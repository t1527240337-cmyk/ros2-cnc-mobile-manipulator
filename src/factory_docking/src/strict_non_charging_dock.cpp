#include "factory_docking/strict_non_charging_dock.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace factory_docking
{

void StrictNonChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  opennav_docking::SimpleNonChargingDock::configure(parent, name, tf);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".angular_threshold", rclcpp::ParameterValue(0.0524));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_nominal_pose_guard", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_refinement_translation", rclcpp::ParameterValue(0.015));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_refinement_yaw", rclcpp::ParameterValue(0.0262));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".nominal_target_translation_x",
    rclcpp::ParameterValue(external_detection_translation_x_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".nominal_target_translation_y",
    rclcpp::ParameterValue(external_detection_translation_y_));

  node_->get_parameter(name + ".angular_threshold", angular_threshold_);
  node_->get_parameter(name + ".use_nominal_pose_guard", use_nominal_pose_guard_);
  node_->get_parameter(
    name + ".max_refinement_translation", max_refinement_translation_);
  node_->get_parameter(name + ".max_refinement_yaw", max_refinement_yaw_);
  node_->get_parameter(
    name + ".nominal_target_translation_x", nominal_target_translation_x_);
  node_->get_parameter(
    name + ".nominal_target_translation_y", nominal_target_translation_y_);

  if (angular_threshold_ <= 0.0 || angular_threshold_ > M_PI) {
    throw std::invalid_argument("angular_threshold must be in (0, pi]");
  }
  if (max_refinement_translation_ < 0.0) {
    throw std::invalid_argument("max_refinement_translation must be non-negative");
  }
  if (max_refinement_yaw_ < 0.0 || max_refinement_yaw_ > M_PI) {
    throw std::invalid_argument("max_refinement_yaw must be in [0, pi]");
  }
}

bool StrictNonChargingDock::getRefinedPose(
  geometry_msgs::msg::PoseStamped & pose,
  std::string id)
{
  auto target_from_reference = [this](
    const geometry_msgs::msg::PoseStamped & reference)
    {
      auto target = reference;
      const double yaw = tf2::getYaw(reference.pose.orientation);
      target.pose.position.x +=
        nominal_target_translation_x_ * std::cos(yaw) -
        nominal_target_translation_y_ * std::sin(yaw);
      target.pose.position.y +=
        nominal_target_translation_x_ * std::sin(yaw) +
        nominal_target_translation_y_ * std::cos(yaw);
      target.pose.position.z = 0.0;
      return target;
    };

  // DockingServer reuses and mutates its pose argument on every control cycle.
  // Capture the database pose only at the start of an approach. A later input
  // near our previous work target is the refined output coming back, not a new
  // calibration reference. The configured nominal translation maps each
  // database dock pose to its repeatable manipulation work pose.
  if (!has_nominal_reference_) {
    nominal_reference_ = pose;
    has_nominal_reference_ = true;
  } else {
    const auto previous_target = target_from_reference(nominal_reference_);
    const double distance_from_previous_target = std::hypot(
      pose.pose.position.x - previous_target.pose.position.x,
      pose.pose.position.y - previous_target.pose.position.y);
    if (pose.header.frame_id != nominal_reference_.header.frame_id ||
      distance_from_previous_target > 0.20)
    {
      nominal_reference_ = pose;
    }
  }

  if (!opennav_docking::SimpleNonChargingDock::getRefinedPose(pose, id)) {
    return false;
  }
  if (!use_nominal_pose_guard_) {
    return true;
  }

  const auto nominal_target = target_from_reference(nominal_reference_);
  double dx = pose.pose.position.x - nominal_target.pose.position.x;
  double dy = pose.pose.position.y - nominal_target.pose.position.y;
  const double translation = std::hypot(dx, dy);
  if (translation > max_refinement_translation_ && translation > 0.0) {
    const double scale = max_refinement_translation_ / translation;
    dx *= scale;
    dy *= scale;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Dock %s visual correction %.3f m exceeds %.3f m; "
      "clamping to calibrated map prior",
      id.c_str(), translation, max_refinement_translation_);
  }
  pose.pose.position.x = nominal_target.pose.position.x + dx;
  pose.pose.position.y = nominal_target.pose.position.y + dy;
  pose.pose.position.z = 0.0;

  const double nominal_yaw =
    tf2::getYaw(nominal_reference_.pose.orientation);
  const double visual_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw_correction = std::remainder(
    visual_yaw - nominal_yaw, 2.0 * M_PI);
  yaw_correction = std::clamp(
    yaw_correction, -max_refinement_yaw_, max_refinement_yaw_);
  tf2::Quaternion bounded_orientation;
  bounded_orientation.setRPY(0.0, 0.0, nominal_yaw + yaw_correction);
  pose.pose.orientation = tf2::toMsg(bounded_orientation);

  // isDocked() evaluates this bounded target, so success means the base
  // reached the calibrated work pose rather than merely following a noisy tag.
  dock_pose_ = pose;
  dock_pose_pub_->publish(dock_pose_);
  return true;
}

bool StrictNonChargingDock::isDocked()
{
  if (dock_pose_.header.frame_id.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.stamp = rclcpp::Time(0);
  base_pose.header.frame_id = base_frame_id_;
  base_pose.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(
      base_pose, base_pose, dock_pose_.header.frame_id);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double target_yaw = tf2::getYaw(dock_pose_.pose.orientation);
  const double dx =
    base_pose.pose.position.x - dock_pose_.pose.position.x;
  const double dy =
    base_pose.pose.position.y - dock_pose_.pose.position.y;
  const double longitudinal_error =
    dx * std::cos(target_yaw) + dy * std::sin(target_yaw);
  const double lateral_error =
    -dx * std::sin(target_yaw) + dy * std::cos(target_yaw);
  const double heading_error = std::remainder(
    tf2::getYaw(base_pose.pose.orientation) -
    target_yaw,
    2.0 * M_PI);

  // Nav2 aims its controller 250 mm beyond the dock and relies on this
  // method to stop the base. A low-rate simulation can step across a small
  // circular tolerance without ever sampling inside it. CNC workstations use
  // a calibrated stopping plane instead: once the bumper reaches that plane,
  // only lateral and heading accuracy remain relevant.
  return longitudinal_error >= -docking_threshold_ &&
         std::abs(lateral_error) < docking_threshold_ &&
         std::abs(heading_error) < angular_threshold_;
}

}  // namespace factory_docking

PLUGINLIB_EXPORT_CLASS(
  factory_docking::StrictNonChargingDock,
  opennav_docking_core::ChargingDock)
