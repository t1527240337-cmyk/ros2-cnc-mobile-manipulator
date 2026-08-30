#ifndef FACTORY_DOCKING__STRICT_NON_CHARGING_DOCK_HPP_
#define FACTORY_DOCKING__STRICT_NON_CHARGING_DOCK_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opennav_docking/simple_non_charging_dock.hpp"

namespace factory_docking
{

class StrictNonChargingDock
  : public opennav_docking::SimpleNonChargingDock
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  bool getRefinedPose(
    geometry_msgs::msg::PoseStamped & pose,
    std::string id) override;

  bool isDocked() override;

private:
  double angular_threshold_{0.0524};
  bool use_nominal_pose_guard_{false};
  double max_refinement_translation_{0.015};
  double max_refinement_yaw_{0.0262};
  double nominal_target_translation_x_{0.0};
  double nominal_target_translation_y_{0.0};
  bool has_nominal_reference_{false};
  geometry_msgs::msg::PoseStamped nominal_reference_;
};

}  // namespace factory_docking

#endif  // FACTORY_DOCKING__STRICT_NON_CHARGING_DOCK_HPP_
