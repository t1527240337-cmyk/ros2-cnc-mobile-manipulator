#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace factory_bringup
{

struct FilterBox
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;

  bool contains(double x, double y, double z) const
  {
    return x >= min_x && x <= max_x &&
           y >= min_y && y <= max_y &&
           z >= min_z && z <= max_z;
  }
};

class LaserSelfFilter : public rclcpp::Node
{
public:
  LaserSelfFilter()
  : Node("laser_self_filter"),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    filter_box_{
      declare_parameter<double>("min_x", -0.53),
      declare_parameter<double>("max_x", 0.53),
      declare_parameter<double>("min_y", -0.38),
      declare_parameter<double>("max_y", 0.38),
      declare_parameter<double>("min_z", -0.20),
      declare_parameter<double>("max_z", 0.30)},
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/scan");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/scan_filtered");

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LaserSelfFilter::filter_scan, this, std::placeholders::_1));
  }

private:
  void filter_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      // The sensor is rigidly attached to base_link, so the latest transform
      // is sufficient and avoids dropping startup scans because of clock skew.
      transform = tf_buffer_.lookupTransform(
        base_frame_, scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot filter scan without %s <- %s transform: %s",
        base_frame_.c_str(), scan->header.frame_id.c_str(), error.what());
      return;
    }

    auto filtered = *scan;
    const auto & translation = transform.transform.translation;
    const auto & rotation_message = transform.transform.rotation;
    const tf2::Quaternion quaternion(
      rotation_message.x, rotation_message.y, rotation_message.z, rotation_message.w);
    const tf2::Matrix3x3 rotation(quaternion);

    double angle = scan->angle_min;
    for (auto & range : filtered.ranges) {
      if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
        const double sensor_x = range * std::cos(angle);
        const double sensor_y = range * std::sin(angle);
        const double base_x =
          rotation[0][0] * sensor_x + rotation[0][1] * sensor_y + translation.x;
        const double base_y =
          rotation[1][0] * sensor_x + rotation[1][1] * sensor_y + translation.y;
        const double base_z =
          rotation[2][0] * sensor_x + rotation[2][1] * sensor_y + translation.z;

        if (filter_box_.contains(base_x, base_y, base_z)) {
          range = std::numeric_limits<float>::infinity();
        }
      }
      angle += scan->angle_increment;
    }
    publisher_->publish(filtered);
  }

  std::string base_frame_;
  FilterBox filter_box_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace factory_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<factory_bringup::LaserSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
