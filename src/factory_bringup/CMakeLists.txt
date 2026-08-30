cmake_minimum_required(VERSION 3.8)
project(factory_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(laser_self_filter src/laser_self_filter.cpp)
ament_target_dependencies(laser_self_filter rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS laser_self_filter DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY behavior_trees config launch maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.orig" EXCLUDE
  PATTERN "*.rej" EXCLUDE
)
ament_package()
