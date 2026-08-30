cmake_minimum_required(VERSION 3.16)
project(factory_task_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(factory_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(factory_task_bt_executor
  src/order_tree_runner.cpp
  src/physical_step_action.cpp
  src/task_tree_runner.cpp
  src/task_bt_executor.cpp
)
target_include_directories(factory_task_bt_executor PRIVATE include)
ament_target_dependencies(factory_task_bt_executor
  ament_index_cpp behaviortree_cpp factory_interfaces rclcpp rclcpp_action
  sensor_msgs)

install(TARGETS factory_task_bt_executor DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.orig" EXCLUDE
  PATTERN "*.rej" EXCLUDE
)

ament_package()
