cmake_minimum_required(VERSION 3.8)
project(factory_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MachineState.msg"
  "msg/RobotTaskState.msg"
  "msg/TraySlotState.msg"
  "msg/TrayOccupancy.msg"
  "action/ExecuteOrder.action"
  action/ExecutePhysicalStep.action
  "action/ExecuteRobotTask.action"
  "action/ManipulatePart.action"
  "srv/ControlTask.srv"
  "srv/ControlProduction.srv"
  "srv/GetFactoryState.srv"
  "srv/ExecuteAgentCommand.srv"
  "srv/GetRobotTaskQueue.srv"
  "srv/MachineCommand.srv"
  "srv/PartTransfer.srv"
  "srv/ReconcileRobotTask.srv"
  "srv/SubmitNaturalLanguage.srv"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
