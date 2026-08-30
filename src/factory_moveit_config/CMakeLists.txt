cmake_minimum_required(VERSION 3.8)
project(factory_moveit_config)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.orig" EXCLUDE
  PATTERN "*.rej" EXCLUDE
)
ament_package()
