cmake_minimum_required(VERSION 3.8)
project(factory_docking)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(factory_docking_plugins SHARED
  src/strict_non_charging_dock.cpp
)
target_compile_features(factory_docking_plugins PUBLIC cxx_std_17)
target_include_directories(factory_docking_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(factory_docking_plugins
  geometry_msgs
  nav2_util
  opennav_docking
  pluginlib
  tf2
  tf2_geometry_msgs
)
# Jazzy exports the simple dock plugin as a runtime library rather than an
# ament CMake target. The derived completion-policy plugin needs its symbols.
find_library(
  SIMPLE_NON_CHARGING_DOCK_LIBRARY
  simple_non_charging_dock
  REQUIRED
)
target_link_libraries(
  factory_docking_plugins
  ${SIMPLE_NON_CHARGING_DOCK_LIBRARY}
)

pluginlib_export_plugin_description_file(
  opennav_docking_core factory_docking_plugins.xml
)

install(TARGETS factory_docking_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(factory_docking_plugins)
ament_export_dependencies(opennav_docking)
ament_package()
