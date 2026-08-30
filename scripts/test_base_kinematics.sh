#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=44
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_base_kinematics.log"
# Use the same DART engine and physical wheel contacts as the production stack.
setsid ros2 launch factory_bringup gazebo.launch.py headless:=true \
  >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

controller_output=""
for _ in $(seq 1 100); do
  controller_output="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  if [[ "${controller_output}" == *"base_controller"*"active"* ]]; then
    break
  fi
  sleep 0.25
done
if [[ "${controller_output}" != *"base_controller"*"active"* ]]; then
  echo "Base controller did not become active"
  tail -120 "${log_file}"
  exit 1
fi

# A short turn hid accumulated heading drift that was large enough to leave
# AprilTags outside the camera view near a station. Exercise roughly a
# half-turn and keep the odometry/truth mismatch below 2.3 degrees.
ros2 run factory_core check_base_kinematics --ros-args \
  -p use_sim_time:=true \
  -p turn_duration:=5.0 \
  -p yaw_tolerance:=0.08 \
  "$@"
