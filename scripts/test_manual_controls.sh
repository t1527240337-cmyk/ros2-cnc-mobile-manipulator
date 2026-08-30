#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source scripts/setup_ros_env.sh
set -u
source "${project_root}/scripts/test_support.sh"
export ROS_DOMAIN_ID=47
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
export GZ_SIM_HEADLESS_RENDERING=1

log_file="/tmp/factory_manual_controls_test.log"
setsid ros2 launch factory_bringup gazebo.launch.py \
  physics_engine:=gz-physics-dartsim-plugin \
  >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

controller_output=""
for _ in $(seq 1 120); do
  controller_output="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  if [[ "${controller_output}" == *"gripper_controller"*"active"* ]]; then
    break
  fi
  sleep 0.25
done
if [[ "${controller_output}" != *"gripper_controller"*"active"* ]]; then
  tail -160 "${log_file}"
  exit 1
fi

echo "controllers_ready"
./scripts/set_cnc_door.sh machine_1 open
./scripts/set_cnc_door.sh machine_1 close
timeout 20 ./scripts/set_gripper.sh close
timeout 20 ./scripts/set_gripper.sh open
echo "manual_controls_ok"
