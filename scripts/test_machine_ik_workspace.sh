#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=154
export GZ_PARTITION=factory_machine_ik_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_machine_ik_workspace.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=false \
  use_moveit:=true \
  enable_perception:=false \
  headless:=true \
  robot_x:=0.8 \
  robot_y:=1.50 \
  robot_yaw:=1.5708 >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

ready=false
for _ in $(seq 1 60); do
  if ros2 service type /compute_ik 2>/dev/null |
    grep -q 'moveit_msgs/srv/GetPositionIK'; then
    ready=true
    break
  fi
  sleep 2
done
if [[ "${ready}" != true ]]; then
  tail -120 "${log_file}"
  exit 1
fi

python3 "${project_root}/scripts/probe_loaded_carry_ik.py" \
  --x 0.80 0.85 0.90 \
  --y 0.0 \
  --z 0.816 0.866 0.996 1.25 \
  --timeout 3.0

echo "camera-clear loaded carry candidates:"
python3 "${project_root}/scripts/probe_loaded_carry_ik.py" \
  --x 0.20 0.30 0.40 \
  --y 0.35 0.45 \
  --z 0.90 1.05 1.20 \
  --timeout 3.0

echo "machine door-clear candidates:"
python3 "${project_root}/scripts/probe_loaded_carry_ik.py" \
  --x 0.52 0.55 0.58 \
  --y 0.0 0.10 0.20 \
  --z 1.10 1.15 1.20 1.25 \
  --timeout 3.0
