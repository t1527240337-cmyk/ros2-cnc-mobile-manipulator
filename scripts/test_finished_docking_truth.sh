#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=47
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_finished_docking_truth.log"
robot_x="${ROBOT_X:-2.85}"
robot_y="${ROBOT_Y:--2.70}"
robot_yaw="${ROBOT_YAW:-0.0}"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true \
  use_moveit:=false \
  enable_perception:=true \
  headless:=true \
  robot_x:="${robot_x}" \
  robot_y:="${robot_y}" \
  robot_yaw:="${robot_yaw}" >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

fail_with_log() {
  echo "$1"
  tail -180 "${log_file}"
  exit 1
}

for _ in $(seq 1 18); do
  lifecycle_state="$(timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true)"
  if grep -q '^active \[3\]$' <<<"${lifecycle_state}"; then
    break
  fi
  sleep 2
done
grep -q '^active \[3\]$' <<<"${lifecycle_state}" ||
  fail_with_log "Docking server did not become active"

timeout 150 ros2 run factory_core dock_station --ros-args \
  -p station:=finished_bin \
  -p navigate_to_staging:=false \
  -p docking_timeout:=120.0 ||
  fail_with_log "Finished-bin local docking failed"

truth_position="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.position)"
truth_orientation="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.orientation)"
truth_x="$(awk '/x:/{print $2; exit}' <<<"${truth_position}")"
truth_y="$(awk '/y:/{print $2; exit}' <<<"${truth_position}")"
truth_z="$(awk '/z:/{print $2; exit}' <<<"${truth_orientation}")"
truth_w="$(awk '/w:/{print $2; exit}' <<<"${truth_orientation}")"

awk -v x="${truth_x}" -v y="${truth_y}" -v z="${truth_z}" -v w="${truth_w}" 'BEGIN {
  target_x = 3.268
  target_y = -2.70
  target_yaw = 0.0
  position_error = sqrt((x-target_x)^2 + (y-target_y)^2)
  yaw = 2.0 * atan2(z, w)
  yaw_error = atan2(sin(yaw-target_yaw), cos(yaw-target_yaw))
  printf "finished_docking_truth x=%.3f y=%.3f position_error=%.3f m ", x, y, position_error
  printf "yaw=%.3f yaw_error=%.3f rad\n", yaw, yaw_error
  if (position_error > 0.03 || sqrt(yaw_error*yaw_error) > 0.0524) exit 1
}'
