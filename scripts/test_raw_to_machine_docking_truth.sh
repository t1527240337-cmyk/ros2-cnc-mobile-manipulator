#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=154
export GZ_PARTITION=factory_raw_to_machine_docking_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_raw_to_machine_docking_truth.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true \
  use_moveit:=false \
  enable_perception:=true \
  headless:=true >"${log_file}" 2>&1 &
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

for _ in $(seq 1 20); do
  lifecycle_state="$(
    timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true
  )"
  if grep -q '^active \[3\]$' <<<"${lifecycle_state}"; then
    break
  fi
  sleep 2
done
grep -q '^active \[3\]$' <<<"${lifecycle_state}" ||
  fail_with_log "Docking server did not become active"

dock() {
  local station="$1"
  timeout 180 ros2 run factory_core dock_station --ros-args \
    -p station:="${station}" \
    -p staging_timeout:=150.0 \
    -p docking_timeout:=150.0 ||
    fail_with_log "Docking failed at ${station}"
}

dock raw_bin
timeout 90 ros2 run factory_core undock_station --ros-args \
  -p dock_type:=factory_bin_station \
  -p stow_arm:=false ||
  fail_with_log "Undocking from raw_bin failed"
dock machine_2

truth_position="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.position)"
truth_orientation="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.orientation)"
truth_x="$(awk '/x:/{print $2; exit}' <<<"${truth_position}")"
truth_y="$(awk '/y:/{print $2; exit}' <<<"${truth_position}")"
truth_z="$(awk '/z:/{print $2; exit}' <<<"${truth_orientation}")"
truth_w="$(awk '/w:/{print $2; exit}' <<<"${truth_orientation}")"

awk -v x="${truth_x}" -v y="${truth_y}" -v z="${truth_z}" -v w="${truth_w}" '
BEGIN {
  target_x = 0.8
  # Collision-bounded CNC work pose; the base front stays outside the door.
  target_y = 1.38
  target_yaw = 1.5708
  position_error = sqrt((x-target_x)^2 + (y-target_y)^2)
  yaw = 2.0 * atan2(z, w)
  yaw_error = atan2(sin(yaw-target_yaw), cos(yaw-target_yaw))
  printf "raw_to_machine_docking_truth x=%.3f y=%.3f ", x, y
  printf "position_error=%.3f m yaw_error=%.3f rad\n", position_error, yaw_error
  if (position_error > 0.03 || sqrt(yaw_error*yaw_error) > 0.0524) exit 1
}'
