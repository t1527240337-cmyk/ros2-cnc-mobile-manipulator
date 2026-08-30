#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=56
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_safe_transit_truth.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true \
  use_moveit:=false \
  headless:=true \
  robot_x:=-1.94 \
  robot_y:=-2.57 \
  robot_yaw:=3.14159 >"${log_file}" 2>&1 &
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
  lifecycle_state="$(
    timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true
  )"
  route_info="$(ros2 action info /navigate_through_poses 2>/dev/null || true)"
  if grep -q '^active \[3\]$' <<<"${lifecycle_state}" &&
    grep -q 'Action servers: 1' <<<"${route_info}"; then
    break
  fi
  sleep 2
done
grep -q '^active \[3\]$' <<<"${lifecycle_state}" ||
  fail_with_log "Docking server did not become active"

timeout 380 ros2 run factory_core navigate_route --ros-args \
  -p route:=raw_bin_to_finished_bin \
  -p navigation_timeout:=360.0 ||
  fail_with_log "Safe transit route failed"

timeout 220 ros2 run factory_core dock_station --ros-args \
  -p station:=finished_bin \
  -p navigate_to_staging:=true \
  -p staging_timeout:=150.0 \
  -p docking_timeout:=180.0 ||
  fail_with_log "Finished-bin docking after safe transit failed"

truth_position="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.position)"
truth_x="$(awk '/x:/{print $2; exit}' <<<"${truth_position}")"
truth_y="$(awk '/y:/{print $2; exit}' <<<"${truth_position}")"

awk -v x="${truth_x}" -v y="${truth_y}" 'BEGIN {
  target_x = 3.268
  target_y = -2.70
  error = sqrt((x-target_x)^2 + (y-target_y)^2)
  printf "safe_transit_truth x=%.3f y=%.3f position_error=%.3f m\n",
    x, y, error
  if (error > 0.03) exit 1
}'
