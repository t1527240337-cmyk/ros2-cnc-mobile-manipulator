#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=46
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_docking_truth.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true use_moveit:="${USE_MOVEIT:-false}" headless:=true >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
  sleep 0.5
  terminate_test_processes
}
trap cleanup EXIT

wait_for_navigation() {
  local state
  for _ in $(seq 1 12); do
    state="$(timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true)"
    if grep -q '^active \[3\]$' <<<"${state}"; then
      return 0
    fi
    sleep 3
  done
  return 1
}

fail_with_launch_log() {
  echo "$1"
  tail -160 "${log_file}"
  exit 1
}

wait_for_navigation ||
  fail_with_launch_log "Docking server did not become active"
timeout 15 ros2 topic echo /base_controller/odom --once >/dev/null 2>&1 ||
  fail_with_launch_log "Base odometry did not become available"

velocity_info="$(ros2 topic info /cmd_vel)"
if ! grep -q 'Subscription count: 1' <<<"${velocity_info}"; then
  fail_with_launch_log "Docking velocity relay is not connected to /cmd_vel"
fi

if ! timeout 260 ros2 run factory_core dock_station --ros-args \
  -p station:=raw_bin \
  -p navigate_to_staging:=true \
  -p staging_timeout:=180.0 \
  -p docking_timeout:=240.0; then
  echo "ground_truth_at_dock_failure:"
  timeout 5 ros2 topic echo /ground_truth/odom --once \
    --field pose.pose || true
  fail_with_launch_log "Dock action failed"
fi

echo "raw_bin_tag_transform:"
timeout 3 ros2 run tf2_ros tf2_echo base_link raw_bin_tag -r 1 ||
  true

truth_position="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.position)"
truth_x="$(awk '/x:/{print $2; exit}' <<<"${truth_position}")"
truth_y="$(awk '/y:/{print $2; exit}' <<<"${truth_position}")"
truth_orientation="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.orientation)"
truth_z="$(awk '/z:/{print $2; exit}' <<<"${truth_orientation}")"
truth_w="$(awk '/w:/{print $2; exit}' <<<"${truth_orientation}")"


awk -v x="${truth_x}" -v y="${truth_y}" -v z="${truth_z}" -v w="${truth_w}" 'BEGIN {
  # The tag is the dock reference. The plugin offset places the robot base
  # center 0.62 m in front of the raw-bin marker.
  target_x = -3.268
  target_y = -2.70
  target_yaw = 3.141592653589793
  error = sqrt((x-target_x)^2 + (y-target_y)^2)
  yaw = 2.0 * atan2(z, w)
  yaw_error = atan2(sin(yaw-target_yaw), cos(yaw-target_yaw))
  printf "docking_truth x=%.3f y=%.3f position_error=%.3f m ", x, y, error
  printf "yaw=%.3f yaw_error=%.3f rad\n", yaw, yaw_error
  if (error > 0.03 || sqrt(yaw_error*yaw_error) > 0.0524) exit 1
}'
