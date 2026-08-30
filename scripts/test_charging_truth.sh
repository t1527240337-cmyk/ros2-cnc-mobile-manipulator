#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=151
export GZ_PARTITION=factory_test_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_charging_truth.log"
lock_file="/tmp/factory_charging_truth.lock"
exec 9>"${lock_file}"
if ! flock -n 9; then
  echo "Another charging acceptance is already running"
  echo "Wait for it to finish instead of sharing ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  exit 75
fi

terminate_test_processes
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true headless:=true 9>&- >"${log_file}" 2>&1 &
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

fail_with_log() {
  echo "$1"
  tail -180 "${log_file}"
  exit 1
}

wait_for_stack() {
  local lifecycle_state battery_info
  for _ in $(seq 1 18); do
    lifecycle_state="$(
      timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true
    )"
    battery_info="$(ros2 topic info /battery_state 2>/dev/null || true)"
    if grep -q '^active \[3\]$' <<<"${lifecycle_state}" &&
      grep -q 'Publisher count: 1' <<<"${battery_info}"; then
      return 0
    fi
    sleep 3
  done
  return 1
}

read_battery() {
  timeout 8 ros2 topic echo /battery_state --once
}

wait_for_stack || fail_with_log "Charging dependencies did not become ready"
initial_message="$(read_battery)" ||
  fail_with_log "No battery state before docking"
initial_percentage="$(
  awk '/percentage:/{print $2; exit}' <<<"${initial_message}"
)"

timeout 360 ros2 run factory_core dock_station --ros-args \
  -p station:=charge_dock \
  -p navigate_to_staging:=true \
  -p staging_timeout:=180.0 \
  -p docking_timeout:=300.0 ||
  fail_with_log "Could not dock at the charging station"

docked_message="$(read_battery)" ||
  fail_with_log "No battery state when charging starts"
docked_percentage="$(
  awk '/percentage:/{print $2; exit}' <<<"${docked_message}"
)"
power_status="$(
  awk '/power_supply_status:/{print $2; exit}' <<<"${docked_message}"
)"
charging_current="$(
  awk '/current:/{print $2; exit}' <<<"${docked_message}"
)"

sleep 3
charged_message="$(read_battery)" ||
  fail_with_log "No battery state after charging interval"
charged_percentage="$(
  awk '/percentage:/{print $2; exit}' <<<"${charged_message}"
)"

truth_position="$(
  timeout 8 ros2 topic echo /ground_truth/odom --once \
    --field pose.pose.position
)" || fail_with_log "No Gazebo ground-truth odometry after docking"
truth_x="$(awk '/x:/{print $2; exit}' <<<"${truth_position}")"
truth_y="$(awk '/y:/{print $2; exit}' <<<"${truth_position}")"

awk \
  -v initial="${initial_percentage}" \
  -v docked="${docked_percentage}" \
  -v charged="${charged_percentage}" \
  -v status="${power_status}" \
  -v current="${charging_current}" \
  -v x="${truth_x}" \
  -v y="${truth_y}" \
  'BEGIN {
    target_x = 0.0
    target_y = -3.087
    error = sqrt((x-target_x)^2 + (y-target_y)^2)
    navigation_delta = docked - initial
    charge_gain = charged - docked
    printf "charging_truth x=%.3f y=%.3f position_error=%.3f m ", x, y, error
    printf "initial=%.3f docked=%.3f navigation_delta=%.3f ",
      initial, docked, navigation_delta
    printf "charged=%.3f charge_gain=%.3f status=%d current=%.1f A\n",
      charged, charge_gain, status, current
    valid = error <= 0.03 && status == 1 && current > 0.5
    if (!valid || charge_gain < 0.01) exit 1
  }' || fail_with_log "Physical charging truth check failed"
