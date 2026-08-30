#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=48
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_manipulation_smoke.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=false headless:=true >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

for _ in $(seq 1 15); do
  if ros2 action list 2>/dev/null | grep -q '^/manipulate_part$'; then
    break
  fi
  sleep 2
done

action_info="$(ros2 action info /manipulate_part 2>/dev/null || true)"
if ! grep -q 'Action servers: 1' <<<"${action_info}"; then
  echo "ManipulatePart action server did not become ready"
  tail -160 "${log_file}"
  exit 1
fi

set +e
rejection_output="$(timeout 10 ros2 action send_goal \
  /manipulate_part factory_interfaces/action/ManipulatePart \
  "{operation: 0, station_id: missing_station, part_id: raw_part_1, placement_slot_id: 0}" \
  2>&1)"
rejection_status=$?
set -e

if ((rejection_status == 124)) ||
  ! grep -qi 'rejected' <<<"${rejection_output}"; then
  echo "Invalid manipulation request was not rejected"
  echo "${rejection_output}"
  tail -160 "${log_file}"
  exit 1
fi

echo "manipulation_action_ready=true invalid_goal_rejected=true"
