#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=49
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_raw_pick_truth.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=false \
  enable_perception:=true \
  headless:=true \
  robot_x:=-3.268 \
  robot_y:=-2.70 \
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

controller_output=""
action_info=""
for _ in $(seq 1 100); do
  controller_output="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  action_info="$(ros2 action info /manipulate_part 2>/dev/null || true)"
  if [[ "${controller_output}" == *"arm_controller"*"active"* \
     && "${controller_output}" == *"gripper_controller"*"active"* ]] &&
    grep -q 'Action servers: 1' <<<"${action_info}"; then
    break
  fi
  sleep 0.25
done

if [[ "${controller_output}" != *"arm_controller"*"active"* \
   || "${controller_output}" != *"gripper_controller"*"active"* ]]; then
  fail_with_log "Manipulation controllers did not become active"
fi

grep -q 'Action servers: 1' <<<"${action_info}" ||
  fail_with_log "Manipulation server did not become active"

# A controller-ready robot is not yet a perception-ready robot.  Require one
# non-empty RGB-D observation before opening the action's fresh-frame window;
# the manipulation server will still require its own three new stable frames.
perception_output=""
for _ in $(seq 1 40); do
  perception_output="$(timeout 3 ros2 topic echo --once \
    /perception/raw_part_candidates 2>/dev/null || true)"
  if grep -q 'position:' <<<"${perception_output}"; then
    break
  fi
done

grep -q 'position:' <<<"${perception_output}" ||
  fail_with_log "RGB-D perception did not publish a non-empty raw-part observation"

# The base starts in a known station fixture so this test isolates manipulation.
# Allow two bounded MoveIt retries plus gripper and sensor-evidence checks.
pick_output="$(timeout 420 ros2 action send_goal --feedback \
  /manipulate_part factory_interfaces/action/ManipulatePart \
  "{operation: 0, station_id: raw_bin, part_id: raw_part_2, placement_slot_id: 0}" \
  2>&1)" || {
    echo "${pick_output}"
    fail_with_log "Raw-bin pick action failed"
  }

grep -q 'success: true' <<<"${pick_output}" || {
  echo "${pick_output}"
  fail_with_log "Raw-bin pick action did not report success"
}
grep -q 'VERIFY_TWO_FINGER_CONTACT' <<<"${pick_output}" ||
  fail_with_log "Pick bypassed physical two-finger contact verification"
grep -q 'VERIFY_LOAD_BEARING_GRASP' <<<"${pick_output}" ||
  fail_with_log "Pick bypassed the unassisted proof-lift phase"

echo \
  "raw_pick_success=true physics=bilateral_contact_and_friction" \
  "proof_lift=unassisted part=raw_part_2 station=raw_bin source_reference=1"
