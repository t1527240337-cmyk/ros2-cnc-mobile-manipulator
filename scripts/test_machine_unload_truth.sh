#!/usr/bin/env bash
set -eo pipefail

# Isolated physical CNC-unload acceptance test.
#
# The pose reset and fixture attach below create the test fixture only. They
# are not part of the manipulation action or its success criteria. The action
# must still close on bilateral physical contact, lift without an attachment,
# verify load-bearing motion, and release the CNC clamp in the normal order.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=158
export GZ_PARTITION=factory_machine_unload_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_machine_unload_truth.log"
fixture_state_file="/tmp/factory_machine_unload_fixture_state.log"
contact_log="/tmp/factory_machine_unload_contacts.log"
terminate_test_processes

setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true \
  use_moveit:=true \
  enable_perception:=true \
  headless:=true \
  robot_x:=0.8 \
  robot_y:=1.25 \
  robot_yaw:=1.5708 >"${log_file}" 2>&1 &
launch_pid=$!
(
  for _ in $(seq 1 90); do
    if ros2 topic type /factory/cnc/contacts 2>/dev/null |
      grep -q 'ros_gz_interfaces/msg/Contacts'; then
      exec env PYTHONUNBUFFERED=1 timeout 600 \
        ros2 topic echo /factory/cnc/contacts
    fi
    sleep 1
  done
  echo "CNC contact topic did not become available"
) >"${contact_log}" 2>&1 &
contact_pid=$!

cleanup() {
  kill "${contact_pid}" 2>/dev/null || true
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

fail_with_log() {
  echo "$1"
  tail -220 "${log_file}"
  echo "Recent CNC collision names:"
  grep 'name:' "${contact_log}" 2>/dev/null |
    tail -80 || true
  exit 1
}

wait_for_stack() {
  local action_info lifecycle_state
  for _ in $(seq 1 24); do
    action_info="$(ros2 action info /manipulate_part 2>/dev/null || true)"
    lifecycle_state="$(
      timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true
    )"
    if grep -q '^active \[3\]$' <<<"${lifecycle_state}" &&
      grep -q 'Action servers: 1' <<<"${action_info}" &&
      gz service -l 2>/dev/null | grep -q '^/world/multi_machine_factory/set_pose$'; then
      return 0
    fi
    sleep 2
  done
  return 1
}
wait_for_machine_tag() {
  local transform
  for _ in $(seq 1 20); do
    transform="$(
      timeout 2 ros2 run tf2_ros tf2_echo \
        base_link machine_2_tag -r 1 2>/dev/null || true
    )"
    if grep -q 'Translation:' <<<"${transform}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

request_fixture_state() {
  local command="$1"
  local expected_state="$2"

  timeout 8 ros2 topic echo \
    /factory/fixture/raw_part_2/attached --once \
    >"${fixture_state_file}" 2>&1 &
  local state_pid=$!
  sleep 0.5

  # The ROS-to-Gazebo bridge may discover a one-shot publisher too late.
  # Repeat this idempotent request, as the production clamp client does.
  timeout 6 ros2 topic pub --rate 4 \
    "/factory/fixture/raw_part_2/${command}" std_msgs/msg/Empty '{}' \
    >/dev/null 2>&1 &
  local request_pid=$!
  if ! wait "${state_pid}"; then
    kill "${request_pid}" 2>/dev/null || true
    fail_with_log "Fixture clamp did not acknowledge ${command}"
  fi
  kill "${request_pid}" 2>/dev/null || true
  wait "${request_pid}" 2>/dev/null || true
  grep -q "data: ${expected_state}" "${fixture_state_file}" ||
    fail_with_log "Fixture clamp reported the wrong state after ${command}"
}


wait_for_stack ||
  fail_with_log "Manipulation server or Gazebo pose service is unavailable"
wait_for_machine_tag ||
  fail_with_log "AprilTag transform for the CNC fixture is unavailable"

door_command="$(
  timeout 15 ros2 service call \
    /factory/machine_command factory_interfaces/srv/MachineCommand \
    "{machine_id: machine_2, command: 0, part_id: ''}" 2>&1
)" || fail_with_log "Could not request the CNC door to open"
if ! grep -Eq 'accepted=(True|true)|accepted: true' <<<"${door_command}"; then
  echo "${door_command}"
  fail_with_log "CNC door-open command was rejected"
fi

timeout 150 ros2 run factory_core dock_station --ros-args \
  -p station:=machine_2 \
  -p navigate_to_staging:=false \
  -p docking_timeout:=120.0 ||
  fail_with_log "Could not establish the visual CNC manipulation pose"

# Source stock begins attached to a world-fixed tray locator. Release that
# initial fixture before moving the isolated test part into the CNC.
request_fixture_state detach detached

# Prepare the isolated stock at the same sensor-derived, calibrated loading
# datum used by production. This is test setup only; the manipulation server
# has no Gazebo pose API and must still acquire the part through contact.
fixture_pose="$(
  timeout 15 python3 "${project_root}/scripts/resolve_machine_fixture_test_pose.py" \
    machine_2 --ros-args -p use_sim_time:=true
)" || fail_with_log "Could not resolve the calibrated CNC test fixture"
read -r fixture_x fixture_y fixture_z <<<"${fixture_pose}"
[[ -n "${fixture_x}" && -n "${fixture_y}" && -n "${fixture_z}" ]] ||
  fail_with_log "Calibrated CNC test fixture returned an invalid pose"
echo "[machine-unload-fixture] calibrated_world_target=${fixture_x},${fixture_y},${fixture_z}"

pose_result="$(
  gz service -s /world/multi_machine_factory/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "name: 'raw_part_2',
      position: {x: ${fixture_x}, y: ${fixture_y}, z: ${fixture_z}},
      orientation: {w: 1.0}" 2>&1
)" || fail_with_log "Could not prepare the CNC workpiece fixture"
grep -qi 'true' <<<"${pose_result}" ||
  fail_with_log "Gazebo rejected the CNC workpiece fixture pose"

# Reattach at the part's current machine pose to model the CNC vise.
request_fixture_state attach attached
echo "[machine-unload-fixture] measured Gazebo pose after clamp:"
timeout 8 ros2 topic echo /factory/workpieces/raw_part_2/pose --once ||
  fail_with_log "No measured workpiece pose after preparing the CNC fixture"

output="$(
  timeout 600 ros2 action send_goal --feedback \
    /manipulate_part factory_interfaces/action/ManipulatePart \
    "{operation: 0, station_id: machine_2, part_id: raw_part_2, placement_slot_id: 0}" \
    2>&1
)" || {
  echo "${output}"
  fail_with_log "CNC unload manipulation action failed"
}

grep -q 'success: true' <<<"${output}" || {
  echo "${output}"
  fail_with_log "CNC unload manipulation action did not succeed"
}
grep -q 'VERIFY_TWO_FINGER_CONTACT' <<<"${output}" ||
  fail_with_log "CNC unload bypassed bilateral contact verification"
grep -q 'VERIFY_LOAD_BEARING_GRASP' <<<"${output}" ||
  fail_with_log "CNC unload bypassed the unassisted proof lift"
grep -q 'RELEASE_FIXTURE_CLAMP' <<<"${output}" ||
  fail_with_log "CNC unload did not release the fixture after grasp proof"
grep -q 'VERIFY_MACHINE_DOOR_OPEN' <<<"${output}" ||
  fail_with_log "CNC unload bypassed the physical door interlock"

echo "machine_unload_pick_success=true machine=machine_2" \
  "part=raw_part_2 physics=bilateral_contact_unassisted_lift"
