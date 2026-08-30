#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=152
export GZ_PARTITION=factory_test_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_raw_to_machine_truth.log"
contact_log="/tmp/factory_raw_to_machine_contacts.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true headless:=true >"${log_file}" 2>&1 &
launch_pid=$!
(
  for _ in $(seq 1 90); do
    if ros2 topic type /factory/cnc/contacts 2>/dev/null |
      grep -q 'ros_gz_interfaces/msg/Contacts'; then
      exec env PYTHONUNBUFFERED=1 timeout 600 ros2 topic echo /factory/cnc/contacts
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
  local lifecycle_state manipulation_info docking_info undocking_info
  local service_info
  for _ in $(seq 1 18); do
    lifecycle_state="$(
      timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true
    )"
    manipulation_info="$(
      ros2 action info /manipulate_part 2>/dev/null || true
    )"
    docking_info="$(ros2 action info /dock_robot 2>/dev/null || true)"
    undocking_info="$(ros2 action info /undock_robot 2>/dev/null || true)"
    service_info="$(ros2 service type /factory/machine_command 2>/dev/null || true)"
    if grep -q '^active \[3\]$' <<<"${lifecycle_state}" &&
      grep -q 'Action servers: 1' <<<"${manipulation_info}" &&
      grep -q 'Action servers: 1' <<<"${docking_info}" &&
      grep -q 'Action servers: 1' <<<"${undocking_info}" &&
      grep -q 'factory_interfaces/srv/MachineCommand' <<<"${service_info}"; then
      return 0
    fi
    sleep 3
  done
  return 1
}

dock_at() {
  local station="$1"
  echo "[machine-cycle] docking at ${station}"
  timeout 360 ros2 run factory_core dock_station --ros-args \
    -p station:="${station}" \
    -p navigate_to_staging:=true \
    -p staging_timeout:=180.0 \
    -p docking_timeout:=300.0 ||
    fail_with_log "Could not dock at ${station}"
}

undock() {
  local dock_type="$1"
  echo "[machine-cycle] leaving the current dock"
  timeout 180 ros2 run factory_core undock_station --ros-args \
    -p dock_type:="${dock_type}" \
    -p undocking_timeout:=75.0 ||
    fail_with_log "Could not leave the current dock"
}

manipulate() {
  local operation="$1"
  local station="$2"
  local slot="$3"
  local part="$4"
  local output
  echo "[machine-cycle] operation=${operation} station=${station} part=${part}"
  output="$(timeout 600 ros2 action send_goal --feedback \
    /manipulate_part factory_interfaces/action/ManipulatePart \
    "{operation: ${operation}, station_id: ${station}, part_id: ${part}, placement_slot_id: ${slot}}" \
    2>&1)" || {
      echo "${output}"
      fail_with_log "Manipulation action failed at ${station}"
    }
  grep -q 'success: true' <<<"${output}" || {
    echo "${output}"
    fail_with_log "Manipulation action did not succeed at ${station}"
  }
}

machine_command() {
  local command="$1"
  local part_id="${2:-}"
  local output
  output="$(timeout 15 ros2 service call \
    /factory/machine_command factory_interfaces/srv/MachineCommand \
    "{machine_id: machine_2, command: ${command}, part_id: '${part_id}'}" \
    2>&1)" || fail_with_log "Machine command ${command} failed"
  if ! grep -Eq 'accepted=(True|true)|accepted: true' <<<"${output}"; then
    echo "${output}"
    fail_with_log "Machine command ${command} was rejected"
  fi
  echo "[machine-cycle] machine_2 accepted command=${command}" \
    "part=${part_id:-none}"
}

read_machine_state() {
  timeout 8 ros2 topic echo /machine_2/state --once
}

wait_until_done() {
  local message state door_open
  for _ in $(seq 1 30); do
    message="$(read_machine_state)" ||
      fail_with_log "No machine_2 state while waiting for completion"
    state="$(awk '/state:/{print $2; exit}' <<<"${message}")"
    door_open="$(awk '/door_open:/{print $2; exit}' <<<"${message}")"
    if [[ "${state}" == "3" && "${door_open}" == "true" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_stack || fail_with_log "Physical action servers did not become ready"

dock_at raw_bin
manipulate 0 raw_bin 0 raw_part_2
undock factory_bin_station

# OPEN_DOOR is accepted only in a safe machine state. The visualizer then
# drives the physical prismatic door while the robot navigates to the station.
machine_command 0
dock_at machine_2
manipulate 1 machine_2 0 raw_part_2

# Register writes happen only after ManipulatePart has verified the physical
# placement. The mobile manipulator leaves the door envelope before the PLC is
# allowed to close the physical door and start machining.
machine_command 7 raw_part_2
undock factory_station
machine_command 1
machine_command 2
wait_until_done ||
  fail_with_log "machine_2 did not finish and expose an open unload state"

final_state="$(read_machine_state)" ||
  fail_with_log "No final machine_2 state"
grep -q 'state: 3' <<<"${final_state}" ||
  fail_with_log "machine_2 is not DONE"
grep -q 'door_open: true' <<<"${final_state}" ||
  fail_with_log "machine_2 did not open its door after processing"
grep -q 'part_id: raw_part_2' <<<"${final_state}" ||
  fail_with_log "machine_2 lost its physical part register"

echo "machine_load_success=true machine=machine_2 part=raw_part_2" \
  "state=DONE door=open physics=contact_verified_constraint"
