#!/usr/bin/env bash
set -eo pipefail

# End-to-end physical factory acceptance test:
# raw tray -> CNC -> processing -> CNC unload -> finished tray.
#
# Every PLC register write follows a verified Gazebo manipulation. A failed
# grasp or placement therefore cannot advance the semantic machine state.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

headless="${HEADLESS:-true}"
case "${headless}" in
  true)
    export GZ_SIM_HEADLESS_RENDERING=1
    ;;
  false)
    unset GZ_SIM_HEADLESS_RENDERING
    echo "[factory-cycle] Gazebo GUI enabled; this test takes about 9 minutes"
    ;;
  *)
    echo "HEADLESS must be true or false"
    exit 2
    ;;
esac

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=154
export GZ_PARTITION=factory_test_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_cycle_truth.log"
contact_log="/tmp/factory_cycle_contacts.log"

# A previously interrupted test can leave a domain-scoped CLI daemon behind.
# Remove only processes belonging to this test's ROS domain / Gazebo partition.
terminate_test_processes

setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true headless:="${headless}" >"${log_file}" 2>&1 &
launch_pid=$!

(
  for _ in $(seq 1 90); do
    if ros2 topic type /factory/cnc/contacts 2>/dev/null |
      grep -q 'ros_gz_interfaces/msg/Contacts'; then
      exec env PYTHONUNBUFFERED=1 timeout 1800 \
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
  tail -240 "${log_file}"
  echo "Recent CNC collision names:"
  grep 'name:' "${contact_log}" 2>/dev/null |
    tail -80 || true
  exit 1
}

wait_for_stack() {
  local lifecycle_state manipulation_info docking_info
  local undocking_info service_info
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
  echo "[factory-cycle] docking at ${station}"
  timeout 360 ros2 run factory_core dock_station --ros-args \
    -p station:="${station}" \
    -p navigate_to_staging:=true \
    -p staging_timeout:=180.0 \
    -p docking_timeout:=300.0 ||
    fail_with_log "Could not dock at ${station}"
}

undock() {
  local dock_type="$1"
  local stow_arm="${2:-false}"
  echo "[factory-cycle] leaving the current dock"
  timeout 240 ros2 run factory_core undock_station --ros-args \
    -p dock_type:="${dock_type}" \
    -p undocking_timeout:=75.0 \
    -p stow_arm:="${stow_arm}" ||
    fail_with_log "Could not leave the current dock"
}

manipulate() {
  local operation="$1"
  local station="$2"
  local slot="$3"
  local part="$4"
  local output
  echo "[factory-cycle] operation=${operation} station=${station}" \
    "slot=${slot} part=${part}"
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
  if [[ "${operation}" == "0" ]]; then
    grep -q 'VERIFY_LOAD_BEARING_GRASP' <<<"${output}" ||
      fail_with_log "Pick at ${station} bypassed the unassisted proof lift"
  fi
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
  echo "[factory-cycle] machine_2 accepted command=${command}" \
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

require_empty_idle_machine() {
  local message
  message="$(read_machine_state)" ||
    fail_with_log "No final machine_2 state"
  grep -q 'state: 0' <<<"${message}" ||
    fail_with_log "machine_2 did not return to IDLE after unload"
  grep -q 'door_open: true' <<<"${message}" ||
    fail_with_log "machine_2 door is not open after unload"
  grep -q "part_id: ''" <<<"${message}" ||
    fail_with_log "machine_2 retained a stale part register after unload"
}

wait_for_stack || fail_with_log "Physical action servers did not become ready"

# Load raw stock only after physical placement has passed.
dock_at raw_bin
manipulate 0 raw_bin 0 raw_part_2
undock factory_bin_station
machine_command 0
dock_at machine_2
manipulate 1 machine_2 0 raw_part_2
machine_command 7 raw_part_2
undock factory_station true
machine_command 1
machine_command 2

# The simulated PLC opens the door and publishes DONE independently. The robot
# then returns, proves the grasp, and only afterwards clears the part register.
wait_until_done ||
  fail_with_log "machine_2 did not finish and expose an open unload state"
dock_at machine_2
manipulate 0 machine_2 0 raw_part_2
machine_command 8 raw_part_2
undock factory_station

# The physical model keeps its stable Gazebo entity ID. Its processed lifecycle
# is represented by the completed CNC handshake, not by teleporting or renaming.
dock_at finished_bin
manipulate 1 finished_bin 2 raw_part_2
undock factory_bin_station
require_empty_idle_machine

echo "factory_cycle_success=true machine=machine_2 part=raw_part_2" \
  "source=raw_bin:2 sink=finished_bin:2" \
  "physics=contact_verified_constraint plc_handshake=verified"
