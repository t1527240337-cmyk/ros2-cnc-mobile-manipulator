#!/usr/bin/env bash
set -eo pipefail

# Formal ROS 2 acceptance test for one physical production order. Unlike the
# lower-level factory-cycle test, this sends only ExecuteOrder and verifies
# that orchestration, physical actions, PLC registers and inventory agree.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_started_at="${SECONDS}"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

headless="${HEADLESS:-true}"
case "${headless}" in
  true) export GZ_SIM_HEADLESS_RENDERING=1 ;;
  false) unset GZ_SIM_HEADLESS_RENDERING ;;
  *) echo "HEADLESS must be true or false"; exit 2 ;;
esac
order_entry="${ORDER_ENTRY:-action}"
case "${order_entry}" in
  action|agent_auto|task_queue) ;;
  *) echo "ORDER_ENTRY must be action, agent_auto or task_queue"; exit 2 ;;
esac

order_id="${ORDER_ID:-physical_truth_1}"
order_quantity="${ORDER_QUANTITY:-1}"
allowed_machines_csv="${ALLOWED_MACHINES:-machine_2}"
raw_bin_seed="${RAW_BIN_SEED:-17}"
raw_part_count="${RAW_PART_COUNT:-6}"
robot_x="${ROBOT_X:-0.0}"
robot_y="${ROBOT_Y:--1.2}"
robot_yaw="${ROBOT_YAW:-0.0}"
initial_battery="${INITIAL_BATTERY_PERCENTAGE:-0.42}"
auto_recharge="${AUTO_RECHARGE:-true}"
expect_recharge="${EXPECT_RECHARGE:-false}"
fault_machine_before_order="${FAULT_MACHINE_BEFORE_ORDER:-}"
fault_machine_during_process="${FAULT_MACHINE_DURING_PROCESS:-}"
expected_execution_machine="${EXPECTED_EXECUTION_MACHINE:-}"
expected_order_success="${EXPECTED_ORDER_SUCCESS:-true}"
expected_completed="${EXPECTED_COMPLETED:-${order_quantity}}"
expected_trapped_part="${EXPECTED_TRAPPED_PART:-}"
if ! [[ "${raw_part_count}" =~ ^[1-6]$ ]]; then
  echo "RAW_PART_COUNT must be an integer from 1 to 6"
  exit 2
fi
if ! [[ "${order_quantity}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ORDER_QUANTITY must be a positive integer"
  exit 2
fi
if ((order_quantity > raw_part_count || order_quantity > 4)); then
  echo "ORDER_QUANTITY exceeds raw inventory or the four-place finished bin"
  exit 2
fi
order_timeout_seconds="${ORDER_TIMEOUT_SECONDS:-$((750 * order_quantity))}"
if ! [[ "${order_timeout_seconds}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ORDER_TIMEOUT_SECONDS must be a positive integer"
  exit 2
fi
if ! [[ "${initial_battery}" =~ ^0([.][0-9]+)?$|^1([.]0+)?$ ]]; then
  echo "INITIAL_BATTERY_PERCENTAGE must be in the range 0 to 1"
  exit 2
fi
for boolean_value in \
  "${auto_recharge}" "${expect_recharge}" "${expected_order_success}"; do
  if [[ "${boolean_value}" != "true" && "${boolean_value}" != "false" ]]; then
    echo "Recharge and expected-success values must be true or false"
    exit 2
  fi
done
if ! [[ "${expected_completed}" =~ ^[0-9]+$ ]] ||
  ((expected_completed > order_quantity)); then
  echo "EXPECTED_COMPLETED must be between zero and ORDER_QUANTITY"
  exit 2
fi
IFS=',' read -r -a allowed_machines <<<"${allowed_machines_csv}"
if ((${#allowed_machines[@]} == 0)); then
  echo "ALLOWED_MACHINES must contain at least one machine id"
  exit 2
fi
allowed_yaml=""
for machine_id in "${allowed_machines[@]}"; do
  if ! [[ "${machine_id}" =~ ^machine_[123]$ ]]; then
    echo "Invalid machine id in ALLOWED_MACHINES: ${machine_id}"
    exit 2
  fi
  allowed_yaml+="${allowed_yaml:+, }${machine_id}"
done
allowed_yaml="[${allowed_yaml}]"
for optional_machine in \
  "${fault_machine_before_order}" "${fault_machine_during_process}" \
  "${expected_execution_machine}"; do
  if [[ -n "${optional_machine}" ]] &&
    ! [[ "${optional_machine}" =~ ^machine_[123]$ ]]; then
    echo "Fault-test machine ids must be machine_1, machine_2 or machine_3"
    exit 2
  fi
done
if [[ -n "${fault_machine_before_order}" ]] &&
  [[ ",${allowed_machines_csv}," != *",${fault_machine_before_order},"* ]]; then
  echo "FAULT_MACHINE_BEFORE_ORDER must be included in ALLOWED_MACHINES"
  exit 2
fi
if [[ -n "${fault_machine_during_process}" ]] &&
  [[ ",${allowed_machines_csv}," != *",${fault_machine_during_process},"* ]]; then
  echo "FAULT_MACHINE_DURING_PROCESS must be included in ALLOWED_MACHINES"
  exit 2
fi
if [[ -n "${fault_machine_before_order}" &&
  -n "${fault_machine_during_process}" ]]; then
  echo "Pre-load and loaded-machine fault injection are mutually exclusive"
  exit 2
fi
if [[ -n "${fault_machine_during_process}" ]] &&
  [[ "${order_entry}" != "action" ]]; then
  echo "Loaded-machine fault acceptance requires ORDER_ENTRY=action"
  exit 2
fi
if [[ "${expected_order_success}" == "false" &&
  -z "${fault_machine_during_process}" ]]; then
  echo "An expected order failure requires a loaded-machine fault target"
  exit 2
fi

if [[ "${order_entry}" == "agent_auto" ]] &&
  ((order_quantity != 1 || ${#allowed_machines[@]} != 1)); then
  echo "agent_auto acceptance currently requires one part and one allowed machine"
  exit 2
fi

if [[ "${order_entry}" == "task_queue" ]]; then
  if ((order_quantity != 3)) ||
    [[ ",${allowed_machines_csv}," != *",machine_1,"* ]] ||
    [[ ",${allowed_machines_csv}," != *",machine_2,"* ]] ||
    [[ ",${allowed_machines_csv}," != *",machine_3,"* ]]; then
    echo "task_queue acceptance requires quantity 3 and all three machines"
    exit 2
  fi
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=155
export GZ_PARTITION=factory_test_${ROS_DOMAIN_ID}
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/physical_order_truth.log"
result_file="/tmp/physical_order_result.log"
summary_file="/tmp/physical_order_summary.log"
factory_state_file="/tmp/physical_order_factory_state.log"
machine_state_file="/tmp/physical_order_machine_states.log"
queue_path="/tmp/physical_order_robot_tasks.json"
fault_injection_file="/tmp/physical_order_fault_injection.log"
world_file="${WORLD_FILE:-}"
record_video_path="${RECORD_VIDEO_PATH:-}"
lock_file="/tmp/factory_physical_order_truth.lock"
exec 9>"${lock_file}"
if ! flock -n 9; then
  echo "Another physical-order acceptance is already running"
  echo "Wait for it to finish instead of sharing ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  exit 75
fi

terminate_test_processes
: >"${result_file}"
: >"${summary_file}"
: >"${factory_state_file}"
: >"${machine_state_file}"
: >"${fault_injection_file}"
rm -f "${queue_path}" "${queue_path}.tmp"

fault_injector_pid=""
recording_bridge_pid=""
recording_pid=""
launch_arguments=(
  use_navigation:=true
  headless:="${headless}"
  robot_x:="${robot_x}"
  robot_y:="${robot_y}"
  robot_yaw:="${robot_yaw}"
  initial_battery_percentage:="${initial_battery}"
  raw_part_count:="${raw_part_count}"
)
if [[ -n "${world_file}" ]]; then
  if [[ ! -r "${world_file}" ]]; then
    echo "WORLD_FILE is not readable: ${world_file}"
    exit 2
  fi
  launch_arguments+=(world_file:="${world_file}")
fi
launch_arguments+=(
  enable_sparse_bin_perception:=true
  randomize_raw_bin:=true
  raw_bin_seed:="${raw_bin_seed}"
  enable_finished_slot_perception:=true
)
if [[ "${order_entry}" == "task_queue" ]]; then
  launch_arguments+=(
    enable_task_queue_runtime:=true
    task_queue_dispatch_enabled:=true
    task_queue_path:="${queue_path}"
  )
fi
setsid ros2 launch factory_bringup physical_stack.launch.py \
  "${launch_arguments[@]}" 9>&- >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  if [[ -n "${recording_pid}" ]]; then
    kill -INT -- "-${recording_pid}" 2>/dev/null || true
    wait "${recording_pid}" 2>/dev/null || true
  fi
  if [[ -n "${recording_bridge_pid}" ]]; then
    kill -TERM -- "-${recording_bridge_pid}" 2>/dev/null || true
    wait "${recording_bridge_pid}" 2>/dev/null || true
  fi
  if [[ -n "${fault_injector_pid}" ]]; then
    kill "${fault_injector_pid}" 2>/dev/null || true
    wait "${fault_injector_pid}" 2>/dev/null || true
  fi
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

fail_with_log() {
  echo "$1"
  echo "--- ExecuteOrder output ---"
  tail -160 "${result_file}" 2>/dev/null || true
  echo "--- Physical stack output ---"
  tail -260 "${log_file}"
  exit 1
}

wait_for_stack() {
  local lifecycle action_info manipulation_info transfer_type
  local task_action_info queue_type queue_ready
  for _ in $(seq 1 24); do
    lifecycle="$(timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true)"
    action_info="$(ros2 action info /factory/execute_order 2>/dev/null || true)"
    manipulation_info="$(ros2 action info /manipulate_part 2>/dev/null || true)"
    transfer_type="$(ros2 service type /factory/part_transfer 2>/dev/null || true)"
    queue_ready=true
    if [[ "${order_entry}" == "task_queue" ]]; then
      task_action_info="$(
        ros2 action info /factory/execute_robot_task 2>/dev/null || true
      )"
      queue_type="$(
        ros2 service type /factory/get_robot_task_queue 2>/dev/null || true
      )"
      if ! grep -q 'Action servers: 1' <<<"${task_action_info}" ||
        ! grep -q 'factory_interfaces/srv/GetRobotTaskQueue' <<<"${queue_type}"; then
        queue_ready=false
      fi
    fi
    if grep -q '^active \[3\]$' <<<"${lifecycle}" &&
      grep -q 'Action servers: 1' <<<"${action_info}" &&
      grep -q 'Action servers: 1' <<<"${manipulation_info}" &&
      grep -q 'factory_interfaces/srv/PartTransfer' <<<"${transfer_type}" &&
      [[ "${queue_ready}" == "true" ]]; then
      return 0
    fi
    sleep 3
  done
  return 1
}

start_optional_recording() {
  local partial_video
  [[ -n "${record_video_path}" ]] || return 0

  mkdir -p "$(dirname "${record_video_path}")"
  setsid ros2 run ros_gz_image image_bridge /demo/overview/image \
    >"${record_video_path%.mp4}.bridge.log" 2>&1 &
  recording_bridge_pid=$!
  timeout 60 ros2 topic echo /demo/overview/image --once >/dev/null ||
    fail_with_log "Overview camera did not publish a recording frame"

  setsid ros2 run factory_perception record_overview --ros-args \
    -p topic:=/demo/overview/image \
    -p output:="${record_video_path}" \
    -p fps:=20.0 \
    >"${record_video_path%.mp4}.recorder.log" 2>&1 &
  recording_pid=$!
  partial_video="${record_video_path%.mp4}.partial.mp4"
  for _ in $(seq 1 60); do
    [[ -s "${partial_video}" ]] && return 0
    sleep 1
  done
  fail_with_log "Overview camera produced no recordable video"
}

run_agent_automatic_order() {
  local active_order machine_number mode_output start_output stop_output
  machine_number="${allowed_machines[0]#machine_}"

  start_output="$(
    timeout 15 ros2 service call \
      /factory_agent/submit factory_interfaces/srv/SubmitNaturalLanguage \
      "{text: '只用${machine_number}号机床启动自动生产'}"
  )" || fail_with_log "Agent could not start automatic production"
  printf '%s\n' "${start_output}" >"${result_file}"
  grep -qi 'accepted=true' <<<"${start_output}" ||
    fail_with_log "Agent rejected automatic production"

  active_order=""
  for _ in $(seq 1 120); do
    mode_output="$(
      timeout 15 ros2 service call \
        /factory/control_production \
        factory_interfaces/srv/ControlProduction \
        "{command: 0, allowed_machine_ids: []}"
    )" || fail_with_log "Could not query automatic production"
    active_order="$(
      grep -o "active_order_id='[^']*'" <<<"${mode_output}" |
        cut -d"'" -f2
    )"
    [[ -n "${active_order}" ]] && break
    sleep 0.25
  done
  [[ -n "${active_order}" ]] ||
    fail_with_log "Automatic production did not dispatch a physical order"

  stop_output="$(
    timeout 15 ros2 service call \
      /factory_agent/submit factory_interfaces/srv/SubmitNaturalLanguage \
      "{text: '完成当前订单后停止自动生产'}"
  )" || fail_with_log "Agent could not request an automatic drain"
  printf '%s\n' "${stop_output}" >>"${result_file}"
  grep -qi 'accepted=true' <<<"${stop_output}" ||
    fail_with_log "Agent rejected the automatic drain request"

  for _ in $(seq 1 1500); do
    mode_output="$(
      timeout 15 ros2 service call \
        /factory/control_production \
        factory_interfaces/srv/ControlProduction \
        "{command: 0, allowed_machine_ids: []}"
    )" || fail_with_log "Could not query automatic production completion"
    if grep -q "state='stopped'" <<<"${mode_output}" &&
      grep -q "completed_parts=${order_quantity}" <<<"${mode_output}"; then
      printf '%s\n' "${mode_output}" >>"${result_file}"
      order_id="${active_order}"
      return
    fi
    if grep -q "state='faulted'" <<<"${mode_output}"; then
      printf '%s\n' "${mode_output}" >>"${result_file}"
      fail_with_log "Automatic physical order entered faulted state"
    fi
    sleep 1
  done
  fail_with_log "Automatic physical order did not finish before timeout"
}

inject_fault_before_order() {
  local response
  [[ -n "${fault_machine_before_order}" ]] || return 0
  response="$(
    timeout 15 ros2 service call \
      /factory/machine_command factory_interfaces/srv/MachineCommand \
      "{machine_id: ${fault_machine_before_order}, command: 4, part_id: ''}"
  )" || fail_with_log "Could not inject the requested pre-load fault"
  grep -Eqi 'accepted=true|accepted: true' <<<"${response}" ||
    fail_with_log "PLC rejected the requested pre-load fault"
  for _ in $(seq 1 20); do
    if timeout 5 ros2 topic echo \
      "/${fault_machine_before_order}/state" --once 2>/dev/null |
      grep -q 'state: 4'; then
      return 0
    fi
  done
  fail_with_log "${fault_machine_before_order} did not publish FAULT"
}

inject_fault_after_processing_starts() {
  local response state
  for _ in $(seq 1 900); do
    state="$(
      timeout 6 ros2 topic echo \
        "/${fault_machine_during_process}/state" --once 2>/dev/null || true
    )"
    if grep -q 'state: 2' <<<"${state}"; then
      response="$(
        timeout 15 ros2 service call \
          /factory/machine_command factory_interfaces/srv/MachineCommand \
          "{machine_id: ${fault_machine_during_process}, command: 4, part_id: ''}"
      )" || {
        echo "fault injection service call failed" >"${fault_injection_file}"
        return 1
      }
      if ! grep -Eqi 'accepted=true|accepted: true' <<<"${response}"; then
        {
          echo "PLC rejected loaded-machine fault injection"
          printf '%s\n' "${response}"
        } >"${fault_injection_file}"
        return 1
      fi
      {
        echo "injected=true"
        echo "machine=${fault_machine_during_process}"
        printf '%s\n' "${response}"
      } >"${fault_injection_file}"
      return 0
    fi
    sleep 0.1
  done
  echo "machine never published PROCESSING" >"${fault_injection_file}"
  return 1
}

wait_for_stack || fail_with_log "Physical order stack did not become ready"
start_optional_recording
provisioned=false
for _ in $(seq 1 120); do
  if grep -q \
    "Unordered raw-material workspace is ready with ${raw_part_count} active part(s)" \
    "${log_file}"; then
    provisioned=true
    break
  fi
  sleep 0.25
done
[[ "${provisioned}" == "true" ]] ||
  fail_with_log "Raw-material provisioning did not reach the requested count"
inject_fault_before_order
if [[ -n "${fault_machine_during_process}" ]]; then
  inject_fault_after_processing_starts &
  fault_injector_pid=$!
fi

if [[ "${order_entry}" == "agent_auto" ]]; then
  run_agent_automatic_order
  grep -q 'BT task .* phase PickPart:' "${log_file}" ||
    fail_with_log "Agent order did not execute the raw pick phase"
  grep -q 'BT task .* phase WaitMachineDone:' "${log_file}" ||
    fail_with_log "Agent order did not execute CNC processing"
elif [[ "${order_entry}" == "task_queue" ]]; then
  queue_finished=false
  for _ in $(seq 1 "${order_timeout_seconds}"); do
    queue_snapshot="$(
      timeout 15 ros2 service call \
        /factory/get_robot_task_queue \
        factory_interfaces/srv/GetRobotTaskQueue \
        "{include_terminal: true}"
    )" || fail_with_log "Could not query the physical robot task queue"
    printf '%s\n' "${queue_snapshot}" >"${result_file}"

    if grep -Eq 'status=(4|5)|status: (4|5)' <<<"${queue_snapshot}"; then
      fail_with_log "A physical robot task failed or was canceled"
    fi
    if grep -Eq 'pending_count=(0|pending_count: 0)|pending_count: 0' \
        <<<"${queue_snapshot}" &&
      grep -Eq 'active_count=(0|active_count: 0)|active_count: 0' \
        <<<"${queue_snapshot}" &&
      grep -Eq 'terminal_count=(6|terminal_count: 6)|terminal_count: 6' \
        <<<"${queue_snapshot}"; then
      succeeded_count="$(
        grep -Eo 'status=3|status: 3' <<<"${queue_snapshot}" | wc -l
      )"
      if ((succeeded_count == 6)); then
        queue_finished=true
        break
      fi
    fi
    sleep 1
  done
  [[ "${queue_finished}" == "true" ]] ||
    fail_with_log "Physical robot task queue did not complete six tasks"
  grep -q 'Robot task .* succeeded: load task' "${log_file}" ||
    fail_with_log "Task queue did not complete a physical load"
  grep -q 'Robot task .* succeeded: unload task' "${log_file}" ||
    fail_with_log "Task queue did not complete a physical unload"
else
  timeout "${order_timeout_seconds}" ros2 action send_goal --feedback \
    /factory/execute_order factory_interfaces/action/ExecuteOrder \
    "{order_id: ${order_id}, quantity: ${order_quantity}, allowed_machine_ids: ${allowed_yaml}, auto_recharge: ${auto_recharge}}" \
    >"${result_file}" 2>&1 ||
    fail_with_log "ExecuteOrder command failed"

  if [[ -n "${fault_injector_pid}" ]]; then
    if ! wait "${fault_injector_pid}"; then
      fault_injector_pid=""
      fail_with_log "Loaded-machine fault injection did not complete"
    fi
    fault_injector_pid=""
    grep -q 'injected=true' "${fault_injection_file}" ||
      fail_with_log "Loaded-machine fault was not recorded"
  fi

  if [[ "${expected_order_success}" == "true" ]]; then
    grep -q 'success: true' "${result_file}" ||
      fail_with_log "ExecuteOrder did not report physical success"
  else
    grep -q 'success: false' "${result_file}" ||
      fail_with_log "Partially completed order unexpectedly succeeded"
    grep -Eq 'error_code: 21|error_code=21' "${result_file}" ||
      fail_with_log "Partial order did not request operator assistance"
    grep -q 'manual intervention' "${result_file}" ||
      fail_with_log "Partial order did not explain its trapped workpiece"
  fi
  grep -q "completed: ${expected_completed}" "${result_file}" ||
    fail_with_log \
      "ExecuteOrder completed count differs from ${expected_completed}"
  grep -q "phase='PickPart'\|phase: PickPart" "${result_file}" ||
    fail_with_log "ExecuteOrder did not expose the raw pick phase"
  grep -q "phase='WaitMachineDone'\|phase: WaitMachineDone" "${result_file}" ||
    fail_with_log "ExecuteOrder did not expose CNC processing feedback"
  if [[ "${expect_recharge}" == "true" ]]; then
    grep -q "detail='.*docking to charge'\|detail: .*docking to charge" "${result_file}" ||
      fail_with_log "Low-battery order did not request charging"
    grep -q "detail='charged to .*resuming production.*'\|detail: charged to .*resuming production" "${result_file}" ||
      fail_with_log "Low-battery order did not resume after charging"
    grep -q 'Physical battery reached' "${log_file}" ||
      fail_with_log "Battery never reached the configured charge target"
  fi
  if [[ -n "${expected_execution_machine}" ]]; then
    grep -Eq \
      "current_machine_id='${expected_execution_machine}'|current_machine_id: ${expected_execution_machine}" \
      "${result_file}" ||
      fail_with_log "Order did not use ${expected_execution_machine}"
    grep -q "phase='MachineReassigned'\\|phase: MachineReassigned" \
      "${result_file}" ||
      fail_with_log "Order did not report deterministic fault reassignment"
  fi
fi
grep -q 'Unordered raw-material workspace is ready' "${log_file}" ||
  fail_with_log "Unordered raw-material provisioning did not finish"
grep -q 'Raw RGB-D target selected' "${log_file}" ||
  fail_with_log "Physical order bypassed RGB-D raw-part selection"
mapfile -t selected_slots < <(
  sed -nE \
    's/.*Finished-bin RGB-D selected empty slot ([0-9]+);.*/\1/p' \
    "${log_file}"
)
((${#selected_slots[@]} == order_quantity)) ||
  fail_with_log "Finished-bin RGB-D did not resolve every placement slot"
unique_selected_slots="$(
  printf '%s\n' "${selected_slots[@]}" | sort -u | wc -l
)"
((unique_selected_slots == order_quantity)) ||
  fail_with_log \
    "Finished-bin RGB-D reused a destination during one physical order"
reserved_slots="$(grep -c 'Reserved finished-bin slot' "${log_file}" || true)"
((reserved_slots == order_quantity)) ||
  fail_with_log "Physical placements did not reserve every selected slot"
grep -q 'Finished-bin slots:' "${log_file}" ||
  fail_with_log "Finished-bin occupancy detector produced no observations"

factory_state="$(timeout 10 ros2 service call \
  /factory/get_state factory_interfaces/srv/GetFactoryState '{}')" ||
  fail_with_log "Could not read final factory state"
printf '%s\n' "${factory_state}" >"${factory_state_file}"
expected_raw=$((raw_part_count - order_quantity))
expected_finished="${expected_completed}"
grep -Eq "raw_part_count=(${expected_raw}|raw_part_count: ${expected_raw})|raw_part_count: ${expected_raw}" <<<"${factory_state}" ||
  fail_with_log "Raw inventory did not decrease to ${expected_raw}"
grep -Eq "finished_part_count=(${expected_finished}|finished_part_count: ${expected_finished})|finished_part_count: ${expected_finished}" <<<"${factory_state}" ||
  fail_with_log "Finished inventory did not increase to ${expected_finished}"
grep -Eq "held_part_id=''|held_part_id: ''" <<<"${factory_state}" ||
  fail_with_log "Robot retained a stale held-part register"

# Validate every machine from the same GetFactoryState response used for
# inventory.  A second sequence of transient topic reads is neither atomic nor
# stronger evidence: DDS transport can disappear after the order while the
# already-returned state service still contains the complete final snapshot.
for machine_id in "${allowed_machines[@]}"; do
  machine_pattern="machine_id='${machine_id}', state=([0-9]+), door_open=(True|False), part_present=(True|False), part_id='([^']*)'"
  if [[ ! "${factory_state}" =~ ${machine_pattern} ]]; then
    fail_with_log \
      "Final atomic factory snapshot omitted ${machine_id}"
  fi
  machine_state_value="${BASH_REMATCH[1]}"
  machine_door_open="${BASH_REMATCH[2]}"
  machine_part_present="${BASH_REMATCH[3]}"
  machine_part_id="${BASH_REMATCH[4]}"
  {
    echo "--- ${machine_id} ---"
    echo "state: ${machine_state_value}"
    echo "door_open: ${machine_door_open}"
    echo "part_present: ${machine_part_present}"
    echo "part_id: '${machine_part_id}'"
  } >>"${machine_state_file}"
  if [[ "${machine_id}" == "${fault_machine_before_order}" ||
    "${machine_id}" == "${fault_machine_during_process}" ]]; then
    [[ "${machine_state_value}" == "4" ]] ||
      fail_with_log "${machine_id} did not remain isolated in FAULT"
    if [[ "${machine_id}" == "${fault_machine_during_process}" ]]; then
      [[ "${machine_part_present}" == "True" ]] ||
        fail_with_log "${machine_id} lost its trapped-part register"
      if [[ -n "${expected_trapped_part}" ]]; then
        [[ "${machine_part_id}" == "${expected_trapped_part}" ]] ||
          fail_with_log \
            "${machine_id} trapped an unexpected workpiece"
      fi
    else
      [[ -z "${machine_part_id}" ]] ||
        fail_with_log "${machine_id} retained an unexpected part"
    fi
  else
    [[ "${machine_state_value}" == "0" ]] ||
      fail_with_log "${machine_id} did not return to IDLE"
    [[ -z "${machine_part_id}" ]] ||
      fail_with_log "${machine_id} retained a stale part register"
  fi
done

echo "--- Physical step timings ---"
grep -E "BT physical step .* completed in" "${log_file}" ||
  echo "No structured step timings found"
raw_layout="unordered_workspace"
mapfile -t measured_raw_targets < <(
  sed -nE \
    's/.*Raw RGB-D target selected at \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\).*/\1,\2,\3/p' \
    "${log_file}"
)
raw_target_count="${#measured_raw_targets[@]}"
((raw_target_count == order_quantity)) ||
  fail_with_log \
    "RGB-D target count ${raw_target_count} differs from order quantity ${order_quantity}"
raw_targets="$({
  local_ifs="${IFS}"
  IFS=';'
  echo "${measured_raw_targets[*]}"
  IFS="${local_ifs}"
})"


summary="physical_order_success=${expected_order_success} order=${order_id}"\
" completed=${expected_completed} machines=${allowed_machines_csv}"\
" initial_raw_inventory=${raw_part_count}"\
" raw_inventory=${expected_raw} finished_inventory=${expected_finished}"\
" orchestration=${order_entry} physics=contact_verified"\
" fault_isolated=${fault_machine_before_order:-${fault_machine_during_process:-none}}"\
" raw_source=rgbd_sparse_bin"\
" raw_seed=${raw_bin_seed}"\
" raw_layout=${raw_layout}"\
" raw_target_count=${raw_target_count}"\
" raw_targets=${raw_targets}"\
" finished_source=rgbd_occupancy"\
" recharge_verified=${expect_recharge}"\
" execution_machine=${expected_execution_machine:-${allowed_machines[0]}}"\
" robot_spawn=${robot_x},${robot_y},${robot_yaw}"\
" wall_seconds=$((SECONDS - test_started_at))"
echo "${summary}" | tee "${summary_file}"
