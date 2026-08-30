#!/usr/bin/env bash
set -eo pipefail

# Prove the restart contract without Gazebo: persist an in-flight physical
# phase, kill the worker, reject unverified replay, then authorize one audited
# retry and finish the original queue without duplicating PLC events.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=58
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_DISABLE_DAEMON=1

queue_path="/tmp/factory_machine_task_recovery.json"
runtime_log="/tmp/factory_task_recovery_runtime.log"
worker_log="/tmp/factory_task_recovery_worker.log"
fake_log="/tmp/factory_task_recovery_executor.log"
rm -f "${queue_path}" "${queue_path}.tmp"

runtime_pid=""
worker_pid=""
fake_pid=""
cleanup() {
  for process_id in "${worker_pid}" "${fake_pid}" "${runtime_pid}"; do
    if [[ -n "${process_id}" ]]; then
      kill -TERM -- "-${process_id}" 2>/dev/null || true
      wait "${process_id}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

start_worker() {
  setsid ros2 run factory_core machine_task_queue \
    --ros-args \
    -p queue_path:="${queue_path}" \
    -p order_id:="restart-recovery-order" \
    -p allow_loading:=true \
    -p dispatch_enabled:=true \
    >"${worker_log}" 2>&1 &
  worker_pid=$!
}

stop_process() {
  local process_id="$1"
  kill -TERM -- "-${process_id}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 -- "-${process_id}" 2>/dev/null; then
      wait "${process_id}" 2>/dev/null || true
      return
    fi
    sleep 0.10
  done
  kill -KILL -- "-${process_id}" 2>/dev/null || true
  wait "${process_id}" 2>/dev/null || true
}

wait_for_queue_service() {
  for _ in $(seq 1 100); do
    if ros2 service type /factory/get_robot_task_queue \
      2>/dev/null | grep -q 'GetRobotTaskQueue'; then
      return
    fi
    sleep 0.20
  done
  return 1
}

queue_snapshot() {
  timeout 10 ros2 service call \
    /factory/get_robot_task_queue \
    factory_interfaces/srv/GetRobotTaskQueue \
    "{include_terminal: true}"
}

setsid ros2 run factory_core factory_runtime \
  --ros-args -p enable_order_execution:=false \
  >"${runtime_log}" 2>&1 &
runtime_pid=$!

setsid python3 "${project_root}/scripts/fake_robot_task_executor.py" \
  --hang-first >"${fake_log}" 2>&1 &
fake_pid=$!
start_worker
wait_for_queue_service || {
  echo "Task queue service did not start"
  exit 1
}

checkpoint_persisted=false
snapshot=""
for _ in $(seq 1 120); do
  snapshot="$(queue_snapshot)"
  if [[ "${snapshot}" == *"active_count=1"* ]] &&
    grep -q '"status": "running"' "${queue_path}" &&
    grep -q '"last_phase": "physical_transfer"' "${queue_path}"; then
    checkpoint_persisted=true
    break
  fi
  sleep 0.20
done
if [[ "${checkpoint_persisted}" != "true" ]]; then
  echo "Worker never persisted the in-flight physical checkpoint"
  echo "${snapshot}"
  cat "${worker_log}" "${fake_log}"
  exit 1
fi

stop_process "${worker_pid}"
worker_pid=""
stop_process "${fake_pid}"
fake_pid=""

setsid python3 "${project_root}/scripts/fake_robot_task_executor.py" \
  >"${fake_log}" 2>&1 &
fake_pid=$!
start_worker
wait_for_queue_service || {
  echo "Task queue service did not return after restart"
  exit 1
}

restart_halted=false
for _ in $(seq 1 100); do
  snapshot="$(queue_snapshot)"
  if [[ "${snapshot}" == *"pending_count=2"* ]] &&
    [[ "${snapshot}" == *"active_count=0"* ]] &&
    [[ "${snapshot}" == *"terminal_count=1"* ]] &&
    [[ "${snapshot}" == *"dispatch halted for manual reconciliation"* ]]; then
    restart_halted=true
    break
  fi
  sleep 0.20
done
if [[ "${restart_halted}" != "true" ]]; then
  echo "Restart did not halt on the unknown physical outcome"
  echo "${snapshot}"
  cat "${worker_log}"
  exit 1
fi
grep -q "last_phase='physical_transfer'" <<<"${snapshot}" || {
  echo "Queue response lost the last physical phase"
  echo "${snapshot}"
  exit 1
}
interrupted_task_id="$(
  grep -o "task_id='[^']*'" <<<"${snapshot}" | head -1 | cut -d"'" -f2
)"
[[ -n "${interrupted_task_id}" ]] || {
  echo "Could not identify the interrupted task"
  exit 1
}

rejected="$(
  timeout 10 ros2 service call \
    /factory/reconcile_robot_task \
    factory_interfaces/srv/ReconcileRobotTask \
    "{task_id: ${interrupted_task_id}, resolution: 0, physical_state_verified: false, operator_note: 'not checked'}"
)"
grep -Eqi 'accepted=false|accepted: false' <<<"${rejected}" || {
  echo "Unverified physical replay was not rejected"
  echo "${rejected}"
  exit 1
}

accepted="$(
  timeout 10 ros2 service call \
    /factory/reconcile_robot_task \
    factory_interfaces/srv/ReconcileRobotTask \
    "{task_id: ${interrupted_task_id}, resolution: 0, physical_state_verified: true, operator_note: 'robot empty and part remains in source slot'}"
)"
grep -Eqi 'accepted=true|accepted: true' <<<"${accepted}" || {
  echo "Verified retry was rejected"
  echo "${accepted}"
  exit 1
}

for _ in $(seq 1 120); do
  snapshot="$(queue_snapshot)"
  if [[ "${snapshot}" == *"pending_count=0"* ]] &&
    [[ "${snapshot}" == *"active_count=0"* ]] &&
    [[ "${snapshot}" == *"terminal_count=3"* ]]; then
    succeeded_count="$(
      grep -Eo 'status=3|status: 3' <<<"${snapshot}" | wc -l
    )"
    if ((succeeded_count == 3)); then
      echo "machine_task_recovery_ros_ok checkpoint=physical_transfer" \
        "rejected_unverified=true retried=${interrupted_task_id}" \
        "succeeded=3"
      exit 0
    fi
  fi
  sleep 0.20
done

echo "Reconciled queue did not finish its original three tasks"
echo "${snapshot}"
cat "${runtime_log}" "${worker_log}" "${fake_log}"
exit 1
