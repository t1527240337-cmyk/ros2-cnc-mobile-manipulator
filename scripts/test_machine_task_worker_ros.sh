#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=57
export ROS_LOCALHOST_ONLY=1
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
ros2 daemon stop >/dev/null 2>&1 || true

queue_path="/tmp/factory_machine_task_worker_acceptance.json"
runtime_log="/tmp/factory_machine_task_worker_runtime.log"
worker_log="/tmp/factory_machine_task_worker.log"
fake_log="/tmp/factory_fake_robot_task_executor.log"
rm -f "${queue_path}" "${queue_path}.tmp"
fail_first="${FAIL_FIRST:-false}"

runtime_pid=""
worker_pid=""
fake_pid=""
cleanup() {
  for process_id in "${worker_pid}" "${fake_pid}" "${runtime_pid}"; do
    if [[ -n "${process_id}" ]]; then
      kill "${process_id}" 2>/dev/null || true
      wait "${process_id}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

ros2 run factory_core factory_runtime \
  --ros-args -p enable_order_execution:=false \
  >"${runtime_log}" 2>&1 &
runtime_pid=$!

fake_arguments=()
if [[ "${fail_first}" == "true" ]]; then
  fake_arguments+=(--fail-first)
fi
python3 "${project_root}/scripts/fake_robot_task_executor.py" \
  "${fake_arguments[@]}" \
  >"${fake_log}" 2>&1 &
fake_pid=$!

# No /clock publisher is started. Dispatch must still advance because its
# orchestration heartbeat deliberately uses a steady clock.
ros2 run factory_core machine_task_queue \
  --ros-args \
  -p queue_path:="${queue_path}" \
  -p order_id:="worker-acceptance-order" \
  -p allow_loading:=true \
  -p dispatch_enabled:=true \
  -p use_sim_time:=true \
  >"${worker_log}" 2>&1 &
worker_pid=$!

for _ in $(seq 1 100); do
  if ros2 service type /factory/get_robot_task_queue >/dev/null 2>&1; then
    break
  fi
  sleep 0.20
done

snapshot=""
for _ in $(seq 1 120); do
  snapshot="$(timeout 10 ros2 service call \
    /factory/get_robot_task_queue \
    factory_interfaces/srv/GetRobotTaskQueue \
    "{include_terminal: true}")"
  if [[ "${fail_first}" == "true" ]]; then
    if [[ "${snapshot}" == *"pending_count=2"* \
      && "${snapshot}" == *"active_count=0"* \
      && "${snapshot}" == *"terminal_count=1"* \
      && "${snapshot}" == *"dispatch halted for manual reconciliation"* ]]; then
      echo "machine_task_worker_halt_ros_ok failed=1 pending=2"
      exit 0
    fi
  else
    if [[ "${snapshot}" == *"pending_count=0"* \
      && "${snapshot}" == *"terminal_count=3"* ]]; then
      echo "machine_task_worker_ros_ok succeeded=3 pending=0"
      exit 0
    fi
  fi
  sleep 0.20
done

if [[ "${fail_first}" == "true" ]]; then
  echo "Queue worker did not halt after the injected physical failure"
else
  echo "Queue worker did not finish three accepted tasks"
fi
echo "${snapshot}"
cat "${runtime_log}" "${worker_log}" "${fake_log}"
exit 1
