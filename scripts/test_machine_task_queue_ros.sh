#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=56
export ROS_LOCALHOST_ONLY=1
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
ros2 daemon stop >/dev/null 2>&1 || true

queue_path="/tmp/factory_machine_task_queue_acceptance.json"
runtime_log="/tmp/factory_machine_task_runtime.log"
queue_log="/tmp/factory_machine_task_queue.log"
rm -f "${queue_path}" "${queue_path}.tmp"

runtime_pid=""
queue_pid=""
cleanup() {
  if [[ -n "${queue_pid}" ]]; then
    kill "${queue_pid}" 2>/dev/null || true
    wait "${queue_pid}" 2>/dev/null || true
  fi
  if [[ -n "${runtime_pid}" ]]; then
    kill "${runtime_pid}" 2>/dev/null || true
    wait "${runtime_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 run factory_core factory_runtime \
  --ros-args -p enable_order_execution:=false \
  >"${runtime_log}" 2>&1 &
runtime_pid=$!

start_queue() {
  ros2 run factory_core machine_task_queue \
    --ros-args \
    -p queue_path:="${queue_path}" \
    -p order_id:="acceptance-order" \
    -p allow_loading:=true \
    >"${queue_log}" 2>&1 &
  queue_pid=$!
}

wait_for_queue() {
  for _ in $(seq 1 80); do
    if ros2 service type /factory/get_robot_task_queue >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_pending_tasks() {
  local expected="$1"
  local output=""
  for _ in $(seq 1 80); do
    output="$(timeout 10 ros2 service call \
      /factory/get_robot_task_queue \
      factory_interfaces/srv/GetRobotTaskQueue \
      "{include_terminal: true}")"
    if [[ "${output}" == *"pending_count=${expected}"* ]]; then
      printf '%s\n' "${output}"
      return 0
    fi
    sleep 0.25
  done
  printf '%s\n' "${output}"
  return 1
}

start_queue
if ! wait_for_queue; then
  echo "Task queue service did not appear"
  cat "${runtime_log}" "${queue_log}"
  exit 1
fi
first_snapshot="$(wait_for_pending_tasks 3)"

kill "${queue_pid}" 2>/dev/null || true
wait "${queue_pid}" 2>/dev/null || true
queue_pid=""
sleep 0.5

start_queue
if ! wait_for_queue; then
  echo "Task queue service did not return after restart"
  cat "${queue_log}"
  exit 1
fi
second_snapshot="$(wait_for_pending_tasks 3)"

if [[ "${second_snapshot}" != *"terminal_count=0"* ]]; then
  echo "Restart changed task status unexpectedly"
  echo "${second_snapshot}"
  exit 1
fi
if [[ ! -s "${queue_path}" ]]; then
  echo "Task queue persistence file was not written"
  exit 1
fi

echo "machine_task_queue_ros_ok pending=3 restart_duplicates=0"
