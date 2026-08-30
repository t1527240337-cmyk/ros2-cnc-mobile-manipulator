#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/setup_ros_env.sh"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-184}"
bt_log="/tmp/factory_task_bt_protocol.log"

ros2 run factory_task_bt factory_task_bt_executor >"${bt_log}" 2>&1 &
bt_pid=$!
cleanup() {
  kill "${bt_pid}" 2>/dev/null || true
  wait "${bt_pid}" 2>/dev/null || true
}
trap cleanup EXIT

python3 "${repo_root}/scripts/test_task_bt_protocol.py"
