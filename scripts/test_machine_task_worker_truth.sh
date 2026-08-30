#!/usr/bin/env bash
set -eo pipefail

# Formal Gazebo acceptance for PLC events -> persistent queue -> physical
# single-task actions. This deliberately does not submit ExecuteOrder.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export ORDER_ENTRY=task_queue
export ORDER_ID=physical_task_queue_1
export ORDER_QUANTITY=3
export ALLOWED_MACHINES=machine_1,machine_2,machine_3
export ORDER_TIMEOUT_SECONDS="${ORDER_TIMEOUT_SECONDS:-2250}"

echo "[machine-task-worker] PLC events drive three physical load/unload cycles"
exec "${project_root}/scripts/test_physical_order_truth.sh"
