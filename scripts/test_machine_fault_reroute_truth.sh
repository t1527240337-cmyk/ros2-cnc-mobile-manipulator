#!/usr/bin/env bash
set -eo pipefail

# Fault the preferred CNC before any workpiece is acquired. The physical order
# must skip it, report reassignment and complete on the next allowed machine.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=false
export ORDER_ID=physical_fault_reroute_1
export ORDER_QUANTITY=1
export ALLOWED_MACHINES=machine_2,machine_1
export FAULT_MACHINE_BEFORE_ORDER=machine_2
export EXPECTED_EXECUTION_MACHINE=machine_1
export ORDER_TIMEOUT_SECONDS="${ORDER_TIMEOUT_SECONDS:-900}"

echo "[fault-reroute] isolate machine_2 and complete the untouched part on machine_1"
exec "${project_root}/scripts/test_physical_order_truth.sh"
