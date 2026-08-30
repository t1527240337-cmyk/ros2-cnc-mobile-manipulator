#!/usr/bin/env bash
set -eo pipefail

# Load two distinct CNCs, then fault machine_2 after its PLC reaches
# PROCESSING. The executor must leave that fixture untouched, collect the
# healthy machine_1 output and report one completed part plus one trapped part.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=false
export ORDER_ID=physical_loaded_fault_1
export ORDER_QUANTITY=2
export ALLOWED_MACHINES=machine_2,machine_1
export FAULT_MACHINE_DURING_PROCESS=machine_2
export EXPECTED_ORDER_SUCCESS=false
export EXPECTED_COMPLETED=1
export EXPECTED_TRAPPED_PART=raw_part_2
export ORDER_TIMEOUT_SECONDS="${ORDER_TIMEOUT_SECONDS:-1500}"

echo "[loaded-machine-fault] isolate machine_2 and recover machine_1 output"
exec "${project_root}/scripts/test_physical_order_truth.sh"
