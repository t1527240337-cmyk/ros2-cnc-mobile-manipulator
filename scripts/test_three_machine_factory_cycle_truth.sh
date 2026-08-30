#!/usr/bin/env bash
set -eo pipefail

# Formal three-CNC acceptance. The physical executor must load all three
# distinct machines before collecting any finished workpiece.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=false
export ORDER_ID=physical_three_machine_1
export ORDER_QUANTITY=3
export ALLOWED_MACHINES=machine_2,machine_1,machine_3
export ORDER_TIMEOUT_SECONDS="${ORDER_TIMEOUT_SECONDS:-2250}"

echo "[three-machine-cycle] fill machine_2, machine_1 and machine_3, then collect all"
exec "${project_root}/scripts/test_physical_order_truth.sh"
