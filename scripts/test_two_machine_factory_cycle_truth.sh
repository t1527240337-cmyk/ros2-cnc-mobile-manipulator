#!/usr/bin/env bash
set -eo pipefail

# Physically load two distinct CNCs before unloading either one. This proves
# that quantity=2 is a pipelined factory order rather than two repetitions on
# the first machine in the preference list.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=false
export ORDER_ID=physical_two_machine_1
export ORDER_QUANTITY=2
export ALLOWED_MACHINES=machine_2,machine_1

echo "[two-machine-cycle] load machine_2 and machine_1, then collect both"
exec "${project_root}/scripts/test_physical_order_truth.sh"
