#!/usr/bin/env bash
set -eo pipefail

# Start below the 25% production threshold. ExecuteOrder must dock, charge to
# 80%, undock and resume the same order before touching raw inventory.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=false
export ORDER_ID=physical_recharge_resume_1
export ORDER_QUANTITY=1
export ALLOWED_MACHINES=machine_2
export INITIAL_BATTERY_PERCENTAGE=0.20
export AUTO_RECHARGE=true
export EXPECT_RECHARGE=true
export ORDER_TIMEOUT_SECONDS="${ORDER_TIMEOUT_SECONDS:-1050}"

echo "[low-battery-cycle] recharge at a safe empty-gripper boundary, then resume"
exec "${project_root}/scripts/test_physical_order_truth.sh"
