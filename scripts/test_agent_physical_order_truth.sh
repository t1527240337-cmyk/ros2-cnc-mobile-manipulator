#!/usr/bin/env bash
set -eo pipefail

# Operator-facing acceptance: Chinese Agent commands enable automatic
# production, one physically verified order completes, and drain-stop prevents
# a second order. Fixed surveyed slots isolate the control plane from the
# separate randomized RGB-D robustness benchmark.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=false
export ORDER_ENTRY=agent_auto
export ORDER_QUANTITY=1
export ALLOWED_MACHINES="${ALLOWED_MACHINES:-machine_2}"

if [[ "${HEADLESS}" == "false" ]]; then
  echo "[agent-physical] Gazebo GUI enabled; use HEADLESS=true for CI"
fi

exec "${project_root}/scripts/test_physical_order_truth.sh"
