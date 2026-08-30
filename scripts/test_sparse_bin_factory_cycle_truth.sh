#!/usr/bin/env bash
set -eo pipefail

# Run the same one-order production acceptance with randomized raw poses and
# RGB-D candidate selection enabled. The underlying test owns process cleanup,
# CNC register checks and final inventory verification.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HEADLESS="${HEADLESS:-false}"
export SPARSE_BIN=true
if [[ "${HEADLESS}" == "false" ]]; then
  echo "[sparse-cycle] Gazebo GUI enabled; use HEADLESS=true for CI"
fi
exec "${project_root}/scripts/test_physical_order_truth.sh"
