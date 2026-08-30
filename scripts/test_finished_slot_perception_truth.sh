#!/usr/bin/env bash
set -eo pipefail

# Formal three-part acceptance for RGB-D empty-slot selection. Four taught
# candidates provide one redundant slot when an arm or fixture occludes a view.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FINISHED_SLOT_PERCEPTION=true
export ORDER_ID=finished_slot_perception_1
export ORDER_QUANTITY=3
export ALLOWED_MACHINES=machine_1,machine_2,machine_3
export ORDER_TIMEOUT_SECONDS="${ORDER_TIMEOUT_SECONDS:-2250}"
exec "${project_root}/scripts/test_physical_order_truth.sh"
