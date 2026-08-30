#!/usr/bin/env bash
set -eo pipefail

# Short physical reach/place acceptance for the redundant finished-bin slot.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RAW_SLOT="${RAW_SLOT:-2}"
export FINISHED_SLOT=4
exec "${project_root}/scripts/test_raw_to_finished_truth.sh"
