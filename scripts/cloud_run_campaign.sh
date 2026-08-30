#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-/home/ubuntu/Embodied_Robotic_Arm}"
seeds="${2:-101-130}"
result_dir="${3:-${workspace}/artifacts/physical_seed_campaign/latest/production}"
profile="${4:-production}"

mkdir -p "${result_dir}"
cd "${workspace}"

# The campaign continues after individual seed failures unless the caller
# explicitly adds --stop-on-failure to the underlying command.
exec ./scripts/run_physical_seed_campaign.sh \
  --profile "${profile}" \
  --seeds "${seeds}" \
  --result-dir "${result_dir}" \
  --minimum-success-rate 0.80 \
  2>&1
