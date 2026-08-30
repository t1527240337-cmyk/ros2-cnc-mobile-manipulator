#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="${1:-${project_root}/artifacts/ucloud_final_showcase}"
mkdir -p "${output_root}"
output_root="$(realpath -m "${output_root}")"
pipeline_log="${output_root}/pipeline.log"
exit_file="${output_root}/exit_code.txt"

cd "${project_root}"
source /opt/ros/jazzy/setup.bash
echo "[cloud-showcase] build started $(date --iso-8601=seconds)" | tee "${pipeline_log}"
colcon build --symlink-install --event-handlers console_cohesion+ \
  >>"${pipeline_log}" 2>&1
echo "[cloud-showcase] recording started $(date --iso-8601=seconds)" \
  | tee -a "${pipeline_log}"

set +e
VIDEO_SPEED_FACTOR="${VIDEO_SPEED_FACTOR:-6}" \
  "${project_root}/scripts/record_final_showcase.sh" "${output_root}" \
  >>"${pipeline_log}" 2>&1
status=$?
set -e
printf '%s\n' "${status}" >"${exit_file}"
echo "[cloud-showcase] finished status=${status} $(date --iso-8601=seconds)" \
  | tee -a "${pipeline_log}"
exit "${status}"
