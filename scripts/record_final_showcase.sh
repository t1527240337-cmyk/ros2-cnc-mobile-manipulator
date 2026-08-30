#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="${1:-${project_root}/artifacts/demo/final_showcase}"
agent_video="${AGENT_VIDEO_PATH:-${project_root}/artifacts/ucloud_20260810/demo/agent_factory_cycle/agent_factory_cycle_presented.mp4}"
speed_factor="${VIDEO_SPEED_FACTOR:-6}"

if ! [[ "${speed_factor}" =~ ^[2-9]$ ]]; then
  echo "VIDEO_SPEED_FACTOR must be an integer from 2 to 9" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
mkdir -p "${output_root}"
output_root="$(realpath -m "${output_root}")"
description_share="$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description"
recording_world="${output_root}/factory_recording.sdf"
python3 "${project_root}/scripts/make_recording_world.py" \
  "${description_share}/worlds/factory.sdf" "${recording_world}"

if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_nvenc &&
  nvidia-smi >/dev/null 2>&1; then
  encoder=(-c:v h264_nvenc -preset p4 -cq 22)
else
  encoder=(-c:v libx264 -preset medium -crf 22)
fi

scenario_failures=()
reuse_completed="${REUSE_COMPLETED_SCENARIOS:-false}"

run_scenario() {
  local name="$1"
  shift
  local scenario_dir="${output_root}/${name}"
  local raw_video="${scenario_dir}/overview_raw.mp4"
  local speed_video="${scenario_dir}/overview_${speed_factor}x.mp4"
  local status
  if [[ "${reuse_completed}" == "true" ]] &&
    [[ -s "${speed_video}" ]] &&
    [[ "$(cat "${scenario_dir}/exit_code.txt" 2>/dev/null || true)" == "0" ]]; then
    echo "[final-video] reusing verified ${name}"
    return
  fi

  mkdir -p "${scenario_dir}"
  echo "[final-video] starting ${name}"
  set +e
  env \
    HEADLESS=true \
    FINISHED_SLOT_PERCEPTION=true \
    RAW_PART_COUNT=6 \
    WORLD_FILE="${recording_world}" \
    RECORD_VIDEO_PATH="${raw_video}" \
    "$@" \
    "${project_root}/scripts/test_physical_order_truth.sh" \
    >"${scenario_dir}/acceptance.log" 2>&1
  status=$?
  set -e

  cp /tmp/physical_order_result.log "${scenario_dir}/action_result.log" 2>/dev/null || true
  cp /tmp/physical_order_summary.log "${scenario_dir}/summary.log" 2>/dev/null || true
  cp /tmp/physical_order_factory_state.log "${scenario_dir}/factory_state.log" 2>/dev/null || true
  cp /tmp/physical_order_machine_states.log "${scenario_dir}/machine_states.log" 2>/dev/null || true
  cp /tmp/physical_order_truth.log "${scenario_dir}/physical_stack.log" 2>/dev/null || true
  printf '%s\n' "${status}" >"${scenario_dir}/exit_code.txt"

  if ((status != 0)) || [[ ! -s "${raw_video}" ]]; then
    scenario_failures+=("${name}")
    echo "[final-video] ${name} failed; evidence was preserved" >&2
    return
  fi

  ffmpeg -y -hide_banner -loglevel warning -i "${raw_video}" \
    -vf "setpts=PTS/${speed_factor}" -an "${encoder[@]}" "${speed_video}"
  ffprobe -v error -show_entries format=duration,size -of json "${speed_video}" \
    >"${scenario_dir}/video_probe.json"
  sha256sum "${raw_video}" "${speed_video}" "${scenario_dir}/action_result.log" \
    >"${scenario_dir}/SHA256SUMS"
  echo "[final-video] completed ${name}"
}

run_scenario multi_machine \
  SPARSE_BIN=false \
  ORDER_ID=video_multi_machine \
  ORDER_QUANTITY=2 \
  ALLOWED_MACHINES=machine_2,machine_1 \
  RAW_BIN_SEED=205 \
  INITIAL_BATTERY_PERCENTAGE=0.72 \
  ORDER_TIMEOUT_SECONDS=2250

run_scenario low_battery_recharge \
  SPARSE_BIN=false \
  ORDER_ID=video_low_battery \
  ORDER_QUANTITY=1 \
  ALLOWED_MACHINES=machine_2 \
  RAW_BIN_SEED=211 \
  INITIAL_BATTERY_PERCENTAGE=0.20 \
  AUTO_RECHARGE=true \
  EXPECT_RECHARGE=true \
  ORDER_TIMEOUT_SECONDS=1050

run_scenario fault_reroute \
  SPARSE_BIN=false \
  ORDER_ID=video_fault_reroute \
  ORDER_QUANTITY=1 \
  ALLOWED_MACHINES=machine_2,machine_1 \
  RAW_BIN_SEED=223 \
  INITIAL_BATTERY_PERCENTAGE=0.55 \
  FAULT_MACHINE_BEFORE_ORDER=machine_2 \
  EXPECTED_EXECUTION_MACHINE=machine_1 \
  ORDER_TIMEOUT_SECONDS=900

if ((${#scenario_failures[@]} > 0)); then
  printf 'Failed scenarios: %s\n' "${scenario_failures[*]}" >&2
  exit 1
fi

final_video="${output_root}/ros2_mobile_manipulator_final_showcase.mp4"
"${project_root}/scripts/package_complex_demo_video.sh" \
  "${agent_video}" \
  "${output_root}/multi_machine/overview_${speed_factor}x.mp4" \
  "${output_root}/low_battery_recharge/overview_${speed_factor}x.mp4" \
  "${output_root}/fault_reroute/overview_${speed_factor}x.mp4" \
  "${final_video}" >"${output_root}/final_video_probe.json"
sha256sum "${final_video}" >"${output_root}/SHA256SUMS"
echo "Final showcase: ${final_video}"
