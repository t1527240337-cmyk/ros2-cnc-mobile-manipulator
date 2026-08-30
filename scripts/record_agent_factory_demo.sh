#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-${project_root}/artifacts/demo/agent_factory_cycle}"
prompt="${2:-Check factory state, then process one raw part and place it in the finished bin.}"
domain_id="${FACTORY_DEMO_DOMAIN_ID:-166}"
mcp_port="${FACTORY_DEMO_MCP_PORT:-8020}"

if [[ -z "${OPENAI_API_KEY:-}" ]]; then
  echo "OPENAI_API_KEY is required for the real Agent demonstration" >&2
  exit 2
fi
if [[ ! -x "${project_root}/.venv-mcp/bin/python" ]]; then
  echo "Run scripts/setup_mcp_env.sh before recording the Agent demo" >&2
  exit 2
fi

mkdir -p "${output_dir}"
output_dir="$(realpath -m "${output_dir}")"
export FACTORY_ROS_DOMAIN_ID="${domain_id}"
export ROS_DOMAIN_ID="${domain_id}"
export FACTORY_MCP_PORT="${mcp_port}"
export FACTORY_MCP_HOST="127.0.0.1"
export FACTORY_MCP_TRANSPORT="streamable-http"
export FACTORY_MCP_URL="http://127.0.0.1:${mcp_port}/mcp"
export FACTORY_AGENT_AUDIT_LOG="${output_dir}/agent_audit.jsonl"
source "${project_root}/scripts/setup_ros_env.sh"
description_share="$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description"
recording_world="${output_dir}/factory_recording.sdf"
python3 "${project_root}/scripts/make_recording_world.py" \
  "${description_share}/worlds/factory.sdf" "${recording_world}"

stack_pid=""
mcp_pid=""
recorder_pid=""
bridge_pid=""

stop_recording_simulator() {
  local pid
  local any_running
  local -a simulator_pids=()
  mapfile -t simulator_pids < <(
    pgrep -f -- "${recording_world}" 2>/dev/null || true
  )
  for pid in "${simulator_pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for _ in $(seq 1 25); do
    any_running=false
    for pid in "${simulator_pids[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        any_running=true
        break
      fi
    done
    [[ "${any_running}" == false ]] && return
    sleep 0.2
  done
  for pid in "${simulator_pids[@]}"; do
    kill -KILL "${pid}" 2>/dev/null || true
  done
}

cleanup() {
  if [[ -n "${recorder_pid}" ]]; then
    kill -TERM -- "-${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]]; then
    kill -TERM -- "-${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi
  if [[ -n "${mcp_pid}" ]]; then
    kill -TERM "${mcp_pid}" 2>/dev/null || true
    wait "${mcp_pid}" 2>/dev/null || true
  fi
  if [[ -n "${stack_pid}" ]]; then
    kill -TERM -- "-${stack_pid}" 2>/dev/null || true
    wait "${stack_pid}" 2>/dev/null || true
  fi
  # gz may daemonize away from the launch process group. Match only the
  # unique derived recording world, never another Gazebo session.
  stop_recording_simulator
}
trap cleanup EXIT INT TERM

printf '%s\n' "${prompt}" > "${output_dir}/operator_prompt.txt"
date --iso-8601=seconds > "${output_dir}/recording_started_at.txt"
nvidia-smi --query-gpu=name,driver_version,memory.total \
  --format=csv,noheader > "${output_dir}/gpu.txt" 2>/dev/null || true

setsid ros2 launch factory_bringup physical_stack.launch.py \
  headless:=true \
  use_navigation:=true \
  world_file:="${recording_world}" \
  use_moveit:=true \
  enable_perception:=true \
  enable_sparse_bin_perception:=true \
  enable_finished_slot_perception:=true \
  randomize_raw_bin:=true \
  raw_bin_seed:=205 \
  raw_part_count:=4 \
  enable_task_queue_runtime:=false \
  > "${output_dir}/physical_stack.log" 2>&1 &
stack_pid=$!

setsid ros2 run ros_gz_image image_bridge /demo/overview/image \
  > "${output_dir}/overview_bridge.log" 2>&1 &
bridge_pid=$!

for _ in $(seq 1 180); do
  if ros2 service type /factory/get_state >/dev/null 2>&1 \
      && ros2 action info /factory/execute_order >/dev/null 2>&1 \
      && ros2 topic info /demo/overview/image >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
ros2 service type /factory/get_state >/dev/null
ros2 action info /factory/execute_order >/dev/null
timeout 60 ros2 topic echo /demo/overview/image --once >/dev/null

for _ in $(seq 1 60); do
  if ros2 service type /factory_agent/command >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
ros2 service type /factory_agent/command >/dev/null
raw_video="${output_dir}/factory_overview_raw.mp4"
setsid ros2 run factory_perception record_overview --ros-args \
  -p topic:=/demo/overview/image \
  -p output:="${raw_video}" \
  -p fps:=20.0 \
  > "${output_dir}/recorder.log" 2>&1 &
recorder_pid=$!
partial_video="${raw_video%.mp4}.partial.mp4"
for _ in $(seq 1 60); do
  [[ -s "${partial_video}" ]] && break
  sleep 1
done
if [[ ! -s "${partial_video}" ]]; then
  echo "Overview camera produced no recordable frames" >&2
  exit 1
fi

"${project_root}/scripts/run_factory_mcp.sh" \
  > "${output_dir}/mcp.log" 2>&1 &
mcp_pid=$!
for _ in $(seq 1 60); do
  if timeout 1 bash -c \
      "</dev/tcp/${FACTORY_MCP_HOST}/${FACTORY_MCP_PORT}" \
      >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

date --iso-8601=seconds > "${output_dir}/agent_request_at.txt"
if ! timeout 180 "${project_root}/scripts/run_factory_operator.sh" \
    --json "${prompt}" > "${output_dir}/agent_trace.json" \
    2> "${output_dir}/agent_stderr.log"; then
  echo "Agent request failed; see agent_stderr.log" >&2
  exit 1
fi
cat "${output_dir}/agent_trace.json"
date --iso-8601=seconds > "${output_dir}/agent_response_at.txt"

"${project_root}/scripts/wait_for_factory_completion.py" \
  --minimum-finished 1 \
  --timeout 900 \
  --output "${output_dir}/final_factory_state.json" \
  | tee "${output_dir}/completion.log"
date --iso-8601=seconds > "${output_dir}/task_completed_at.txt"

kill -INT -- "-${recorder_pid}"
wait "${recorder_pid}"
recorder_pid=""

final_video="${output_dir}/agent_factory_cycle_2x.mp4"
if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_nvenc \
    && nvidia-smi >/dev/null 2>&1; then
  encoder=( -c:v h264_nvenc -preset p4 -cq 22 )
else
  encoder=( -c:v libx264 -preset medium -crf 22 )
fi
ffmpeg -y -hide_banner -loglevel warning -i "${raw_video}" \
  -vf "setpts=0.5*PTS" -an "${encoder[@]}" "${final_video}"
ffprobe -v error -show_entries format=duration,size \
  -of json "${final_video}" > "${output_dir}/video_probe.json"
presented_video="${output_dir}/agent_factory_cycle_presented.mp4"
"${project_root}/scripts/package_demo_video.sh" \
  "${final_video}" "${presented_video}" \
  > "${output_dir}/presented_video_probe.json"
sha256sum "${final_video}" "${presented_video}" \
  "${output_dir}/agent_trace.json" \
  "${output_dir}/agent_audit.jsonl" \
  "${output_dir}/final_factory_state.json" > "${output_dir}/SHA256SUMS"

echo "Agent-to-robot demo evidence: ${output_dir}"
