#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

venv_python="${project_root}/.venv-mcp/bin/python"
if [[ ! -x "${venv_python}" ]]; then
  echo "MCP environment is missing; run ./scripts/setup_mcp_env.sh"
  exit 2
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${FACTORY_LLM_TEST_DOMAIN_ID:-48}"
export GZ_PARTITION="factory_llm_test_${ROS_DOMAIN_ID}"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

mcp_host="127.0.0.1"
mcp_port="${FACTORY_LLM_TEST_MCP_PORT:-8013}"
mcp_url="http://${mcp_host}:${mcp_port}/mcp"
llm_host="127.0.0.1"
llm_port="${FACTORY_LLM_TEST_API_PORT:-18081}"
llm_base_url="http://${llm_host}:${llm_port}/v1"

launch_log="/tmp/factory_llm_semantic_${ROS_DOMAIN_ID}.log"
mcp_log="/tmp/factory_llm_mcp_${ROS_DOMAIN_ID}.log"
llm_log="/tmp/factory_llm_api_${ROS_DOMAIN_ID}.log"

ros2 launch factory_bringup semantic_demo.launch.py \
  semantic_step_period:=0.05 >"${launch_log}" 2>&1 &
launch_pid=$!
mcp_pid=""
llm_pid=""

cleanup() {
  if [[ -n "${llm_pid}" ]]; then
    kill "${llm_pid}" 2>/dev/null || true
    wait "${llm_pid}" 2>/dev/null || true
  fi
  if [[ -n "${mcp_pid}" ]]; then
    kill "${mcp_pid}" 2>/dev/null || true
    wait "${mcp_pid}" 2>/dev/null || true
  fi
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  if ros2 service type /factory_agent/command >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! ros2 service type /factory_agent/command >/dev/null 2>&1; then
  echo "Factory Agent ROS service did not start"
  cat "${launch_log}"
  exit 1
fi

"${venv_python}" -m factory_agent.mcp_server \
  --transport streamable-http \
  --host "${mcp_host}" \
  --port "${mcp_port}" >"${mcp_log}" 2>&1 &
mcp_pid=$!

"${venv_python}" "${project_root}/scripts/fake_openai_tool_server.py" \
  --host "${llm_host}" \
  --port "${llm_port}" >"${llm_log}" 2>&1 &
llm_pid=$!

mcp_ready=false
llm_ready=false
for _ in $(seq 1 100); do
  mcp_status="$(curl --silent --output /dev/null \
    --write-out "%{http_code}" "${mcp_url}" || true)"
  llm_status="$(curl --silent --output /dev/null \
    --write-out "%{http_code}" \
    "http://${llm_host}:${llm_port}/health" || true)"
  if [[ "${mcp_status}" != "000" ]]; then
    mcp_ready=true
  fi
  if [[ "${llm_status}" == "200" ]]; then
    llm_ready=true
  fi
  if [[ "${mcp_ready}" == true && "${llm_ready}" == true ]]; then
    break
  fi
  sleep 0.1
done

if [[ "${mcp_ready}" != true || "${llm_ready}" != true ]]; then
  echo "MCP or fake LLM server did not start"
  cat "${mcp_log}"
  cat "${llm_log}"
  exit 1
fi

if ! timeout 75 "${venv_python}" \
  "${project_root}/scripts/llm_mcp_acceptance_client.py" \
  --mcp-url "${mcp_url}" \
  --llm-base-url "${llm_base_url}" \
  --timeout 30; then
  echo "MCP server log:"
  cat "${mcp_log}"
  echo "Fake LLM server log:"
  cat "${llm_log}"
  echo "Semantic factory log:"
  cat "${launch_log}"
  exit 1
fi
