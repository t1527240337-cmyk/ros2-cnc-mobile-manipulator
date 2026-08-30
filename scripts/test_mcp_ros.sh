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
export ROS_DOMAIN_ID="${FACTORY_MCP_TEST_DOMAIN_ID:-47}"
export GZ_PARTITION="factory_mcp_test_${ROS_DOMAIN_ID}"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

mcp_host="${FACTORY_MCP_TEST_HOST:-127.0.0.1}"
mcp_port="${FACTORY_MCP_TEST_PORT:-8011}"
mcp_url="http://${mcp_host}:${mcp_port}/mcp"
launch_log="/tmp/factory_mcp_semantic_${ROS_DOMAIN_ID}.log"
mcp_log="/tmp/factory_mcp_server_${ROS_DOMAIN_ID}.log"

ros2 launch factory_bringup semantic_demo.launch.py \
  semantic_step_period:=0.05 >"${launch_log}" 2>&1 &
launch_pid=$!
mcp_pid=""

cleanup() {
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

mcp_ready=false
for _ in $(seq 1 100); do
  status="$(curl --silent --output /dev/null \
    --write-out "%{http_code}" "${mcp_url}" || true)"
  if [[ "${status}" != "000" ]]; then
    mcp_ready=true
    break
  fi
  sleep 0.1
done
if [[ "${mcp_ready}" != true ]]; then
  echo "MCP server did not start at ${mcp_url}"
  cat "${mcp_log}"
  exit 1
fi

if ! timeout 60 "${venv_python}" \
  "${project_root}/scripts/mcp_acceptance_client.py" \
  --url "${mcp_url}" \
  --quantity 2 \
  --machine machine_2 \
  --timeout 30; then
  echo "MCP server log:"
  cat "${mcp_log}"
  echo "Semantic factory log:"
  cat "${launch_log}"
  exit 1
fi
