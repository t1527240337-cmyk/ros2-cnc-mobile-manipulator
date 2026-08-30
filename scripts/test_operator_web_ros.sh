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
export ROS_DOMAIN_ID="${FACTORY_WEB_TEST_DOMAIN_ID:-49}"
export GZ_PARTITION="factory_web_test_${ROS_DOMAIN_ID}"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

mcp_host="127.0.0.1"
mcp_port="${FACTORY_WEB_TEST_MCP_PORT:-8014}"
mcp_url="http://${mcp_host}:${mcp_port}/mcp"
llm_host="127.0.0.1"
llm_port="${FACTORY_WEB_TEST_API_PORT:-18082}"
llm_base_url="http://${llm_host}:${llm_port}/v1"
web_host="127.0.0.1"
web_port="${FACTORY_WEB_TEST_PORT:-8081}"
web_url="http://${web_host}:${web_port}"

launch_log="/tmp/factory_web_semantic_${ROS_DOMAIN_ID}.log"
mcp_log="/tmp/factory_web_mcp_${ROS_DOMAIN_ID}.log"
llm_log="/tmp/factory_web_llm_${ROS_DOMAIN_ID}.log"
web_log="/tmp/factory_web_server_${ROS_DOMAIN_ID}.log"

ros2 launch factory_bringup semantic_demo.launch.py \
  semantic_step_period:=0.05 >"${launch_log}" 2>&1 &
launch_pid=$!
mcp_pid=""
llm_pid=""
web_pid=""

cleanup() {
  if [[ -n "${web_pid}" ]]; then
    kill "${web_pid}" 2>/dev/null || true
    wait "${web_pid}" 2>/dev/null || true
  fi
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

OPENAI_API_KEY="local-web-acceptance-key" \
OPENAI_BASE_URL="${llm_base_url}" \
FACTORY_AGENT_MODEL="factory-fake-tool-model" \
"${venv_python}" -m factory_agent.operator_web \
  --host "${web_host}" \
  --port "${web_port}" \
  --mcp-url "${mcp_url}" \
  --quiet >"${web_log}" 2>&1 &
web_pid=$!

mcp_ready=false
llm_ready=false
web_ready=false
for _ in $(seq 1 120); do
  mcp_status="$(curl --silent --output /dev/null \
    --write-out "%{http_code}" "${mcp_url}" || true)"
  llm_status="$(curl --silent --output /dev/null \
    --write-out "%{http_code}" \
    "http://${llm_host}:${llm_port}/health" || true)"
  web_status="$(curl --silent --output /dev/null \
    --write-out "%{http_code}" "${web_url}/api/health" || true)"
  if [[ "${mcp_status}" != "000" ]]; then
    mcp_ready=true
  fi
  if [[ "${llm_status}" == "200" ]]; then
    llm_ready=true
  fi
  if [[ "${web_status}" == "200" ]]; then
    web_ready=true
  fi
  if [[
    "${mcp_ready}" == true &&
    "${llm_ready}" == true &&
    "${web_ready}" == true
  ]]; then
    break
  fi
  sleep 0.1
done

if [[
  "${mcp_ready}" != true ||
  "${llm_ready}" != true ||
  "${web_ready}" != true
]]; then
  echo "Operator web acceptance services did not start"
  cat "${mcp_log}"
  cat "${llm_log}"
  cat "${web_log}"
  exit 1
fi

page="$(curl --fail --silent "${web_url}/")"
health="$(curl --fail --silent "${web_url}/api/health")"
if [[ "${page}" != *"移动机械臂操作台"* ]]; then
  echo "Operator page content is missing"
  exit 1
fi
if [[
  "${health}" != *'"llm_configured": true'* ||
  "${health}" == *"local-web-acceptance-key"*
]]; then
  echo "Operator health response is invalid or leaks a secret"
  echo "${health}"
  exit 1
fi

chat_output="$(curl --fail --silent \
  --header "Content-Type: application/json" \
  --request POST \
  --data '{"message":"请只使用2号机床加工2个零件，并告诉我订单编号"}' \
  "${web_url}/api/chat")"

if [[
  "${chat_output}" != *'"name": "submit_order"'* ||
  "${chat_output}" != *'"accepted": true'* ||
  "${chat_output}" != *'"order_id": "agent-'*
]]; then
  echo "Web request did not produce an accepted bounded order"
  echo "${chat_output}"
  cat "${web_log}"
  exit 1
fi

state_output=""
for _ in $(seq 1 200); do
  state_output="$(timeout 10 ros2 service call \
    /factory/get_state factory_interfaces/srv/GetFactoryState '{}')"
  if [[ "${state_output}" == *"finished_part_count=2"* ]]; then
    echo "operator_web_ros_acceptance_ok finished_part_count=2"
    exit 0
  fi
  sleep 0.1
done

echo "Web-submitted order did not finish"
echo "${state_output}"
cat "${launch_log}"
exit 1
