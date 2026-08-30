#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${project_root}/scripts/setup_ros_env.sh"
venv_python="${project_root}/.venv-mcp/bin/python"

if [[ ! -x "${venv_python}" ]]; then
  echo "MCP environment is missing."
  echo "Run: ./scripts/setup_mcp_env.sh"
  exit 2
fi

if [[ -z "${OPENAI_API_KEY:-}" ]]; then
  echo "OPENAI_API_KEY is not configured."
  echo "Set OPENAI_BASE_URL and FACTORY_AGENT_MODEL for another compatible provider."
  exit 2
fi

exec "${venv_python}" -m factory_agent.operator_web \
  --host "${FACTORY_OPERATOR_WEB_HOST:-127.0.0.1}" \
  --port "${FACTORY_OPERATOR_WEB_PORT:-8080}" \
  --mcp-url "${FACTORY_MCP_URL:-http://127.0.0.1:8000/mcp}" \
  "$@"
