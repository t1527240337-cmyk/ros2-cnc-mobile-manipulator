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

if ! "${venv_python}" -c "from mcp.server.fastmcp import FastMCP" >/dev/null 2>&1; then
  echo "MCP SDK is missing from ${project_root}/.venv-mcp"
  echo "Run: ./scripts/setup_mcp_env.sh"
  exit 2
fi

transport="${FACTORY_MCP_TRANSPORT:-streamable-http}"
host="${FACTORY_MCP_HOST:-127.0.0.1}"
port="${FACTORY_MCP_PORT:-8000}"
exec "${venv_python}" -m factory_agent.mcp_server \
  --transport "${transport}" \
  --host "${host}" \
  --port "${port}"
