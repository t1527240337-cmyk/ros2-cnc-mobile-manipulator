#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_path="${project_root}/.venv-mcp"

python3 -m venv --system-site-packages "${venv_path}"
"${venv_path}/bin/python" -m pip install \
  --disable-pip-version-check \
  --retries 2 \
  --timeout 20 \
  "mcp>=1.27,<2"

echo "MCP environment ready: ${venv_path}"
echo "Start the ROS factory, then run:"
echo "  FACTORY_MCP_TRANSPORT=streamable-http ./scripts/run_factory_mcp.sh"
echo "For a local stdio client use:"
echo "  FACTORY_MCP_TRANSPORT=stdio ./scripts/run_factory_mcp.sh"
