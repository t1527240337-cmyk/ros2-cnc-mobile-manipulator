#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-/home/ubuntu/Embodied_Robotic_Arm}"
port="${FACTORY_MCP_PORT:-8001}"

cd "${workspace}"
source scripts/setup_ros_env.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-222}"

export FACTORY_MCP_TRANSPORT=streamable-http FACTORY_MCP_HOST=127.0.0.1 FACTORY_MCP_PORT="${port}"
exec ./scripts/run_factory_mcp.sh
