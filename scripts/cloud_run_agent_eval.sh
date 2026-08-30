#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <api-key-base64>" >&2
  exit 2
fi

workspace="/home/ubuntu/Embodied_Robotic_Arm"
cd "${workspace}"
source scripts/setup_ros_env.sh
mkdir -p artifacts/agent_eval

export OPENAI_API_KEY
OPENAI_API_KEY="$(printf '%s' "$1" | base64 -d)"
trap 'unset OPENAI_API_KEY' EXIT

exec .venv-mcp/bin/python scripts/evaluate_real_agent.py \
  --mcp-url http://127.0.0.1:8001/mcp \
  --base-url https://api.deepseek.com --model deepseek-v4-flash \
  --timeout 45 --minimum-rate 0.80 \
  --output artifacts/agent_eval/deepseek_v4_flash_15_cases.json
