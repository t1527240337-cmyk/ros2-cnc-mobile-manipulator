#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <api-key-base64>" >&2
  exit 2
fi

workspace="/home/ubuntu/Embodied_Robotic_Arm"
cd "${workspace}"

export OPENAI_API_KEY
OPENAI_API_KEY="$(printf '%s' "$1" | base64 -d)"
export OPENAI_BASE_URL="https://api.deepseek.com"
export FACTORY_AGENT_MODEL="deepseek-v4-flash"
export FACTORY_AGENT_LLM_TIMEOUT="30.0"
trap 'unset OPENAI_API_KEY' EXIT

exec ./scripts/record_agent_factory_demo.sh \
  "${workspace}/artifacts/demo/agent_factory_cycle" \
  "请先查询工厂状态，再加工一个原料并将成品放入成品区；不得调用任何底层运动接口。"
