#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${project_root}/install/setup.bash" ]]; then
  source "${project_root}/install/setup.bash"
fi
export PYTHONPATH="${project_root}/src/factory_core${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 -m factory_core.physical_campaign \
  --project-root "${project_root}" "$@"
