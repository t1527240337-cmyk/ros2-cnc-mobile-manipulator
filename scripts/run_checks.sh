#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS message packages are imported by a few adapter modules during unit-test
# discovery, so initialize Jazzy before Python loads those modules.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
project_pythonpath="${project_root}/src/factory_core:${project_root}/src/factory_agent:${project_root}/src/factory_perception"
export PYTHONPATH="${project_pythonpath}${PYTHONPATH:+:${PYTHONPATH}}"

# Build generated ROS interfaces before Python test discovery imports adapter
# modules that depend on them. This keeps the script valid in a clean clone,
# where install/setup.bash does not exist yet.
ros_workspace_available=false
if command -v colcon >/dev/null 2>&1 && [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ros_workspace_available=true
  cd "${project_root}"
  colcon build --symlink-install --event-handlers console_direct+
  source "${project_root}/install/setup.bash"
fi

# Pytest runs both unittest.TestCase suites and function-style ROS tests.
# unittest discovery silently skipped the latter and could report a green
# build while control-contract tests had never executed.
python3 -m pytest -q "${project_root}/src/factory_core/test"
python3 -m pytest -q "${project_root}/src/factory_agent/test"
python3 -m pytest -q "${project_root}/src/factory_perception/test"
python3 -m compileall -q "${project_root}/src"
python3 "${project_root}/scripts/validate_assets.py"
python3 "${project_root}/scripts/generate_factory_map.py" --check

if [[ "${ros_workspace_available}" == true ]]; then
  cd "${project_root}"
  ./scripts/test_task_bt_protocol.sh
  colcon test --event-handlers console_direct+
  colcon test-result --verbose
else
  echo "ROS 2 Jazzy not installed; skipped colcon build."
fi
