#!/usr/bin/env bash
set -euo pipefail

workspace="${1:-/home/ubuntu/Embodied_Robotic_Arm}"
log_file="${2:-${workspace}/artifacts/regression_build.log}"

mkdir -p "$(dirname "${log_file}")"
cd "${workspace}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build --symlink-install --packages-select \
  factory_agent \
  factory_core \
  factory_perception \
  factory_task_bt \
  factory_bringup \
  mobile_manipulator_description 2>&1 | tee "${log_file}"
