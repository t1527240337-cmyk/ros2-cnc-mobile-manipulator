#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"

duration="${1:-3}"
speed="${2:-0.20}"

timeout "${duration}" ros2 topic pub -r 10 \
  /base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: ${speed}}}}" \
  >/dev/null || [[ "$?" -eq 124 ]]

ros2 topic pub --once \
  /base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {}}" \
  >/dev/null
