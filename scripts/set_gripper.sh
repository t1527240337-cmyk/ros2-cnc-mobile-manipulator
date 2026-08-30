#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 open|close|stop" >&2
  exit 2
fi

case "$1" in
  open)
    effort=-2.0
    settle_seconds=1.0
    stop_after=true
    ;;
  close)
    effort=12.0
    settle_seconds=1.0
    stop_after=false
    ;;
  stop)
    effort=0.0
    settle_seconds=0.0
    stop_after=false
    ;;
  *)
    echo "Usage: $0 open|close|stop" >&2
    exit 2
    ;;
esac

script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_directory}/setup_ros_env.sh"
set -u

ros2 topic pub --once /gripper_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [${effort}, ${effort}]}"

if [[ "${settle_seconds}" != "0.0" ]]; then
  sleep "${settle_seconds}"
fi

if [[ "${stop_after}" == true ]]; then
  ros2 topic pub --once /gripper_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0]}'
fi
