#!/usr/bin/env bash

# Source this file in every interactive terminal that talks to the demo:
#   source scripts/setup_ros_env.sh
# Keep setup files ahead of `set -u`: Jazzy setup reads optional variables.
factory_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${factory_project_root}/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${FACTORY_ROS_DOMAIN_ID:-0}"
export ROS2CLI_DISABLE_DAEMON=1
