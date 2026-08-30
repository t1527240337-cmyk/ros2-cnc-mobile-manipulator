#!/usr/bin/env bash
set -eo pipefail

script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_directory}/setup_ros_env.sh"
set -u

echo "Starting the persistent Gazebo demo on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
echo "In another terminal run: source scripts/setup_ros_env.sh"
echo "Press Ctrl+C to stop the demo."
exec ros2 launch factory_bringup gazebo.launch.py
