#!/usr/bin/env bash
set -euo pipefail

if ! grep -q 'VERSION_ID="24.04"' /etc/os-release; then
  echo "This installer targets Ubuntu 24.04 (Noble)." >&2
  exit 2
fi

sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

ros_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p')"
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-opennav-docking \
  ros-jazzy-slam-toolbox ros-jazzy-moveit ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-apriltag-ros ros-jazzy-cv-bridge \
  ros-jazzy-ur-description ros-jazzy-robotiq-description \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-pydantic python3-numpy python3-yaml

echo "ROS 2 Jazzy dependencies installed. Run: source /opt/ros/jazzy/setup.bash"
