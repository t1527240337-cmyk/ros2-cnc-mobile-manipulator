#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
ros2 daemon stop >/dev/null 2>&1 || true


log_file="/tmp/factory_semantic_launch.log"
ros2 launch factory_bringup semantic_demo.launch.py >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if ros2 service type /factory_agent/submit >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

submit_output="$(timeout 15 ros2 service call /factory_agent/submit factory_interfaces/srv/SubmitNaturalLanguage \
  "{text: '加工3个零件'}")"
if [[ "${submit_output}" != *"accepted=True"* && "${submit_output}" != *"accepted=true"* ]]; then
  echo "Agent did not accept the order"
  echo "${submit_output}"
  cat "${log_file}"
  exit 1
fi

for _ in $(seq 1 80); do
  state_output="$(timeout 15 ros2 service call /factory/get_state factory_interfaces/srv/GetFactoryState '{}')"
  if [[ "${state_output}" == *"finished_part_count=3"* ]]; then
    echo "semantic_ros_ok finished_part_count=3"
    exit 0
  fi
  sleep 0.25
done

echo "Order did not finish before timeout"
echo "${state_output}"
cat "${log_file}"
exit 1
