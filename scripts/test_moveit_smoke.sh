#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=44
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
ros2 daemon stop >/dev/null 2>&1 || true

log_file="/tmp/factory_moveit_smoke.log"
ros2 launch factory_moveit_config move_group.launch.py >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

node_output=""
for _ in $(seq 1 80); do
  node_output="$(ros2 node list 2>/dev/null || true)"
  if [[ "${node_output}" == *"/move_group"* ]]; then
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "MoveGroup exited before becoming ready"
    tail -160 "${log_file}"
    exit 1
  fi
  sleep 0.25
done

if [[ "${node_output}" != *"/move_group"* ]]; then
  echo "MoveGroup node did not become ready"
  tail -160 "${log_file}"
  exit 1
fi

service_output="$(ros2 service list)"
action_output="$(ros2 action list)"
pipeline_output="$(ros2 param get /move_group planning_pipelines)"

if [[ "${service_output}" != *"/plan_kinematic_path"* \
   || "${action_output}" != *"/move_action"* \
   || "${pipeline_output}" != *"ompl"* \
   || "${pipeline_output}" != *"pilz_industrial_motion_planner"* ]]; then
  echo "MoveGroup interfaces or planning pipelines are incomplete"
  echo "${pipeline_output}"
  tail -160 "${log_file}"
  exit 1
fi

echo "moveit_smoke_ok pipelines=ompl+pilz node=/move_group"
