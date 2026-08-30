#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${FACTORY_AGENT_TEST_DOMAIN_ID:-46}"
export GZ_PARTITION="factory_agent_test_${ROS_DOMAIN_ID}"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_agent_command_launch_${ROS_DOMAIN_ID}.log"
ros2 launch factory_bringup semantic_demo.launch.py \
  semantic_step_period:=0.05 >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  if ros2 service type /factory_agent/command >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

state_output="$(timeout 15 ros2 service call \
  /factory_agent/command \
  factory_interfaces/srv/ExecuteAgentCommand \
  "{source: integration_test, operation: get_factory_state, auto_recharge: true}")"
if [[ "${state_output}" != *"accepted=True"* &&
      "${state_output}" != *"accepted=true"* ]]; then
  echo "Structured state query was rejected"
  echo "${state_output}"
  cat "${log_file}"
  exit 1
fi
if [[ "${state_output}" != *"raw_parts"* ||
      "${state_output}" != *"battery"* ]]; then
  echo "Structured state query did not include machine-readable data"
  echo "${state_output}"
  exit 1
fi

capability_output="$(timeout 15 ros2 service call \
  /factory_agent/command \
  factory_interfaces/srv/ExecuteAgentCommand \
  "{source: integration_test, operation: list_capabilities, auto_recharge: true}")"
if [[ "${capability_output}" != *"submit_order"* ||
      "${capability_output}" == *"/cmd_vel"* ]]; then
  echo "Agent capability allow-list is invalid"
  echo "${capability_output}"
  exit 1
fi

invalid_output="$(timeout 15 ros2 service call \
  /factory_agent/command \
  factory_interfaces/srv/ExecuteAgentCommand \
  "{source: integration_test, operation: hold_machine, target_machine_id: machine_99, auto_recharge: true}")"
if [[ "${invalid_output}" != *"accepted=False"* &&
      "${invalid_output}" != *"accepted=false"* ]]; then
  echo "Invalid machine escaped Agent schema validation"
  echo "${invalid_output}"
  exit 1
fi

order_output="$(timeout 15 ros2 service call \
  /factory_agent/command \
  factory_interfaces/srv/ExecuteAgentCommand \
  "{source: integration_test, operation: submit_order, quantity: 2, allowed_machine_ids: [machine_2], auto_recharge: true}")"
if [[ "${order_output}" != *"accepted=True"* &&
      "${order_output}" != *"accepted=true"* ]]; then
  echo "Structured order was not accepted by the deterministic executor"
  echo "${order_output}"
  cat "${log_file}"
  exit 1
fi

for _ in $(seq 1 160); do
  state_output="$(timeout 15 ros2 service call \
    /factory/get_state factory_interfaces/srv/GetFactoryState '{}')"
  if [[ "${state_output}" == *"finished_part_count=2"* ]]; then
    task_output="$(timeout 15 ros2 service call \
      /factory_agent/command \
      factory_interfaces/srv/ExecuteAgentCommand \
      "{source: integration_test, operation: get_task_status, auto_recharge: true}")"
    if [[ "${task_output}" != *"accepted=True"* &&
          "${task_output}" != *"accepted=true"* ]] ||
       [[ "${task_output}" != *"complete"* ]]; then
      echo "Completed task was not retained by the Agent status cache"
      echo "${task_output}"
      exit 1
    fi

    unknown_task_output="$(timeout 15 ros2 service call \
      /factory_agent/command \
      factory_interfaces/srv/ExecuteAgentCommand \
      "{source: integration_test, operation: get_task_status, task_id: missing-task, auto_recharge: true}")"
    if [[ "${unknown_task_output}" != *"accepted=False"* &&
          "${unknown_task_output}" != *"accepted=false"* ]]; then
      echo "Unknown task id was not rejected"
      echo "${unknown_task_output}"
      exit 1
    fi

    echo "agent_command_ros_ok finished_part_count=2"
    exit 0
  fi
  sleep 0.1
done

echo "Structured Agent order did not finish before timeout"
echo "${state_output}"
cat "${log_file}"
exit 1
