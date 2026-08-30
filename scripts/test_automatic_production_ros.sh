#!/usr/bin/env bash
set -eo pipefail

# Semantic ROS acceptance for automatic production and Agent control. The
# physical ExecuteOrder implementation uses the same typed action and service
# boundaries; this fast test intentionally excludes Gazebo motion.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=47
export GZ_PARTITION=factory_auto_test_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_automatic_production.log"
ros2 launch factory_bringup semantic_demo.launch.py \
  automatic_max_batch_size:=1 \
  semantic_step_period:=0.25 >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

fail_with_log() {
  echo "$1"
  tail -180 "${log_file}"
  exit 1
}

for _ in $(seq 1 80); do
  if ros2 service type /factory_agent/submit 2>/dev/null |
    grep -q 'factory_interfaces/srv/SubmitNaturalLanguage' &&
    ros2 service type /factory/control_production 2>/dev/null |
    grep -q 'factory_interfaces/srv/ControlProduction'; then
    break
  fi
  sleep 0.25
done

start_output="$(
  timeout 15 ros2 service call \
    /factory_agent/submit factory_interfaces/srv/SubmitNaturalLanguage \
    "{text: '启动自动生产'}"
)"
grep -qi 'accepted=true' <<<"${start_output}" ||
  fail_with_log "Agent did not enable automatic production"

active_order=""
for _ in $(seq 1 100); do
  mode_output="$(
    timeout 15 ros2 service call \
      /factory/control_production factory_interfaces/srv/ControlProduction \
      "{command: 0, allowed_machine_ids: []}"
  )"
  active_order="$(
    grep -o "active_order_id='[^']*'" <<<"${mode_output}" |
      cut -d"'" -f2
  )"
  [[ -n "${active_order}" ]] && break
  sleep 0.10
done
[[ -n "${active_order}" ]] ||
  fail_with_log "Automatic coordinator did not dispatch an order"

stop_output="$(
  timeout 15 ros2 service call \
    /factory_agent/submit factory_interfaces/srv/SubmitNaturalLanguage \
    "{text: '完成当前订单后停止自动生产'}"
)"
grep -qi 'accepted=true' <<<"${stop_output}" ||
  fail_with_log "Agent did not request a draining stop"

for _ in $(seq 1 160); do
  mode_output="$(
    timeout 15 ros2 service call \
      /factory/control_production factory_interfaces/srv/ControlProduction \
      "{command: 0, allowed_machine_ids: []}"
  )"
  state_output="$(
    timeout 15 ros2 service call \
      /factory/get_state factory_interfaces/srv/GetFactoryState '{}'
  )"
  if grep -q "state='stopped'" <<<"${mode_output}" &&
    grep -q 'completed_parts=1' <<<"${mode_output}" &&
    grep -q 'raw_part_count=2' <<<"${state_output}" &&
    grep -q 'finished_part_count=1' <<<"${state_output}"; then
    echo "automatic_production_ros_ok order=${active_order} completed=1"
    exit 0
  fi
  sleep 0.15
done

echo "Last production mode response:"
echo "${mode_output}"
echo "Last factory state response:"
echo "${state_output}"
fail_with_log "Automatic production did not drain after one complete order"
