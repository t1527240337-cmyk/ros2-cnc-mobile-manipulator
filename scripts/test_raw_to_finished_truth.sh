#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"
headless="${HEADLESS:-true}"
case "${headless}" in
  true)
    export GZ_SIM_HEADLESS_RENDERING=1
    ;;
  false)
    unset GZ_SIM_HEADLESS_RENDERING
    echo "[transfer] Gazebo GUI enabled; the full run takes about 4-5 minutes"
    ;;
  *)
    echo "HEADLESS must be true or false"
    exit 2
    ;;
esac
finished_slot="${FINISHED_SLOT:-2}"
if [[ ! "${finished_slot}" =~ ^[1-4]$ ]]; then
  echo "FINISHED_SLOT must be 1, 2, 3, or 4"
  exit 2
fi
part_id="${PART_ID:-raw_part_2}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=150
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_raw_to_finished_truth.log"

setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true headless:="${headless}" >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

fail_with_log() {
  echo "$1"
  tail -200 "${log_file}"
  exit 1
}

wait_for_stack() {
  local lifecycle_state action_info
  for _ in $(seq 1 18); do
    lifecycle_state="$(timeout 5 ros2 lifecycle get /docking_server 2>/dev/null || true)"
    action_info="$(ros2 action info /manipulate_part 2>/dev/null || true)"
    if grep -q '^active \[3\]$' <<<"${lifecycle_state}" &&
      grep -q 'Action servers: 1' <<<"${action_info}"; then
      return 0
    fi
    sleep 3
  done
  return 1
}

dock_at() {
  local station="$1"
  echo "[transfer] docking at ${station}"
  timeout 360 ros2 run factory_core dock_station --ros-args \
    -p station:="${station}" \
    -p navigate_to_staging:=true \
    -p staging_timeout:=180.0 \
    -p docking_timeout:=300.0 ||
    fail_with_log "Could not dock at ${station}"
}

undock() {
  local dock_type="$1"
  echo "[transfer] leaving the current dock"
  timeout 180 ros2 run factory_core undock_station --ros-args \
    -p dock_type:="${dock_type}" \
    -p undocking_timeout:=75.0 ||
    fail_with_log "Could not leave the current dock"
}

navigate_route() {
  local route="$1"
  echo "[transfer] following safe transit route ${route}"
  timeout 380 ros2 run factory_core navigate_route --ros-args \
    -p route:="${route}" \
    -p navigation_timeout:=360.0 ||
    fail_with_log "Could not complete transit route ${route}"
}

manipulate() {
  local operation="$1"
  local station="$2"
  local slot="$3"
  local part="$4"
  local output
  echo "[transfer] operation=${operation} station=${station} slot=${slot} part=${part}"

  output="$(timeout 300 ros2 action send_goal --feedback \
    /manipulate_part factory_interfaces/action/ManipulatePart \
    "{operation: ${operation}, station_id: ${station}, part_id: ${part}, placement_slot_id: ${slot}}" \
    2>&1)" || {
      echo "${output}"
      fail_with_log "Manipulation action failed at ${station}"
    }

  grep -q 'success: true' <<<"${output}" || {
    echo "${output}"
    fail_with_log "Manipulation action did not succeed at ${station}"
  }
  if [[ "${operation}" == "0" ]]; then
    grep -q 'VERIFY_LOAD_BEARING_GRASP' <<<"${output}" ||
      fail_with_log "Pick bypassed the unassisted proof-lift phase"
  fi
  echo "[transfer] manipulation succeeded at ${station}"
}

wait_for_stack || fail_with_log "Physical action servers did not become ready"
dock_at raw_bin
manipulate 0 raw_bin 0 "${part_id}"
undock factory_bin_station

navigate_route raw_bin_to_finished_bin
dock_at finished_bin
manipulate 1 finished_bin "${finished_slot}" "${part_id}"
undock factory_bin_station

echo "transfer_success=true physics=contact_and_sensor_verified" \
  "proof_lift=unassisted part=${part_id}" \
  "source=raw_bin:auto_rgbd sink=finished_bin:${finished_slot}"
