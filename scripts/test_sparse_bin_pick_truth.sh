#!/usr/bin/env bash
set -eo pipefail

# Physical acceptance for a randomized, single-layer raw-material bin.
# The action must consume a fresh RGB-D candidate before simulator truth is
# allowed to validate the residual and the subsequent contact grasp.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

raw_bin_seed="${RAW_BIN_SEED:-17}"
raw_part_count="${RAW_PART_COUNT:-6}"
if ! [[ "${raw_part_count}" =~ ^[1-6]$ ]]; then
  echo "RAW_PART_COUNT must be an integer from 1 to 6"
  exit 2
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=163
export GZ_PARTITION=factory_test_${ROS_DOMAIN_ID}
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_sparse_bin_pick_truth.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=false \
  enable_perception:=true \
  enable_sparse_bin_perception:=true \
  randomize_raw_bin:=true \
  raw_bin_seed:="${raw_bin_seed}" \
  raw_part_count:="${raw_part_count}" \
  headless:=true \
  robot_x:=-3.268 \
  robot_y:=-2.70 \
  robot_yaw:=3.14159 >"${log_file}" 2>&1 &
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
  tail -220 "${log_file}"
  exit 1
}

controller_output=""
action_info=""
for _ in $(seq 1 120); do
  controller_output="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  action_info="$(ros2 action info /manipulate_part 2>/dev/null || true)"
  if [[ "${controller_output}" == *"arm_controller"*"active"* \
     && "${controller_output}" == *"gripper_controller"*"active"* ]] &&
    grep -q 'Action servers: 1' <<<"${action_info}"; then
    break
  fi
  sleep 0.25
done

if [[ "${controller_output}" != *"arm_controller"*"active"* \
   || "${controller_output}" != *"gripper_controller"*"active"* ]]; then
  fail_with_log "Manipulation controllers did not become active"
fi
grep -q 'Action servers: 1' <<<"${action_info}" ||
  fail_with_log "Manipulation server did not become active"

candidate_output=""
candidate_count=0
stable_count=0
for _ in $(seq 1 80); do
  candidate_output="$(timeout 3 ros2 topic echo \
    /perception/raw_part_candidates --once 2>/dev/null || true)"
  candidate_count="$(grep -c 'position:' <<<"${candidate_output}" || true)"
  if (( candidate_count >= 1 )) &&
    grep -q 'Unordered raw-material workspace is ready' "${log_file}"; then
    stable_count=$((stable_count + 1))
    if (( stable_count >= 3 )); then
      break
    fi
  else
    stable_count=0
  fi
  sleep 0.5
done

if (( stable_count < 3 )); then
  echo "${candidate_output}"
  fail_with_log "RGB-D did not publish a usable raw-part candidate stream"
fi
grep -q 'Unordered raw-material workspace is ready' "${log_file}" ||
  fail_with_log "Raw-bin randomizer did not finish"

pick_started="${SECONDS}"
pick_output="$(timeout 480 ros2 action send_goal --feedback \
  /manipulate_part factory_interfaces/action/ManipulatePart \
  "{operation: 0, station_id: raw_bin, part_id: raw_part_2, placement_slot_id: 0}" \
  2>&1)" || {
    echo "${pick_output}"
    fail_with_log "Sparse-bin pick action failed"
  }

grep -q 'success: true' <<<"${pick_output}" || {
  echo "${pick_output}"
  fail_with_log "Sparse-bin pick did not report success"
}
grep -q 'PERCEIVE_RAW_PART' <<<"${pick_output}" ||
  fail_with_log "Pick bypassed the RGB-D target-selection phase"
grep -q 'VERIFY_TWO_FINGER_CONTACT' <<<"${pick_output}" ||
  fail_with_log "Pick bypassed two-finger contact verification"
grep -q 'VERIFY_LOAD_BEARING_GRASP' <<<"${pick_output}" ||
  fail_with_log "Pick bypassed the unassisted proof lift"

pick_elapsed=$((SECONDS - pick_started))
selected_target="$(sed -nE \
  's/.*Raw RGB-D target selected at \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\).*/\1,\2,\3/p' \
  "${log_file}" | tail -1)"
[[ -n "${selected_target}" ]] ||
  fail_with_log "Pick succeeded without a recorded RGB-D target"
physical_part_id="$(sed -nE \
  's/.*physical_part_id: ([^ ]+).*/\1/p' <<<"${pick_output}" | tail -1)"
[[ -n "${physical_part_id}" ]] ||
  fail_with_log "Pick succeeded without a tactile-bound physical identity"
echo "sparse_bin_pick_success=true candidates=${candidate_count}" \
  "target=rgbd truth_role=residual_guard grasp=contact_verified" \
  "layout=unordered_workspace raw_seed=${raw_bin_seed}" \
  "raw_part_count=${raw_part_count} selected_xyz=${selected_target}" \
  "physical_part=${physical_part_id}" \
  "manipulation_seconds=${pick_elapsed}"
