#!/usr/bin/env bash
set -eo pipefail

# Focused black-box acceptance for the finished-bin RGB-D detector. Gazebo's
# pose service is used only to arrange the test specimen; the detector receives
# camera images and TF exactly as it does during production execution.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=158
export GZ_PARTITION=factory_test_${ROS_DOMAIN_ID}
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

log_file="/tmp/factory_finished_slot_sensor_truth.log"
fixture_state_file="/tmp/factory_finished_slot_sensor_fixture.txt"

setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=false \
  use_moveit:=false \
  enable_sparse_bin_perception:=false \
  enable_finished_slot_perception:=true \
  headless:=true \
  robot_x:=3.268 \
  robot_y:=-2.70 \
  robot_yaw:=0.0 >"${log_file}" 2>&1 &
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
  tail -180 "${log_file}"
  exit 1
}

wait_for_interfaces() {
  local topics
  for _ in $(seq 1 60); do
    topics="$(ros2 topic list 2>/dev/null || true)"
    if grep -q '^/perception/finished_bin_slots$' <<<"${topics}" &&
      gz service -l 2>/dev/null |
        grep -q '^/world/multi_machine_factory/set_pose$'; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

request_source_fixture_release() {
  timeout 10 ros2 topic echo \
    /factory/fixture/raw_part_2/attached --once \
    >"${fixture_state_file}" 2>&1 &
  local state_pid=$!
  sleep 0.5

  timeout 6 ros2 topic pub --rate 4 \
    /factory/fixture/raw_part_2/detach std_msgs/msg/Empty '{}' \
    >/dev/null 2>&1 &
  local request_pid=$!
  if ! wait "${state_pid}"; then
    kill "${request_pid}" 2>/dev/null || true
    fail_with_log "Source fixture did not acknowledge release"
  fi
  kill "${request_pid}" 2>/dev/null || true
  wait "${request_pid}" 2>/dev/null || true
  grep -q 'data: detached' "${fixture_state_file}" ||
    fail_with_log "Source fixture remained attached"
}

set_part_pose() {
  local x="$1"
  local y="$2"
  local z="$3"
  local result
  result="$(
    gz service -s /world/multi_machine_factory/set_pose \
      --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean \
      --timeout 5000 \
      --req "name: \"raw_part_2\", position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {w: 1.0}" \
      2>&1
  )" || fail_with_log "Could not arrange the RGB-D test specimen"
  grep -qi 'true' <<<"${result}" ||
    fail_with_log "Gazebo rejected the RGB-D test specimen pose"
}

read_slot_two() {
  local message block
  message="$(
    timeout 6 ros2 topic echo --once \
      /perception/finished_bin_slots 2>/dev/null || true
  )"
  block="$(grep -A3 'slot_id: 2' <<<"${message}" | head -4)"
  if grep -q 'observable: true' <<<"${block}"; then
    if grep -q 'occupied: true' <<<"${block}"; then
      printf '%s\n' occupied
    else
      printf '%s\n' empty
    fi
  else
    printf '%s\n' unknown
  fi
}

require_three_consecutive_frames() {
  local expected="$1"
  local consecutive=0
  local state
  for _ in $(seq 1 30); do
    state="$(read_slot_two)"
    if [[ "${state}" == "${expected}" ]]; then
      consecutive=$((consecutive + 1))
      if ((consecutive == 3)); then
        return 0
      fi
    else
      consecutive=0
    fi
  done
  return 1
}

wait_for_interfaces || fail_with_log "Finished-slot detector did not start"

# The production world starts with no part at the finished bin. This first
# assertion prevents a permanently-occupied or badly projected ROI from
# passing the test.
require_three_consecutive_frames empty ||
  fail_with_log "Finished slot 2 was not observably empty before placement"

request_source_fixture_release

# The robot pose is the surveyed finished-bin manipulation pose. Slot 2 is at
# base_link (0.72, 0.05); its 120 mm cylinder rests on the 0.554 m world table.
set_part_pose 3.988 -2.650 0.614
sleep 2
require_three_consecutive_frames occupied ||
  fail_with_log "RGB-D did not observe the supported specimen in slot 2"

# Move the specimen to a clear floor location and require the same physical
# sensor path to report an empty slot again.
set_part_pose 2.0 -1.0 0.061
sleep 2
require_three_consecutive_frames empty ||
  fail_with_log "RGB-D slot 2 remained occupied after specimen removal"

echo "finished_slot_sensor_success=true evidence=rgbd_three_frames" \
  "slot=2 transitions=empty_occupied_empty"
