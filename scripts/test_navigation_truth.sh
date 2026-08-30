#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=45
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
export ROS2CLI_DISABLE_DAEMON=1

# Recover from an earlier interrupted run before allocating CycloneDDS
# participants in this test-owned domain and Gazebo partition.
terminate_test_processes

log_file="/tmp/factory_navigation_truth.log"
setsid ros2 launch factory_bringup physical_stack.launch.py \
  use_navigation:=true use_moveit:=false headless:=true >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  terminate_test_processes
  sleep 0.5
  terminate_test_processes
}
trap cleanup EXIT

# Software rendering can delay controller and sensor-node creation. The 35 s
# budget is measured on the supported WSL setup; message receipt still proves
# readiness with the sensor topic's actual Best-Effort QoS.
sleep 35
if ! timeout 30 ros2 topic echo /scan_filtered --once \
    --qos-reliability best_effort >/dev/null 2>&1; then
  echo "Filtered laser scan did not publish a message"
  tail -160 "${log_file}"
  exit 1
fi

timeout 170 ros2 run factory_core navigate_station --ros-args \
  -p station:=raw_bin -p navigation_timeout:=150.0

echo "Controller wheel odometry at Nav2 success:"
timeout 5 ros2 topic echo /base_controller/odom --once --field pose.pose
echo "AMCL map pose at Nav2 success:"
timeout 5 ros2 topic echo /amcl_pose --once --field pose.pose

station_config="${project_root}/src/factory_core/config/stations.yaml"
read -r target_x target_y target_yaw < <(
  python3 - "${station_config}" <<'PY'
from pathlib import Path
import sys

from factory_core.station_config import load_station_definitions

station = load_station_definitions(Path(sys.argv[1]))["raw_bin"]
print(station.staging_pose.x, station.staging_pose.y, station.staging_pose.yaw)
PY
)

truth_position="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.position)"
truth_x="$(awk '/x:/{print $2; exit}' <<<"${truth_position}")"
truth_y="$(awk '/y:/{print $2; exit}' <<<"${truth_position}")"
truth_orientation="$(timeout 5 ros2 topic echo /ground_truth/odom --once \
  --field pose.pose.orientation)"
truth_z="$(awk '/z:/{print $2; exit}' <<<"${truth_orientation}")"
truth_w="$(awk '/w:/{print $2; exit}' <<<"${truth_orientation}")"
awk \
  -v x="${truth_x}" \
  -v y="${truth_y}" \
  -v z="${truth_z}" \
  -v w="${truth_w}" \
  -v target_x="${target_x}" \
  -v target_y="${target_y}" \
  -v target_yaw="${target_yaw}" \
  'BEGIN {
  # The independent OdometryPublisher reports the base pose in the world
  # frame; no drive plugin publishes a second origin-relative odometry topic.
  error = sqrt((x-target_x)^2 + (y-target_y)^2)
  yaw = 2.0 * atan2(z, w)
  yaw_error = atan2(sin(yaw-target_yaw), cos(yaw-target_yaw))
  printf "navigation_truth x=%.3f y=%.3f position_error=%.3f m ", x, y, error
  printf "yaw=%.3f yaw_error=%.3f rad\n", yaw, yaw_error
  if (error > 0.40 || sqrt(yaw_error*yaw_error) > 0.20) exit 1
}'

if ! timeout 15 ros2 topic echo /camera/image_raw --once >/dev/null; then
  echo "RGB image stream stopped after navigation"
  tail -120 "${log_file}"
  exit 1
fi

detections="$(timeout 20 ros2 topic echo /detections --once 2>/dev/null || true)"
if ! grep -q 'id: 10' <<<"${detections}"; then
  echo "AprilTag 10 was not detected at the raw-bin staging pose"
  echo "${detections}"
  tail -120 "${log_file}"
  exit 1
fi

ros2 topic pub --once --qos-durability transient_local \
  /perception/target_tag_id std_msgs/msg/Int32 "{data: 10}" >/dev/null
detected_pose="$(timeout 10 ros2 topic echo /detected_dock_pose --once 2>/dev/null || true)"
if [[ -z "${detected_pose}" ]]; then
  echo "Targeted AprilTag pose was not published"
  tail -120 "${log_file}"
  exit 1
fi
echo "Raw camera-frame Tag 10 pose:"
echo "${detected_pose}"
