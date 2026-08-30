#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u
source "${project_root}/scripts/test_support.sh"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export ROS_DOMAIN_ID=43
export GZ_PARTITION=factory_test_$ROS_DOMAIN_ID
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_HEADLESS_RENDERING=1
ros2 daemon stop >/dev/null 2>&1 || true

log_file="/tmp/factory_gazebo_smoke.log"
setsid ros2 launch factory_bringup gazebo.launch.py headless:=true >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  terminate_test_processes
}
trap cleanup EXIT

controller_output=""
for _ in $(seq 1 100); do
  controller_output="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  if [[ "${controller_output}" == *"base_controller"*"active"* \
     && "${controller_output}" == *"arm_controller"*"active"* \
     && "${controller_output}" == *"gripper_controller"*"active"* \
     && "${controller_output}" == *"joint_state_broadcaster"*"active"* ]]; then
    break
  fi
  sleep 0.25
done

if [[ "${controller_output}" != *"base_controller"*"active"* \
   || "${controller_output}" != *"arm_controller"*"active"* \
   || "${controller_output}" != *"gripper_controller"*"active"* \
   || "${controller_output}" != *"joint_state_broadcaster"*"active"* ]]; then
  echo "Controllers did not all become active"
  echo "${controller_output}"
  tail -120 "${log_file}"
  exit 1
fi

topic_output="$(ros2 topic list)"
for expected_topic in \
  /clock /scan /ground_truth/odom /base_controller/odom \
  /camera/image_raw /camera/depth/image_raw /camera/camera_info \
  /camera_aux/image_raw /camera_aux/depth/image_raw \
  /camera_aux/camera_info \
  /tag_camera/image_raw /tag_camera/camera_info \
  /joint_states /battery_state /factory/charging/contacts \
  /machine_1/door_position_cmd /machine_2/door_position_cmd /machine_3/door_position_cmd; do
  if [[ "${topic_output}" != *"${expected_topic}"* ]]; then
    echo "Missing expected topic: ${expected_topic}"
    tail -120 "${log_file}"
    exit 1
  fi
done

if ! timeout 5 ros2 topic echo /base_controller/odom --once >/dev/null; then
  echo "Wheel-integrated controller odometry exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 5 ros2 topic echo /ground_truth/odom --once >/dev/null; then
  echo "Ground-truth odometry topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /camera/image_raw --once >/dev/null; then
  echo "RGB image topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /camera/camera_info --once >/dev/null; then
  echo "Camera-info topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /camera_aux/image_raw --once >/dev/null; then
  echo "Auxiliary RGB image topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /camera_aux/depth/image_raw --once >/dev/null; then
  echo "Auxiliary depth topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /camera_aux/camera_info --once >/dev/null; then
  echo "Auxiliary camera-info topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /tag_camera/image_raw --once >/dev/null; then
  echo "High station-tag camera topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

if ! timeout 10 ros2 topic echo /tag_camera/camera_info --once >/dev/null; then
  echo "High station-tag camera-info topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
fi

joint_state_output="$(timeout 10 ros2 topic echo /joint_states --once)" || {
  echo "Joint-state topic exists but publishes no data"
  tail -120 "${log_file}"
  exit 1
}
for passive_joint in \
  front_caster_swivel_joint front_caster_wheel_joint \
  rear_caster_swivel_joint rear_caster_wheel_joint; do
  if [[ "${joint_state_output}" != *"${passive_joint}"* ]]; then
    echo "Missing passive joint state: ${passive_joint}"
    tail -120 "${log_file}"
    exit 1
  fi
done

node_output="$(ros2 node list)"
if [[ "${node_output}" != *"/machine_door_visualizer"* ]]; then
  echo "Machine door visualizer is not running"
  tail -120 "${log_file}"
  exit 1
fi

echo "gazebo_smoke_ok controllers=4 sensors=lidar+dual_lower_rgbd+station_tag_color wheel_odom=active truth_odom=observer_only" \
  "passive_casters=4 cnc_doors=3"
