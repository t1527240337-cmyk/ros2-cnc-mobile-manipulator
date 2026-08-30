#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

dds_mode="${FACTORY_DDS_PROBE_MODE:-cyclone_config}"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_LOCALHOST_ONLY=1
case "${dds_mode}" in
  cyclone_config)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://${project_root}/scripts/cyclonedds_localhost.xml"
    ;;
  cyclone_ros_localhost)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    unset CYCLONEDDS_URI
    export ROS_LOCALHOST_ONLY=1
    ;;
  fastdds_ros_localhost)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI
    export ROS_LOCALHOST_ONLY=1
    ;;
  *)
    echo "Unknown FACTORY_DDS_PROBE_MODE=${dds_mode}" >&2
    exit 2
    ;;
esac
export ROS_DOMAIN_ID="${FACTORY_DDS_PROBE_DOMAIN_ID:-159}"
export ROS2CLI_DISABLE_DAEMON=1

probe_log="$(mktemp)"
subscriber_pid=""
cleanup() {
  if [[ -n "${subscriber_pid}" ]]; then
    kill "${subscriber_pid}" 2>/dev/null || true
  fi
  rm -f "${probe_log}"
}
trap cleanup EXIT

timeout 12 ros2 topic echo \
  /factory/dds_probe std_msgs/msg/String --once >"${probe_log}" 2>&1 &
subscriber_pid=$!
sleep 1
timeout 10 ros2 topic pub --rate 5 --times 25 \
  /factory/dds_probe std_msgs/msg/String "{data: probe}"
wait "${subscriber_pid}"

grep -q 'data: probe' "${probe_log}"
echo "dds_localhost_probe=true mode=${dds_mode}"
