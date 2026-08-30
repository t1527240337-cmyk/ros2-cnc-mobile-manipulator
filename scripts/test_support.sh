#!/usr/bin/env bash

# Physical integration tests run in one WSL instance. Jazzy's native
# localhost mode keeps discovery and data traffic off changing host/VPN links.
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1

# Helpers shared by Gazebo integration tests. Each test owns one GZ_PARTITION,
# so cleanup never touches an interactive simulator or another test.
_partition_gazebo_pids() {
  local environment_file pid command_line

  for environment_file in /proc/[0-9]*/environ; do
    if ! grep -Fzqx "GZ_PARTITION=${GZ_PARTITION}" \
      "${environment_file}" 2>/dev/null; then
      continue
    fi

    pid="${environment_file#/proc/}"
    pid="${pid%/environ}"
    command_line="$(
      tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true
    )"
    if [[ "${command_line}" == *"gz sim"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done
}

terminate_gazebo_partition() {
  local attempt
  local -a gazebo_pids=()

  mapfile -t gazebo_pids < <(_partition_gazebo_pids)
  if ((${#gazebo_pids[@]} == 0)); then
    return
  fi

  kill -TERM "${gazebo_pids[@]}" 2>/dev/null || true
  for attempt in $(seq 1 20); do
    mapfile -t gazebo_pids < <(_partition_gazebo_pids)
    if ((${#gazebo_pids[@]} == 0)); then
      return
    fi
    sleep 0.1
  done

  kill -KILL "${gazebo_pids[@]}" 2>/dev/null || true
}

_ros_domain_pids() {
  local environment_file pid command_line

  [[ -n "${ROS_DOMAIN_ID:-}" ]] || return
  for environment_file in /proc/[0-9]*/environ; do
    if ! grep -Fzqx "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
      "${environment_file}" 2>/dev/null; then
      continue
    fi

    pid="${environment_file#/proc/}"
    pid="${pid%/environ}"
    [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
    command_line="$(
      tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true
    )"
    if [[ "${command_line}" == *"/opt/ros/jazzy/"* ||
      "${command_line}" == *"/Embodied_Robotic_Arm/install/"* ||
      "${command_line}" == *"ros2cli.daemon"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done
}

terminate_ros_domain() {
  local attempt
  local -a ros_pids=()

  mapfile -t ros_pids < <(_ros_domain_pids)
  if ((${#ros_pids[@]} == 0)); then
    return
  fi

  kill -TERM "${ros_pids[@]}" 2>/dev/null || true
  for attempt in $(seq 1 20); do
    mapfile -t ros_pids < <(_ros_domain_pids)
    if ((${#ros_pids[@]} == 0)); then
      return
    fi
    sleep 0.1
  done

  kill -KILL "${ros_pids[@]}" 2>/dev/null || true
}

terminate_test_processes() {
  terminate_gazebo_partition
  terminate_ros_domain
}
