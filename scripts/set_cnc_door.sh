#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 machine_1|machine_2|machine_3 open|close" >&2
  exit 2
fi

machine_id="$1"
operation="$2"
if [[ ! "${machine_id}" =~ ^machine_[123]$ ]]; then
  echo "Invalid machine id: ${machine_id}" >&2
  exit 2
fi

case "${operation}" in
  open) command=0 ;;
  close) command=1 ;;
  *) echo "Operation must be open or close" >&2; exit 2 ;;
esac

script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_directory}/setup_ros_env.sh"
set -u

ros2 service call /factory/machine_command factory_interfaces/srv/MachineCommand \
  "{machine_id: '${machine_id}', command: ${command}}"
