#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-/home/ubuntu/Embodied_Robotic_Arm}"
result_dir="${workspace}/artifacts/docker_validation"
image="${FACTORY_DOCKER_IMAGE:-embodied-factory:jazzy}"

mkdir -p "${result_dir}"
cd "${workspace}"

build_output=()
if sudo docker buildx version >/dev/null 2>&1; then
  build_output=(--progress plain)
fi

base_image="${FACTORY_DOCKER_BASE_IMAGE:-osrf/ros:jazzy-desktop-full-noble}"
pull_attempts="${FACTORY_DOCKER_PULL_ATTEMPTS:-4}"

for ((attempt = 1; attempt <= pull_attempts; attempt++)); do
  if sudo docker pull "${base_image}" \
    2>&1 | tee "${result_dir}/base_image_pull.log"; then
    break
  fi
  if ((attempt == pull_attempts)); then
    echo "Unable to pull ${base_image} after ${pull_attempts} attempts" >&2
    exit 1
  fi
  echo "Base image pull attempt ${attempt} failed; retrying in 10 seconds" >&2
  sleep 10
done

sudo docker build "${build_output[@]}" \
  --build-arg "ROS_BASE_IMAGE=${base_image}" \
  -t "${image}" . \
  2>&1 | tee "${result_dir}/build.log"
sudo docker run --rm "${image}" ./scripts/run_checks.sh \
  2>&1 | tee "${result_dir}/run_checks.log"
sudo docker image inspect "${image}" \
  --format '{{json .RepoDigests}} {{.Id}}' \
  | tee "${result_dir}/image_identity.txt"
