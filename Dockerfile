ARG ROS_BASE_IMAGE=osrf/ros:jazzy-desktop-full-noble
FROM ${ROS_BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
    git \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-venv \
    python3-vcstool \
    ros-jazzy-rmw-cyclonedds-cpp \
    tmux \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src ./src
COPY scripts ./scripts
COPY factory.repos README.md ./

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    ./scripts/setup_mcp_env.sh && \
    rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /ros_entrypoint_factory.sh
RUN chmod +x /ros_entrypoint_factory.sh

ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1

ENTRYPOINT ["/ros_entrypoint_factory.sh"]
CMD ["bash"]
