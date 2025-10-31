# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
ARG base_img=nickodema/kobuki:humble-22.04-100625

FROM ${base_img}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-foxglove-bridge \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

ENV WS=/workspace
WORKDIR ${WS}

COPY workspace/ ${WS}/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
