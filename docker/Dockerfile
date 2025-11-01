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
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y cmake g++ wget unzip
RUN mkdir -p ~/opencv && cd ~/opencv && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.12.0.zip && \
    unzip opencv.zip && \
    mkdir -p build && cd build && \
    cmake ../opencv-4.12.0 && cmake --build . && \
    sudo make install

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs

ENV KOBUKI_WS=/kobuki_ws
ENV SOLUTION_WS=/workspace

# Build the hardware stack once and cache its layers.
COPY kobuki/workspace/ ${KOBUKI_WS}/
WORKDIR ${KOBUKI_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# Overlay solution workspace (rebuilt frequently).
COPY solution/workspace/ ${SOLUTION_WS}/
COPY launch/ ${SOLUTION_WS}/launch/
COPY docker/ ${SOLUTION_WS}/docker/
WORKDIR ${SOLUTION_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${KOBUKI_WS}/install/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${KOBUKI_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "source ${SOLUTION_WS}/install/setup.bash" >> /root/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
WORKDIR ${SOLUTION_WS}
