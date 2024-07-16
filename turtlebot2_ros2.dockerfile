# Copyright 2023 Ingot Robotics

ARG from_image=ros:iron
ARG robot_workspace="/root/robot"

#FROM osrf/ros:iron-desktop
FROM $from_image AS builder
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get upgrade -y && apt-get install wget -y

ARG robot_workspace
ENV ROBOT_WORKSPACE=$robot_workspace

# Use Multi-stage build to just have Kobuki final built package files, similar to a binary install
# https://www.docker.com/blog/intro-guide-to-dockerfile-best-practices/
# https://docs.docker.com/develop/develop-images/guidelines/

# Build Kobuki drivers (https://kobuki.readthedocs.io/en/release-1.0.x/software.html)
#RUN apt-get update && apt-get install wget python3-venv -y

ENV KOBUKI_BUILD_SPACE=$ROBOT_WORKSPACE/kobuki_build_space
WORKDIR $KOBUKI_BUILD_SPACE

RUN wget https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/colcon.meta && \
    wget https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/kobuki_standalone.repos
#   wget https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/venv.bash

# Update kobuki_standalone.repos to build on iron
# comment out foxy ament tools
RUN sed -i 's/ament_/#&/g' $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
# edit kobuki_standalone.repos to add line for kobuki_ros
RUN echo "  cmv_vel_mux      : { type: 'git', url: 'https://github.com/kobuki-base/cmd_vel_mux.git', version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 
RUN echo "  kobuki_ros       : { type: 'git', url: 'https://github.com/kobuki-base/kobuki_ros.git',  version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 
# update ecl_lite version to 1.2.x in kobuki_standalone.repos
# (see https://github.com/stonier/ecl_lite/pull/38 )
RUN sed -i '/ecl_lite/s/release\/1.1.x/release\/1.2.x/g' $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 

RUN mkdir -p $ROBOT_WORKSPACE/src && vcs import $ROBOT_WORKSPACE/src < $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
RUN touch $ROBOT_WORKSPACE/src/eigen/AMENT_IGNORE

# Install dependencies
WORKDIR $ROBOT_WORKSPACE
RUN apt-get update && rosdep install --from-paths ./src -y --ignore-src

SHELL ["/bin/bash", "-c"]

# Build release with debug symbols
ARG parallel_jobs=8
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd $ROBOT_WORKSPACE && colcon build --parallel-workers $parallel_jobs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
#    -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo


FROM $from_image AS mobile_base

ARG robot_workspace
ENV ROBOT_WORKSPACE=$robot_workspace

WORKDIR $ROBOT_WORKSPACE
RUN mkdir -p $ROBOT_WORKSPACE/install
COPY --from=builder $ROBOT_WORKSPACE/install/ install

# Install dependencies with rosdep
WORKDIR $ROBOT_WORKSPACE
RUN apt-get update && apt-get upgrade -y && rosdep install --from-paths ./install/*/ -y --ignore-src

# Install URG node
RUN apt-get update && apt-get install ros-$ROS_DISTRO-urg-node -y

# Patch urg_node_launch.py to point to the actual executable
RUN sed -i "s/node_executable='urg_node'/executable='urg_node_driver'/g" /opt/ros/$ROS_DISTRO/share/urg_node/launch/urg_node_launch.py

# Patch urg_node_serial.yaml (temporary fix, really should be changing the launch file)
RUN sed -i 's/laser_frame_id: "laser"/laser_frame_id: "nav_laser"/g' /opt/ros/$ROS_DISTRO/share/urg_node/launch/urg_node_serial.yaml

# Install RealSense (for D435)
RUN apt-get update && apt-get install ros-$ROS_DISTRO-realsense2-* -y

# Setup Turtlebot2 URDF
#COPY turtlebot2_description/ $ROBOT_WORKSPACE/src/turtlebot2_description
#COPY turtlebot2_bringup/ $ROBOT_WORKSPACE/src/turtlebot2_bringup
#RUN --mount=type=bind,source=.,target=$ROBOT_WORKSPACE/src,readonly \
#    mkdir -p $ROBOT_WORKSPACE/src
# Pre-copied URDF and xacro files from https://github.com/turtlebot/turtlebot.git into our turtlebot2_description folder


# Install Nav2 bringup package
#RUN apt-get update && apt-get install -y \
#    ros-$ROS_DISTRO-nav2-bringup \

# Install dependencies
WORKDIR $ROBOT_WORKSPACE
RUN --mount=type=bind,source=.,target=$ROBOT_WORKSPACE/src,readonly \
    apt-get update && rosdep install --from-paths ./src -y --ignore-src

# Build turtlebot2_description and turtlebot2_bringup
SHELL ["/bin/bash", "-c"]
ARG parallel_jobs=8
RUN --mount=type=bind,source=.,target=$ROBOT_WORKSPACE/src,readonly \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $ROBOT_WORKSPACE && \
    colcon build --packages-select turtlebot2_description turtlebot2_bringup --parallel-workers $parallel_jobs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


# Kobuki udev rules for host machine
# `wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules`
# and put that file in `/etc/udev/rules.d/`

# Launch container with Hokuyo and Kobuki USB connections
# `docker run -it --device=/dev/ttyUSB0 --device=/dev/kobuki --device=/dev/ttyACM0 <container name>`
# If running rviz on a separate machine, adding `--network=host` is a
# docker networking work-around to allow containers to communicate
#
# Launch turtlebot2 with
# `ros2 launch turtlebot2_bringup turtlebot2_bringup.launch.py`

# Run second container to do keyboard control, and in that container
# `ros2 run kobuki_keyop kobuki_keyop_node`
# or
# `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
