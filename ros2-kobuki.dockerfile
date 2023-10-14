# Copyright 2023 Ingot Robotics

FROM osrf/ros:rolling-desktop
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get upgrade -y
#RUN apt-get update && apt-get install ros-rolling-kobuki* -y

#RUN apt-get install ros-rolling-urg-node -y

# How to add udev rule
#ADD  /[source]/[destination]


# Need to create a urg_nde.yaml in the container
# launch command:
# ros2 run urg_node urg_node_driver --ros-args --params-file urg04.yaml

# Kobuki udev rules
# wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules

# Build Kobuki drivers
# https://kobuki.readthedocs.io/en/release-1.0.x/software.html
#
RUN apt-get update && apt-get install wget python3-venv -y
ARG ROBOT_WORKSPACE=/home/robot
ARG KOBUKI_BUILD_SPACE=$ROBOT_WORKSPACE/kobuki_build_space
RUN mkdir -p "$KOBUKI_BUILD_SPACE" && cd "$KOBUKI_BUILD_SPACE" && \
    wget https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/venv.bash && \
    wget https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/colcon.meta && \
    wget https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/kobuki_standalone.repos


# Update ecl_lite version to 1.2.x in kobuki_standalone.repos
# See https://github.com/stonier/ecl_lite/pull/38
RUN sed -i '/ecl_lite/s/release\/1.1.x/release\/1.2.x/g' $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 

# edit kobuki_standalone.repos to add line for kobuki_ros --> also need to `apt-get install ros-rolling-kobuki-ros-interfaces ros-rolling-diagnostic-updater -y` first
# 'echo hello world' >> kobuki_standalone.repos
#RUN echo "  kobuki_ros       : { type: 'git', url: 'https://github.com/kobuki-base/kobuki_ros.git',  version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 

# Source virtual environment, upgrade setuptools (58.2 is last version that doesn't issue a warning)
SHELL ["/bin/bash", "-c"]
RUN source "$KOBUKI_BUILD_SPACE/venv.bash" && pip install --upgrade setuptools

#ENV VIRTUAL_ENV="$KOBUKI_BUILD_SPACE/.venv"
#ENV PATH="$VIRTUAL_ENV/bin:$PATH"

RUN mkdir -p $ROBOT_WORKSPACE/src && vcs import $ROBOT_WORKSPACE/src < $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
RUN touch $ROBOT_WORKSPACE/src/eigen/AMENT_IGNORE

# Build release with debug symbols
RUN cd $ROBOT_WORKSPACE && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo

# update the source workspace
#RUN vcs pull /home/robot/src && deactivate





# Launch container with Hokuyo and Kobuki USB connections
# docker run -it --device=/dev/kobuki --device=/dev/ttyACM0 ingot/kobuki_build:latest

# The above command needs some serious udev work if the device is unplugged while the container
# is running, but I think we can handle this.
