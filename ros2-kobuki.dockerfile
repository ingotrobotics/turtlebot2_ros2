# Copyright 2023 Ingot Robotics

FROM osrf/ros:rolling-desktop
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get upgrade -y

# Build Kobuki drivers (https://kobuki.readthedocs.io/en/release-1.0.x/software.html)
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
RUN echo "  cmv_vel_mux      : { type: 'git', url: 'https://github.com/kobuki-base/cmd_vel_mux.git', version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 
RUN echo "  kobuki_ros       : { type: 'git', url: 'https://github.com/kobuki-base/kobuki_ros.git',  version: 'devel' }" >> $KOBUKI_BUILD_SPACE/kobuki_standalone.repos 

#ENV VIRTUAL_ENV="$KOBUKI_BUILD_SPACE/.venv"
#ENV PATH="$VIRTUAL_ENV/bin:$PATH"

RUN mkdir -p $ROBOT_WORKSPACE/src && vcs import $ROBOT_WORKSPACE/src < $KOBUKI_BUILD_SPACE/kobuki_standalone.repos
RUN touch $ROBOT_WORKSPACE/src/eigen/AMENT_IGNORE

# Install dependencies with rosdep
RUN apt-get update && cd $ROBOT_WORKSPACE && rosdep install --from-paths src -y --ignore-src

# Source virtual environment, upgrade setuptools (58.2 is last version that doesn't issue a warning)
SHELL ["/bin/bash", "-c"]
RUN sed -i '/setuptools/s/45.2.0/68.2.2/' $KOBUKI_BUILD_SPACE/venv.bash && source "$KOBUKI_BUILD_SPACE/venv.bash"

# Build release with debug symbols
RUN source /opt/ros/rolling/setup.bash && cd $ROBOT_WORKSPACE && colcon build --merge-install --parallel-workers 6 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo

# update the source workspace
#RUN vcs pull /home/robot/src && deactivate



#RUN apt-get install ros-rolling-urg-node -y

# Need to create a urg_nde.yaml in the container
# launch command:
# ros2 run urg_node urg_node_driver --ros-args --params-file urg04.yaml




# Consider updating kobuki_standalone.repos to point to rolling, not foxy

# Use Multi-stage build to just have Kobuki final built package files, similar to a binary install
# https://www.docker.com/blog/intro-guide-to-dockerfile-best-practices/
# https://docs.docker.com/develop/develop-images/guidelines/

# Launch container with Hokuyo and Kobuki USB connections
# docker run -it --device=/dev/kobuki --device=/dev/ttyACM0 ingot/kobuki_build:latest

# The above command needs some serious udev work if the device is unplugged while the container
# is running, but I think we can handle this.

# How to add udev rule
#ADD  /[source]/[destination]

# Kobuki udev rules
# wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules

# Run second container to do keyboard control
# ros2 run kobuki_keyop kobuki_keyop_node --ros-args --remap /cmd_vel:=/commands/velocity
# Or use dev laptop (also works in container)
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/commands/velocity




