# Copyright 2023 Ingot Robotics

FROM ingot/turtlebot2-ros-iron:hokuyo
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-slam-toolbox

# Setup Turtlebot2 URDF
COPY turtlebot2_description/ $ROBOT_WORKSPACE/src/turtlebot2_description
COPY turtlebot2_bringup/ $ROBOT_WORKSPACE/src/turtlebot2_bringup

# Pre-copied URDF and xacro files from https://github.com/turtlebot/turtlebot.git into our turtlebot2_description folder


# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd $ROBOT_WORKSPACE && colcon build --merge-install --packages-select turtlebot2_description turtlebot2_bringup --parallel-workers 6 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo

