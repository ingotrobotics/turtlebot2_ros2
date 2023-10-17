# Copyright 2023 Ingot Robotics

FROM ingot/turtlebot2-ros-iron:hokuyo
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

