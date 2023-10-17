# Copyright 2023 Ingot Robotics

FROM ingot/turtlebot2-ros-iron:kobuki
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get install ros-$ROS_DISTRO-urg-node -y

# Need to create a urg_node.yaml in the container, ROBOT_WORKSPACE environment variable set in Kobuki build stages
# Using COPY right now because we have the file in the same repo
COPY urg-04lx-ug01.yaml $ROBOT_WORKSPACE/install/share/urg_node/config/


# launch command:
# ros2 run urg_node urg_node_driver --ros-args --params-file install/share/urg_node/config/urg-04lx-ug01.yaml


# Launch container with Hokuyo and Kobuki USB connections
# docker run -it --device=/dev/kobuki --device=/dev/ttyACM0 ingot/turtlebot2-ros:latest
