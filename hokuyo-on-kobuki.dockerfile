# Copyright 2023 Ingot Robotics

FROM ingot/turtlebot2-ros-iron:kobuki
MAINTAINER proan@ingotrobotics.com

RUN apt-get update && apt-get install ros-$ROS_DISTRO-urg-node -y

# Need to create a urg_node.yaml in the container, ROBOT_WORKSPACE environment variable set in Kobuki build stages
# Using COPY right now because we have the file in the same repo
#COPY urg-04lx-ug01.yaml $ROBOT_WORKSPACE/install/share/urg_node/config/

# Patch urg_node_launch.py
RUN sed -i "s/node_executable='urg_node'/executable='urg_node_driver'/g" /opt/ros/$ROS_DISTRO/share/urg_node/launch/urg_node_launch.py

# Patch urg_node_serial.yaml (until I can learn how to pass in my config file in the nested launch files)
RUN sed -i 's/laser_frame_id: "laser"/laser_frame_id: "nav_laser"/g' /opt/ros/$ROS_DISTRO/share/urg_node/launch/urg_node_serial.yaml

# launch command:
# ros2 run urg_node urg_node_driver --ros-args --params-file install/share/urg_node/config/urg-04lx-ug01.yaml


# Launch container with Hokuyo and Kobuki USB connections
# docker run -it --device=/dev/kobuki --device=/dev/ttyACM0 ingot/turtlebot2-ros:latest
