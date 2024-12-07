# Copyright 2023-2024 Ingot Robotics

ARG from_image=ros:iron
ARG robot_workspace="/root/robot"

FROM $from_image AS kobuki_builder
LABEL org.opencontainers.image.authors="proan@ingotrobotics.com"

RUN apt-get update && apt-get upgrade -y && apt-get install wget -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

ARG robot_workspace
ENV ROBOT_WORKSPACE=$robot_workspace

# Use Multi-stage build to just have Kobuki final built package files, similar to a binary install
# https://www.docker.com/blog/intro-guide-to-dockerfile-best-practices/
# https://docs.docker.com/develop/develop-images/guidelines/

# Build Kobuki drivers (https://kobuki.readthedocs.io/en/release-1.0.x/software.html)
ENV KOBUKI_BUILD_SPACE=$ROBOT_WORKSPACE/kobuki_build_space
WORKDIR $KOBUKI_BUILD_SPACE

# Get list of repos needed to build from source
#RUN wget -q https://raw.githubusercontent.com/kobuki-base/kobuki_documentation/release/1.0.x/resources/kobuki_standalone.repos
COPY kobuki_standalone.repos .

# Use vcs to import source repos
RUN mkdir -p $ROBOT_WORKSPACE/src && vcs import $ROBOT_WORKSPACE/src < $KOBUKI_BUILD_SPACE/kobuki_standalone.repos

# Install dependencies
WORKDIR $ROBOT_WORKSPACE
RUN apt-get update && rosdep install --from-paths ./src -y --ignore-src && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Build release with debug symbols
ARG parallel_jobs=8
WORKDIR $ROBOT_WORKSPACE
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && colcon build --parallel-workers $parallel_jobs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

FROM $from_image AS mobile_base

ARG robot_workspace
ENV ROBOT_WORKSPACE=$robot_workspace

WORKDIR $ROBOT_WORKSPACE
RUN mkdir -p $ROBOT_WORKSPACE/install
COPY --from=kobuki_builder $ROBOT_WORKSPACE/install/ install

# Install Kobuki dependencies with rosdep
WORKDIR $ROBOT_WORKSPACE
RUN apt-get update && apt-get upgrade -y && rosdep install --from-paths ./install/*/ -y --ignore-src && rm -rf /var/lib/apt/lists/*

# Install URG node
RUN apt-get update && apt-get install "ros-$ROS_DISTRO-urg-node" -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Patch urg_node_launch.py to point to the actual executable
RUN sed -i "s/node_executable='urg_node'/executable='urg_node_driver'/g" "/opt/ros/$ROS_DISTRO/share/urg_node/launch/urg_node_launch.py"

# Patch urg_node_serial.yaml (temporary fix, really should be changing the launch file)
RUN sed -i 's/laser_frame_id: "laser"/laser_frame_id: "nav_laser"/g' "/opt/ros/$ROS_DISTRO/share/urg_node/launch/urg_node_serial.yaml"

# Install RealSense drivers and ROS nodes
RUN apt-get update && apt-get install "ros-$ROS_DISTRO"-realsense2-* -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Install Nav2 bringup package
RUN apt-get update && apt-get install "ros-$ROS_DISTRO-nav2-bringup" -y --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Install Turtlebot dependencies with rosdep
WORKDIR "$ROBOT_WORKSPACE"
RUN mkdir -p "$ROBOT_WORKSPACE/src"
#RUN --mount=type=bind,source=.,target="$ROBOT_WORKSPACE/src",readonly \
COPY ./turtlebot2_description ./src/turtlebot2_description
COPY ./turtlebot2_bringup ./src/turtlebot2_bringup
RUN apt-get update && rosdep install --from-paths ./src -y --ignore-src && rm -rf /var/lib/apt/lists/*

# Build turtlebot2_description and turtlebot2_bringup
# The URDF and xacro files come from https://github.com/turtlebot/turtlebot.git
# and have been copied into the turtlebot2_description folder
SHELL ["/bin/bash", "-c"]
ARG parallel_jobs=8
WORKDIR $ROBOT_WORKSPACE
#RUN --mount=type=bind,source=.,target="$ROBOT_WORKSPACE/src",readonly \
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    colcon build --packages-select turtlebot2_description turtlebot2_bringup --parallel-workers $parallel_jobs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Install Zenoh
RUN git clone https://github.com/ros2/rmw_zenoh.git "$ROBOT_WORKSPACE/src/rmw_zenoh"
RUN apt-get update && \
    rosdep install --from-paths ./src -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN mkdir -p "$ROBOT_WORKSPACE/zenoh_confs" && \
    cp "$ROBOT_WORKSPACE/src/rmw_zenoh/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5" "$ROBOT_WORKSPACE/zenoh_confs/ROUTER_CONFIG.json5"

# Set ROS to use Zenoh as the middleware
RUN echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> /root/.bashrc && \
    echo "export RUST_LOG=info" >> /root/.bashrc

# Copy script for launching Zenoh in the background
COPY background_zenoh_router.sh "$ROBOT_WORKSPACE"
RUN chmod +x "$ROBOT_WORKSPACE/background_zenoh_router.sh"

# Kobuki udev rules for host machine
# `wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules`
# and put that file in `/etc/udev/rules.d/`

# Launch container with Hokuyo and Kobuki USB connections
# `docker run -it --rm -p 7447:7447 --device=/dev/ttyUSB0 --device=/dev/kobuki --device=/dev/ttyACM0 <container name>`
# Launch Zenoh router with
# `./background_zenoh_router.sh`
# Launch turtlebot2 with
# `source install/setup.bash && ros2 launch turtlebot2_bringup turtlebot2_bringup.launch.py`
# Run second container to do keyboard control, and in that container
# `./background_zenoh_router.sh`, `source install/setup.bash`, and then
# `ros2 run kobuki_keyop kobuki_keyop_node` or
# `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
