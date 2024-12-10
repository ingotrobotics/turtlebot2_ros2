# Turtlebot2 on ROS2 with Zenoh as RMW

### Maintainer: [Ingot Robotics](https://ingotrobotics.com)

There have been previous efforts to support Turtlebot2 (based on Yujin's Kobuki mobile base), but none of them are docker-based nor supporting ROS2 Iron. This repository is our attempt to address this. We followed and then edited the instructions for the release-1.0.x for the Kobuki <https://kobuki.readthedocs.io/en/release-1.0.x/software.html>, and we took inspiration from the ROS 2 Bouncy Turtlebot2 repo: <https://github.com/ros2/turtlebot2_demo/tree/master>.

The repository contains a dockerfile, three udev rules files, and two packages to run a Turtlebot2 using a Hokuyo URG-04 as the navigation sensor and an Intel RealSense as a depth camera.

Additional navigation sensors may be added if requested, and pull requests are always appreciated.

## Setup

After building up the Turtlebot2 and installing Ubuntu (22.04 or 24.04) on the robot computer, install Docker (`docker.io`), git, and any other favorite packages. We use a separate laptop for controlling the robot, so the robot computer also needs Open-SSH server and tmux.

Clone this repo, and from that directory, the dockerfile can be built with the command
`docker build -t ingot/turtlebot2-ros2:jazzy-desktop-zenoh -f turtlebot2_ros2.dockerfile --build-arg from_image=osrf/ros:jazzy-desktop --build-arg parallel_jobs=4 .`
The dockerfile has default values for the base image (`ros:iron`) and parallel build jobs (8), but the command above overrides the defaults. Adjust as fits your needs.

The robot computer needs device udev rules installed in `/etc/udev/rules.d/`. The Kobuki udev rule comes from the [Kobuki](https://github.com/kobuki-base) organization: <https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules>. The Hokuyo udev rule can be found many places online. The RealSense udev rule comes from the [IntelRealSense](https://github.com/IntelRealSense) organization: <https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules>.

After the docker container is built, it can be launched with the command
```docker run -it --rm -p 7447:7447 --device=/dev/ttyACM0 --device=/dev/kobuki --device=/ttyUSB0 -v /dev:/dev --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw" ingot/turtlebot2-ros2:jazzy-desktop-zenoh```
If you are not running any of the Kobuki utilities like `kobuki-version-info`, you can omit the `--device=/dev/kobuki` switch. The RealSense grabs multiple video devices, and we will look to improve the access to them so the `-v /dev:/dev` can be replaced.

Inside the docker container, first start the Zenoh router, then source the overlay, and then launch the Turtlebot2:
```
./background_zenoh_router.sh
source install/setup.bash
ros2 launch turtlebot2_bringup turtlebot2_bringup.launch.py
```

The Turtlebot2 can be controlled by a separate instance of the same container. To run this instance, drop all of the `--device` switches from the `docker run` command above, and add the robot's address as an environment variable: `--env='ZENOH_TARGET="tcp/<robot address>:7447"'`. Once in the container, start the Zenoh router in the background, source the overlay, and then run a keyboard or joystick control node, such as `ros2 run kobuki_keyop kobuki_keyop_node` or `ros2 run teleop_twist_keyboard teleop_twist_keyboard`. Alternatively, `rviz2` and the Navigation2 stacks allow the use of naviation goals on the map.

If you want to interact with the Kobuki base without bringing up the full Turtlebot2 stack, launch the Kobuki with `ros2 launch kobuki_node kobuki_node-launch.py` and run keyboard teleoperation with remapping for the velocity commands:
`ros2 run kobuki_keyop kobuki_keyop_node --ros-args --remap /cmd_vel:=/commands/velocity` or `ros2 run teleop_twist teleop_twist_keyboard --ros-args -r /cmd_vel:=/commands/velocity`.

 Note that for MacOS environment, you'll need to add the flag `--platform linux/amd64` for all docker commands to specify non-arm variant
