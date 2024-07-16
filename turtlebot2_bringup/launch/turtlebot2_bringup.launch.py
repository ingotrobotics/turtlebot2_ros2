# Copyright 2023 Ingot Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import launch_ros

def generate_launch_description():

    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('urg_node'),
                'launch',
                'urg_node_launch.py'
            ])
        ]),
        launch_arguments={
            'sensor_interface': 'serial'
        }.items()
    )        

    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot2_description'),
                'launch',
                'turtlebot2_description.launch.py'
            ])
        ])
    )

    kobuki_node = GroupAction(
        actions=[
            launch_ros.actions.SetRemap( src='/commands/velocity', dst='/cmd_vel'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('kobuki_node'),
                        'launch',
                        'kobuki_node-launch.py'
                    ])
                ])
            )
        ]
    )

    robot_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot2_bringup'),
                'launch',
                'ekf_node.launch.py'
            ])
        ])
    )

    slam_node = GroupAction(
        actions=[
            launch_ros.actions.SetRemap( src='/scan', dst='/scan_filtered'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ])
            )
        ]
    )

    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )

    laser_filter = LaunchDescription()
    laser_filter_node = launch_ros.actions.Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('turtlebot2_bringup'),
                'config',
                'turtlebot2_laser_scan_filter.yaml'
            ])],
        )
    laser_filter.add_action( laser_filter_node )
    
    return LaunchDescription([
        robot_model,
        laser_node,
        laser_filter,
        kobuki_node,
        #robot_localization_node
        slam_node,
        nav_node
    ])
