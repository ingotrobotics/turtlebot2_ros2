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

import os
import launch
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
import launch_ros

def generate_launch_description():

  robot_localization_node = launch_ros.actions.Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[
          PathJoinSubstitution([ FindPackageShare('turtlebot2_bringup'), 'config', 'ekf.yaml' ]),
          {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )
  
  return launch.LaunchDescription([
      launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                           description='Flag to enable use_sim_time'),
      robot_localization_node
  ])
