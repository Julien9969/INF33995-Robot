# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths

    # Node pour identification:
        # Bridge ROS topics and Gazebo messages for establishing communication
    identify = Node(
        package='py_identify_server',
        executable='identify',
        output='screen',
        namespace=f'robot{os.environ["ROBOT_NUM"]}',
    )
    # Node pour mission control (start et stop mission):
    # mission_switch = Node(
    #     package='mission_control',
    #     executable='mission_switch',
    #     output='screen',
    #     # namespace=f'robot{os.environ["ROBOT_NUM"]}',
    # )
    mission_switch = Node(
        package='limo_info',
        executable='publisher',
        output='screen',
        namespace=f'robot{os.environ["ROBOT_NUM"]}',
    )
    # info_publisher = Node(
    #     package='limo_info',
    #     executable='publisher',
    #     output='screen',
    #     namespace=f'robot{os.environ["ROBOT_NUM"]}',
    # )




    return LaunchDescription([
        identify,
        mission_switch,
        # info_publisher
    ])
