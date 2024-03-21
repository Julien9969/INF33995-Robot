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
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    # Configure ROS nodes for launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local', default='true')

    # Setup project paths
    launch_dir = os.path.join(get_package_share_directory('simulation_bringup'), 'launch')

    return LaunchDescription([
        GroupAction( # pour remap le topic cmd_vel + odom + scan + imu dans nav
            actions=[
                # TODO: test if robot actually moves / why not?? (is there a robot2/... somewhere?)
                # SetRemap(src='/cmd_vel',dst=f'/robot2/cmd_vel'),
                # SetRemap(src='cmd_vel',dst=f'/robot2/cmd_vel'),
                # SetRemap(src='/odom',dst=f'/robot2/odom'),
                # SetRemap(src='odom',dst=f'/robot2/odom'),
                # SetRemap(src='/scan',dst=f'/robot2/scan'),
                # SetRemap(src='scan',dst=f'/robot2/scan'),
                # SetRemap(src='/imu',dst=f'/robot2/imu'),
                # SetRemap(src='imu',dst=f'/robot2/imu'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_dir, '/slam_toolbox_launch.py']),
                    launch_arguments={
                        # 'map': map_dir,
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local' : map_subscribe_transient_local,
                    }.items(),
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_dir, '/navigation_launch.py']),
                    launch_arguments={
                        # 'map': map_dir,
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local' : map_subscribe_transient_local,
                    }.items(),
                ),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/simulation.launch.py']),
            launch_arguments={
            }.items(),
        ),
    ])