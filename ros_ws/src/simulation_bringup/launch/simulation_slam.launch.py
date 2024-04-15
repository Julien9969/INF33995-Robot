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

from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Configure ROS nodes for launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local', default='true')

    # Setup project paths
    launch_dir = os.path.join(get_package_share_directory('simulation_bringup'), 'launch')

    return LaunchDescription([
        GroupAction( # pour remap le topic cmd_vel + odom + scan + imu dans nav
            actions=[
                # PushRosNamespace(f'robot{os.environ("ROBOT_NUM")}'),
                # SetRemap(src='/cmd_vel',dst='cmd_vel'),
                # SetRemap(src='/odom',dst='odom'),
                SetRemap(src='/scan',dst=f'/robot{os.environ("ROBOT_NUM")}/scan'),

                SetRemap(src='/map',dst=f'/robot{os.environ("ROBOT_NUM")}/map'),
                SetRemap(src='/map_metadata',dst=f'/robot{os.environ("ROBOT_NUM")}/map_metadata'),
                SetRemap(src='/slam_toolbox/feedback',dst=f'/robot{os.environ("ROBOT_NUM")}/slam_toolbox/feedback'),
                SetRemap(src='/slam_toolbox/graph_visualization',dst=f'/robot{os.environ("ROBOT_NUM")}/slam_toolbox/graph_visualization'),
                SetRemap(src='/slam_toolbox/scan_visualization',dst=f'/robot{os.environ("ROBOT_NUM")}/slam_toolbox/scan_visualization'),
                SetRemap(src='/slam_toolbox/update',dst=f'/robot{os.environ("ROBOT_NUM")}/slam_toolbox/update'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_dir, '/slam_toolbox_launch.py']),
                    launch_arguments={
                        # 'map': map_dir,
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local' : map_subscribe_transient_local,
                        'slam_params_file': os.path.join(get_package_share_directory("simulation_bringup"), #TODO:Changer le src pour un vrai setup
                                   'config',f'slam_toolbox_params_{os.environ("ROBOT_NUM")}.yaml'),
                    }.items(),
                ),
            ]
        ),




    ])