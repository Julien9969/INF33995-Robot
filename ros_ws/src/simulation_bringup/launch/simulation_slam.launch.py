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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local', default='true')
    log_level = LaunchConfiguration('log_level', default='error')

    # Setup project paths
    launch_dir = os.path.join(get_package_share_directory('simulation_bringup'), 'launch')

    return LaunchDescription([
        GroupAction( # pour remap le topic cmd_vel + odom + scan + imu dans nav
            actions=[
                PushRosNamespace(namespace='robot1'),
                SetRemap(src='/cmd_vel',dst='cmd_vel'),
                SetRemap(src='/odom',dst='odom'),
                SetRemap(src='/scan',dst='scan'),
                SetRemap(src='/imu',dst='imu'),
                # SetRemap(src='tf',dst='/tf'),
                # SetRemap(src='tf_static',dst='/tf_static'),

                # SetRemap(src='/scan',dst='/robot1/scan'),

                # TODO 2: mb try pose --> /pose or smthng, OR remap everything manually
                # SetRemap(src='/clock',dst='/robot1/clock'),
                # SetRemap(src='/parameter_events',dst='/robot1/parameter_events'),
                SetRemap(src='/map',dst='/robot1/map'),
                SetRemap(src='/map_metadata',dst='/robot1/map_metadata'),
                SetRemap(src='/slam_toolbox/feedback',dst='/robot1/slam_toolbox/feedback'),
                SetRemap(src='/slam_toolbox/graph_visualization',dst='/robot1/slam_toolbox/graph_visualization'),
                SetRemap(src='/slam_toolbox/scan_visualization',dst='/robot1/slam_toolbox/scan_visualization'),
                SetRemap(src='/slam_toolbox/update',dst='/robot1/slam_toolbox/update'),

                # TODO: test if robot actually moves / why not?? (is there a robot1/... somewhere?)
                # SetRemap(src='/cmd_vel',dst=f'/robot1/cmd_vel'),
                # SetRemap(src='cmd_vel',dst=f'/robot1/cmd_vel'),
                # SetRemap(src='/odom',dst=f'/robot1/odom'),
                # SetRemap(src='odom',dst=f'/robot1/odom'),
                # SetRemap(src='/scan',dst=f'/robot1/scan'),
                # SetRemap(src='scan',dst=f'/robot1/scan'),
                # SetRemap(src='/imu',dst=f'/robot1/imu'),
                # SetRemap(src='imu',dst=f'/robot1/imu'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_dir, '/navigation_launch.py']),
                    launch_arguments={
                        # 'map': map_dir,
                        'namespace':'robot1',
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local' : map_subscribe_transient_local,
                        'params_file': os.path.join(get_package_share_directory('simulation_bringup'), 'config','nav2_params_1.yaml'),
                        'log_level': log_level,
                    }.items(),
                ),
            ]
        ),
        GroupAction( # pour remap le topic cmd_vel + odom + scan + imu dans nav
            actions=[
                # PushRosNamespace('robot1'),
                # SetRemap(src='/cmd_vel',dst='cmd_vel'),
                # SetRemap(src='/odom',dst='odom'),
                SetRemap(src='/scan',dst='/robot1/scan'),

                # TODO 2: mb try pose --> /pose or smthng, OR remap everything manually
                # SetRemap(src='/clock',dst='/robot1/clock'),
                # SetRemap(src='/parameter_events',dst='/robot1/parameter_events'),
                SetRemap(src='/map',dst='/robot1/map'),
                SetRemap(src='/map_metadata',dst='/robot1/map_metadata'),
                SetRemap(src='/slam_toolbox/feedback',dst='/robot1/slam_toolbox/feedback'),
                SetRemap(src='/slam_toolbox/graph_visualization',dst='/robot1/slam_toolbox/graph_visualization'),
                SetRemap(src='/slam_toolbox/scan_visualization',dst='/robot1/slam_toolbox/scan_visualization'),
                SetRemap(src='/slam_toolbox/update',dst='/robot1/slam_toolbox/update'),




                # SetRemap(src='/imu',dst='imu'),
                # SetRemap(src='tf',dst='/tf'),
                # SetRemap(src='tf_static',dst='/tf_static'),

                # TODO: test if robot actually moves / why not?? (is there a robot1/... somewhere?)
                # SetRemap(src='/cmd_vel',dst=f'/robot1/cmd_vel'),
                # SetRemap(src='cmd_vel',dst=f'/robot1/cmd_vel'),
                # SetRemap(src='/odom',dst=f'/robot1/odom'),
                # SetRemap(src='odom',dst=f'/robot1/odom'),
                # SetRemap(src='/scan',dst=f'/robot1/scan'),
                # SetRemap(src='scan',dst=f'/robot1/scan'),
                # SetRemap(src='/imu',dst=f'/robot1/imu'),
                # SetRemap(src='imu',dst=f'/robot1/imu'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_dir, '/slam_toolbox_launch.py']),
                    launch_arguments={
                        # 'map': map_dir,
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local' : map_subscribe_transient_local,
                        'slam_params_file': os.path.join(get_package_share_directory("simulation_bringup"), #TODO:Changer le src pour un vrai setup
                                   'config','slam_toolbox_params_1.yaml'),
                        'log_level': log_level,
                    }.items(),
                ),
            ]
        ),

        GroupAction( # pour remap le topic cmd_vel + odom + scan + imu dans nav
            actions=[
                PushRosNamespace(namespace='robot2'),
                SetRemap(src='/cmd_vel',dst='cmd_vel'),
                SetRemap(src='/odom',dst='odom'),
                SetRemap(src='/scan',dst='scan'),
                SetRemap(src='/imu',dst='imu'),
                # SetRemap(src='tf',dst='/tf'),
                # SetRemap(src='tf_static',dst='/tf_static'),

                # SetRemap(src='/scan',dst='/robot2/scan'),

                # TODO 2: mb try pose --> /pose or smthng, OR remap everything manually
                # SetRemap(src='/clock',dst='/robot2/clock'),
                # SetRemap(src='/parameter_events',dst='/robot2/parameter_events'),
                SetRemap(src='/map',dst='/robot2/map'),
                SetRemap(src='/map_metadata',dst='/robot2/map_metadata'),
                SetRemap(src='/slam_toolbox/feedback',dst='/robot2/slam_toolbox/feedback'),
                SetRemap(src='/slam_toolbox/graph_visualization',dst='/robot2/slam_toolbox/graph_visualization'),
                SetRemap(src='/slam_toolbox/scan_visualization',dst='/robot2/slam_toolbox/scan_visualization'),
                SetRemap(src='/slam_toolbox/update',dst='/robot2/slam_toolbox/update'),

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
                    PythonLaunchDescriptionSource([launch_dir, '/navigation_launch.py']),
                    launch_arguments={
                        # 'map': map_dir,
                        'namespace':'robot2',
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local' : map_subscribe_transient_local,
                        'params_file': os.path.join(get_package_share_directory('simulation_bringup'), 'config','nav2_params_2.yaml'),
                        'log_level': log_level,
                    }.items(),
                ),
            ]
        ),
        GroupAction( # pour remap le topic cmd_vel + odom + scan + imu dans nav
            actions=[
                # PushRosNamespace('robot2'),
                # SetRemap(src='/cmd_vel',dst='cmd_vel'),
                # SetRemap(src='/odom',dst='odom'),
                SetRemap(src='/scan',dst='/robot2/scan'),

                # TODO 2: mb try pose --> /pose or smthng, OR remap everything manually
                # SetRemap(src='/clock',dst='/robot2/clock'),
                # SetRemap(src='/parameter_events',dst='/robot2/parameter_events'),
                SetRemap(src='/map',dst='/robot2/map'),
                SetRemap(src='/map_metadata',dst='/robot2/map_metadata'),
                SetRemap(src='/slam_toolbox/feedback',dst='/robot2/slam_toolbox/feedback'),
                SetRemap(src='/slam_toolbox/graph_visualization',dst='/robot2/slam_toolbox/graph_visualization'),
                SetRemap(src='/slam_toolbox/scan_visualization',dst='/robot2/slam_toolbox/scan_visualization'),
                SetRemap(src='/slam_toolbox/update',dst='/robot2/slam_toolbox/update'),




                # SetRemap(src='/imu',dst='imu'),
                # SetRemap(src='tf',dst='/tf'),
                # SetRemap(src='tf_static',dst='/tf_static'),

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
                        'slam_params_file': os.path.join(get_package_share_directory("simulation_bringup"), #TODO:Changer le src pour un vrai setup
                                   'config','slam_toolbox_params_2.yaml'),
                        'log_level': log_level,
                    }.items(),
                ),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/simulation.launch.py']),
            launch_arguments={
                'log_level': log_level,
            }.items(),
        ),
    ])