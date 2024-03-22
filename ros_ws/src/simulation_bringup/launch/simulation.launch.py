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
    pkg_project_bringup = get_package_share_directory('simulation_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # # Load the SDF files from "description" package
    # sdf_file_1 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive_1', 'model.sdf')
    # with open(sdf_file_1, 'r') as infp:
    #     robot_desc_1 = infp.read()

    # Load the SDF files from "description" package
    sdf_file_2 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive_2', 'model.sdf')
    with open(sdf_file_2, 'r') as infp:
        robot_desc_2 = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    # robot_state_publisher_1 = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher_1',
    #     output='both',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc_1},
    #     ]
    # )
    robot_state_publisher_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_2',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc_2},
        ]
    )

    # Visualize in RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'limo_diff_drive.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'simulation_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # # Node pour identification:
    #     # Bridge ROS topics and Gazebo messages for establishing communication
    # identify1 = Node(
    #     package='py_identify_server',
    #     executable='identify',
    #     # parameters=[{
    #     #     'config_file': os.path.join(pkg_project_bringup, 'config', 'simulation_bridge.yaml'),
    #     #     'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     # }],
    #     namespace='robot1',
    #     output='screen'
    # )
    
    # Node pour identification:
        # Bridge ROS topics and Gazebo messages for establishing communication
    # identify2 = Node(
    #     package='py_identify_server',
    #     executable='identify',
    #     # parameters=[{
    #     #     'config_file': os.path.join(pkg_project_bringup, 'config', 'simulation_bridge.yaml'),
    #     #     'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     # }],
    #     namespace='robot2',
    #     output='screen'
    # )
    # # Node pour mission control (start et stop mission):
    # mission_switch1 = Node(
    #     package='mission_control',
    #     executable='mission_switch',
    #     output='screen',
    #     namespace='robot1',
    # )
        # Node pour mission control (start et stop mission):
    mission_switch2 = Node(
        package='mission_control',
        executable='mission_switch',
        output='screen',
        namespace='robot2',
    )

    # explore_2 = Node(
    #     package='py_exploration_server',
    #     executable='explore',
    #     output='screen',
    #     # namespace='robot2',
    # )

    # robot_localization_node = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[os.path.join(pkg_project_bringup, 'config', 'ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    return LaunchDescription([
        # DeclareLaunchArgument(name='use_sim_time', default_value='True',
        #                                     description='Flag to enable use_sim_time'),
        # robot_localization_node,
        gz_sim,
        # DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        # robot_state_publisher_1,
        robot_state_publisher_2,
        # rviz,
        # mission_switch1,
        mission_switch2,
        # explore_2,
        # identify1,
        # identify2
    ])