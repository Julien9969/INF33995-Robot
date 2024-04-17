import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True
    slam_params_file = LaunchConfiguration('slam_params_file')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("simulation_bringup"), #TODO:Changer le src pour un vrai setup
                                   'config','slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_log_level_argument = DeclareLaunchArgument(
        'log_level',
        default_value='error',
        description='Log level')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_log_level_argument)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
