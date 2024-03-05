from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            output='screen',
            namespace=f'robot{os.environ["ROBOT_NUM"]}',
        ),
        Node(
            package='py_navigate_server',
            executable='navigate',
            output='screen',
            namespace=f'robot{os.environ["ROBOT_NUM"]}',
        ),
    ])
