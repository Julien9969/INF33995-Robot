from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_navigation_server',
            executable='navigation',
            output='screen',
            namespace=f'robot{os.environ["ROBOT_NUM"]}',
        ),
    ])
