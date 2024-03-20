from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_exploration_server',
            executable='exploration',
            output='screen',
            namespace=f'robot{os.environ["ROBOT_NUM"]}',
        ),
    ])
