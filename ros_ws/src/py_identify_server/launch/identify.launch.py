from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_identify_server',
            executable='identify',
            output='screen',
        ),
    ])
