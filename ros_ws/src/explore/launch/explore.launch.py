import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    config1 = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params_1.yaml"
    )

    config2 = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params_2.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    # namespace = LaunchConfiguration("namespace")
    namespace = ""

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    # declare_namespace_argument = DeclareLaunchArgument(
    #     "namespace",
    #     default_value="",
    #     description="Namespace for the explore node",
    # )

    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "/tf"), ("/tf_static", "/tf_static")]

    node1 = Node(
        package="explore_lite",
        name="explore_node",
        namespace='robot1',
        executable="explore",
        parameters=[config1, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
        # arguments=['--ros-args', '--log-level', 'DEBUG' ]
    )

    node2 = Node(
        package="explore_lite",
        name="explore_node",
        namespace='robot2',
        executable="explore",
        parameters=[config2, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
        # arguments=['--ros-args', '--log-level', 'DEBUG' ]
    )

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(node1)
    ld.add_action(node2)
    return ld
