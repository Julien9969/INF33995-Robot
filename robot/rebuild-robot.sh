#!/bin/bash

killall ros2
cd /home/nvidia/INF3995-Robot/ros_ws/ && rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/
colcon build
source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py

source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
ros2 launch file_server robot_bringup.launch.py