#!/bin/bash
# 
# unset AMENT_PREFIX_PATH
# unset CMAKE_PREFIX_PATH
# source /opt/ros/humble/setup.bash
apt update -y
rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/
cd /home/nvidia/INF3995-Robot/ros_ws/ 
rosdep install --from-paths /home/nvidia/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble

colcon build
source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py &

source /home/nvidia/agilex_ws/install/setup.bash
ros2 launch limo_bringup limo_start.launch.py &

source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
ros2 launch file_server_bringup robot_bringup.launch.py &
