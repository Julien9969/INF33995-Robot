#!/bin/bash

pkill ros2
pkill gazebo

apt update -y
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
rosdep install --from-paths /root/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
rosdep install --from-paths /root/INF3995-Robot/file_transfer_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 5
cd /root/INF3995-Robot/file_transfer_ws
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.sh
ros2 launch files_server_simulation simulation.launch.py &

sleep 1
cd /root/INF3995-Robot/ros_ws
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.sh

ros2 launch simulation_bringup simulation.launch.py
