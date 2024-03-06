#!/bin/bash

pkill ros2
apt update -y

echo "export ROBOT_ENV='ROBOT'" >> /home/nvidia/.bashrc

rm -rf /home/nvidia/INF3995-Robot/file_transfer_ws/build/ /home/nvidia/INF3995-Robot/file_transfer_ws/install/ /home/nvidia/INF3995-Robot/file_transfer_ws/log/
rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/


# rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update

rosdep install --from-paths /home/nvidia/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
rosdep install --from-paths /home/nvidia/INF3995-Robot/file_transfer_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 5

cd /home/nvidia/INF3995-Robot/ros_ws
# colcon build
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py &

source /home/nvidia/agilex_ws/install/setup.bash
ros2 launch limo_bringup limo_start.launch.py &

cd /home/nvidia/INF3995-Robot/file_transfer_ws 
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
ros2 launch file_server_bringup robot_bringup.launch.py &

sleep 1

export ROBOT_ENV="ROBOT"