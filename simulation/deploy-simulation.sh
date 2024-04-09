#!/bin/bash

apt update -y
mkdir -p "/usr/local/include/" && cp /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/ /usr/local/include/ -r
apt update
rm -r /root/INF3995-Robot/simulation/.venv
apt install python3.10-venv -y
cd /root/INF3995-Robot/simulation && python3 -m venv .venv
source /root/INF3995-Robot/simulation/.venv/bin/activate && pip install python-sdformat catkin_pkg empy==3.3.4 && pip uninstall em && pip3 install lark --prefer-binary && python3 /root/INF3995-Robot/simulation/random_map_generator.py

cd /root/INF3995-Robot/ros_ws
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
rosdep install --from-paths /root/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
rosdep install --from-paths /root/INF3995-Robot/file_transfer_ws/src --ignore-src -r -i -y --rosdistro humble

sleep 1
mkdir -p "/usr/local/include/" && cp /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/ /usr/local/include/ -r
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.sh
ros2 launch simulation_bringup simulation_slam.launch.py

# sleep 5
# cd /root/INF3995-Robot/file_transfer_ws
# colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
# source install/setup.sh
# ros2 launch files_server_simulation simulation.launch.py &