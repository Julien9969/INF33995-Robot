cd /root/INF3995-Robot/ros_ws
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
rosdep install --from-paths /home/nvidia/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 5
colcon build
source install/setup.bash
