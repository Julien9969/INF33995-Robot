cd INF3995-Robot/file_transfer_ws
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
rosdep install --from-paths /root/INF3995-Robot/file_transfer_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 5
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.sh
ros2 launch robot_bringup robot_bringup.launch.py