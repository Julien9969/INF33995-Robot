apt update -y
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
cd /root/INF3995-Robot/file_transfer_ws
rosdep install --from-paths /root/INF3995-Robot/file_transfer_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 5
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source /root/INF3995-Robot/file_transfer_ws/install/setup.sh
ros2 launch files_server_simulation simulation.launch.py