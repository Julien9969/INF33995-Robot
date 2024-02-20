apt update
cd INF3995-Robot/ros_ws
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
rosdep install --from-paths /root/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 5
mkdir -p "/usr/local/include/" && cp /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/ /usr/local/include/ -r
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.sh
ros2 launch simulation_bringup simulation.launch.py
