#!/bin/bash

processes=("ros2" "files_server" "static_transfor" "ydlidar_ros2_dr" "limo_base" "mission_switch" "identify" "parameter_bridg" "robot_state_pub" "cartographer_oc" "publisher" "static_transfor" "component_conta" "cartographer_oc")
for process in "${processes[@]}"; do
    killall -9 $process
done

# apt update -y

echo "export ROBOT_ENV='ROBOT'" >> /home/nvidia/.bashrc

# rm -rf /home/nvidia/INF3995-Robot/file_transfer_ws/build/ /home/nvidia/INF3995-Robot/file_transfer_ws/install/ /home/nvidia/INF3995-Robot/file_transfer_ws/log/
# rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/

# rm /etc/ros/rosdep/sources.list.d/20-default.list
# rosdep init
# rosdep update

# rosdep install --from-paths /home/nvidia/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
# rosdep install --from-paths /home/nvidia/INF3995-Robot/file_transfer_ws/src --ignore-src -r -i -y --rosdistro humble
sleep 3

cd /home/nvidia/INF3995-Robot/ros_ws
colcon build
source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py &

sleep 3

cd /home/nvidia/INF3995-Robot/file_transfer_ws 
colcon build
source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
ros2 launch file_server_bringup robot_bringup.launch.py &

sleep 4
# TODO log arguments see agilex repo start-robot.sh
cd ~/agilex_ws 
source install/setup.sh
./start-robot.sh error &

export ROBOT_ENV="ROBOT"
