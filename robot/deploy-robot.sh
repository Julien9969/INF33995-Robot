#!/bin/bash

processes=("ros2" "files_server" "static_transfor" "ydlidar_ros2_dr" "limo_base" "mission_switch" "identify" "parameter_bridg" "robot_state_pub" "cartographer_oc" "publisher" "static_transfor" "component_conta" "cartographer_no" "python3")
for process in "${processes[@]}"; do
    killall -9 $process
done

echo "export ROBOT_ENV='ROBOT'" >> /home/nvidia/.bashrc

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

ros2 launch limo_bringup limo_start.launch.py pub_odom_tf:=true &
ros2 launch limo_bringup cartographer.launch.py log_level:=error &
ros2 launch limo_bringup nav_bringup_launch.py log_level:=error &

export ROBOT_ENV="ROBOT"
