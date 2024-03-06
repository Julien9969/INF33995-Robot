#!/bin/bash

# ps aux | grep -v 'bash' | awk '{print $2}' | xargs -r kill
sleep 5
echo $ROBOT_ENV

processes=("ros2" "files_server" "static_transfor" "ydlidar_ros2_dr" "limo_base" "mission_switch" "identify" "parameter_bridg" "robot_state_pub")

for process in "${processes[@]}"; do
    pkill $process
done


if [ "$ROBOT_ENV" = "SIMULATION" ]; then
    pkill ruby
    bash /root/deploy-simulation.sh
    echo "SIMULATION REBUILD DONE"
else

    cd /home/nvidia/INF3995-Robot/ros_ws/ 
    rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/
    rosdep install --from-paths /home/nvidia/INF3995-Robot/ros_ws/src --ignore-src -r -i -y --rosdistro humble
    colcon build
    source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
    ros2 launch robot_bringup robot_bringup.launch.py &

    source /home/nvidia/agilex_ws/install/setup.bash
    ros2 launch limo_bringup limo_start.launch.py &

    source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
    ros2 launch file_server_bringup robot_bringup.launch.py &

    echo "ROBOT REBUILD DONE"
fi