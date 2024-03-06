#!/bin/bash

ps aux | awk '/files_server|parameter_bridg|robot_state_pub|mission_switch|identify/ {print $2}' | xargs -I {} pkill {}
# ps aux | grep -v 'bash' | awk '{print $2}' | xargs -r kill

echo $ROBOT_ENV
if [ "$ROBOT_ENV" = "SIMULATION" ]; then
    pkill ruby
    bash /root/deploy-simulation.sh
    echo "SIMULATION REBUILD DONE"
else
    ps aux | awk '/static_transfor|ydlidar_ros2_dr|limo_base/ {print $2}' | xargs -I {} pkill {}
    cd /home/nvidia/INF3995-Robot/ros_ws/ && rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/
    colcon build
    source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
    ros2 launch robot_bringup robot_bringup.launch.py &

    source /home/nvidia/agilex_ws/install/setup.bash
    ros2 launch limo_bringup limo_start.launch.py &

    source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
    ros2 launch file_server robot_bringup.launch.py &

    echo "ROBOT REBUILD DONE"
fi