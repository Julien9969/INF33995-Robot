#!/bin/bash


echo $ROBOT_ENV
if [ "$ROBOT_ENV" = "SIMULATION" ]; then
    bash /root/deploy-simulation.sh
    echo "SIMULATION REBUILD DONE"
else
    pkill ros2
    cd /home/nvidia/INF3995-Robot/ros_ws/ && rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/
    colcon build
    source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
    ros2 launch robot_bringup robot_bringup.launch.py &

    source /home/nvidia/agilex/install/setup.bash
    ros2 launch limo_bringup limo_start.launch.py &

    source /home/nvidia/INF3995-Robot/file_transfer_ws/install/setup.bash
    ros2 launch file_server robot_bringup.launch.py &

    echo "ROBOT REBUILD DONE"
fi