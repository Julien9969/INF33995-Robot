#!/bin/bash

# TODO : Enlever la fonctionnalité de relancer quand ça casse

function log {
    if ! [ -d /home/nvidia/INF3995-Robot/robot/log ]; then
        mkdir /home/nvidia/INF3995-Robot/robot/log
    fi
    echo $(date)" | $1" >> /home/nvidia/INF3995-Robot/robot/log/rebuild-robot.log
}

function revert_changes {
    log "Reverting changes"
    sudo rm -rf /home/nvidia/INF3995-Robot/ros_ws
    cd /home/nvidia/INF3995-Robot
    unzip ros_ws.backup.zip
    rm /home/nvidia/INF3995-Robot/ros_ws.backup.zip
    source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
    ros2 launch robot_bringup robot_bringup.launch.py
}

if [ -f /home/nvidia/INF3995-Robot/robot/update_queue/update.txt ]; then
    NEW_FILE_LOCATION=/home/nvidia/INF3995-Robot/robot/update_queue/$(cat /home/nvidia/INF3995-Robot/robot/update_queue/update.txt | awk '{print $1}')
    NEW_FILE_DESTINATION=/home/nvidia/INF3995-Robot/ros_ws/src/$(cat /home/nvidia/INF3995-Robot/robot/update_queue/update.txt | awk '{print $2}')
    rm /home/nvidia/INF3995-Robot/robot/update_queue/update.txt
else
    log "ERROR: Update queue is empty"
    exit 1
fi

cd /home/nvidia/INF3995-Robot
zip ros_ws.backup.zip -r ros_ws
mv $NEW_FILE_LOCATION $NEW_FILE_DESTINATION
cd /home/nvidia/INF3995-Robot/ros_ws/ && rm -rf /home/nvidia/INF3995-Robot/ros_ws/build/ /home/nvidia/INF3995-Robot/ros_ws/install/ /home/nvidia/INF3995-Robot/ros_ws/log/
colcon build
if [ $? -ne 0 ]; then
    log "ERROR: Build failed"
    revert_changes
    exit 2
fi

source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py

if [ $? -ne 0 ]; then
    log "ERROR: Build failed"
    revert_changes
    exit 3
fi
