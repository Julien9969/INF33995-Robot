#!/bin/bash

function log {
    if [ -d /home/nvidia/INF3995-Robot/robot/log ]; then
        mkdir /home/nvidia/INF3995-Robot/robot/log
    fi
    echo $(date)" | $1" >> /home/nvidia/INF3995-Robot/robot/log/rebuild-robot.log
}

function revert_changes {
    log "Reverting changes"
    rm -rf /home/nvidia/INF3995-Robot/ros_ws
    unzip /home/nvidia/INF3995-Robot/ros_ws.backup.zip -d /home/nvidia/INF3995-Robot/
    rm /home/nvidia/INF3995-Robot/ros_ws.backup.zip
    source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash
    ros2 launch robot_bringup robot_bringup.launch.py
}

if [ -f /home/nvidia/INF3995-Robot/robot/update_queue/update.txt ]; then
    NEW_FILE_LOCATION=/home/nvidia/INF3995-Robot/robot/update_queue/$(cat /home/nvidia/INF3995-Robot/update_queue/update.txt | awk '{print $1}')
    NEW_FILE_DESTINATION=/home/nvidia/INF3995-Robot/ros_ws/src/$(cat /home/nvidia/INF3995-Robot/update_queue/update.txt | awk '{print $2}')
    rm /home/nvidia/INF3995-Robot/robot/update_queue/update.txt
else
    log "ERROR: Update queue is empty"
    exit 1
fi

zip /home/nvidia/INF3995-Robot/ros_ws.backup.zip -r /home/nvidia/INF3995-Robot/ros_ws
mv $NEW_FILE_LOCATION $NEW_FILE_DESTINATION
cd /home/nvidia/INF3995-Robot/ros_ws/ && rm -r build/ install/ log/
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
