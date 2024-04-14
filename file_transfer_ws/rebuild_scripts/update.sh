#!/bin/bash

# ps aux | grep -v 'bash' | awk '{print $2}' | xargs -r kill
sleep 5
echo $ROBOT_ENV

# processes=("smoother_server" "ros2" "files_server" "static_transfor" "ydlidar_ros2_dr" "limo_base" "mission_switch" "identify" "parameter_bridg" "robot_state_pub" "cartographer_oc" "publisher" "static_transfor")
processes=("smoother_server" "ros2" "files_server" "static_transfor" "ydlidar_ros2_dr" "limo_base" "mission_switch" "identify" "parameter_bridg" "robot_state_pub" "cartographer_oc" "publisher" "static_transfor" "controller_serv" "planner_server" "behavior_server" "bt_navigator" "waypoint_follow" "velocity_smooth" "lifecycle_manag" "async_slam_tool")
for process in "${processes[@]}"; do
    pkill $process
done

apt update -y
rosdep init
rosdep update

if [ "$ROBOT_ENV" = "SIMULATION" ]; then
    pkill ruby
    bash /root/deploy-simulation.sh
    echo "SIMULATION REBUILD DONE"
else
    # bash /home/nvidia/INF3995-Robot/robot/rebuild-robot.sh
    bash /home/nvidia/INF3995-Robot/robot/deploy-robot.sh
    echo "ROBOT REBUILD DONE"
fi

 