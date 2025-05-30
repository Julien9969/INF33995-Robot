#!/bin/bash

# ps aux | grep -v 'bash' | awk '{print $2}' | xargs -r kill
sleep 5
echo $ROBOT_ENV

processes=("ros2" "files_server" "static_transfor" "ydlidar_ros2_dr" "limo_base" "mission_switch" "identify" "parameter_bridg" "robot_state_pub" "cartographer_oc" "publisher" "static_transfor" "component_conta" "cartographer_oc" "python3" "cartographer_no")

for process in "${processes[@]}"; do
    killall -9 $process
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

 