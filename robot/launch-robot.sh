#!/bin/bash

BRANCH="main"
ROBOTS=("192.168.0.102" "192.168.0.106")
USERNAME=nvidia
COMMAND="ros2 launch robot_bringup robot_bringup.launch.py"

function print_usage {
    echo "Usage: ./launch-robot.sh [-b|--branch <branch-name>] [-r|--robots <ip-addresses-of-robots-separated-by-space>] [-h|--help] [-c|--command <ros2-command-to-run>]"
    exit 1
}

while [[ $# -gt 0 ]]
do
    key="$1"
    case $key in
        -b|--branch)
            BRANCH="$2"
            shift
            shift
            ;;
        -r|--robots)
            ROBOTS=()
            shift
            # Parse until the next option or end of ip addresses
            while [[ $# -gt 0 && ! "$1" =~ ^- ]]; do
                ROBOTS+=("$1")
                shift
            done
            ;;
        -c|--command)
            COMMAND="$2"
            shift
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

NUM_ROBOTS=${#ROBOTS[@]}

for i in $(seq 1 $NUM_ROBOTS);
do
    ssh $USERNAME@${ROBOTS[$((i-1))]}  """
        if [ -z '\$(cat /home/nvidia/.bashrc | grep ROS_DOMAIN_ID)' ]; then
            echo "export ROS_DOMAIN_ID=62" >> /home/nvidia/.bashrc
        else
            sed -i 's/ROS_DOMAIN_ID=[0-9]*/ROS_DOMAIN_ID=62/g' /home/nvidia/.bashrc
        fi

        if [ -d /home/nvidia/INF3995-Robot ]; then
            cd /home/nvidia/INF3995-Robot && git pull && git switch $BRANCH && git pull;
        else
            git clone git@gitlab.com:polytechnique-montr-al/inf3995/20241/equipe-102/INF3995-Robot.git -b $BRANCH;
        fi

        cd /home/nvidia/INF3995-Robot/robot
        docker build -t docker-robot .
        docker run -d --rm --network=host --ipc=host --pid=host --device=/dev/ttyTHS1 --device=/dev/ydlidar -v /home/nvidia/INF3995-Robot:/root/INF3995-Robot -v /tmp/.X11-unix:/tmp/.X11-unix --env ROS_DOMAIN_ID=62 --env ROBOT_NUM=$i docker-robot bash -c '/root/clean_workspace.sh && source /opt/ros/humble/setup.bash && cd root && /root/deploy-robot.sh && source /root/INF3995-Robot/ros_ws/install/setup.bash && $COMMAND'
        """
done
