#!/bin/bash

BRANCH="main"
ROBOTS=("192.168.0.102" "192.168.0.106")
USERNAME=nvidia
REINSTALL="false"
COMMAND="ros2 launch robot_bringup robot_bringup.launch.py"
SCRIPT_PATH=$(dirname $0)

function print_usage {
    echo "Usage: ./launch-robot.sh [-b|--branch <branch-name>] [-r|--robots <ip-addresses-of-robots-separated-by-space>] [-h|--help] [-c|--command <ros2-command-to-run>] [--reinstall]"
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
        --reinstall)
            REINSTALL="true"
            shift
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
    if [ "$REINSTALL" == "true" ]; then
        ssh-copy-id -i $SCRIPT_PATH/ssh_keys/robot_ssh $USERNAME@${ROBOTS[$((i-1))]}
        scp -i $SCRIPT_PATH/ssh_keys/robot_ssh $SCRIPT_PATH/setup-robot.sh scp://$USERNAME@${ROBOTS[$((i-1))]}//home/nvidia/setup-robot.sh
    fi

    ssh -i $SCRIPT_PATH/ssh_keys/robot_ssh -t $USERNAME@${ROBOTS[$((i-1))]}  """
        if [ "$REINSTALL" == "true" ]; then
            chmod +x /home/nvidia/setup-robot.sh
            sudo /home/nvidia/setup-robot.sh
        fi

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

        export ROBOT_NUM=$i
        /home/nvidia/INF3995-Robot/robot/clean_workspace.sh && source /opt/ros/humble/setup.bash && /home/nvidia/INF3995-Robot/robot/deploy-robot.sh && source /home/nvidia/INF3995-Robot/ros_ws/install/setup.bash && $COMMAND
        """
        # docker build -t docker-robot .
        # docker run -d --rm -p 22900:22900 -p 22901:22901 -p 22902:22902 -p 22910-22921:22910-22921 --device=/dev/ttyTHS1 --device=/dev/ydlidar -v /home/nvidia/INF3995-Robot:/root/INF3995-Robot -v /tmp/.X11-unix:/tmp/.X11-unix --env ROS_DOMAIN_ID=62 --env ROBOT_NUM=$i docker-robot bash -c '/root/clean_workspace.sh && source /opt/ros/humble/setup.bash && cd root && /root/deploy-robot.sh && source /root/INF3995-Robot/ros_ws/install/setup.bash && $COMMAND'
done


# TODO : Ajouter param si on veut cloner tout from scratch.
# Si on veut tout pull, rerouler tout ce qui est dans le dockerfile

# TODO : Voir si sudo cause probleme