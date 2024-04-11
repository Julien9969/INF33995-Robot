#! /usr/bin/env python3

import signal
import sys
import argparse
import os
import random
import subprocess
import sys
import time, rclpy
from nav_to_pos import navigateToPos, square_nav, set_end_nav

RANGE_VALUES_POS = 2
VALUE_TO_SIMULATE_RANGE = RANGE_VALUES_POS/2
NOT_IMPORTANT_VALUE = 0.0
PROTECTED_ZONE = 0.75

sys.path.append('.')


def navigateToRandomLocation(name_space):
    new_x_goal = 0
    new_y_goal = 0
    while new_x_goal > -PROTECTED_ZONE and new_x_goal < PROTECTED_ZONE:
        new_x_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    while new_y_goal > -PROTECTED_ZONE and new_y_goal < PROTECTED_ZONE:
        new_y_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    new_w_goal = NOT_IMPORTANT_VALUE

    # print(new_x_goal, new_y_goal, new_w_goal)
    # return navigateToPos([new_x_goal, new_y_goal, new_w_goal], name_space)

    pos = [(4, 4), (4, -4), (-4, -4), (-4, 4)]
    while True:
        for p in pos:
            navigateToPos([p[0], p[1], 0], name_space)
        
    return 
    return go_to_poses()


def main(name_space):
    rclpy.init()

    square_nav(name_space)
    # while True:
    #     navigateToRandomLocation(name_space)
    #     time.sleep(5)
    rclpy.shutdown()


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    # navigator.cancelTask()
    # set_end_nav()
    sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name_space", type=str, help="Namespace for the robot", required=True)
    args = parser.parse_args()
    # navProcess = subprocess.Popen(['/bin/bash', 'cd', '../../../', '&&', 'source', 'install/setup.bash', '&&', 'ros2', 'launch', 'explore_lite', 'explore.launch.py'])
    # navProcess = subprocess.Popen(['/bin/bash', '-c', 'cd ../../../ && source install/setup.bash && ros2 launch explore_lite explore.launch.py'])
    print(args.name_space)
    signal.signal(signal.SIGINT, signal_handler)
    main(args.name_space)

    # main(args.name_space)