#! /usr/bin/env python3

import argparse
import os
import random
import subprocess
import sys
import time, rclpy
from nav_to_pos import navigateToPos, go_to_poses

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
    return navigateToPos([10, 0, new_w_goal], name_space)
    return go_to_poses()


def main(name_space):
    rclpy.init()
    while True:
        navigateToRandomLocation(name_space)
        time.sleep(5)
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name_space", type=str, help="Namespace for the robot", required=True)
    args = parser.parse_args()
    # navProcess = subprocess.Popen(['/bin/bash', 'cd', '../../../', '&&', 'source', 'install/setup.bash', '&&', 'ros2', 'launch', 'explore_lite', 'explore.launch.py'])
    # navProcess = subprocess.Popen(['/bin/bash', '-c', 'cd ../../../ && source install/setup.bash && ros2 launch explore_lite explore.launch.py'])

    main(args.name_space)
    while True:
        pass
    # main(args.name_space)