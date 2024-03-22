#! /usr/bin/env python3

import os
import random
import sys
import time, rclpy
from nav_to_pos import navigateToPos, go_to_poses

RANGE_VALUES_POS = 5
VALUE_TO_SIMULATE_RANGE = RANGE_VALUES_POS/2
NOT_IMPORTANT_VALUE = 0.0
PROTECTED_ZONE = 0.75

sys.path.append('.')

def navigateToRandomLocation():
    new_x_goal = 0
    new_y_goal = 0
    while new_x_goal > -PROTECTED_ZONE and new_x_goal < PROTECTED_ZONE:
        new_x_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    while new_y_goal > -PROTECTED_ZONE and new_y_goal < PROTECTED_ZONE:
        new_y_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    new_w_goal = NOT_IMPORTANT_VALUE

    # print(new_x_goal, new_y_goal, new_w_goal)
    return navigateToPos([new_x_goal, new_y_goal, new_w_goal])
    return go_to_poses()


if __name__ == "__main__":
    rclpy.init()
    while True:
        navigateToRandomLocation()
        time.sleep(5)
    rclpy.shutdown()