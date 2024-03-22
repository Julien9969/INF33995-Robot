#! /usr/bin/env python3

import os
import random
import sys
import time, rclpy
from nav_to_pos import navigateToPos

RANGE_VALUES_POS = 3.4
VALUE_TO_SIMULATE_RANGE = RANGE_VALUES_POS/2
NOT_IMPORTANT_VALUE = 0.0
from rclpy.node import Node

sys.path.append('.')

def navigateToRandomLocation():
    # Node.get_logger().info('Navigating to random location')
    new_x_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    new_y_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    new_w_goal = NOT_IMPORTANT_VALUE

    print(new_x_goal, new_y_goal, new_w_goal)
    print(os.listdir())
    return navigateToPos([new_x_goal, new_y_goal, new_w_goal])


if __name__ == "__main__":
    rclpy.init()
    while True:
        navigateToRandomLocation()
        time.sleep(5)
    rclpy.shutdown()