#! /usr/bin/env python3

import random
from nav_to_pos import navigateToPos

RANGE_VALUES_POS = 3.4
VALUE_TO_SIMULATE_RANGE = RANGE_VALUES_POS/2
NOT_IMPORTANT_VALUE = 0.0

def navigateToRandomLocation():
    new_x_goal, new_y_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    new_w_goal = NOT_IMPORTANT_VALUE

    print(new_x_goal, new_y_goal, new_w_goal)
    return navigateToPos([new_x_goal, new_y_goal, new_w_goal])
