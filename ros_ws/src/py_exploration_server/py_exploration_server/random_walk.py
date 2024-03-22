#! /usr/bin/env python3

import random
from py_exploration_server.nav_to_pos import navigateToPos

RANGE_VALUES_POS = 2
VALUE_TO_SIMULATE_RANGE = RANGE_VALUES_POS/2
NOT_IMPORTANT_VALUE = 0.0
PROTECTED_ZONE = 0.75


def navigateToRandomLocation():
    new_x_goal = 0
    new_y_goal = 0
    while new_x_goal > -PROTECTED_ZONE and new_x_goal < PROTECTED_ZONE:
        new_x_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    while new_y_goal > -PROTECTED_ZONE and new_y_goal < PROTECTED_ZONE:
        new_y_goal = VALUE_TO_SIMULATE_RANGE - random.random()*RANGE_VALUES_POS
    new_w_goal = NOT_IMPORTANT_VALUE

    print(new_x_goal, new_y_goal, new_w_goal)
    return navigateToPos([new_x_goal, new_y_goal, new_w_goal])
while True:
    navigateToRandomLocation()
# 
#      x: 0.0004669085003282022
#       y: 0.0035518087427087244
    
    