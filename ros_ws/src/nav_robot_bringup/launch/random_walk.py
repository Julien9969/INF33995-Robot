#! /usr/bin/env python3

import random
import os

def main():
    new_x_goal = 2.5 - random.random()*5
    new_y_goal = 2.5 - random.random()*5
    new_w_goal = 1 - random.random()*2
    print(new_x_goal, new_y_goal, new_w_goal)
    os.system(f'./nav_to_pose.launch.py {new_x_goal} {new_y_goal} {new_w_goal}')
    print(f'./nav_to_pose.py {new_x_goal} {new_y_goal} {new_w_goal}')

if __name__ == '__main__':
    main()