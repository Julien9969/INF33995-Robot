#!/bin/bash

colcon build
source install/setup.bash
ros2 run my_pcl_pkg my_node