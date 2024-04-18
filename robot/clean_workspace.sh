#!/bin/bash

if [ -d /home/nvidia/INF3995-Robot/ros_ws/build ]; then
	rm -rf /home/nvidia/INF3995-Robot/ros_ws/build
fi

if [ -d /home/nvidia/INF3995-Robot/ros_ws/install ]; then
	rm -rf /home/nvidia/INF3995-Robot/ros_ws/install
fi

if [ -d /home/nvidia/INF3995-Robot/ros_ws/log ]; then
	rm -rf /home/nvidia/INF3995-Robot/ros_ws/log
fi
