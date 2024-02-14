#!/bin/bash

if [ -d /root/INF3995-Robot/ros_ws/build ]; then
	rm -rf /root/INF3995-Robot/ros_ws/build
fi

if [ -d /root/INF3995-Robot/ros_ws/install ]; then
	rm -rf /root/INF3995-Robot/ros_ws/install
fi

if [ -d /root/INF3995-Robot/ros_ws/log ]; then
	rm -rf /root/INF3995-Robot/ros_ws/log
fi

