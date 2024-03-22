#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import sys

"""
Basic navigation demo to go to pose.
"""

# INIT_X_POS_IN_ARGS = 0
# INIT_Y_POS_IN_ARGS = 1
# INIT_Z_ORIENTATION_IN_ARGS = 2

GOAL_X_POS_IN_ARGS = 0
GOAL_Y_POS_IN_ARGS = 1
GOAL_W_ORIENTATION_IN_ARGS = 2

def setGoalPos(navigator, goalPosInfo):
    # Go to our demos first goal pose
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(goalPosInfo[GOAL_X_POS_IN_ARGS])
    goal_pose.pose.position.y = float(goalPosInfo[GOAL_Y_POS_IN_ARGS])
    goal_pose.pose.orientation.w = float(goalPosInfo[GOAL_W_ORIENTATION_IN_ARGS])

    return goal_pose

def navigateToPos(goalPosInfo):
    # rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = float(argumentsPassedToScript[INIT_X_POS_IN_ARGS])
    # initial_pose.pose.position.y = float(argumentsPassedToScript[INIT_Y_POS_IN_ARGS])
    # initial_pose.pose.orientation.z = float(argumentsPassedToScript[INIT_Z_ORIENTATION_IN_ARGS])
    # initial_pose.pose.orientation.w = 0.5 #que es?
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    goal_pose = setGoalPos(navigator, goalPosInfo)
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        feedback = navigator.getFeedback()

        # print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')

            # Some navigation timeout to demo cancellation
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                navigator.cancelTask()
                break
            # Some navigation request change to demo preeto demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        result = 'success'
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        result = "canceled"
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        result = 'failed'
        print('Goal failed!')
    else:
        result = 'invalid return status'
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()
    return result