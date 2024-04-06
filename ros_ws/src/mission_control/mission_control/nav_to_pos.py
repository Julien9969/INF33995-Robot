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

from copy import deepcopy
import random
import time
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

TIMEOUT_TO_CANCEL = 20.0

INCREMENT = [
    [0.5, 0.5],
    [0.5, -0.5],
    [-0.5, -0.5],
    [-0.5, 0.5]
]

def go_to_poses(name_space):
    navigator = BasicNavigator(namespace=name_space)
    try:

        # Security route, probably read in from a file for a real application
        # from either a map or drive and repeat.
        security_route = [
            [1.792, 2.144],
            [1.792, -5.44],
            [1.792, -9.427],
            [-3.665, -9.427],
            [-3.665, -4.303],
            [-3.665, 2.330],
            [-3.665, 9.283]]


        # Do security route until dead
        while rclpy.ok():
            # Send our route
            route_poses = []
            pose = PoseStamped()
            pose.header.frame_id = f'{name_space}/odom'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.orientation.w = 1.0
            for pt in security_route:
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                route_poses.append(deepcopy(pose))
            navigator.goThroughPoses(route_poses)

            # Do something during our route (e.x. AI detection on camera images for anomalies)
            # Simply print ETA for the demonstation
            i = 0
            while not navigator.isTaskComplete():
                i += 1
                feedback = navigator.getFeedback()
                    # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=90.0):
                    navigator.cancelTask()

            # If at end of route, reverse the route to restart
            security_route.reverse()

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Route complete! Restarting...')
            elif result == TaskResult.CANCELED:
                print('Security route was canceled, exiting.')
                exit(1)
            elif result == TaskResult.FAILED:
                print('Security route failed! Restarting from other side...')
        return result
    except:
        navigator.cancelTask()
        rclpy.shutdown()
        sys.exit()



def setGoalPos(navigator, goalPosInfo, name_space):
    # Go to our demos first goal pose
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = f'{name_space}/map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(goalPosInfo[GOAL_X_POS_IN_ARGS])
    goal_pose.pose.position.y = float(goalPosInfo[GOAL_Y_POS_IN_ARGS])
    # goal_pose.pose.orientation.w = float(goalPosInfo[GOAL_W_ORIENTATION_IN_ARGS])

    return goal_pose

def compute_new_square(square, goals_results):
    new_square = []
    for i in range(len(square)):
        if goals_results[i] == TaskResult.SUCCEEDED:
            print('adddddddddd')
            new_square.append([square[i][0] + INCREMENT[i][0], square[i][1] + INCREMENT[i][1]])
        else:
            print('subbbbbbbbb')
            new_square.append([square[i][0] - INCREMENT[i][0], square[i][1] - INCREMENT[i][1]])
    return new_square

# pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.15159579452578884, y=-0.11735794085747109, z=0.0)
def new_square_from_poses(pose): 
    new_square = []
    for i in range(len(INCREMENT)):
        print(pose)
        new_square.append([pose.position.x + INCREMENT[i][0], pose.position.y + INCREMENT[i][1]])

    random.shuffle(new_square)
    print(new_square)
    return new_square

def square_nav(name_space):
    navigator = BasicNavigator(namespace=name_space)

    square = deepcopy(INCREMENT)

    while True:
        goals_results = []
        not_far_from_goal = False
        try:
            for pt in square:
                not_far_from_goal = False
                    
                goal_pose = setGoalPos(navigator, pt, name_space)
                navigator.goToPose(goal_pose)

                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    # print(feedback)

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                        navigator.cancelTask()
                        break
                    elif feedback.distance_remaining < 0.20:
                        navigator.cancelTask()
                        not_far_from_goal = True
                        print(feedback)

                        break
                    time.sleep(0.5)


                result = navigator.getResult()
                goals_results.append(result if not not_far_from_goal else TaskResult.SUCCEEDED)

                if result == TaskResult.SUCCEEDED or not_far_from_goal:
                    print('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')
                else:
                    print('Goal has an invalid return status!')

            # square = compute_new_square(square, goals_results)
            print(navigator.getFeedback().current_pose.pose)
            square = new_square_from_poses(navigator.getFeedback().current_pose.pose)

        except:
            navigator.cancelTask()

# nav2_msgs.action.NavigateToPose_Feedback(current_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1712436970, nanosec=892889298), frame_id='robot1/map'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.2572483882703879, y=-0.4146586193160936, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.8712225799436084, w=-0.4908881911356219))), navigation_time=builtin_interfaces.msg.Duration(sec=0, nanosec=120231597), estimated_time_remaining=builtin_interfaces.msg.Duration(sec=0, nanosec=0), number_of_recoveries=0, distance_remaining=0.17135821282863617)


def navigateToPos(goalPosInfo, name_space):
    navigator = BasicNavigator(namespace=name_space)
    try:

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

        goal_pose = setGoalPos(navigator, goalPosInfo, name_space)
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
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=TIMEOUT_TO_CANCEL):
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
    except:
        navigator.cancelTask()
        rclpy.shutdown()
        sys.exit()
    navigator.destroy_node()
    return result