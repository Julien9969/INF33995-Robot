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
import signal
import sys
import time, os
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

GOAL_X_POS_IN_ARGS = 0
GOAL_Y_POS_IN_ARGS = 1
GOAL_W_ORIENTATION_IN_ARGS = 2

TIMEOUT_TO_CANCEL = 20.0

if os.environ.get("ROBOT_ENV") == "SIMULATION":
    INCREMENT = 1.4
    NOT_FAR = 0.80
    print('Simulation')
else:
    INCREMENT = 0.7
    NOT_FAR = 0.40
    print('Real')

INCREMENT_POINTS = [
    [INCREMENT, INCREMENT],
    [-INCREMENT, INCREMENT],
    [INCREMENT, -INCREMENT],
    [-INCREMENT, -INCREMENT]
]

def signal_handler(sig, frame):
    global navigator
    print('Stop navigation!')
    try:
        navigator.cancelTask()
        navigator.destroyNode()
    except:
        print("Navigator not initialized")
    sys.exit(0)

def setGoalPos(navigator, goalPosInfo, name_space):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = f'{name_space}/map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(goalPosInfo[GOAL_X_POS_IN_ARGS])
    goal_pose.pose.position.y = float(goalPosInfo[GOAL_Y_POS_IN_ARGS])

    return goal_pose

def compute_new_square(square, goals_results) -> list[list[float, float]]:
    """Creat increment points for the next square

    Args:
        square : current square
        goals_results : results of the previous goals

    Returns:
        new square
    """
    new_square = []
    for i in range(len(square)):
        if goals_results[i] == TaskResult.SUCCEEDED:
            new_square.append([square[i][0] + INCREMENT_POINTS[i][0], square[i][1] + INCREMENT_POINTS[i][1]])
        else:
            new_square.append([square[i][0] - INCREMENT_POINTS[i][0], square[i][1] - INCREMENT_POINTS[i][1]])
    return new_square

def new_square_from_poses(pose): 
    new_square = []
    for i in range(len(INCREMENT_POINTS)):
        new_square.append([pose.position.x + INCREMENT_POINTS[i][0], pose.position.y + INCREMENT_POINTS[i][1]])

    random.shuffle(new_square)
    print(new_square)
    return new_square

def square_nav(name_space):
    signal.signal(signal.SIGINT, signal_handler)

    global navigator
    navigator = BasicNavigator(namespace=name_space)

    if os.environ.get("ROBOT_NUM") is None:
        namespace_value = int(name_space.split('robot')[1])
        if namespace_value % 2 == 0:
            square = [[namespace_value, INCREMENT]]
        else:
            square = [[namespace_value, 0]]
    else: 
        square = random.shuffle(deepcopy(INCREMENT_POINTS))

    print(square)

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

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
                        # print(feedback)
                        navigator.cancelTask()
                        break
                    elif feedback.distance_remaining < NOT_FAR:
                        navigator.cancelTask()
                        not_far_from_goal = True
                        navigator.get_logger().info(f'[{name_space}] Not far from goal!')
                        break
                    
                    time.sleep(0.5)

                result = navigator.getResult()
                goals_results.append(result if not not_far_from_goal else TaskResult.SUCCEEDED)

                if result == TaskResult.SUCCEEDED or not_far_from_goal:
                    navigator.get_logger().info(f'[{name_space}] Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    navigator.get_logger().info(f'[{name_space}] Goal was canceled!')
                elif result == TaskResult.FAILED:
                    navigator.get_logger().info(f'[{name_space}] Goal failed!')
                else:
                    navigator.clearAllCostmaps()
                    navigator.get_logger().info(f'[{name_space}] Goal has an invalid return status!')
            # square = compute_new_square(square, goals_results)
            square = new_square_from_poses(navigator.getFeedback().current_pose.pose)
        except:
            try:
                navigator.cancelTask()
            except:
                print(f'[{name_space}] Navigator not initialized')

# nav2_msgs.action.NavigateToPose_Feedback(current_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1712436970, nanosec=892889298), frame_id='robot1/map'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.2572483882703879, y=-0.4146586193160936, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.8712225799436084, w=-0.4908881911356219))), navigation_time=builtin_interfaces.msg.Duration(sec=0, nanosec=120231597), estimated_time_remaining=builtin_interfaces.msg.Duration(sec=0, nanosec=0), number_of_recoveries=0, distance_remaining=0.17135821282863617)
