import argparse
from copy import deepcopy
import os
import random
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import sys

if os.environ.get("ROBOT_ENV") == "SIMULATION":
    HOME_POS = [-4.0, 0.0] # -4 -0.35 0.35
    NOT_FAR = 0.70
else:
    HOME_POS = [0.0, 0.0]
    NOT_FAR = 0.10

def back_to_home(name_space):
    """
    Navigate the robot back to the home position

    Args:
        name_space : namespace of the robot
    """

    rclpy.init()
    navigator = BasicNavigator(namespace=name_space)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = f'{name_space}/map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = HOME_POS[0]
    goal_pose.pose.position.y = HOME_POS[1]
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        try:
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=25.0) or feedback.distance_remaining < NOT_FAR:
                navigator.cancelTask()
                break
            time.sleep(0.5)
        except:
            print("distance remaining has not been calculated yet")

    rclpy.shutdown()
    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name_space", type=str, help="Namespace for the robot", required=True)
    args = parser.parse_args()
    back_to_home(args.name_space)