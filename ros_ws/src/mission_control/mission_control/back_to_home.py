import argparse
from copy import deepcopy
import random
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import sys


def back_to_home(name_space):
    rclpy.init()
    navigator = BasicNavigator(name_space)
    navigator.start()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = f'{name_space}/map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.orientation.w = 0
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()


        if feedback.distance_remaining < 0.25:
            navigator.cancelTask()
            break
        time.sleep(0.5)

    rclpy.shutdown()
    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name_space", type=str, help="Namespace for the robot", required=True)
    args = parser.parse_args()
    back_to_home(args.name_space)