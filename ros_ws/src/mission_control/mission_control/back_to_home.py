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

INFINITY = float('inf')

if os.environ.get("ROBOT_ENV") == "SIMULATION":
    HOME_POS = [-4.0, 0.0] # -4 -0.35 0.35
    NOT_FAR = 0.80
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
    try:
        navigator = BasicNavigator(namespace=name_space)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = f'{name_space}/map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = HOME_POS[0]
        goal_pose.pose.position.y = HOME_POS[1]

        distance_remaining = INFINITY
        feedback = None

        while distance_remaining > NOT_FAR:
            navigator.goToPose(goal_pose)
            
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                try:
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
                        navigator.cancelTask()
                        break
                    time.sleep(0.5)
                except:
                    print("distance remaining has not been calculated yet")

            if feedback is not None:
                distance_remaining = feedback.distance_remaining
                print(f"Distance remaining: {distance_remaining}")

        navigator.get_logger().info(f"Robot {name_space} back to home DONE!")
        rclpy.shutdown()
    except Exception as e:
        print(f"An error occurred: {e}")
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name_space", type=str, help="Namespace for the robot", required=True)
    args = parser.parse_args()
    back_to_home(args.name_space)