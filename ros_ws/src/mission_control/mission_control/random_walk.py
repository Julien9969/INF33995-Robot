#! /usr/bin/env python3

import signal
import sys
import argparse
import sys
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_to_pos import square_nav

sys.path.append('.')


def main(name_space):
    rclpy.init()

    square_nav(name_space)

    rclpy.shutdown()


def signal_handler(sig, frame):
    print('Stop navigation!')
    try:
        navigator = BasicNavigator(namespace=args.name_space)
        navigator.cancelTask()
        navigator.destroyNode()
    except:
        print("Navigator not initialized")
    sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name_space", type=str, help="Namespace for the robot", required=True)
    args = parser.parse_args()
    print(args.name_space)
    signal.signal(signal.SIGINT, signal_handler)
    main(args.name_space)