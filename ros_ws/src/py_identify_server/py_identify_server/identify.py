import time
from interfaces.srv import Identify
import os

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class IdentifyService(Node):

    def __init__(self):
        super().__init__('identify_service')
        self.srv = self.create_service(Identify, 'identify', self.serve)
        self.publisher_ = self.create_publisher(Twist, f'/robot{os.environ["ROBOT_NUM"]}/cmd_vel', 10)

    def serve(self, request, response):
        response.b = request.a * 2
        self.get_logger().info(f'Incoming request, a: {request.a}')

        for i in range(6):
            self.trigger()

        return response

    def trigger(self):
        rotate_msg = Twist()
        rotate_msg.angular.z = 3.14
        self.publisher_.publish(rotate_msg)


def main():
    rclpy.init()

    identify_service = IdentifyService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()