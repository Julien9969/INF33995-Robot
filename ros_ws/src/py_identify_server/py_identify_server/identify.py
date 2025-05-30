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
        self.publisher_ = self.create_publisher(Twist, f'cmd_vel', 10)

    def serve(self, request, response):
        response.b = request.a * 2
        self.get_logger().info(f'Incoming request, a: {request.a}')

        for i in range(5):
            self.trigger()
            time.sleep(0.5)

        rotate_msg = Twist()
        rotate_msg.angular.z = 0.0
        rotate_msg.linear.x = 0.0

        self.get_logger().info(f'Outgoing response, b: {response.b}')

        for i in range(5):
            self.publisher_.publish(rotate_msg)
            time.sleep(0.3)

        return response

    def trigger(self):
        rotate_msg = Twist()

        if os.environ.get("ROBOT_ENV") == "SIMULATION":
            rotate_msg.angular.z = -5.5
            rotate_msg.linear.x = 0.5
        else:
            rotate_msg.angular.z = -1.5
            rotate_msg.linear.x = 0.0
                
        self.publisher_.publish(rotate_msg)


def main():
    rclpy.init()

    identify_service = IdentifyService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()