import time
from interfaces.srv import Identify

import sys
import rclpy
from rclpy.node import Node
from .spin_robot import SpinRobot


class IdentifyService(Node):

    def __init__(self):
        super().__init__('identify_service')
        self.srv = self.create_service(Identify, 'identify', self.serve)

    def serve(self, request, response):
        if request.identify == 0:
            response.response = "Identify request not treated"
            return response

        self.get_logger().info(f'Incoming request, identify: {bool(request.identify)}')

        SpinRobot().trigger()
        time.sleep(10)
        SpinRobot().stop()

        response.response = "Identify request completed"
        return response


def main():
    rclpy.init()

    identify_service = IdentifyService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()