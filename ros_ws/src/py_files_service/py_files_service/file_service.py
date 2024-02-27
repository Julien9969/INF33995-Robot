import time
from interfaces.srv import FilesService

import sys
import rclpy
from rclpy.node import Node


class FileService(Node):

    def __init__(self):
        super().__init__('files_service')
        self.srv = self.create_service(FilesService, 'files', self.serve)

    def serve(self, request, response):
        response.b = request.a * 2
        self.get_logger().info(f'Incoming request, a: {request.a}')

        return response


def main():
    rclpy.init()

    identify_service = FileService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()