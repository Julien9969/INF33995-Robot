import time
from interfaces.srv import FilesService

import sys
import rclpy
from rclpy.node import Node
from .files_tree import build_json_file_tree

from enum import Enum

class Commands(Enum):
    FILES_TREE = "files-tree" 
    GET_FILE = "get-file"
    RECEIVE_FILE = "receive-file"

class FileService(Node):

    def __init__(self):
        super().__init__('files_service')
        self.srv = self.create_service(FilesService, 'files', self.serve)

    def serve(self, request, response):
        
        self.get_logger().info(f'Files Service Incoming request, {request.command}')

        if request.command == Commands.FILES_TREE:
            try:
                response.content = build_json_file_tree()
                response.message = "Success"
            except Exception as e:
                response.content = ""
                response.message = f"Error : {e}"
            
            return response



def main():
    rclpy.init()

    identify_service = FileService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()