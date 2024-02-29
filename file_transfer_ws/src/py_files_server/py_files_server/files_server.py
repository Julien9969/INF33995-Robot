import time
from interfaces.srv import FilesServer

import sys
import rclpy
from rclpy.node import Node
# from .files_tree import build_json_file_tree

from enum import Enum

class Commands(Enum):
    FILES_TREE = "files-tree" 
    GET_FILE = "get-file"
    RECEIVE_FILE = "receive-file"

class FileServer(Node):

    def __init__(self):
        super().__init__('files_service')
        self.srv = self.create_service(FilesServer, 'files', self.serve)

    def serve(self, request, response):
        command:str = str(request.command)
        self.get_logger().info(f'Files Service Incoming request, {command}')

        if command == "files-tree":
            try:
                # response.content = build_json_file_tree()
                response.content = "build_json_file_tree()"
                response.message = "Success"
            except Exception as e:
                response.content = "Error"
                response.message = f"{str(e)}"
        else :
            response.message = "Command not found"
            response.content = ""

        self.get_logger().info(f'response: {response.message}')
        return response



def main():
    rclpy.init()

    files_server = FileServer()

    rclpy.spin(files_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()