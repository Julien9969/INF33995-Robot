import json
from interfaces.srv import FilesServer

import sys, os
import subprocess
import rclpy
from rclpy.node import Node
from py_files_server.files_tree import get_file_tree, get_full_path

from enum import Enum

class Commands(Enum):
    FILES_TREE = "files-tree" 
    GET_FILE = "get-file"
    EDIT_FILE = "edit-file"
    UPDATE_ROBOT = "update-robot"

class FileServer(Node):

    def __init__(self):
        super().__init__('files_service')
        self.srv = self.create_service(FilesServer, 'files', self.serve)

    def serve(self, request, response):
        command:str = str(request.command)
        
        self.get_logger().info(f'Files Service Incoming request, {command}')

        try:
            if command == Commands.FILES_TREE.value:
                response.content = get_file_tree()
                response.message = "Success"
            
            elif command == Commands.GET_FILE.value:
                fileDict = json.loads(request.content)
                file_path = get_full_path(fileDict["id"], fileDict["name"])

                if os.path.exists(file_path):
                    with open(file_path, 'r') as file:
                        response.content = file.read()
                        response.message = "Success"
                else:
                    response.message = "Error"
                    response.content = "File not found or moved"
            
            elif command == Commands.EDIT_FILE.value:
                fileDict = json.loads(request.content)
                file_path = get_full_path(fileDict["id"], fileDict["name"])
                if os.path.exists(file_path):

                    with open(file_path, 'w') as file:
                        file.write(fileDict["content"])
                        response.message = "Success"
                        response.content = "File received"  
                else:
                    response.message = "Error"
                    response.content = "File not found or moved"

            elif command == Commands.UPDATE_ROBOT.value:
                self.get_logger().info(f'Update: {os.environ.get("ROBOT_ENV")}')

                if os.environ.get("ROBOT_ENV") == "SIMULATION":
                    path = "./"
                else:
                    path = "/home/nvidia/INF3995-Robot/ros_ws/"

                with open(os.path.join(path, "stdout.txt"), "w") as f:
                    subprocess.Popen(["bash", os.path.join(path, "rebuild_scripts/rebuild-robot.sh")], stdout=f, stderr=subprocess.STDOUT)
    
                response.message = "Success"
                response.content = "update en cours"
            
            else :
                response.message = "Error"
                response.content = "Command not found"
        
        except Exception as e:
            response.content = f"{str(e)}"
            response.message = "Error"

        self.get_logger().info(f'response: {response.message} {"" if response.message == "Success" else response.content}')
        return response

def main():
    rclpy.init()

    files_server = FileServer()

    rclpy.spin(files_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()