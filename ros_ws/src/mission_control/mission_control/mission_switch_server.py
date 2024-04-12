from enum import Enum
import time
import os
from interfaces.srv import MissionSwitch

import sys, subprocess, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
# from mission_control.random_walk import main as random_walk_main

class State(Enum):
    ON = "ON" 
    OFF = "OFF"


START = "start"
STOP = "stop"
HOME = "home"

class MissionSwitchService(Node): #TODO : print parameter of mission switch robot_id
    navProcess = None
    robot_id = ''

    def __init__(self):
        super().__init__('mission_switch')
        self.srv = self.create_service(MissionSwitch, 'mission_switch', self.serve)
        try:
            self.declare_parameter('robot_id', rclpy.Parameter.Type.INTEGER)
            self.robot_id = str(self.get_parameter('robot_id').value)
            self.get_logger().info(f"[robot{self.robot_id}] Simulation mission switch started")
        except Exception as e:
            self.robot_id = os.environ.get("ROBOT_NUM")
            self.get_logger().error(f'Error getting robot_id : {e} can be ignored if running with real robots')
        self.state = State.OFF
    
    def start_process(self):
        if os.environ.get("ROBOT_ENV") == "SIMULATION":
            self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py', '-n', f'robot{self.robot_id}'])
        else:
            self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py', '-n', f'robot{os.environ.get("ROBOT_NUM")}'])
        
        self.get_logger().info(f'[robot{self.robot_id}] Started random walk')

    def stop_process(self):
        try:
            self.navProcess.send_signal(signal.SIGINT)
            time.sleep(0.1)
            self.navProcess.send_signal(signal.SIGINT)
            time.sleep(0.5)
            self.navProcess.send_signal(signal.SIGKILL)
            self.navProcess = None
            
            self.get_logger().info(f'[robot{self.robot_id}] Stopped random walk')
        except Exception as e:
            self.get_logger().error(f'[robot{self.robot_id}] Error stopping random walk : {e}')

    def call_back_home(self):
        self.state = State.OFF
        try:
            if self.navProcess is not None:
                self.stop_process()

            if os.environ.get("ROBOT_ENV") == "SIMULATION":
                self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/back_to_home.py', '-n', f'robot{self.robot_id}'])
                time.sleep(2)
            else:
                self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/back_to_home.py', '-n', f'robot{os.environ.get("ROBOT_NUM")}'])
            self.get_logger().info(f'[robot{self.robot_id}] Going back home')
        except Exception as e:
            self.get_logger().error(f'[robot{self.robot_id}] Error going back home : {e}')

    def get_environment(self):
        return 'simulated' if 'ROBOT_NUM' not in os.environ else 'real'

    def serve(self, request, response):
        command:str = str(request.command)

        self.get_logger().info(f'[robot{self.robot_id}] Incoming request, command: {command}, current state: {self.state}')

        if command == START and self.state == State.OFF:
            self.state = State.ON
            self.start_process()
            response.answer = f'{command} executed'
        elif command == STOP and self.state == State.ON:
            self.state = State.OFF

            self.stop_process()
            response.answer = f'{command} executed'
        elif command == HOME:
            self.state = State.OFF
            self.call_back_home()
            response.answer = f'{command} executed'
        else:
            response.answer = 'unknown'

        response.environment = self.get_environment()

        return response


def main():
    try:
        rclpy.init()

        identify_service = MissionSwitchService()

        rclpy.spin(identify_service)

        rclpy.shutdown()
    except:
        pass



if __name__ == '__main__':
    main()