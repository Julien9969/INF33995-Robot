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
    navProcess2 = None

    def __init__(self):
        super().__init__('mission_switch')
        self.srv = self.create_service(MissionSwitch, 'mission_switch', self.serve)
        self.declare_parameter('robot_id', rclpy.Parameter.Type.INTEGER)
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.timer= self.create_timer(0.5, self.timer_callback)
        self.state = State.OFF
        robot_id = self.get_parameter('robot_id')
        self.get_logger().info("int: %s" %(str(robot_id.value),))
    
    # def timer_callback(self):
    #     if(self.state == State.ON ):
    #         rotate_msg = Twist()
    #         rotate_msg.linear.x = 1.0
    #         rotate_msg.angular.z = 0.5
    #         self.publisher_.publish(rotate_msg)

    def start_process(self):
        # self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py'])
        if os.environ.get("ROBOT_ENV") == "SIMULATION":
            # self.navProcess = subprocess.Popen(['/bin/bash', '-c', 'source install/setup.bash && ros2 launch explore_lite explore.launch.py use_sim_time:=false'])
            self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py', '-n', f'robot1'])
            self.navProcess2 = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py', '-n', f'robot2'])
            # self.navProcess2 = subprocess.Popen(['/bin/bash', '-c', 'source install/setup.bash && ros2 launch explore_lite explore.launch2.py use_sim_time:=false'])
        else:
            print(f'robot{os.environ.get("ROBOT_NAME_SPACE")}')
            self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py', '-n', f'robot{os.environ.get("ROBOT_NUM")}'])
            # self.navProcess2 = subprocess.Popen(['/bin/bash', '-c', 'source install/setup.bash && ros2 launch explore_lite explore.launch2.py use_sim_time:=false'])
        
        self.get_logger().info('Started random walk')

    def stop_process(self):
        try:
            self.navProcess.send_signal(signal.SIGINT)
            time.sleep(0.1)
            self.navProcess.send_signal(signal.SIGINT)
            time.sleep(0.5)
            self.navProcess.send_signal(signal.SIGKILL)
            self.navProcess = None
            
            if os.environ.get("ROBOT_ENV") == "SIMULATION":
                self.navProcess2.send_signal(signal.SIGINT)
                time.sleep(0.1)
                self.navProcess2.send_signal(signal.SIGINT)
                time.sleep(0.5)
                self.navProcess2.send_signal(signal.SIGKILL)
                self.navProcess2 = None

            self.get_logger().info('Stopped random walk')
        except Exception as e:
            self.get_logger().info(f'Error stopping random walk : {e}')

    def call_back_home(self):
        self.state = State.OFF
        try:
            if self.navProcess is not None:
                self.stop_process()

            if os.environ.get("ROBOT_ENV") == "SIMULATION":
                self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/back_to_home.py', '-n', f'robot1'])
                time.sleep(2)
                self.navProcess2 = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/back_to_home.py', '-n', f'robot2'])
            else:
                self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/back_to_home.py', '-n', f'robot{os.environ.get("ROBOT_NUM")}'])
            self.get_logger().info('Going back home')
        except Exception as e:
            self.get_logger().info(f'Error going back home : {e}')

    def get_environment(self):
        return 'simulated' if 'ROBOT_NUM' not in os.environ else 'real'

    def serve(self, request, response):
        command:str = str(request.command)

        self.get_logger().info(f'Incoming request, command: {command}, current state: {self.state}')

        if command == START and self.state == State.OFF:
            self.state = State.ON
            self.start_process()
            response.answer = f'{command} executed'
        elif command == STOP and self.state == State.ON:
            self.state = State.OFF

            self.stop_process()
            response.answer = f'{command} executed'
            # self.publisher_.publish(Twist())
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