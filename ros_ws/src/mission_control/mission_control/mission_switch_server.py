from enum import Enum
import time
from interfaces.srv import MissionSwitch

import sys, subprocess, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class State(Enum):
    ON = "ON" 
    OFF = "OFF"


START = "start"
STOP = "stop"

class MissionSwitchService(Node):

    def __init__(self):
        super().__init__('mission_switch')
        self.srv = self.create_service(MissionSwitch, 'mission_switch', self.serve)
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.timer= self.create_timer(0.5, self.timer_callback)
        self.state = State.OFF
    
    # def timer_callback(self):
    #     if(self.state == State.ON ):
    #         rotate_msg = Twist()
    #         rotate_msg.linear.x = 1.0
    #         rotate_msg.angular.z = 0.5
    #         self.publisher_.publish(rotate_msg)

    def start_process(self):
        # self.navProcess = subprocess.Popen(['python3', '-u', 'src/mission_control/mission_control/random_walk.py'])
        self.navProcess = subprocess.Popen(['/bin/bash', '-c', 'source install/setup.bash && ros2 launch explore_lite explore.launch.py use_sim_time:=false'])
        # self.navProcess2 = subprocess.Popen(['/bin/bash', '-c', 'source install/setup.bash && ros2 launch explore_lite explore.launch2.py use_sim_time:=false'])
        
        self.get_logger().info('Started random walk')

    def serve(self, request, response):
        command:str = str(request.command)

        self.get_logger().info(f'Incoming request, command: {command}, current state: {self.state}')

        if command == START and self.state == State.OFF:
            self.state = State.ON
            self.start_process()
            response.answer = f'{command} executed'
        elif command == STOP and self.state == State.ON:
            self.state = State.OFF

            self.navProcess.send_signal(signal.SIGINT)
            self.navProcess2.send_signal(signal.SIGINT)

            # self.navProcess.send_signal(signal.SIGKILL)
            # self.navProcess2.send_signal(signal.SIGKILL)

            response.answer = f'{command} executed'
            # self.publisher_.publish(Twist())
        else:
            response.answer = 'unknown'

        return response


def main():
    try:
        rclpy.init()

        identify_service = MissionSwitchService()

        rclpy.spin(identify_service)

        rclpy.shutdown()
    except:
        if identify_service is not None and identify_service.navProcess is not None:
            identify_service.navProcess.send_signal(signal.SIGINT)
            identify_service.navProcess2.send_signal(signal.SIGINT)



if __name__ == '__main__':
    main()