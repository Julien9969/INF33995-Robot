from enum import Enum
import time
from interfaces.srv import MissionSwitch
import os

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class State(Enum):
    ON = "ON" 
    OFF = "OFF"


START = "start"
STOP = "stop"
# ROBOT_ID = os.environ["ROBOT_NUM"]

class MissionSwitchService(Node):

    def __init__(self):
        super().__init__('mission_switch')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer= self.create_timer(2, self.timer_callback)
        self.srv = self.create_service(MissionSwitch, 'mission_switch', self.serve)
        self.state = State.OFF
        # self.onGoingNavigation = False
        self.cli = self.create_client(Empty, "/explore")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('explore service not available, waiting again...')
        self.req = Empty.Request()

    def timer_callback(self):
        if(self.state == State.ON ):
            self.get_logger().info(f'state ON, sending explore')
            response = self.send_request(int(sys.argv[1]), int(sys.argv[2]))
        # if(self.state == State.OFF):
        #     return
        # if(not self.onGoingNavigation):
        #     self.onGoingNavigation = True
        #     result = navigateToRandomLocation()
        #     self.onGoingNavigation = False
        # else:
        #     self.get_logger().info(f'random walk ongoing, waiting again...')

    def serve(self, request, response):
        command:str = str(request.command)
        self.get_logger().info(f'Incoming request, command: {command}, current state: {self.state}')
        if command == START and self.state == State.OFF:
            self.state = State.ON
            response.answer = f'{command} executed'
        elif command == STOP and self.state == State.ON:
            self.state = State.OFF
            response.answer = f'{command} executed'
            self.publisher_.publish(Twist())
        else:
            response.answer = 'unknown '
        return response

# TODO: test send request from mission to make sure ok / doesn't block mission from stopping for example
    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    identify_service = MissionSwitchService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()