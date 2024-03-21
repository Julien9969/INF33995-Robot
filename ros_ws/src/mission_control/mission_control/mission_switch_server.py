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
from interfaces.srv import Exploration
from launch import LaunchDescription

class State(Enum):
    ON = True 
    OFF = False


START = "start"
STOP = "stop"
# ROBOT_ID = os.environ["ROBOT_NUM"]

class MissionSwitchService(Node):
    numberOfcallbacked = 0
    last_state = State.ON
    state = State.OFF

    def __init__(self):
        super().__init__('mission_switch')
        # self.create_node_timer()
        self.create_connection_to_explore_service()

    # def timer_callback(self):
    #     self.numberOfcallbacked+=1
    #     self.pause_timer()
    #     self.get_logger().info(f'calling back timer in mission switch----')
    #     self.get_logger().info(f'state ON, sending explore')
    #     self.send_request() #est async! donc on bloque rien ici sadly
    #     self.get_logger().info(f'explore RES RECEIVED AND WE HAVE MADE {self.numberOfcallbacked} CALLBACKS')
    #     self.restart_timer()

    def serve(self, request, response):
        command:str = str(request.command)
        self.get_logger().info(f'Incoming request, command: {command}, current state: {self.state}')
        if command == START and self.state == State.OFF:
            self.changeState()
            # self.restart_timer()
            response.answer = f'{command} executed'
        elif command == STOP and self.state == State.ON:
            self.changeState()
            response.answer = f'{command} executed'
            # self.pause_timer()
        else:
            response.answer = 'unknown '
        return response

# TODO: test send request from mission to make sure ok / doesn't block mission from stopping for example
    def send_request(self):
        self.get_logger().info(f'Mission sending explore request...')
        self.future = self.cli.call_async(self.req) #sync maintenant?
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'Mission received explore!!: MIAWIW:{self.future.result()}')
        return self.future.result()

    # def create_node_timer(self):
    #     self.timer = self.create_timer(2, self.timer_callback)
    #     self.pause_timer()


    def create_connection_to_explore_service(self):
        self.srv = self.create_service(MissionSwitch, 'mission_switch', self.serve)
        self.cli = self.create_client(Exploration, "/explore")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('explore service not available, waiting again...')
        self.req = Exploration.Request()
        self.get_logger().info('explore service available in mission!')

    # def pause_timer(self):
    #     self.get_logger().info('Timer has been paused')
    #     self.timer.cancel()

    # def restart_timer(self):
    #     self.get_logger().info('Timer has been restarted')
    #     self.timer.reset()

    def changeState(self):
        self.get_logger().info(f'ChangedState from {self.state} to {not self.state}')
        self.last_state = self.state
        if self.state_started():
            self.state = State.OFF
        else:
            self.state = State.ON

    def state_started(self):
        return self.state == State.ON

    def generate_launch_description(self):
        explore_2 = Node(
            package='py_exploration_server',
            executable='explore',
            output='screen',
            # namespace='robot2',
        )
        return LaunchDescription([explore_2])

    def launch_exploration_node(self):
        launch = self.generate_launch_description()
        launch.start()


def main():
    rclpy.init()

    mission_service = MissionSwitchService()
    while True:
        rclpy.spin_once(mission_service, timeout_sec=0.5)
        if mission_service.state_started():
            mission_service.send_request()

    rclpy.shutdown()


if __name__ == '__main__':
    main()