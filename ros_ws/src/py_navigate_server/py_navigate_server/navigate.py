from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import  PoseStamped
from rclpy.node import Node
from interfaces.srv import Navigate
import os
from geometry_msgs.msg import Twist
# from nav_msgs.msg import RobotState
INIT_POSE = PoseStamped()
INIT_POSE.pose.position.x = 0.0
INIT_POSE.pose.position.y = 0.0
INIT_POSE.pose.position.z = 0.0

GOAL_POSE = PoseStamped()
GOAL_POSE.pose.position.x = 8.0
GOAL_POSE.pose.position.y = 0.0
GOAL_POSE.pose.position.z = 0.0

TIMEOUT_ERROR = 600


# {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}

class NavigateService(Node):
    initPose = INIT_POSE
    goalPose = GOAL_POSE
    
    def __init__(self):
        super().__init__('navigate_service')
        self.srv = self.create_service(Navigate, 'navigate', self.navigate)
        self.nav = BasicNavigator()
        self.nav.setInitialPose(self.initPose)
        self.publisher_ = self.create_publisher(Twist, f'/robot2/cmd_vel', 10)
        
        rotate_msg = Twist()
        rotate_msg.angular.z = 3.14
        self.publisher_.publish(rotate_msg)

        # client = self.create_client(GetRobotState, '/amcl/get_state')
        self.nav.lifecycleStartup()
        self.nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()
        self.getPath()
        self.goToPosition()

    # ...
    def navigate(self, request, response):
        self.get_logger().info(f'Incoming request, a: {request.a}')
        self.nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        self.getPath()
        self.goToPosition()
        response.b = self.result

        return response

    #Initializes the path the robot takes to go to goal position
    def getPath(self):
        self.path = self.nav.getPath(self.initPose, self.goalPose)
        self.smoothed_path = self.nav.smoothPath(self.path)
    
    #The robot uses nav2 to go to previously setted position
    def goToPosition(self):
        self.nav.goToPose(self.goalPose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
        if feedback.navigate_duration > TIMEOUT_ERROR:
            self.nav.cancelTask()
        self.result = self.nav.getResult()
        if self.result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif self.result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif self.result == TaskResult.FAILED:
            print('Goal failed!')
            





def main():
    rclpy.init()

    navigate_service = NavigateService()

    rclpy.spin(navigate_service)



if __name__ == '__main__':
    main()