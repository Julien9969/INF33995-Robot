from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
INIT_POSE = PoseStamped()
INIT_POSE.pose.x = 0.0
INIT_POSE.pose.y = 0.0
INIT_POSE.pose.z = 0.0

GOAL_POSE = PoseStamped()
GOAL_POSE.pose.x = 8.0
GOAL_POSE.pose.y = 0.0
GOAL_POSE.pose.z = 0.0

TIMEOUT_ERROR = 600


{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}

class NavigationService(Node):
    self.initPose = INIT_POSE
    self.goalPose = GOAL_POSE
    
    def __init__():
        super().__init__('identify_service')
        self.srv = self.create_service(Navigation, 'identify', self.navigate)
        rclpy.init()
        self.nav = BasicNavigator()
        self.nav.setInitialPose(self.initPose)


    # ...
    def navigate(self, request, response):
        self.get_logger().info(f'Incoming request, a: {request.a}')
        self.nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        self.getPath()
        self.goToPosition()
        response.b = self.result
        return response

    #Initializes the path the robot takes to go to goal position
    def getPath():
        self.path = self.nav.getPath(self.initPose, self.goalPose)
        self.smoothed_path = self.nav.smoothPath(self.path)
    
    #The robot uses nav2 to go to previously setted position
    def goToPosition():
        self.nav.goToPose(self.goalPose)
        while not self.nav.isTaskComplete():
        feedback = self.nav.getFeedback()
        if feedback.navigation_duration > TIMEOUT_ERROR:
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

    navigation_service = NavigationService()

    rclpy.spin(navigation_service)



if __name__ == '__main__':
    main()