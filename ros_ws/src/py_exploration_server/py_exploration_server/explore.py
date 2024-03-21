from py_exploration_server.random_walk import navigateToRandomLocation
from rclpy.node import Node
from std_srvs.srv import Empty
import rclpy

class ExplorationService(Node):
    onGoingNavigation = False
    def __init__(self):
        super().__init__('exploration_service')
        self.srv = self.create_service(Empty, '/explore', self.explore)
    
    def explore(self, req, res):
        self.get_logger().info(f'Incoming request EXPLORE')
        if not self.onGoingNavigation:
            self.onGoingNavigation = True
            result = navigateToRandomLocation()
            self.onGoingNavigation = False
            return res # TODO: test if this was what was missing, made service crash when returning from service call
        


def main():
    rclpy.init()

    explorationService = ExplorationService()

    rclpy.spin(explorationService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()