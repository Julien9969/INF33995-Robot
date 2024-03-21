from py_exploration_server.random_walk import navigateToRandomLocation
from rclpy.node import Node
from std_srvs.srv import Empty
import rclpy
from interfaces.srv import Exploration

class ExplorationService(Node):
    onGoingNavigation = False
    def __init__(self):
        super().__init__('exploration_service')
        self.srv = self.create_service(Exploration, '/explore', self.explore)
    
    def explore(self, req, res):
        self.get_logger().info(f'Incoming request EXPLORE')
        self.onGoingNavigation = True
        self.get_logger().info(f'Envoie d un message de navigation')
        result = navigateToRandomLocation()
        self.onGoingNavigation = False
        self.get_logger().info(f'Received this result from navigation:')
        self.get_logger().info(f'Returning explore task with this result: {result}')
        res.result = result
        return res # TODO: test if this was what was missing, made service crash when returning from service call
        


def main():
    rclpy.init()

    explorationService = ExplorationService()

    rclpy.spin(explorationService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()