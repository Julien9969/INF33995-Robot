from scripts.random_walk import navigateToRandomLocation
from rclpy.node import Node
from std_srvs.srv import Empty
import rclpy

class ExplorationService(Node):
    onGoingNavigation = False
    def __init__(self):
        super().__init__('exploration service')
        self.srv = self.create_service(Empty, 'exploration', self.explore)
    
    def explore(self):
        if not self.onGoingNavigation:
            self.onGoingNavigation = True
            # result = navigateToRandomLocation()
            self.onGoingNavigation = False
        


def main():
    rclpy.init()

    explorationService = ExplorationService()

    rclpy.spin(explorationService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()