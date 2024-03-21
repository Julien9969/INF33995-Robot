import rclpy
from rclpy.node import Node
from limo_msgs.msg import LimoStatus
import os

class LimoInfoPublisher(Node):

    def __init__(self):
        super().__init__('limo_info_publisher')
        self.battery_level_percentage_ = 100

        if 'ROBOT_NUM' not in os.environ:
            timer_period = 20  # seconds
            self.timer = self.create_timer(timer_period, self.simulation_timer_callback)
        else:
            robot_num = os.environ['ROBOT_NUM']
            self.limo_status_subscriber_ = self.create_subscription(LimoStatus, (f'/robot{robot_num}/limo_status'), self.limo_status_listener_callback, 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.robot_timer_callback)

    def limo_status_listener_callback(self, msg):
        new_percentage = int((msg.battery_voltage - 9) / 3 * 100)
        self.battery_level_percentage_ = new_percentage if new_percentage < self.battery_level_percentage_ else self.battery_level_percentage_

    def simulation_timer_callback(self):
        self.battery_level_percentage_ -= 1
        if self.battery_level_percentage_ < 0:
            self.battery_level_percentage_ = 0
        self.get_logger().info(f'BATTERY: {self.battery_level_percentage_}%')

    def robot_timer_callback(self):
        self.get_logger().info(f'BATTERY: {self.battery_level_percentage_}%')


def main(args=None):
    rclpy.init(args=args)
    limo_info_publisher = LimoInfoPublisher()
    rclpy.spin(limo_info_publisher)
    limo_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
