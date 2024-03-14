import rclpy
from rclpy.node import Node
from interfaces.msg import LimoInfo, LimoStatus

class LimoInfoPublisher(Node):

    def __init__(self):
        super().__init__('limo_info_publisher')
        self.battery_level_percentage_ = 12

        self.publisher_ = self.create_publisher(LimoInfo, 'limo_info', 10)
        self.limo_status_subscriber_ = self.create_subscription(LimoStatus, 'limo_status', self.limo_status_listener_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def limo_status_listener_callback(self, msg):
        self.battery_level_percentage_ = msg.battery_voltage / 12 * 100

    def timer_callback(self):
        msg = LimoInfo()
        msg.battery_level_percentage = self.battery_level_percentage_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: \n\tBattery Level (%): {msg.battery_level_percentage}%')


def main(args=None):
    rclpy.init(args=args)
    limo_info_publisher = LimoInfoPublisher()
    rclpy.spin(limo_info_publisher)
    limo_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
