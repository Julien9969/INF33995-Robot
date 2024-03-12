# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
