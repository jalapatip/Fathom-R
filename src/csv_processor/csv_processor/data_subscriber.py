import rclpy
from rclpy.node import Node

from datetime import datetime

from std_msgs.msg import Float32
from message_interfaces.msg import Data


class DataSubscriber(Node):
    time_format = "%Y-%m-%d %H:%M:%S%z"

    def __init__(self):
        super().__init__('data_subscriber')
        self.data_subscription = self.create_subscription(Data, 'data', self.process_data_message, 10)
        self.data_subscription
        self.previous_time = None
        self.diff_publisher = self.create_publisher(Float32, 'diff', 10)

    def process_data_message(self, msg):
        current_time = datetime.strptime(msg.time, self.time_format).timestamp()

        if self.previous_time:
            msg = Float32()
            msg.data = current_time - self.previous_time
            self.diff_publisher.publish(msg)
        self.previous_time = current_time


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)

    data_subscriber.destroy_node()
    rclpy.shutdown()
