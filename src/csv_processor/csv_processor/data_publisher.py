import rclpy
from rclpy.node import Node

from os import path
from ament_index_python.packages import get_package_share_directory

import csv
import codecs

from message_interfaces.msg import Data


class DataPublisher(Node):

    def __init__(self):
        super().__init__('data_publisher')
        self.data_publisher = self.create_publisher(Data, 'data', 10)
        file_path = path.join(get_package_share_directory('csv_processor'), 'Dataset.csv')
        self.data_csv = open(file_path, 'rb')
        # Using csv_reader to stream by line for better memory efficiency
        self.data_reader = csv.reader(codecs.iterdecode(self.data_csv, 'utf-8'))
        self.header = self.data_reader.__next__()
        self.column_length = len(self.header)
        self.frequency = 1.0 / 3.0
        self.timer = self.create_timer(self.frequency, self.publish_message)

    # Avoiding exception chaining to keep the implementation more readable
    # especially given the data files is consistent
    # Using generic instead of specific exceptions for the same reason
    def line_processor(self, line):
        if len(line) != self.column_length:
            self.get_logger().error(f'Missing data in row: {line}')

        try:
            longitude = float(line[1])
            latitude = float(line[2])
            altitude = float(line[3])
            time = line[4]
            actual_speed = float(line[5])
            if time.strip() == '':
                self.get_logger().error(f'Time is not populated: {line}')
                return None
            return longitude, latitude, altitude, time, actual_speed
        except Exception as e:
            self.get_logger().error(f"Unexpected {e=}, {type(e)=}")

    def publish_message(self):
        msg = Data()
        line = self.data_reader.__next__()
        values = self.line_processor(line)
        if values:
            msg.longitude, msg.latitude, msg.altitude, msg.time, msg.actual_speed = values
            self.data_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    data_publisher = DataPublisher()

    rclpy.spin(data_publisher)

    data_publisher.destroy_node()
    rclpy.shutdown()
