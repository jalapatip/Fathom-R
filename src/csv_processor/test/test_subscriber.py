import os
import sys
import time
import unittest
import csv
import codecs

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

from std_msgs.msg import Float32
from message_interfaces.msg import Data

data_csv = open(R'/home/jalapatip/Downloads/Dataset.csv', 'rb')


def generate_test_description():
    file_path = os.path.dirname(__file__)
    publisher_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "csv_processor", "data_publisher.py")],
    )
    subscriber_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "csv_processor", "data_subscriber.py")],
    )
    return(
        launch.LaunchDescription([
            publisher_node,
            subscriber_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'publisher': publisher_node,
            'subscriber': subscriber_node,
        },
    )


def generate_sample_data_message():
    msg = Data()
    msg.longitude = float(-105.434187)
    msg.latitude = float(41.233283)
    msg.altitude = float(2606.7)
    msg.time = "2014-07-26 13:03:55+00:00"
    msg.actual_speed = float(2.77628018691555)
    return msg


class TestSubscriber(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_subscriber")

    def tearDown(self):
        self.node.destroy_node()

    def test_subscriber_diff_outbound(self):
        msg_rx = []
        sub = self.node.create_subscription(
            Float32,
            "diff",
            lambda message: msg_rx.append(message),
            10
        )

        try:
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msg_rx) > 2:
                    break

            self.assertGreater(2, len(msg_rx))

            data_reader = csv.reader(codecs.iterdecode(data_csv, 'utf-8'))
            data_reader.__next__()

            for msg in msg_rx:
                line = data_reader.__next__()
                self.assertEqual(msg.data, line[6])

        finally:
            self.node.destroy_subscription(sub)
