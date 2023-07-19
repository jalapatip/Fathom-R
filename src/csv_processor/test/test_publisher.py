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

from message_interfaces.msg import Data


def generate_test_description():
    file_path = os.path.dirname(__file__)
    publisher_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "csv_processor", "data_publisher.py")],
    )
    return(
        launch.LaunchDescription([
            publisher_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'publisher': publisher_node,
        },
    )


def create_message_for_row(line):
    msg = Data()
    msg.longitude, msg.latitude, msg.altitude, msg.time, msg.actual_speed = float(line[1]), float(line[2]), float(line[3]), line[4], float(line[5])
    return msg


class TestPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_publisher")

    def tearDown(self):
        self.node.destroy_node()

    def test_publisher_data_outboud(self):
        msg_rx = []
        sub = self.node.create_subscription(
            Data,
            "data",
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

            data_csv = open(R'/home/jalapatip/Downloads/Dataset.csv', 'rb')
            data_reader = csv.reader(codecs.iterdecode(data_csv, 'utf-8'))
            data_reader.__next__()

            for msg in msg_rx:
                line = data_reader.__next__()
                self.assertEqual(msg, create_message_for_row(line))

        finally:
            self.node.destroy_subscription(sub)
