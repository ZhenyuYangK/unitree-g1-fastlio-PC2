"""Exercise the installed adapter over DDS using only synthetic odometry."""
import argparse
import math
import subprocess
import tempfile
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class NavigationOdometryNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_navigation_odometry')
        cls.received = []
        cls.publisher = cls.node.create_publisher(
            Odometry, '/test/navigation_odometry/input', 10)
        cls.subscription = cls.node.create_subscription(
            Odometry, '/test/navigation_odometry/output', cls.received.append,
            qos_profile_sensor_data)
        cls.log = tempfile.TemporaryFile(mode='w+')
        cls.process = subprocess.Popen([
            NODE_EXECUTABLE, '--ros-args',
            '-p', 'input_topic:=/test/navigation_odometry/input',
            '-p', 'output_topic:=/test/navigation_odometry/output',
        ], stdout=cls.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 8.0
        while cls.publisher.get_subscription_count() == 0:
            if cls.process.poll() is not None or time.monotonic() > deadline:
                cls.log.seek(0)
                raise RuntimeError('Adapter did not start: ' + cls.log.read())
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        cls.process.terminate()
        cls.process.wait(timeout=5)
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.log.close()

    def setUp(self):
        self.spin_for(0.1)
        self.received.clear()

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def message(self):
        message = Odometry()
        message.header.frame_id = 'camera_init'
        message.header.stamp.sec = 42
        message.header.stamp.nanosec = 1234
        message.child_frame_id = 'body'
        message.pose.pose.orientation.x = 1.0
        message.pose.pose.orientation.w = 0.0
        message.pose.pose.position.x = 2.5
        message.twist.twist.linear.x = 0.7
        message.twist.twist.linear.y = -0.2
        message.twist.twist.angular.z = -0.4
        return message

    def send_and_receive(self, message):
        deadline = time.monotonic() + 3.0
        while not self.received and time.monotonic() < deadline:
            self.publisher.publish(message)
            self.spin_for(0.05)
        self.assertIsNone(self.process.poll(), 'Adapter exited')
        self.assertTrue(self.received, 'No converted odometry received')
        return self.received[-1]

    def test_forward_turn_and_sensor_stamp_reach_navigation_topic(self):
        message = self.message()
        output = self.send_and_receive(message)
        self.assertEqual(output.header, message.header)
        self.assertEqual(output.child_frame_id, 'nav_base')
        self.assertEqual(output.pose.pose.position.x, 2.5)
        self.assertAlmostEqual(abs(output.pose.pose.orientation.w), 1.0)
        self.assertAlmostEqual(output.twist.twist.linear.x, 0.7)
        self.assertAlmostEqual(output.twist.twist.linear.y, 0.2)
        self.assertAlmostEqual(output.twist.twist.angular.z, 0.4)

    def test_reverse_and_opposite_turn_keep_their_signs(self):
        message = self.message()
        message.twist.twist.linear.x = -0.3
        message.twist.twist.angular.z = 0.4
        output = self.send_and_receive(message)
        self.assertAlmostEqual(output.twist.twist.linear.x, -0.3)
        self.assertAlmostEqual(output.twist.twist.angular.z, -0.4)

    def test_invalid_frame_and_nonfinite_velocity_are_dropped(self):
        for kind in ('frame', 'velocity', 'orientation'):
            message = self.message()
            if kind == 'frame':
                message.child_frame_id = 'wrong_frame'
            elif kind == 'velocity':
                message.twist.twist.linear.x = math.nan
            else:
                message.pose.pose.orientation.x = 0.0
            self.publisher.publish(message)
            self.spin_for(0.2)
            self.assertFalse(self.received, kind)
        self.assertIsNone(self.process.poll())

    def test_stopping_input_does_not_republish_old_feedback(self):
        self.send_and_receive(self.message())
        self.spin_for(0.1)
        self.received.clear()
        self.spin_for(0.3)
        self.assertFalse(self.received)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True)
    args, remaining = parser.parse_known_args()
    NODE_EXECUTABLE = args.node
    unittest.main(argv=['test_navigation_odometry_node'] + remaining)
