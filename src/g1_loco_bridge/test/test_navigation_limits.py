"""Measure configured limits and slew rates through the actual DryRunBackend."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--bridge', required=True)
ARGS, REMAINING = parser.parse_known_args()
os.environ['ROS_DOMAIN_ID'] = '175'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('CYCLONEDDS_URI', None)

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('generator', str(ROOT / 'src/g1_nav2_bringup/tools/generate_nav2_params.py'))
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class NavigationLimitsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='g1_p4_limits_')
        self.path = Path(self.directory.name)
        self.process = None
        self.node = None
        self.addCleanup(self.cleanup)
        os.environ['ROS_LOG_DIR'] = str(self.path / 'ros_logs')
        self.logs = []
        self.permit = None
        self.profile = 'target' if 'target' in self._testMethodName else 'safe'
        self.cap = 0.7 if self.profile == 'target' else 0.2
        nav = generator.navigation_parameters(generator.load_yaml(ROOT / 'src/g1_nav2_bringup/config/nav2_params.yaml'), self.profile)
        config = generator.bridge_parameters(generator.load_yaml(ROOT / 'src/g1_loco_bridge/config/g1_loco_bridge_nav2_dry_run.yaml'), nav, 'dry')
        params = config['g1_cmd_vel_dry_run']['ros__parameters']
        # Standalone bench checks need no localization. P2 and the readiness
        # integration suite independently exercise the mandatory navigation guard.
        params.update(cmd_vel_topic='/limits_test/cmd', enable_service_name='/limits_test/enable',
                      require_localization=False, report_period_sec=0.05,
                      require_navigation_permit='navigation_permit' in self._testMethodName)
        (self.path / 'config.yaml').write_text(yaml.safe_dump(config))
        rclpy.init()
        self.node = rclpy.create_node('limits_test')
        self.publisher = self.node.create_publisher(Twist, '/limits_test/cmd', 10)
        self.status = self.node.create_publisher(String, '/g1_navigation/status', 1)
        self.client = self.node.create_client(SetBool, '/limits_test/enable')
        self.process = subprocess.Popen([ARGS.bridge, '--ros-args', '--params-file', str(self.path / 'config.yaml')],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.reader = threading.Thread(target=self.read_logs, daemon=True)
        self.reader.start()
        self.assertTrue(self.client.wait_for_service(timeout_sec=6), ''.join(self.logs))
        future = self.client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)

    def read_logs(self):
        for line in self.process.stdout:
            self.logs.append(line)

    def pump(self, duration, command=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.permit is not None:
                self.status.publish(String(data=json.dumps(dict(
                    schema='G1_NAVIGATION_WAIT_V1', active=True,
                    state='MOVING' if self.permit else 'BLOCKED', permit=self.permit,
                    stamp_ns=self.node.get_clock().now().nanoseconds))))
            if command is not None:
                self.publisher.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertIsNone(self.process.poll(), ''.join(self.logs))

    def samples(self, start=0):
        pattern = re.compile(r'\[([0-9]+\.[0-9]+)\].*safe_output\[x=([-0-9.]+) y=([-0-9.]+) yaw=([-0-9.]+)\]')
        return [tuple(float(v) for v in match.groups()) for line in self.logs[start:]
                for match in [pattern.search(line)] if match]

    def exercise_profile(self):
        command = Twist()
        command.linear.x = self.cap + 0.2
        command.linear.y = 0.1
        command.angular.z = 0.7
        self.pump(self.cap / 0.2 + 0.5, command)
        forward = self.samples()
        self.assertGreater(len(forward), 8, ''.join(self.logs))
        self.assertAlmostEqual(forward[-1][1], self.cap, delta=0.002)
        self.assertAlmostEqual(forward[-1][3], 0.4, delta=0.002)
        for previous, current in zip(forward, forward[1:]):
            dt = current[0] - previous[0]
            self.assertLessEqual(current[1] - previous[1], 0.2 * dt + 0.003)
            self.assertLessEqual(abs(current[3] - previous[3]), 4.0 * dt + 0.01)
        start = len(self.logs)
        command.linear.x = -0.1
        command.angular.z = -0.7
        self.pump(self.cap / 0.3 + 0.5, command)
        stopping = self.samples(start)
        self.assertTrue(stopping)
        self.assertAlmostEqual(stopping[-1][1], 0.0, delta=0.002)
        self.assertAlmostEqual(stopping[-1][3], -0.4, delta=0.002)
        for previous, current in zip(stopping, stopping[1:]):
            dt = current[0] - previous[0]
            self.assertLessEqual(previous[1] - current[1], 0.3 * dt + 0.003)
            self.assertLessEqual(abs(current[3] - previous[3]), 4.0 * dt + 0.01)
        for _, x, y, angular in self.samples():
            self.assertGreaterEqual(x, 0.0)
            self.assertLessEqual(x, self.cap + 0.001)
            self.assertEqual(y, 0.0)
            self.assertLessEqual(abs(angular), 0.401)
        self.pump(0.7)
        self.assertIn('Watchdog timeout', ''.join(self.logs))

    def test_safe_profile_limits_acceleration_deceleration_and_axis_gates(self):
        self.exercise_profile()

    def test_target_profile_limits_acceleration_deceleration_and_axis_gates(self):
        self.exercise_profile()

    def test_target_profile_fine_turning_raises_small_requests_above_stalled_band(self):
        # Exercise the generated navigation config through the actual backend.
        # In the field, 0.2 rad/s was accepted by the SDK while the robot stalled.
        # The user reports 0.25 still stalls; test the requested 0.3 minimum.
        cases = ((.06, .3), (.1, .3), (.2, .3), (.25, .3), (.29, .3),
                 (.3, .3), (.35, .35), (.4, .4), (0., 0.), (-.06, -.3),
                 (-.1, -.3), (-.2, -.3), (-.25, -.3), (-.29, -.3),
                 (-.3, -.3), (-.35, -.35), (-.4, -.4), (.04, 0.), (-.04, 0.), (0., 0.))
        for requested, expected in cases:
            with self.subTest(requested=requested):
                command = Twist()
                command.angular.z = requested
                start = len(self.logs)
                self.pump(.45, command)
                samples = self.samples(start)
                self.assertGreaterEqual(len(samples), 3, ''.join(self.logs[start:]))
                for _, x, y, yaw in samples[-3:]:
                    self.assertEqual(x, 0.)
                    self.assertEqual(y, 0.)
                    self.assertAlmostEqual(yaw, expected, delta=.002)

    def test_navigation_permit_stops_immediately_and_discards_probe_commands(self):
        command = Twist()
        command.linear.x = .2
        self.permit = True
        self.pump(1.5, command)
        self.assertGreater(self.samples()[-1][1], .15, ''.join(self.logs))
        start = len(self.logs)
        self.permit = False
        self.pump(.15, command)
        self.assertIn('zero state: navigation permit withheld or stale', ''.join(self.logs[start:]))
        self.permit = True
        start = len(self.logs)
        self.pump(.2)  # Restoring permission must not replay cached probe Twist.
        self.assertTrue(all(s[1] == 0. for s in self.samples(start)), ''.join(self.logs[start:]))
        self.pump(.4, command)
        self.assertGreater(self.samples()[-1][1], .01)
        start = len(self.logs)
        self.permit = None  # Continue fresh Twist but lose guard heartbeat.
        self.pump(.5, command)
        self.assertIn('zero state: navigation permit withheld or stale', ''.join(self.logs[start:]))
        self.permit = True
        future = self.client.call_async(SetBool.Request(data=False))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        self.assertTrue(future.result().success)
        start = len(self.logs)
        self.pump(.4, command)
        self.assertTrue(all(s[1] == 0. for s in self.samples(start)), ''.join(self.logs[start:]))

    def cleanup(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if hasattr(self, 'reader'):
            self.reader.join(timeout=2)
        if self.process:
            self.process.stdout.close()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.directory.cleanup()


if __name__ == '__main__':
    unittest.main(argv=['test_navigation_limits'] + REMAINING)
