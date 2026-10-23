#!/usr/bin/env python3
"""ROS integration check; starts only the non-motion DryRunBackend on private topics.

Run after sourcing ROS + workspace: python3 test_localization_guard.py --bridge PATH
The isolated ROS domain and localhost restriction keep this test off robot topics.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest


parser = argparse.ArgumentParser()
parser.add_argument("--bridge", required=True)
args, unittest_args = parser.parse_known_args()
BRIDGE = str(Path(args.bridge).resolve())
# Set transport isolation before importing/initializing ROS.
os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 60)
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ.pop("CYCLONEDDS_URI", None)

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


class LocalizationGuardTest(unittest.TestCase):
    def setUp(self):
        self.log_dir = tempfile.TemporaryDirectory(prefix="g1_guard_test_")
        self.addCleanup(self.cleanup)
        os.environ["ROS_LOG_DIR"] = self.log_dir.name
        rclpy.init()
        self.node = rclpy.create_node("g1_guard_test")
        self.status_pub = self.node.create_publisher(
            String,
            "/guard_test/status",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.twist_pub = self.node.create_publisher(Twist, "/guard_test/cmd_vel", 10)
        self.enable_client = self.node.create_client(SetBool, "/guard_test/enable")
        self.logs = []
        self.logs_lock = threading.Lock()
        guard_required = self._testMethodName != "test_optional_guard_preserves_independent_dry_run"
        self.process = subprocess.Popen(
            [BRIDGE, "--ros-args", "-p", "dry_run:=true", "-p",
             "require_localization:=" + str(guard_required).lower(),
             "-p", "localization_status_topic:=/guard_test/status",
             "-p", "cmd_vel_topic:=/guard_test/cmd_vel",
             "-p", "enable_service_name:=/guard_test/enable",
             "-p", "max_angular_z:=0.5", "-p", "report_period_sec:=0.05",
             "-p", "localization_status_timeout_sec:=0.4",
             "-p", "localization_success_timeout_sec:=0.7"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        self.reader = threading.Thread(target=self.read_logs, daemon=True)
        self.reader.start()
        self.assertTrue(self.enable_client.wait_for_service(timeout_sec=10), self.log_text())
        self.pump(0.5)
        self.assertIn("Selected backend: dry_run, real_backend=false", self.log_text())

    def cleanup(self):
        if hasattr(self, "process") and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if hasattr(self, "reader"):
            self.reader.join(timeout=2)
        if hasattr(self, "process"):
            self.process.stdout.close()
        if hasattr(self, "node"):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.log_dir.cleanup()

    def read_logs(self):
        for line in self.process.stdout:
            with self.logs_lock:
                self.logs.append(line)

    def log_text(self, start=0):
        with self.logs_lock:
            return "".join(self.logs[start:])

    def log_position(self):
        with self.logs_lock:
            return len(self.logs)

    def status(self, **changes):
        now = self.node.get_clock().now().nanoseconds
        value = {"state": "TRACKING", "valid": True, "stamp_ns": now,
                 "last_success_stamp_ns": now}
        value.update(changes)
        return json.dumps(value)

    def pump(self, duration, status_factory=None, twists=False):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if status_factory:
                self.status_pub.publish(String(data=status_factory()))
            if twists:
                message = Twist()
                message.linear.x = 0.2
                self.twist_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertIsNone(self.process.poll(), self.log_text())

    def enable(self, expected, message_part=None):
        future = self.enable_client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        self.assertTrue(future.done(), self.log_text())
        response = future.result()
        self.assertEqual(expected, response.success, response.message + "\n" + self.log_text())
        if message_part:
            self.assertIn(message_part, response.message)

    def prepare_enabled_motion(self):
        self.pump(0.1, self.status)
        self.enable(True)
        start = self.log_position()
        self.pump(0.2, self.status, twists=True)
        self.assertIn("Bridge accepted fresh Twist", self.log_text(start))
        self.assertIn("safe_output[x=", self.log_text(start))

    def assert_disabled_with_fresh_twists(self, start):
        self.pump(0.1, twists=True)
        text = self.log_text(start)
        self.assertIn("Localization guard DISABLED both safety gates", text)
        self.assertIn("DryRunBackend DISABLED and reset to zero", text)
        self.assertIn("DryRunBackend entered zero state: localization guard:", text)
        after_stop = text.split("Localization guard DISABLED both safety gates", 1)[1]
        self.assertNotIn("Bridge accepted fresh Twist", after_stop)
        self.assertNotIn("SAFETY raw[", after_stop)

    def test_enable_requires_strict_current_localization(self):
        self.enable(False, "no localization status received")
        # Each payload must independently fail; finite integer timestamps and bools
        # are required, including integer overflow and string coercion edge cases.
        invalid_payloads = [
            "{", "[]", "null", self.status(valid="true"), self.status(valid=False),
            self.status(state="LOST"), self.status(stamp_ns=1.5),
            self.status(stamp_ns=True), self.status(stamp_ns="123"),
            self.status(stamp_ns=2**64 - 1), self.status(last_success_stamp_ns=0),
            self.status(stamp_ns=self.node.get_clock().now().nanoseconds + 5_000_000_000),
            self.status(last_success_stamp_ns=self.node.get_clock().now().nanoseconds +
                        5_000_000_000),
            '{"state":"TRACKING","valid":true,"stamp_ns":1e400,"last_success_stamp_ns":1}',
        ]
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                self.pump(0.05, lambda: payload)
                self.enable(False, "localization guard")
        self.pump(0.1, lambda: self.status(state="LOCALIZED"))
        self.enable(True)

    def test_optional_guard_preserves_independent_dry_run(self):
        self.enable(True)
        start = self.log_position()
        self.pump(0.2, twists=True)
        self.assertIn("Bridge accepted fresh Twist", self.log_text(start))
        self.assertIn("SAFETY raw[", self.log_text(start))

    def test_lost_disables_and_recovery_requires_reenable_and_fresh_twist(self):
        self.prepare_enabled_motion()
        start = self.log_position()
        self.pump(0.15, lambda: self.status(state="LOST", valid=False), twists=True)
        self.assert_disabled_with_fresh_twists(start)
        start = self.log_position()
        self.pump(0.15, self.status, twists=True)
        self.assertNotIn("Bridge accepted fresh Twist", self.log_text(start))
        self.assertNotIn("SAFETY raw[", self.log_text(start))
        self.enable(True)
        start = self.log_position()
        self.pump(0.15, self.status)  # Cached Twist from before enable must not resume.
        self.assertNotIn("SAFETY raw[", self.log_text(start))
        self.pump(0.15, self.status, twists=True)
        self.assertIn("Bridge accepted fresh Twist", self.log_text(start))
        self.assertIn("SAFETY raw[", self.log_text(start))

    def test_status_receive_timeout_disables_despite_fresh_twists(self):
        self.prepare_enabled_motion()
        start = self.log_position()
        self.pump(0.55, twists=True)
        self.assert_disabled_with_fresh_twists(start)
        self.enable(False, "expired")

    def test_stale_ros_status_disables_despite_periodic_receive(self):
        self.prepare_enabled_motion()
        frozen_stamp = self.node.get_clock().now().nanoseconds
        start = self.log_position()
        self.pump(0.55, lambda: self.status(stamp_ns=frozen_stamp,
                                         last_success_stamp_ns=frozen_stamp), twists=True)
        self.assert_disabled_with_fresh_twists(start)
        self.enable(False, "ROS timestamp expired")

    def test_stale_success_scan_disables_despite_fresh_status_and_twists(self):
        self.prepare_enabled_motion()
        frozen_stamp = self.node.get_clock().now().nanoseconds
        start = self.log_position()
        self.pump(0.85, lambda: self.status(last_success_stamp_ns=frozen_stamp), twists=True)
        self.assert_disabled_with_fresh_twists(start)
        self.enable(False, "successful localization scan expired")

    def test_future_or_malformed_status_disables_active_gate(self):
        for factory in (lambda: "{", lambda: self.status(
                stamp_ns=self.node.get_clock().now().nanoseconds + 5_000_000_000)):
            self.prepare_enabled_motion()
            start = self.log_position()
            self.pump(0.1, factory, twists=True)
            self.assert_disabled_with_fresh_twists(start)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0], *unittest_args])
