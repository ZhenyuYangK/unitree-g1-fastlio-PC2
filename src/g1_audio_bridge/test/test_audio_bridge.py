#!/usr/bin/env python3
"""Exercise the real bundled AudioClient against a local mock, without hardware.

Run from a sourced workspace. Transport and logs are isolated; temporary logs
are removed even when a test fails. No native Unitree SDK or robot is required.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest


parser = argparse.ArgumentParser()
parser.add_argument("--bridge", required=True)
args, remaining = parser.parse_known_args()
BRIDGE = str(Path(args.bridge).resolve())
os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 60)
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ.pop("CYCLONEDDS_URI", None)

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from g1_task_interfaces.action import Speak  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from unitree_api.msg import Request, Response  # noqa: E402


class AudioBridgeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="g1_audio_test_")
        self.addCleanup(self.cleanup)
        os.environ["ROS_LOG_DIR"] = self.temp.name
        rclpy.init()
        self.node = rclpy.create_node("audio_bridge_test")
        self.client = ActionClient(self.node, Speak, "/g1_audio/speak")
        self.responses = self.node.create_publisher(Response, "/api/voice/response", 10)
        self.subscription = self.node.create_subscription(
            Request, "/api/voice/request", self.respond, 10)
        self.requests = []
        self.held = []
        self.hold_api = set()
        self.failure_api = {}
        self.logs = []
        self.process = None
        self.reader = None

    def cleanup(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=3)
            self.reader.join(timeout=2)
            self.process.stdout.close()
        if hasattr(self, "client"):
            self.client.destroy()
        if hasattr(self, "node"):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.temp.cleanup()

    def start(self, *, dry_run=None, volume=None):
        command = [BRIDGE, "--ros-args", "-p", "app_name:=test_tour"]
        if dry_run is not None:
            command += ["-p", "dry_run:=" + str(dry_run).lower()]
        if volume is not None:
            command += ["-p", "volume:=" + str(volume)]
        self.process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.reader = threading.Thread(target=self.read_logs, daemon=True)
        self.reader.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=8), "".join(self.logs))
        self.pump(0.3)

    def read_logs(self):
        for line in self.process.stdout:
            self.logs.append(line)

    def respond(self, request):
        api = request.header.identity.api_id
        self.requests.append((api, json.loads(request.parameter or "{}"), time.monotonic()))
        response = Response()
        response.header.identity = request.header.identity
        response.header.status.code = self.failure_api.get(api, 0)
        response.data = json.dumps({"volume": 42} if api == 1005 else {})
        if api in self.hold_api:
            self.held.append(response)
        else:
            self.responses.publish(response)

    def release(self):
        for response in self.held:
            self.responses.publish(response)
        self.held.clear()

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.02, end - time.monotonic()))

    def until(self, predicate, timeout=8):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(predicate(), "Timed out; bridge output:\n" + "".join(self.logs))

    def future(self, future, timeout=8):
        self.until(future.done, timeout)
        return future.result()

    def goal(self, text="测试讲解", speaker_id=0, wait_sec=0.25, feedback=None):
        message = Speak.Goal(text=text, speaker_id=speaker_id, wait_sec=wait_sec)
        return self.future(self.client.send_goal_async(message, feedback_callback=feedback))

    def api_ids(self):
        return [entry[0] for entry in self.requests]

    def test_default_dry_run_waits_without_robot_requests(self):
        self.start()
        feedback = []
        start = time.monotonic()
        handle = self.goal(wait_sec=0.35, feedback=lambda msg: feedback.append(msg.feedback))
        self.assertTrue(handle.accepted)
        result = self.future(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.result.success)
        self.assertGreaterEqual(time.monotonic() - start, 0.34)
        self.assertEqual(self.requests, [])
        self.assertIn("DRY_RUN_WAITING", [item.state for item in feedback])
        self.assertIn("not confirmed", result.result.message)
        # Bilingual missions submit the next language immediately on success.
        for speaker_id in [0, 1] * 4:
            next_goal = self.goal(speaker_id=speaker_id, wait_sec=0.001)
            self.assertTrue(next_goal.accepted)
            self.assertEqual(self.future(next_goal.get_result_async()).status,
                             GoalStatus.STATUS_SUCCEEDED)

    def test_real_client_volume_and_wait_starts_after_tts_response(self):
        self.start(dry_run=False, volume=63)
        self.hold_api.add(1001)
        handle = self.goal(text="Welcome to the lab.", speaker_id=1, wait_sec=0.3)
        result_future = handle.get_result_async()
        self.until(lambda: bool(self.held))
        self.pump(0.35)
        self.assertFalse(result_future.done())
        acknowledged = time.monotonic()
        self.release()
        result = self.future(result_future)
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreaterEqual(time.monotonic() - acknowledged, 0.29)
        self.assertEqual(self.api_ids(), [1005, 1006, 1001])
        self.assertEqual(self.requests[1][1], {"volume": 63})
        self.assertEqual(self.requests[2][1]["speaker_id"], 1)
        self.assertEqual(self.requests[2][1]["text"], "Welcome to the lab.")

    def test_busy_rejected_cancel_wait_and_next_goal(self):
        self.start(dry_run=False)
        handle = self.goal(wait_sec=30.0)
        self.until(lambda: 1001 in self.api_ids())
        self.assertFalse(self.goal().accepted)
        cancel = self.future(handle.cancel_goal_async())
        self.assertEqual(len(cancel.goals_canceling), 1)
        result = self.future(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(result.result.success)
        self.assertEqual(self.api_ids(), [1005, 1001, 1004])
        self.assertEqual(self.requests[-1][1], {"app_name": "test_tour"})
        next_goal = self.goal(speaker_id=1)
        self.assertTrue(next_goal.accepted)
        self.assertEqual(self.future(next_goal.get_result_async()).status,
                         GoalStatus.STATUS_SUCCEEDED)

    def test_cancel_during_sdk_call_does_not_block_action_callbacks(self):
        self.start(dry_run=False)
        self.hold_api.add(1001)
        handle = self.goal(wait_sec=30.0)
        self.until(lambda: bool(self.held))
        start = time.monotonic()
        cancel = self.future(handle.cancel_goal_async(), timeout=1)
        self.assertEqual(len(cancel.goals_canceling), 1)
        self.assertLess(time.monotonic() - start, 0.8)
        self.assertFalse(self.goal().accepted)
        self.release()
        result = self.future(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(self.api_ids(), [1005, 1001, 1004])

    def test_stop_timeout_keeps_ros_callbacks_responsive(self):
        self.start(dry_run=False)
        self.hold_api.add(1004)
        handle = self.goal(wait_sec=30.0)
        self.until(lambda: 1001 in self.api_ids())
        self.future(handle.cancel_goal_async(), timeout=1)
        self.until(lambda: 1004 in self.api_ids())
        start = time.monotonic()
        self.assertFalse(self.goal().accepted)
        self.assertLess(time.monotonic() - start, 0.8)
        self.assertEqual(self.future(handle.get_result_async()).status,
                         GoalStatus.STATUS_CANCELED)
        self.assertIn("PlayStop failed", "".join(self.logs))

    def test_tts_failure_aborts_and_requests_stop(self):
        self.start(dry_run=False)
        self.failure_api[1001] = 4321
        handle = self.goal()
        result = self.future(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)
        self.assertIn("4321", result.result.message)
        self.assertEqual(self.api_ids(), [1005, 1001, 1004])

    def test_volume_failure_aborts_before_tts(self):
        self.start(dry_run=False, volume=50)
        self.failure_api[1006] = 1234
        result = self.future(self.goal().get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertIn("SetVolume failed", result.result.message)
        self.assertEqual(self.api_ids(), [1005, 1006])

    def test_invalid_goals_are_rejected(self):
        self.start()
        for values in [{"text": " \n"}, {"speaker_id": -1}, {"wait_sec": 0.0},
                       {"wait_sec": -1.0}, {"wait_sec": float("nan")},
                       {"wait_sec": float("inf")}]:
            with self.subTest(values=values):
                self.assertFalse(self.goal(**values).accepted)


if __name__ == "__main__":
    unittest.main(argv=[__file__] + remaining)
