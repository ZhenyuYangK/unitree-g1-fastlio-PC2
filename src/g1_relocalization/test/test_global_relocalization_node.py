#!/usr/bin/env python3
"""Exercise the real localization node with independent synthetic sensor data.

Only a relocalization process runs. ROS uses a private domain and localhost;
there is no driver, navigation stack, velocity publisher, or motion backend.
Run after sourcing ROS: python3 test_relocalization_node.py --node PATH
"""

import argparse
import hashlib
import json
import math
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
parser.add_argument("--node", required=True)
args, unittest_args = parser.parse_known_args()
EXECUTABLE = str(Path(args.node).resolve())
# Configure transport before importing ROS, including the child process.
os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 40)
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ.pop("CYCLONEDDS_URI", None)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy, QoSProfile, QoSReliabilityPolicy as ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Float64, String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


YAW = math.radians(137.0)
POSITION = np.array([2.1, -1.2, 0.0])
ODOM_POSITION = np.array([0.8, -0.45, 0.12])
RZ = np.array([[math.cos(YAW), -math.sin(YAW), 0.0],
               [math.sin(YAW), math.cos(YAW), 0.0], [0.0, 0.0, 1.0]])
BODY_ROLL = np.diag([1.0, -1.0, -1.0])


def room(sample_offset=0.0):
    """Asymmetric room and two different interior structures, sampled as walls."""
    segments = [(-4, -3, 5, -3), (-4, -3, -4, 4), (-4, 4, 1, 4),
                (1, 4, 1, 1), (1, 1, 5, 1), (5, 1, 5, -3),
                (-1.5, -1, -0.5, -1), (-1.5, -1, -1.5, 0.5),
                (2.5, -1.8, 2.5, -0.4), (2.5, -0.4, 3.8, -0.4)]
    points = []
    for x0, y0, x1, y1 in segments:
        steps = math.ceil(math.hypot(x1 - x0, y1 - y0) / 0.12)
        for i in range(steps + 1):
            fraction = (i + sample_offset) / steps
            for z in np.arange(-0.6 + sample_offset * 0.05, 1.40001, 0.14):
                points.append((x0 + fraction * (x1 - x0),
                               y0 + fraction * (y1 - y0), z))
    return np.asarray(points, dtype=np.float32)


def feature(sample_offset=0.0):
    points = []
    for x in [-.2, 3.5]:
        for y in np.arange(-2.6 + sample_offset * .05, 3.2, .10):
            for z in np.arange(-.55, 1.35, .13):
                points.append((x, y, z))
    return np.asarray(points, dtype=np.float32)


def independent_scan(sample_offset, unique_feature=False):
    # Samples differ from the map; remove observations and add sensor noise.
    source = room(sample_offset)
    if unique_feature:
        source = np.concatenate((source, feature(sample_offset)))
    points = (source - POSITION) @ (RZ @ BODY_ROLL)
    indices = np.arange(len(points))
    points = points[indices % 7 != 0].copy()
    indices = indices[indices % 7 != 0]
    points[:, 0] += 0.012 * np.sin(indices * 0.61 + sample_offset * 17)
    points[:, 1] += 0.009 * np.cos(indices * 0.37 + sample_offset * 19)
    points[:, 2] += 0.008 * np.sin(indices * 0.29 + sample_offset * 11)
    return np.column_stack((points, np.ones(len(points)))).astype("<f4").tobytes()


def write_pcd(path, points):
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
              "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(points)}\nDATA ascii\n")
    with open(path, "w", encoding="utf-8") as output:
        output.write(header)
        for point in points:
            output.write(f"{point[0]:.7f} {point[1]:.7f} {point[2]:.7f} 1\n")


class RelocalizationNodeTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="g1_localization_test_")
        self.addCleanup(self.cleanup)
        os.environ["ROS_LOG_DIR"] = self.directory.name
        rclpy.init()
        self.node = rclpy.create_node("g1_relocalization_integration_test")
        self.statuses, self.poses, self.fitnesses, self.transforms = [], [], [], []
        self.node.create_subscription(
            String, "/relocalization_test/status", self.receive_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(PoseWithCovarianceStamped,
                                      "/relocalization_test/pose", self.poses.append, 10)
        self.node.create_subscription(Float64, "/relocalization_test/fitness",
                                      self.fitnesses.append, 10)
        self.node.create_subscription(TFMessage, "/relocalization_test/tf",
                                      self.receive_tf, 10)
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, "/relocalization_test/cloud", sensor_qos)
        self.odom_pub = self.node.create_publisher(
            Odometry, "/relocalization_test/odom", sensor_qos)
        self.initial_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/relocalization_test/initialpose", 10)
        self.logs, self.logs_lock = [], threading.Lock()
        self.sequence = 0
        self.last_cloud = None
        self.scan_data = [independent_scan(0.41 + i * 0.07) for i in range(5)]

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
        self.directory.cleanup()

    def receive_status(self, message):
        value = json.loads(message.data)
        self.assertIs(type(value.get("valid")), bool, message.data)
        self.assertIs(type(value.get("stamp_ns")), int, message.data)
        json.dumps(value, allow_nan=False)
        self.statuses.append(value)

    def receive_tf(self, message):
        self.transforms.extend(message.transforms)

    def log_text(self):
        with self.logs_lock:
            return "".join(self.logs)

    def read_logs(self):
        for line in self.process.stdout:
            with self.logs_lock:
                self.logs.append(line)

    def launch(self, mode="global", duplicate=False, unique_feature=False, **overrides):
        points = room()
        if duplicate:
            points = np.concatenate((points, points + np.array([14.0, 0.0, 0.0])))
        if unique_feature:
            points = np.concatenate((points, feature()))
        map_path = Path(self.directory.name) / "map.pcd"
        write_pcd(map_path, points)
        parameters = {
            "map_path": str(map_path), "initialization_mode": mode,
            "input_cloud_topic": "/relocalization_test/cloud",
            "odometry_topic": "/relocalization_test/odom",
            "initial_pose_topic": "/relocalization_test/initialpose",
            "global_use_search_bounds": True, "global_use_ground_plane": False,
            "global_min_x": -5.0, "global_max_x": 20.0 if duplicate else 6.0,
            "global_min_y": -4.0, "global_max_y": 5.0,
            "global_coarse_yaw_step_deg": 15.0,
            "global_retained_modes": 64 if duplicate else 40,
            "global_ndt_candidates": 16 if duplicate else 10,
            "global_unresolved_score_gap": 0.015,
            "scan_voxel_size": 0.18, "ndt_resolution": 0.8,
            "global_max_fitness": 0.4, "global_collection_sec": 0.25,
            "global_min_scan_count": 3, "global_search_timeout_sec": 10.0,
            "verification_timeout_sec": 5.0, "confirmation_scan_count": 3,
            "global_initialization_timeout_sec": 15.0,
            "diagnostics_directory": self.directory.name,
            "localization_period_ms": 100, "max_scan_age_sec": 0.8,
            "localization_valid_timeout_sec": 1.2,
        }
        parameters.update(overrides)
        command = [EXECUTABLE, "--ros-args"]
        for key, value in parameters.items():
            if isinstance(value, bool):
                value = str(value).lower()
            command.extend(("-p", f"{key}:={value}"))
        for before, after in (("/g1_relocalization/status", "/relocalization_test/status"),
                              ("/localization_pose", "/relocalization_test/pose"),
                              ("/localization_fitness", "/relocalization_test/fitness"),
                              ("/tf", "/relocalization_test/tf")):
            command.extend(("-r", f"{before}:={after}"))
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True)
        self.reader = threading.Thread(target=self.read_logs, daemon=True)
        self.reader.start()
        self.wait_for(lambda: bool(self.statuses), timeout=7)
        self.wait_for(lambda: self.cloud_pub.get_subscription_count() > 0 and
                      self.odom_pub.get_subscription_count() > 0 and
                      self.initial_pub.get_subscription_count() > 0, timeout=3)

    def publish_sensor(self, repeat=False, cloud_frame="body", odom_frame="camera_init", odom_offset=0.):
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = float(ODOM_POSITION[0] + odom_offset)
        odom.pose.pose.position.y = float(ODOM_POSITION[1])
        odom.pose.pose.position.z = float(ODOM_POSITION[2])
        odom.pose.pose.orientation.x = 1.0  # FAST-LIO body roll is 180 degrees.
        odom.pose.pose.orientation.w = 0.0
        self.odom_pub.publish(odom)
        if repeat and self.last_cloud is not None:
            self.cloud_pub.publish(self.last_cloud)
            return
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = cloud_frame
        cloud.height = 1
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32,
                                   count=1) for i, name in enumerate(("x", "y", "z", "intensity"))]
        cloud.data = self.scan_data[self.sequence % len(self.scan_data)]
        cloud.point_step = 16
        cloud.width = len(cloud.data) // cloud.point_step
        cloud.row_step = len(cloud.data)
        cloud.is_dense = True
        self.cloud_pub.publish(cloud)
        self.last_cloud = cloud
        self.sequence += 1

    def pump(self, duration, sensor=False, **sensor_options):
        deadline, next_publication = time.monotonic() + duration, 0.0
        while time.monotonic() < deadline:
            if sensor and time.monotonic() >= next_publication:
                self.publish_sensor(**sensor_options)
                next_publication = time.monotonic() + 0.07
            rclpy.spin_once(self.node, timeout_sec=0.015)
            self.assertIsNone(self.process.poll(), self.log_text())

    def wait_for(self, condition, timeout=10, sensor=False):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.pump(0.08, sensor=sensor)
        self.assertTrue(condition(), self.log_text() + "\n" + str(self.statuses[-5:]))

    def initial_pose(self, frame="map", zero_quaternion=False):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = float(POSITION[0])
        message.pose.pose.position.y = float(POSITION[1])
        if not zero_quaternion:
            message.pose.pose.orientation.z = math.sin(YAW / 2)
            message.pose.pose.orientation.w = math.cos(YAW / 2)
        else:
            message.pose.pose.orientation.w = 0.0
        self.initial_pub.publish(message)
        self.pump(0.1)

    def assert_no_localization(self):
        self.assertFalse(any(status["valid"] for status in self.statuses), self.log_text())
        self.assertFalse(self.poses, self.log_text())
        self.assertFalse(self.transforms, self.log_text())
        self.assertFalse(any(math.isfinite(value.data) for value in self.fitnesses), self.log_text())

    def assert_correct_localization(self):
        self.wait_for(lambda: self.poses and self.transforms and self.statuses[-1]["valid"],
                      sensor=True)
        status = self.statuses[-1]
        self.assertIn(status["state"], ("LOCALIZED", "TRACKING"))
        self.assertGreater(status["coverage"], 0.9)
        self.assertGreater(status["last_success_stamp_ns"], 0)
        self.assertLess((self.node.get_clock().now().nanoseconds -
                         status["last_success_stamp_ns"]) / 1e9, 0.8)
        pose = self.poses[-1]
        self.assertEqual(pose.header.frame_id, "map")
        actual = np.array([pose.pose.pose.position.x, pose.pose.pose.position.y,
                           pose.pose.pose.position.z])
        self.assertLess(np.linalg.norm(actual - POSITION), 0.2, self.log_text())
        q = pose.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.assertLess(abs(math.atan2(math.sin(yaw - YAW), math.cos(yaw - YAW))),
                        math.radians(3), self.log_text())
        tf = self.transforms[-1]
        self.assertEqual((tf.header.frame_id, tf.child_frame_id), ("map", "camera_init"))
        translation = tf.transform.translation
        expected = POSITION - RZ @ ODOM_POSITION
        self.assertLess(np.linalg.norm(np.array([translation.x, translation.y, translation.z])
                                       - expected), 0.2, self.log_text())

    def test_global_bootstrap_without_initialpose_and_repeated_scan_expiry(self):
        self.launch()
        self.pump(0.3)
        self.assertEqual(self.statuses[-1]["state"], "WAITING_DATA")
        self.assert_no_localization()
        # There is deliberately no initial_pose() call before automatic success.
        self.assert_correct_localization()
        self.assertTrue(any(status["state"] == "COLLECTING" for status in self.statuses))
        self.assertEqual(self.statuses[-1]["initialization_mode"], "global")
        records = list(Path(self.directory.name).rglob('global_*.json'))
        self.assertTrue(records)
        record = json.loads(records[0].read_text())
        session = json.loads(next(Path(self.directory.name).rglob('session.json')).read_text())
        self.assertEqual(session['map_sha256'], hashlib.sha256(
            (Path(self.directory.name) / 'map.pcd').read_bytes()).hexdigest())
        canonical = json.dumps(session['parameters'], ensure_ascii=False, separators=(',', ':'))
        self.assertEqual(session['config_sha256'], hashlib.sha256(canonical.encode()).hexdigest())
        self.assertEqual(record['map_sha256'], session['map_sha256'])
        self.assertEqual(record['config_sha256'], session['config_sha256'])
        self.assertTrue(record['candidates'])
        self.assertIn('initial_yaw', record['candidates'][0])
        self.assertTrue(records[0].with_suffix('.pcd').is_file())
        comparisons = list(Path(self.directory.name).rglob('verification_*.jsonl'))
        self.assertTrue(comparisons)
        for line in comparisons[0].read_text().splitlines():
            row = json.loads(line)
            self.assertTrue(row['candidates'])
            self.assertTrue(Path(row['input_pcd']).is_file())
        self.pump(0.3, sensor=True, repeat=True)  # Drain any earlier fresh observation.
        baseline_success = self.statuses[-1]["last_success_stamp_ns"]
        self.pump(1.6, sensor=True, repeat=True)
        self.assertEqual(self.statuses[-1]["state"], "LOST", self.log_text())
        self.assertFalse(self.statuses[-1]["valid"])
        self.assertEqual(self.statuses[-1]["last_success_stamp_ns"], baseline_success)
        tf_count, pose_count = len(self.transforms), len(self.poses)
        self.pump(0.3, sensor=True, repeat=True)
        self.assertEqual(len(self.transforms), tf_count)
        self.assertEqual(len(self.poses), pose_count)
        self.assert_correct_localization()  # Distinct new scans can reconfirm.

    def test_duplicate_rooms_fail_closed_and_manual_pose_can_resolve(self):
        self.launch(duplicate=True, verification_timeout_sec=0.7, global_max_attempts=2,
                    global_direct_accept_coverage=1.0)
        self.wait_for(lambda: self.statuses[-1]["state"] in ("AMBIGUOUS", "FAILED"),
                      sensor=True)
        self.assertEqual(self.statuses[-1]["state"], "AMBIGUOUS", self.log_text())
        self.assertTrue(any(status['state'] == 'VERIFYING_CANDIDATES' for status in self.statuses))
        self.assertTrue(any(status.get('attempt') == 2 for status in self.statuses))
        self.pump(0.4, sensor=True)
        self.assert_no_localization()
        self.initial_pose()
        self.assert_correct_localization()

    def test_ambiguous_candidates_retry_and_new_geometry_selects_correct_location(self):
        self.launch(duplicate=True, unique_feature=True, verification_timeout_sec=0.8,
                    global_max_attempts=3, global_direct_accept_coverage=1.0)
        # Initial scans omit the distinctive walls: both locations are plausible.
        self.wait_for(lambda: any(s.get('attempt', 0) >= 2 for s in self.statuses), sensor=True)
        self.assert_no_localization()
        self.scan_data = [independent_scan(.41 + i*.07, unique_feature=True) for i in range(5)]
        self.assert_correct_localization()
        self.assertIn('Verified candidate', self.log_text())
        self.assertTrue(any(len(s.get('candidates', [])) >= 2 for s in self.statuses))

    def test_total_budget_expires_during_input_outage_without_false_confirmation(self):
        self.launch(global_initialization_timeout_sec=0.8)
        self.publish_sensor()
        self.pump(1.2, sensor=True, repeat=True)
        self.assertEqual(self.statuses[-1]['state'], 'FAILED', self.log_text())
        self.assertIn('Total automatic initialization time budget', self.statuses[-1]['reason'])
        self.assert_no_localization()

    def test_budget_starts_at_first_synchronized_frame_not_node_start(self):
        self.launch(global_initialization_timeout_sec=0.5)
        self.pump(0.8)
        self.assertEqual(self.statuses[-1]['state'], 'WAITING_DATA')
        self.publish_sensor()
        self.pump(0.8, sensor=True, repeat=True)
        self.assertEqual(self.statuses[-1]['state'], 'FAILED', self.log_text())
        self.assert_no_localization()
        self.initial_pose()
        self.assert_correct_localization()

    def test_total_budget_cancels_search_without_publishing_partial_match(self):
        self.launch(global_collection_sec=0.1, global_min_scan_count=2,
                    global_initialization_timeout_sec=0.18, global_coarse_xy_step=0.1,
                    global_fine_xy_step=0.05)
        self.wait_for(lambda: self.statuses[-1]['state'] == 'FAILED', sensor=True)
        self.assertIn('Total automatic initialization time budget', self.statuses[-1]['reason'])
        self.assert_no_localization()

    def test_motion_during_stationary_collection_rejects_attempt(self):
        self.launch(global_collection_sec=1., global_max_attempts=1)
        self.publish_sensor()
        self.pump(.15)
        self.publish_sensor(odom_offset=1.)
        self.pump(.4)
        self.assertEqual(self.statuses[-1]['state'], 'FAILED', self.log_text())
        self.assertIn('moved during stationary', self.statuses[-1]['reason'])
        self.assert_no_localization()

    def test_candidate_bank_does_not_count_repeated_timestamp(self):
        self.launch(duplicate=True, unique_feature=True, global_initialization_timeout_sec=2.0,
                    global_direct_accept_coverage=1.0)
        self.wait_for(lambda: self.statuses[-1]['state'] == 'VERIFYING_CANDIDATES', sensor=True)
        self.pump(.5, sensor=True, repeat=True)
        self.assert_no_localization()
        self.wait_for(lambda: self.statuses[-1]['state'] in ['FAILED', 'AMBIGUOUS'], timeout=4)
        self.assert_no_localization()

    def test_truncated_candidate_bank_cannot_claim_uniqueness(self):
        self.launch(duplicate=True, global_verification_candidates=1,
                    verification_timeout_sec=0.7, global_max_attempts=1,
                    global_direct_accept_coverage=1.0)
        self.wait_for(lambda: self.statuses[-1]['state'] == 'AMBIGUOUS', sensor=True)
        self.assert_no_localization()

    def test_high_coverage_primary_is_accepted_despite_truncated_candidate_bank(self):
        self.launch(duplicate=True, global_verification_candidates=1)
        self.wait_for(lambda: self.poses and self.transforms and self.statuses[-1]['valid'],
                      sensor=True)
        self.assertIn('Primary candidate directly accepted', self.log_text())
        files = list(Path(self.directory.name).rglob('verification_*.jsonl'))
        self.assertTrue(files)
        rows = [json.loads(line) for line in files[-1].read_text().splitlines()]
        self.assertGreaterEqual(len(rows), 3)
        self.assertTrue(rows[-1]['omitted_candidates'])
        self.assertTrue(rows[-1]['candidates'][0]['stable'])
        coverage = sum(row['candidates'][0]['coverage'] for row in rows[-3:]) / 3
        self.assertGreater(coverage, .91)

    def test_manual_pose_interrupts_automatic_candidate_comparison(self):
        self.launch(duplicate=True, global_direct_accept_coverage=1.0)
        self.wait_for(lambda: self.statuses[-1]['state'] == 'VERIFYING_CANDIDATES', sensor=True)
        self.assert_no_localization()
        self.initial_pose()
        self.assert_correct_localization()

    def test_manual_pose_and_scans_require_correct_frames_and_distinct_timestamps(self):
        self.launch(mode="manual")
        self.pump(0.35, sensor=True, odom_frame="map")
        self.assert_no_localization()
        self.initial_pose(frame="odom")
        self.initial_pose(zero_quaternion=True)
        self.pump(0.4, sensor=True)
        self.assertEqual(self.statuses[-1]["state"], "WAITING_INITIAL_POSE")
        self.assert_no_localization()
        self.initial_pose()
        self.pump(0.35, sensor=True, cloud_frame="camera_init")
        self.assert_no_localization()
        self.publish_sensor()
        self.pump(0.45, sensor=True, repeat=True)
        self.assert_no_localization()  # One timestamp cannot satisfy three confirmations.
        self.assert_correct_localization()

    def test_verification_timeout_is_terminal_until_new_initialpose(self):
        self.launch(mode="manual", verification_timeout_sec=0.6)
        self.initial_pose()
        self.publish_sensor()
        self.pump(0.3, sensor=True, repeat=True)
        self.assertEqual(self.statuses[-1]["state"], "VERIFYING", self.log_text())
        self.pump(0.7, sensor=True, repeat=True)
        self.assertEqual(self.statuses[-1]["state"], "FAILED", self.log_text())
        self.assert_no_localization()
        self.pump(0.45, sensor=True)
        self.assertEqual(self.statuses[-1]["state"], "FAILED", self.log_text())
        self.assert_no_localization()
        self.initial_pose()
        self.assert_correct_localization()

    def test_invalid_map_returns_failure(self):
        missing = Path(self.directory.name) / "absent.pcd"
        result = subprocess.run([EXECUTABLE, "--ros-args", "-p", f"map_path:={missing}"],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Cannot load PCD map", result.stdout)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0], *unittest_args])
