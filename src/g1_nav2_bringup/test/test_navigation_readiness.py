"""DDS integration: actual adapter/DryRunBackend, synthetic lifecycle servers."""
import argparse
import importlib.util
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
parser.add_argument('--adapter', required=True)
parser.add_argument('--bridge', required=True)
ARGS, REMAINING = parser.parse_known_args()
os.environ['ROS_DOMAIN_ID'] = '174'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('CYCLONEDDS_URI', None)

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('generator', str(ROOT / 'src/g1_nav2_bringup/tools/generate_nav2_params.py'))
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class NavigationReadinessTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='g1_p4_readiness_')
        self.path = Path(self.directory.name)
        os.environ['ROS_LOG_DIR'] = str(self.path / 'ros_logs')
        self.processes = []
        self.nodes = []
        self.logs = []
        self.executor = None
        self.addCleanup(self.cleanup)
        rclpy.init()
        record = generator.generate(ROOT / 'src/g1_nav2_bringup/config/nav2_params.yaml', self.path / 'nav.yaml',
            'safe', {mode: (ROOT / ('src/g1_loco_bridge/config/g1_loco_bridge_nav2_' + suffix + '.yaml'),
                           self.path / (mode + '.yaml')) for mode, suffix in (('dry', 'dry_run'), ('real', 'real'))},
            self.path / 'manifest.json')
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.fake_nodes = {}
        for name in ('controller_server', 'planner_server', 'recoveries_server', 'bt_navigator'):
            node = rclpy.create_node(name)
            self.fake_nodes[name] = node
            for key, value in record['expected'].get(name, {}).items():
                if node.has_parameter(key):
                    node.set_parameters([Parameter(key, value=value)])
                else:
                    node.declare_parameter(key, value)
            node.create_service(GetState, '/' + name + '/get_state', self.active)
            self.nodes.append(node)
            self.executor.add_node(node)
        self.source = rclpy.create_node('p4_synthetic_source')
        self.nodes.append(self.source)
        self.executor.add_node(self.source)
        self.publisher = self.source.create_publisher(Odometry, '/Odometry', 10)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.source.create_publisher(String, '/g1_relocalization/status', qos)
        self.stale = False
        self.active_id = 3
        self.source.create_timer(0.05, self.publish)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.start([ARGS.adapter])
        # Actual backend reports must ignore even conflicting startup overrides.
        self.start([ARGS.bridge, '--ros-args', '--params-file', str(self.path / 'dry.yaml'),
                    '-p', 'real_backend:=true', '-p', 'backend_name:=incorrect'])
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() and any(
                    name == '/g1_loco_bridge/enable' for name, _ in self.source.get_service_names_and_types()):
                break
            time.sleep(0.05)

    def active(self, request, response):
        response.current_state.id = self.active_id
        response.current_state.label = 'active'
        return response

    def start(self, command):
        log = tempfile.TemporaryFile(mode='w+')
        self.logs.append(log)
        self.processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))

    def publish(self):
        now = self.source.get_clock().now()
        message = Odometry()
        message.header.frame_id = 'camera_init'
        message.child_frame_id = 'body'
        message.header.stamp = now.to_msg()
        if self.stale:
            message.header.stamp.sec -= 5
        message.pose.pose.orientation.x = 1.0
        message.pose.pose.orientation.w = 0.0
        message.twist.twist.linear.x = 0.1
        message.twist.twist.angular.z = -0.2
        self.publisher.publish(message)
        self.status.publish(String(data=json.dumps({'state': 'TRACKING', 'valid': True,
            'stamp_ns': now.nanoseconds, 'last_success_stamp_ns': now.nanoseconds})))

    def check(self, mode='dry', enable=True, timeout='5'):
        command = ['python3', str(ROOT / 'script/g1_navigation_readiness.py'),
                   '--manifest', str(self.path / 'manifest.json'), '--mode', mode, '--timeout', timeout]
        if enable:
            command.append('--enable')
        return subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=12)

    def log_text(self):
        text = ''
        for log in self.logs:
            log.seek(0)
            text += log.read()
        return text

    def test_correct_dry_backend_feedback_and_parameters_allow_enable(self):
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + self.log_text())
        self.assertIn('DryRunBackend ENABLED', self.log_text())

    def test_same_named_dry_backend_is_rejected_by_real_check(self):
        result = self.check('real')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('dry_run', result.stdout)
        self.assertNotIn('DryRunBackend ENABLED', self.log_text())

    def test_loaded_nav2_limit_mismatch_prevents_enable(self):
        self.fake_nodes['controller_server'].set_parameters([Parameter('FollowPath.max_vel_x', value=1.0)])
        result = self.check()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('FollowPath.max_vel_x', result.stdout)
        self.assertNotIn('DryRunBackend ENABLED', self.log_text())

    def test_stale_feedback_prevents_enable(self):
        self.stale = True
        time.sleep(0.2)
        result = self.check(timeout='3')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('/Odometry_nav', result.stdout)
        self.assertNotIn('DryRunBackend ENABLED', self.log_text())

    def test_inactive_nav2_prevents_enable(self):
        self.active_id = 2
        result = self.check()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('不是 active', result.stdout)
        self.assertNotIn('DryRunBackend ENABLED', self.log_text())

    def test_cached_mode_and_limit_parameters_are_read_only(self):
        client = self.source.create_client(SetParameters, '/g1_cmd_vel_dry_run/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        request = SetParameters.Request(parameters=[Parameter(name, value=value).to_parameter_msg()
            for name, value in (('dry_run', False), ('real_backend', True), ('max_linear_x', 1.0))])
        future = client.call_async(request)
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done())
        self.assertTrue(all(not result.successful for result in future.result().results))
        result = self.check(enable=False)
        self.assertEqual(result.returncode, 0, result.stdout)

    def cleanup(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        if self.executor:
            self.executor.shutdown(timeout_sec=3)
        if hasattr(self, 'thread'):
            self.thread.join(timeout=3)
        for node in self.nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in self.logs:
            log.close()
        self.directory.cleanup()


if __name__ == '__main__':
    unittest.main(argv=['test_navigation_readiness'] + REMAINING)
