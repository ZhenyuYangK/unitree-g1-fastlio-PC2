"""Exercise the real recorder/task health methods with live-looking TF snapshots.

No ROS node or motion backend is started; transport calls are mocks. Requires ROS
Python messages to import the consumers. Pure contract tests live alongside this.
"""
import json
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock, patch

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from g1_task_manager.nav2_client import Nav2WaypointClient, Nav2NavigationError
from g1_task_manager.waypoint_recorder import WaypointRecorderNode, WaypointRecordError
from g1_task_manager.mission_ros import MissionTransport
from g1_task_manager.mission_engine import execute_mission


class LocalizationConsumersTest(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.status = dict(state='TRACKING', valid=True, stamp_ns=self.now,
                           last_success_stamp_ns=self.now - 200_000_000)
        transform = TransformStamped()
        transform.header.stamp.sec = 10
        transform.transform.rotation.w = 1.
        self.snapshot = SimpleNamespace(
            localization_status=self.status, localization_status_time=time.monotonic(),
            latest_fitness=.1, latest_fitness_time=time.monotonic(),
            tf_buffer=SimpleNamespace(lookup_transform=Mock(return_value=transform)),
            global_frame='map', base_frame='nav_base',
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now)))

    def check_consumers(self, healthy):
        if healthy:
            Nav2WaypointClient.check_health(self.snapshot)
            WaypointRecorderNode._check_fitness(self.snapshot, 2., 15.)
        else:
            with self.assertRaises(Nav2NavigationError):
                Nav2WaypointClient.check_health(self.snapshot)
            with self.assertRaises(WaypointRecordError):
                WaypointRecorderNode._check_fitness(self.snapshot, 2., 15.)

    def test_fresh_confirmed_status_allows_both_consumers(self):
        self.check_consumers(True)

    def test_fresh_tf_and_fitness_cannot_hide_old_sensor_evidence(self):
        self.status['last_success_stamp_ns'] = self.now - 4_000_000_000
        self.check_consumers(False)

    def test_long_legacy_fitness_age_does_not_relax_status_watchdog(self):
        self.snapshot.localization_status_time = time.monotonic() - 1.1
        self.check_consumers(False)

    def test_lost_verifying_and_false_valid_all_block(self):
        for state in ('WAITING_INITIAL_POSE', 'VERIFYING', 'LOST', 'FAILED'):
            self.status['state'] = state
            self.check_consumers(False)
        self.status['state'] = 'TRACKING'
        self.status['valid'] = False
        self.check_consumers(False)

    def test_malformed_callback_replaces_previous_valid_status(self):
        for method in (Nav2WaypointClient._status_callback, WaypointRecorderNode._status_callback):
            method(self.snapshot, String(data=json.dumps(self.status)))
            self.check_consumers(True)
            method(self.snapshot, String(data='{'))
            self.assertIsNone(self.snapshot.localization_status)
            self.check_consumers(False)

    def test_mission_checks_again_after_goal_acceptance_before_opening(self):
        transport = MissionTransport.__new__(MissionTransport)
        transport.node = SimpleNamespace(check_health=Mock(side_effect=Nav2NavigationError('LOST')))
        transport.health_limits = {}
        transport.result_future = None
        transport.monitoring = False
        transport._gate = Mock()
        with self.assertRaises(Nav2NavigationError):
            transport.open_gate()
        transport._gate.assert_not_called()
        self.assertFalse(transport.monitoring)

    def test_loss_during_mission_monitoring_stops_execution(self):
        transport = MissionTransport.__new__(MissionTransport)
        transport.node = SimpleNamespace(check_health=Mock(side_effect=Nav2NavigationError('LOST')))
        transport.monitoring = True
        transport.cleanup = False
        transport.health_limits = {}
        with patch('rclpy.ok', return_value=True), patch('rclpy.spin_once'):
            with self.assertRaises(Nav2NavigationError):
                transport._spin(.1)

    def test_failed_localization_preserves_cleanup_and_terminal_requirement(self):
        transport = Mock()
        transport.ready.side_effect = Nav2NavigationError('LOST')
        transport.cancel_pending.return_value = False
        mission = dict(id='p2_test', steps=[dict(waypoint='a', timeout_sec=1., dwell_sec=0.)])
        report = execute_mission(mission, transport, lambda message: None)
        self.assertEqual(report['state'], 'STOP_UNCONFIRMED')
        self.assertTrue(report['gate_closed'])
        self.assertFalse(report['goals_terminal'])
        transport.open_gate.assert_not_called()
        transport.prepare_goal.assert_not_called()
        transport.begin_cleanup.assert_called_once()
        transport.cancel_pending.assert_called_once()


if __name__ == '__main__':
    unittest.main()
