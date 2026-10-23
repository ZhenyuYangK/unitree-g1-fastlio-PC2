"""Exercise CLI locking and durable stop confirmation with no ROS nodes."""
import contextlib
import fcntl
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from g1_task_manager.mission_cli import run_command
from g1_task_manager.waypoint_store import WaypointConfigError, WaypointStore


class MissionRecoveryTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        env = patch.dict(os.environ, G1_WS=str(self.root))
        env.start()
        self.addCleanup(env.stop)
        points = self.root / 'points.yaml'
        points.write_text(yaml.safe_dump(dict(schema_version=1, map_id='map',
            frame_id='map', base_frame='nav_base',
            waypoints={'a': dict(pose=dict(x=0., y=0., yaw=0.))})))
        self.store = WaypointStore(points).load()
        missions = self.root / 'missions.yaml'
        missions.write_text(yaml.safe_dump(dict(schema_version=1, map_id='map',
            missions={'old': dict(steps=[dict(waypoint='a')])})))
        self.args = SimpleNamespace(command='mission', mission_id='old',
            missions_file=str(missions), check_only=False, simulate=False, time_scale=1.,
            gate_service='/unused', max_tf_age=.5, max_fitness=2., max_fitness_age=5.)
        self.runtime = self.root / 'runtime/missions'
        self.runtime.mkdir(parents=True)
        self.blocker = self.runtime / 'recovery_required.json'

    def run_cli(self):
        with contextlib.redirect_stdout(io.StringIO()):
            return run_command(self.args, self.store)

    def test_same_lock_as_cruise_blocks_even_simulation(self):
        self.args.simulate = True
        with (self.runtime / 'mission.lock').open('a+') as cruise_lock:
            fcntl.flock(cruise_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(WaypointConfigError, '已有多点任务'):
                self.run_cli()

    def test_previous_recovery_marker_blocks_real_execution_without_modification(self):
        original = b'{"state":"STOP_UNCONFIRMED"}\n'
        self.blocker.write_bytes(original)
        with patch('g1_task_manager.mission_cli._run_ros') as run:
            with self.assertRaisesRegex(WaypointConfigError, '未完成收尾'):
                self.run_cli()
            run.assert_not_called()
        self.assertEqual(self.blocker.read_bytes(), original)

    def finish(self, gate, goals, state):
        transport = Mock()
        report = dict(state=state, gate_closed=gate, goals_terminal=goals,
                      elapsed_sec=.1, errors=[], steps=[])
        with patch('rclpy.init'), patch('rclpy.ok', return_value=False), patch(
                'g1_task_manager.mission_ros.create_transport', return_value=transport), patch(
                'g1_task_manager.mission_engine.execute_mission', return_value=report):
            rc = self.run_cli()
        paths = list(self.runtime.glob('*_old.json'))
        self.assertEqual(len(paths), 1)
        record = json.loads(paths[0].read_text())
        self.assertEqual(record['state'], state)
        transport.destroy.assert_called_once()
        return rc

    def test_unconfirmed_stop_keeps_marker_and_durable_report(self):
        self.assertEqual(self.finish(True, False, 'STOP_UNCONFIRMED'), 2)
        record = json.loads(self.blocker.read_text())
        self.assertEqual(record['state'], 'STOP_UNCONFIRMED')
        self.assertTrue(Path(record['report']).exists())

    def test_confirmed_cancel_persists_result_then_removes_marker(self):
        self.assertEqual(self.finish(True, True, 'CANCELED'), 130)
        self.assertFalse(self.blocker.exists())

    def test_transport_start_failure_keeps_crash_recovery_marker(self):
        with patch('rclpy.init'), patch('rclpy.ok', return_value=False), patch(
                'g1_task_manager.mission_ros.create_transport', side_effect=RuntimeError('unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'unavailable'):
                self.run_cli()
        self.assertTrue(self.blocker.exists())
        self.assertEqual(json.loads(self.blocker.read_text())['state'], 'RUNNING')


if __name__ == '__main__':
    unittest.main()
