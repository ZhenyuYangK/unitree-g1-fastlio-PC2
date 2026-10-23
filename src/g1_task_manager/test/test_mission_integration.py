"""Opt-in ROS tests: source ROS/install first, then set G1_RUN_ROS_TESTS=1.

All transport activity uses simulation, a separate DDS domain, and loopback.
Temporary configs, worker reports, and ROS/process logs are removed on exit.
"""
from concurrent.futures import Future
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

import yaml


@unittest.skipUnless(os.environ.get('G1_RUN_ROS_TESTS') == '1',
                     'set G1_RUN_ROS_TESTS=1 in a sourced ROS environment')
class MissionIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='g1_mission_integration_')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.logs = []
        # Avoid the robot network and any task services in the normal domain.
        environment = {
            'G1_WS': str(self.root),
            'G1_TASK_EVENTS': '1',
            'ROS_LOG_DIR': str(self.root / 'ros_logs'),
            'ROS_DOMAIN_ID': os.environ.get('G1_TEST_ROS_DOMAIN_ID', str(140 + os.getpid() % 60)),
            # The explicit Cyclone lo interface below already limits traffic;
            # ROS_LOCALHOST_ONLY would add lo a second time in some releases.
            'ROS_LOCALHOST_ONLY': '0',
            'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
            'CYCLONEDDS_URI': (
                '<CycloneDDS><Domain><General>'
                '<NetworkInterfaceAddress>lo</NetworkInterfaceAddress>'
                '<AllowMulticast>false</AllowMulticast></General><Discovery>'
                '<ParticipantIndex>auto</ParticipantIndex>'
                '<MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>'
                '<Peers><Peer Address="127.0.0.1"/></Peers>'
                '</Discovery></Domain></CycloneDDS>'),
            'PYTHONUNBUFFERED': '1',
            'PYTHONDONTWRITEBYTECODE': '1',
        }
        environment_patch = patch.dict(os.environ, environment)
        environment_patch.start()
        self.addCleanup(environment_patch.stop)
        # Importing only here keeps ordinary, non-ROS unittest discovery usable.
        import rclpy
        from g1_task_interfaces.srv import ContinueMission, GetMissionState

        self.rclpy = rclpy
        self.ContinueMission = ContinueMission
        self.GetMissionState = GetMissionState
        rclpy.init(args=[])
        self.addCleanup(self._shutdown_ros)
        self.node = rclpy.create_node('g1_integration_' + uuid.uuid4().hex[:8])
        self.addCleanup(self.node.destroy_node)
        self.state_client = self.node.create_client(GetMissionState, '/g1_task/get_state')
        self.continue_client = self.node.create_client(ContinueMission, '/g1_task/continue')
        self.points = self.root / 'points.yaml'
        self.missions = self.root / 'missions.yaml'
        self._write_config()

    def _shutdown_ros(self):
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def _write_config(self, speech_wait=1.0):
        self.points.write_text(yaml.safe_dump({
            'schema_version': 1, 'map_id': 'offline_map',
            'frame_id': 'map', 'base_frame': 'nav_base',
            'waypoints': {
                'point_{:02d}'.format(index): {
                    'pose': {'x': float(index), 'y': 0.0, 'yaw': 0.0},
                } for index in (1, 2)
            },
        }), encoding='utf-8')
        self.missions.write_text(yaml.safe_dump({
            'schema_version': 1, 'map_id': 'offline_map',
            'missions': {'offline_tour': {'name': '离线讲解测试', 'steps': [
                {
                    'waypoint': 'point_{:02d}'.format(index),
                    'timeout_sec': 2.0, 'dwell_sec': 0.0, 'advance_mode': 'manual',
                    'speech': {
                        'mode': 'tts', 'language': 'both', 'failure_policy': 'abort',
                        'content': {
                            'zh': {'speaker_id': 0, 'text': '中文{}'.format(index),
                                   'wait_sec': speech_wait},
                            'en': {'speaker_id': 1, 'text': 'English {}'.format(index),
                                   'wait_sec': speech_wait},
                        },
                    },
                } for index in (1, 2)
            ]}},
        }, allow_unicode=True), encoding='utf-8')

    def _stop_process(self, process):
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if process.poll() is not None:
                return
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=3.0)
                return
            except subprocess.TimeoutExpired:
                pass
        self.fail('test subprocess failed to stop')

    def _spawn(self, command):
        path = self.root / ('process_{}.log'.format(len(self.logs)))
        self.logs.append(path)
        stream = path.open('wb')
        self.addCleanup(stream.close)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   env=dict(os.environ), start_new_session=True)
        self.addCleanup(self._stop_process, process)
        return process, path

    def _diagnostics(self):
        return '\n'.join(path.read_text(encoding='utf-8', errors='replace')[-12000:]
                         for path in self.logs)

    def _future(self, future, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(future.done(), 'ROS future timed out\n' + self._diagnostics())
        return future.result()

    def _service(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), self._diagnostics())
        return self._future(client.call_async(request), timeout=5.0)

    def _state(self):
        return self._service(self.state_client, self.GetMissionState.Request())

    def _continue(self, state, run_id=None, wait_id=None):
        return self._service(self.continue_client, self.ContinueMission.Request(
            run_id=state.run_id if run_id is None else run_id,
            wait_id=state.wait_id if wait_id is None else wait_id))

    def _wait_state(self, state, step, process, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(process.poll(), 'worker exited early\n' + self._diagnostics())
            current = self._state()
            if current.state == state and current.current_step == step:
                return current
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        self.fail('timed out waiting for {} at station {}\n{}'.format(
            state, step, self._diagnostics()))

    def _start_mission(self):
        return self._spawn([
            '/usr/bin/python3', '-m', 'g1_task_manager.task_cli', '--file', str(self.points),
            'mission', 'offline_tour', '--missions-file', str(self.missions),
            '--simulate', '--time-scale', '0.01'])

    def _control_cli(self, command):
        process, path = self._spawn([
            '/usr/bin/python3', '-m', 'g1_task_manager.task_cli',
            '--file', str(self.points), command])
        code = process.wait(timeout=10.0)
        return code, path.read_text(encoding='utf-8')

    def _events(self, path):
        prefix = 'G1_TASK_EVENT '
        return [json.loads(line[len(prefix):])
                for line in path.read_text(encoding='utf-8').splitlines()
                if line.startswith(prefix)]

    def test_terminal_services_and_cli_release_exactly_one_station(self):
        # Keep speech active long enough to test early terminal requests.
        self._write_config(speech_wait=300.0)
        process, path = self._start_mission()
        speaking = self._wait_state('SPEAKING_ZH', 1, process)
        self.assertFalse(self._continue(speaking).accepted)
        code, output = self._control_cli('continue')
        self.assertEqual(code, 2, output)
        self.assertIn('不能放行', output)

        first = self._wait_state('WAITING_FOR_CONTINUE', 1, process)
        self.assertTrue(first.active)
        self.assertTrue(first.simulated)
        self.assertEqual(first.next_waypoint, 'point_02')
        code, output = self._control_cli('status')
        self.assertEqual(code, 0, output)
        self.assertIn('第 1 / 2 点', output)
        self.assertIn('WAITING_FOR_CONTINUE', output)
        self.assertIn('模拟演练', output)
        # Querying status or leaving the control terminal must not release it.
        unchanged = self._state()
        self.assertEqual((unchanged.current_step, unchanged.wait_id), (1, first.wait_id))
        self.assertFalse(self._continue(first, run_id='another-run').accepted)
        self.assertTrue(self._continue(first).accepted)
        self.assertFalse(self._continue(first).accepted)

        last = self._wait_state('WAITING_FOR_CONTINUE', 2, process)
        self.assertEqual(last.run_id, first.run_id)
        self.assertNotEqual(last.wait_id, first.wait_id)
        self.assertEqual(last.next_waypoint, '')
        self.assertIn('最后一站', last.message)
        self.assertFalse(self._continue(first).accepted)
        self.assertEqual(self._state().wait_id, last.wait_id)
        self.assertIsNone(process.poll())
        code, output = self._control_cli('continue')
        self.assertEqual(code, 0, output)
        self.assertIn('放行已接受', output)
        self.assertEqual(process.wait(timeout=10.0), 0, self._diagnostics())

        events = self._events(path)
        narration = [(event['current_step'], event['state']) for event in events
                     if event['kind'] == 'progress' and event['state'] in (
                         'SPEAKING_ZH', 'SPEAKING_EN', 'WAITING_FOR_CONTINUE')]
        self.assertEqual(narration, [
            (1, 'SPEAKING_ZH'), (1, 'SPEAKING_EN'), (1, 'WAITING_FOR_CONTINUE'),
            (2, 'SPEAKING_ZH'), (2, 'SPEAKING_EN'), (2, 'WAITING_FOR_CONTINUE')])
        result = [event for event in events if event['kind'] == 'result'][-1]
        self.assertEqual(result['state'], 'SUCCEEDED')
        self.assertTrue(result['gate_closed'])
        self.assertTrue(result['goals_terminal'])

    def test_sigint_while_waiting_confirms_cleanup_and_returns_130(self):
        process, path = self._start_mission()
        self._wait_state('WAITING_FOR_CONTINUE', 1, process)
        process.send_signal(signal.SIGINT)
        self.assertEqual(process.wait(timeout=10.0), 130, self._diagnostics())
        result = [event for event in self._events(path) if event['kind'] == 'result'][-1]
        self.assertEqual(result['state'], 'CANCELED')
        self.assertTrue(result['gate_closed'])
        self.assertTrue(result['goals_terminal'])
        self.assertEqual(len(result['steps']), 1)

    def test_audio_accepted_after_cancel_is_canceled_before_transport_destruction(self):
        from g1_task_manager.mission_ros import MissionTransport
        from unique_identifier_msgs.msg import UUID

        events = []
        transport = MissionTransport.__new__(MissionTransport)
        transport.cleanup = False
        transport.monitoring = True
        transport.audio_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        transport.audio_send = Future()
        transport.audio_handle = None
        # Playback completion never arrives; cancellation acknowledgement must
        # suffice without waiting for PlayStop or the original speech result.
        transport.audio_result = Future()
        transport.audio_cancel_futures = []
        before_accept = Future()
        before_accept.set_result(SimpleNamespace(goals_canceling=[]))
        after_accept = Future()
        transport.audio_cancel = SimpleNamespace(call_async=Mock(
            side_effect=lambda request: events.append('cancel_before_accept') or before_accept))
        handle = SimpleNamespace(accepted=True, cancel_goal_async=Mock(
            side_effect=lambda: events.append('cancel_after_accept') or after_accept))
        transport.audio = SimpleNamespace(destroy=lambda: events.append('destroy_audio'))
        transport.node = SimpleNamespace(
            destroy_node=lambda: events.append('destroy_node'), get_logger=lambda: Mock())
        transport.audio_send.add_done_callback(transport._audio_accepted)

        def spin(duration):
            self.assertNotIn('destroy_node', events)
            if not transport.audio_send.done():
                events.append('accept')
                transport.audio_send.set_result(handle)
            elif not after_accept.done():
                events.append('cancel_ack')
                after_accept.set_result(SimpleNamespace(goals_canceling=[object()]))
            else:
                self.fail('audio cleanup spun after receiving its cancellation acknowledgement')

        transport._spin = spin
        transport.begin_cleanup()
        self.assertTrue(transport.cleanup)
        self.assertFalse(transport.monitoring)
        self.assertFalse(transport.audio_send.done())
        request = transport.audio_cancel.call_async.call_args.args[0]
        self.assertEqual(request.goal_info.goal_id, transport.audio_goal_id)
        self.assertEqual(transport.finish_cleanup(), [])
        self.assertFalse(transport.audio_result.done())
        self.assertEqual(handle.cancel_goal_async.call_count, 1)
        transport.destroy()
        self.assertEqual(events, ['cancel_before_accept', 'accept', 'cancel_after_accept',
                                  'cancel_ack', 'destroy_audio', 'destroy_node'])

    def _audio_fixture(self, dry_run=False, name='g1_audio_bridge', action=True):
        from rclpy.action import ActionClient, ActionServer
        from rclpy.executors import SingleThreadedExecutor
        from g1_task_interfaces.action import Speak

        bridge = self.rclpy.create_node(name)
        self.addCleanup(bridge.destroy_node)
        bridge.declare_parameter('dry_run', dry_run)

        def unexpected_speech(_):
            self.fail('readiness checking submitted a speech request')

        if action:
            server = ActionServer(bridge, Speak, '/g1_audio/speak', unexpected_speech)
            self.addCleanup(server.destroy)
        audio = ActionClient(self.node, Speak, '/g1_audio/speak')
        self.addCleanup(audio.destroy)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.add_node(bridge)
        self.addCleanup(executor.shutdown)
        return audio, lambda seconds: executor.spin_once(timeout_sec=seconds)

    def test_audio_preflight_accepts_real_mode_without_playing_any_sound(self):
        from g1_task_manager.audio_readiness import wait_for_audio
        from g1_task_manager.mission_ros import MissionTransport

        audio, spin = self._audio_fixture()
        wait_for_audio(self.node, audio, spin, timeout=3.0)
        transport = MissionTransport.__new__(MissionTransport)
        transport.node, transport.audio, transport._spin = self.node, audio, spin
        transport.preflight({'steps': [{'speech': {'language': 'zh'}}]})
        # A legacy mission must not require any audio services.
        with patch('g1_task_manager.mission_ros.wait_for_audio') as check:
            transport.preflight({'steps': [{}]})
        check.assert_not_called()

    def test_audio_preflight_rejects_dry_run(self):
        from g1_task_manager.audio_readiness import AudioReadinessError, wait_for_audio

        audio, spin = self._audio_fixture(dry_run=True)
        with self.assertRaisesRegex(AudioReadinessError, 'dry_run=true'):
            wait_for_audio(self.node, audio, spin, timeout=3.0)

    def test_audio_probe_distinguishes_absence_from_incomplete_bridge(self):
        from rclpy.action import ActionClient
        from g1_task_interfaces.action import Speak
        from g1_task_manager.audio_readiness import AudioAbsent, AudioReadinessError, wait_for_audio

        audio = ActionClient(self.node, Speak, '/g1_audio/speak')
        self.addCleanup(audio.destroy)
        with self.assertRaises(AudioAbsent):
            wait_for_audio(self.node, audio,
                           lambda seconds: self.rclpy.spin_once(self.node, timeout_sec=seconds),
                           timeout=0.2, allow_absent=True)
        audio, spin = self._audio_fixture(action=False)
        with self.assertRaises(AudioReadinessError) as error:
            wait_for_audio(self.node, audio, spin, timeout=0.4, allow_absent=True)
        self.assertNotIsInstance(error.exception, AudioAbsent)

    def test_audio_probe_rejects_an_action_owned_by_an_unexpected_node(self):
        from g1_task_manager.audio_readiness import AudioAbsent, AudioReadinessError, wait_for_audio

        audio, spin = self._audio_fixture(name='unexpected_audio')
        with self.assertRaises(AudioReadinessError) as error:
            wait_for_audio(self.node, audio, spin, timeout=0.4, allow_absent=True)
        self.assertNotIsInstance(error.exception, AudioAbsent)

    def test_audio_preflight_rejects_duplicate_bridge_nodes(self):
        from g1_task_manager.audio_readiness import AudioReadinessError, wait_for_audio

        audio, spin = self._audio_fixture()
        duplicate = self.rclpy.create_node('g1_audio_bridge', enable_rosout=False)
        self.addCleanup(duplicate.destroy_node)
        with self.assertRaisesRegex(AudioReadinessError, '多个'):
            wait_for_audio(self.node, audio, spin, timeout=3.0)

    def _start_action_server(self, simulated_worker=False):
        from g1_task_interfaces.action import ExecuteMission
        from rclpy.action import ActionClient

        arguments = [
            '/usr/bin/python3', '-m', 'g1_task_manager.g1_task_manager_node', '--ros-args',
            '-p', 'waypoints_file:=' + str(self.points),
            '-p', 'missions_file:=' + str(self.missions),
        ]
        if simulated_worker:
            # Exercise the EXECUTE orchestration while the test shim forces its
            # worker onto the official simulation transport before CLI parsing.
            wrapper = self.root / 'simulated_taskctl'
            wrapper.write_text(
                '#!/usr/bin/python3\n'
                'import sys\n'
                'from g1_task_manager.task_cli import main\n'
                'args = sys.argv[1:]\n'
                'if "--execute" in args:\n'
                '    args[args.index("--execute")] = "--simulate"\n'
                '    args.extend(["--time-scale", "0.01"])\n'
                'raise SystemExit(main(args))\n', encoding='utf-8')
            wrapper.chmod(0o700)
            arguments.extend(['-p', 'execute_real:=true',
                              '-p', 'taskctl_executable:=' + str(wrapper)])
        process, _ = self._spawn(arguments)
        client = ActionClient(self.node, ExecuteMission, '/g1_task/execute_mission')
        self.addCleanup(client.destroy)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            self.assertIsNone(process.poll(), self._diagnostics())
            if client.wait_for_server(timeout_sec=0.1):
                return process, client, ExecuteMission
        self.fail('task Action server not discovered\n' + self._diagnostics())

    def test_action_preview_accepts_valid_mission_and_rejects_unknown_mission(self):
        _, client, action = self._start_action_server()
        rejected = self._future(client.send_goal_async(action.Goal(mission_id='missing')))
        self.assertFalse(rejected.accepted)
        accepted = self._future(client.send_goal_async(action.Goal(mission_id='offline_tour')))
        self.assertTrue(accepted.accepted, self._diagnostics())
        reply = self._future(accepted.get_result_async())
        self.assertEqual(reply.status, 4)
        self.assertTrue(reply.result.success)
        self.assertEqual(reply.result.state, 'CHECKED')
        self.assertFalse(reply.result.gate_closed)
        self.assertFalse(reply.result.goals_terminal)

    def test_single_point_action_uses_the_same_worker_and_confirms_stop(self):
        _, client, action = self._start_action_server(simulated_worker=True)
        original = self.missions.read_bytes()
        accepted = self._future(client.send_goal_async(action.Goal(mission_id='waypoint:point_02')))
        self.assertTrue(accepted.accepted, self._diagnostics())
        reply = self._future(accepted.get_result_async())
        self.assertEqual(reply.result.state, 'SUCCEEDED', self._diagnostics())
        self.assertTrue(reply.result.gate_closed and reply.result.goals_terminal)
        self.assertEqual(self.missions.read_bytes(), original)
        identifier = str(uuid.UUID(bytes=bytes(accepted.goal_id.uuid)))
        record = json.loads((self.root / 'runtime/missions/actions' / (identifier + '.json')).read_text())
        self.assertEqual(record['goal_id'], identifier)
        self.assertEqual(record['mission_id'], 'waypoint:point_02')

    def test_action_worker_forwards_station_feedback_release_and_cancellation(self):
        process, client, action = self._start_action_server(simulated_worker=True)
        feedback = []
        handle = self._future(client.send_goal_async(
            action.Goal(mission_id='offline_tour'),
            feedback_callback=lambda message: feedback.append(message.feedback)))
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        first = self._wait_state('WAITING_FOR_CONTINUE', 1, process)
        self.assertTrue(self._continue(first).accepted)
        last = self._wait_state('WAITING_FOR_CONTINUE', 2, process)
        self.assertFalse(self._continue(first).accepted)
        self.assertFalse(result_future.done())
        self.assertTrue(self._continue(last).accepted)
        result = self._future(result_future)
        self.assertEqual(result.status, 4)
        self.assertEqual(result.result.state, 'SUCCEEDED')
        self.assertTrue(result.result.gate_closed)
        self.assertTrue(result.result.goals_terminal)
        waits = [item for item in feedback if item.state == 'WAITING_FOR_CONTINUE']
        self.assertEqual([item.current_step for item in waits], [1, 2])
        self.assertEqual([item.wait_id for item in waits], [first.wait_id, last.wait_id])
        self.assertTrue(all(item.simulated for item in waits))
        self.assertEqual([item.next_waypoint for item in waits], ['point_02', ''])

        # A second run must have a new token and remain cancelable via Action.
        canceled = self._future(client.send_goal_async(action.Goal(mission_id='offline_tour')))
        self.assertTrue(canceled.accepted)
        canceled_result = canceled.get_result_async()
        current = self._wait_state('WAITING_FOR_CONTINUE', 1, process)
        self.assertNotEqual(current.run_id, first.run_id)
        self.assertFalse(self._continue(first).accepted)
        cancellation = self._future(canceled.cancel_goal_async())
        self.assertTrue(cancellation.goals_canceling)
        result = self._future(canceled_result)
        self.assertEqual(result.status, 5)
        self.assertEqual(result.result.state, 'CANCELED')
        self.assertTrue(result.result.gate_closed)
        self.assertTrue(result.result.goals_terminal)


if __name__ == '__main__':
    unittest.main()
