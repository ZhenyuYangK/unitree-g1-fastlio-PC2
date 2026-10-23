"""Mapping integration uses disposable files and owned sleep processes, never ROS hardware."""
import asyncio
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
import uuid
from unittest.mock import patch

from fastapi.testclient import TestClient

from g1_web.app import create_app
from g1_web.cloud import MappingPreview
from g1_web.config import Settings
from g1_web.mapping import MappingManager, SCRIPT_SOURCE, write_json
from g1_web.runtime import process, group_state, pcd_info

PCD = b'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n'


class FakeRunner:
    def __init__(self, root):
        self.root = root
        self.children = []
        self.calls = []
        self.failure = ''
        self.no_update = False
        self.false_stop = False
        self.delay = 0.02

    def spawn(self, name):
        child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)'],
                                 start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(child)
        return '{} {} {}\n'.format(name, child.pid, process(child.pid)['token'])

    async def __call__(self, record):
        self.calls.append(record['action'])
        await asyncio.sleep(self.delay)
        if self.failure == record['action']:
            return 1
        runtime = self.root / 'runtime/mapping'
        runtime.mkdir(parents=True, exist_ok=True)
        if record['action'] == 'mapping.start':
            (runtime / 'mapping.pids').write_text(self.spawn('livox') + self.spawn('fast_lio'))
            write_json(runtime / 'session.json', {'id': record['session_id'], 'output': record['output'], 'started_at': time.time()})
            (runtime / 'logs').mkdir(exist_ok=True)
            (runtime / 'logs/fast_lio.log').write_text('fixture cloud ready\n')
        if record['action'] in {'mapping.save', 'mapping.finish'} and not self.no_update:
            path = Path(record['output'])
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(PCD)
        if record['action'] in {'mapping.finish', 'mapping.stop'}:
            if not self.false_stop:
                self.cleanup()
            (runtime / 'mapping.pids').unlink(missing_ok=True)
        return 0

    def cleanup(self):
        for child in self.children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=3)


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='g1-mapping-test-')
        self.root = Path(self.temp.name)
        (self.root / 'script').mkdir()
        (self.root / 'script/g1_mapping.sh').write_text('# fixture only\n')
        self.settings = Settings(workspace=str(self.root))
        self.runner = FakeRunner(self.root)
        self.manager = MappingManager(self.settings, self.runner)
        self.app = create_app(self.settings, mapping=self.manager)
        self.client = TestClient(self.app).__enter__()
        self.headers = {'X-Control-Lease': self.client.post('/api/v1/control/acquire', json={'owner': 'mapping test'}).json()['lease_id']}

    def tearDown(self):
        self.client.__exit__(None, None, None)
        self.runner.cleanup()
        self.temp.cleanup()

    def submit(self, action, name='', identifier=None, headers=None):
        return self.client.post('/api/v1/operations', headers=self.headers if headers is None else headers,
                                json={'request_id': identifier or str(uuid.uuid4()), 'operation': 'mapping.' + action, 'name': name})

    def wait(self, response, expected='SUCCEEDED'):
        self.assertEqual(response.status_code, 202, response.text)
        identifier = response.json()['operation_id']
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            result = self.client.get('/api/v1/operations/' + identifier).json()
            if result['state'] not in {'QUEUED', 'RUNNING'}:
                self.assertEqual(result['state'], expected, result)
                return result
            time.sleep(0.01)
        self.fail('mapping operation did not complete')

    def state(self):
        return self.client.get('/api/v1/mapping').json()

    def test_full_start_save_download_finish_and_navigation_map_is_untouched(self):
        original = self.root / 'maps/map_multilayer.yaml'
        original.parent.mkdir()
        original.write_text('keep production map')
        self.wait(self.submit('start', '实验室东区'))
        self.assertEqual(self.state()['phase'], 'RUNNING')
        self.assertEqual(self.submit('start').status_code, 409)
        self.wait(self.submit('save'))
        state = self.state()
        self.assertTrue(state['session']['saved'])
        self.assertEqual(self.client.get(state['session']['download_url']).content, PCD)
        self.assertIn('fixture cloud ready', self.client.get('/api/v1/mapping/logs').text)
        self.assertEqual(self.state()['phase'], 'RUNNING')
        self.wait(self.submit('finish'))
        self.assertEqual(self.state()['phase'], 'STOPPED')
        self.assertTrue(self.state()['session']['saved'])
        self.assertEqual(original.read_text(), 'keep production map')

    def test_requires_lease_valid_name_and_same_origin(self):
        self.assertEqual(self.submit('start', headers={}).status_code, 409)
        self.assertEqual(self.submit('start', '../../overwrite').status_code, 422)
        self.assertEqual(self.submit('start', 'a;touch file').status_code, 422)
        self.assertEqual(self.submit('start', headers=dict(self.headers, Origin='https://foreign.example')).status_code, 403)
        self.assertEqual(self.runner.calls, [])

    def test_async_busy_idempotency_and_durable_replay(self):
        self.runner.delay = 0.15
        identifier = str(uuid.uuid4())
        first = self.submit('start', identifier=identifier)
        repeat = self.submit('start', identifier=identifier)
        self.assertEqual(first.json(), repeat.json())
        self.assertEqual(self.submit('save', identifier=identifier).status_code, 409)
        self.assertEqual(self.submit('start').status_code, 409)
        self.wait(first)
        self.assertEqual(self.runner.calls, ['mapping.start'])
        restarted = MappingManager(self.settings, self.runner)
        replay = self.client.portal.call(restarted.submit, uuid.UUID(identifier), 'mapping.start', '')
        self.assertEqual(replay, first.json())
        self.assertEqual(self.runner.calls, ['mapping.start'])

    def test_save_failure_preserves_capture_and_old_pcd_is_not_success(self):
        self.wait(self.submit('start'))
        self.wait(self.submit('save'))
        self.runner.no_update = True
        self.wait(self.submit('save'), 'FAILED')
        self.assertEqual(self.state()['phase'], 'RUNNING')
        self.runner.failure = 'mapping.finish'
        self.wait(self.submit('finish'), 'FAILED')
        self.assertEqual(self.state()['phase'], 'RUNNING')

    def test_stop_without_save_and_false_stop_detected_even_if_pid_file_removed(self):
        self.wait(self.submit('start'))
        self.runner.false_stop = True
        self.wait(self.submit('stop'), 'FAILED')
        self.assertTrue(any(child.poll() is None for child in self.runner.children))
        self.runner.cleanup()
        self.runner.false_stop = False
        self.wait(self.submit('start'))
        self.wait(self.submit('stop'))
        self.assertFalse(self.state()['session']['saved'])

    def test_navigation_mission_and_recovery_blocks_start(self):
        path = self.root / 'runtime/navigation/navigation.pids'
        path.parent.mkdir(parents=True)
        path.write_text(self.runner.spawn('controller_server'))
        self.assertEqual(self.submit('start').status_code, 409)
        bridge = path.with_name('real_bridge.pid')
        bridge.write_text(path.read_text().split()[1])
        path.unlink()
        self.assertEqual(self.submit('start').status_code, 409)
        self.runner.cleanup()
        runtime = self.root / 'runtime/missions'
        runtime.mkdir(exist_ok=True)
        (runtime / 'recovery_required.json').write_text('{}')
        self.assertEqual(self.submit('start').status_code, 409)
        (runtime / 'recovery_required.json').unlink()
        with (runtime / 'mission.lock').open('a') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(self.submit('start').status_code, 409)
        self.assertEqual(self.runner.calls, [])

    def test_old_pid_identity_is_not_used_for_stop(self):
        runtime = self.root / 'runtime/mapping'
        runtime.mkdir(parents=True)
        (runtime / 'mapping.pids').write_text(self.runner.spawn('livox').rsplit(' ', 1)[0] + '\n')
        self.assertEqual(self.state()['phase'], 'DEGRADED')
        self.assertFalse(self.state()['can_stop'])
        self.assertEqual(self.submit('stop').status_code, 409)
        self.assertEqual(self.runner.calls, [])

    def test_gateway_restart_and_invalid_download_paths(self):
        self.wait(self.submit('start'))
        record = {'operation_id': str(uuid.uuid4()), 'state': 'RUNNING', 'command_pid': 99999999}
        write_json(self.manager.records / (record['operation_id'] + '.json'), record)
        self.assertEqual(MappingManager(self.settings).operation(record['operation_id'])['state'], 'INTERRUPTED')
        self.assertEqual(self.client.get('/api/v1/mapping/artifact?session_id=other').status_code, 404)
        session = self.manager.session()
        self.assertEqual(self.client.get('/api/v1/mapping/artifact?session_id=' + session['id']).status_code, 409)
        session['output'] = '/etc/passwd'
        write_json(self.manager.runtime / 'session.json', session)
        self.assertEqual(self.client.get('/api/v1/mapping/artifact?session_id=' + session['id']).status_code, 403)

    def test_default_subprocess_runner_uses_fixed_script_and_captures_failure(self):
        script = self.root / 'script/g1_mapping.sh'
        script.write_text('printf "fixture process runner log\\n"\nexit 7\n')
        self.manager.runner = self.manager._run_script
        result = self.wait(self.submit('start'), 'FAILED')
        self.assertEqual(result['exit_code'], 7)
        self.assertIn('fixture process runner log', self.client.get('/api/v1/mapping/logs').text)
        self.assertTrue(result.get('command_pid'))

    def test_failed_trigger_does_not_trust_existing_pcd(self):
        import g1_mapping_save
        path = self.root / 'old.pcd'
        path.write_bytes(PCD)
        future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=False, message='rejected fixture'), cancelled=lambda:False)
        client = SimpleNamespace(wait_for_service=lambda **kw: True, call_async=lambda request: future)
        node = SimpleNamespace(create_client=lambda *args: client, destroy_node=lambda: None)
        ros = SimpleNamespace(init=lambda **kw: None, create_node=lambda name: node, ok=lambda: True, shutdown=lambda: None, spin_until_future_complete=lambda *a, **kw:None)
        service = SimpleNamespace(Request=lambda: None)
        with patch.dict(sys.modules, {'rclpy': ros, 'std_srvs.srv': SimpleNamespace(Trigger=service)}):
            with self.assertRaisesRegex(RuntimeError, 'success=false'):
                g1_mapping_save.save(path, 5)
        self.assertEqual(path.read_bytes(), PCD)

    def test_unknown_mapping_state_blocks_configuration_writes(self):
        path = self.root / 'runtime/mapping/mapping.pids'
        path.parent.mkdir(parents=True)
        path.write_text('invalid pid record')
        self.assertEqual(self.state()['phase'], 'UNKNOWN')
        result = self.client.post('/api/v1/waypoints', headers=dict(self.headers, **{'If-Match': 'old'}),
                                  json={'id': 'point_test', 'name': 'test', 'x': 0, 'y': 0, 'yaw': 0})
        self.assertEqual(result.status_code, 409, result.text)
        self.assertEqual(result.json()['error']['code'], 'MAPPING_UNKNOWN')

    def test_mock_mode_does_not_expose_site_mapping_metadata(self):
        self.wait(self.submit('start'))
        self.wait(self.submit('save'))
        with TestClient(create_app(Settings(mode='mock', workspace=str(self.root)))) as client:
            state = client.get('/api/v1/mapping').json()
            self.assertFalse(state['available'])
            self.assertEqual(state['processes'], [])
            self.assertIsNone(state['session'])
            self.assertEqual(client.get('/api/v1/mapping/logs').json()['logs'], [])
            self.assertEqual(client.get('/api/v1/mapping/artifact?session_id=anything').status_code, 501)

    def test_cli_stop_uses_owned_process_tokens_and_shared_navigation_lock(self):
        self.wait(self.submit('start'))
        scripts = self.root / 'script'
        self.runner.cleanup()
        (self.root / 'runtime/mapping/mapping.pids').unlink()
        import g1_process
        with patch.dict(os.environ, G1_WS=str(self.root)):
            for name in ('livox', 'fast_lio'):
                g1_process.start(self.root / 'runtime/mapping/mapping.pids', name, self.root / (name+'.log'), [sys.executable, '-c', 'import time; time.sleep(120)'])
                self.runner.children.append(g1_process.started_processes[-1])
        for name in ('g1_mapping.sh', 'g1_mapping_state.py', 'g1_navigation.sh', 'g1_process.py', 'g1_livox_config.py', 'g1_env.sh'):
            shutil.copy2(SCRIPT_SOURCE / name, scripts / name)
        env = dict(os.environ, G1_WS=str(self.root))
        runtime = self.root / 'runtime'
        with (runtime / 'stack.lock').open('a') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for name, action in [('g1_mapping.sh', 'stop'), ('g1_navigation.sh', 'start-auto')]:
                result = subprocess.run(['bash', str(scripts / name), action], env=env, capture_output=True, text=True, timeout=5)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('另一个建图/导航管理命令', result.stdout + result.stderr)
        result = subprocess.run(['bash', str(scripts / 'g1_mapping.sh'), 'stop'], env=env, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for child in self.runner.children:
            child.wait(timeout=3)
        self.assertEqual(self.state()['phase'], 'STOPPED')


class CloudTests(unittest.TestCase):
    def message(self, endian=False):
        data = bytearray(56)
        codec = '>' if endian else '<'
        for offset, xyz in zip([0, 12, 28, 40], [(1., 2., 3.), (float('nan'), 0., 0.), (4., 5., 6.), (1., 2., 3.)]):
            struct.pack_into(codec + 'fff', data, offset, *xyz)
        return SimpleNamespace(fields=[SimpleNamespace(name=name, offset=i * 4, datatype=7) for i, name in enumerate(('x', 'y', 'z'))],
                               point_step=12, row_step=28, width=2, height=2, data=data, is_bigendian=endian,
                               header=SimpleNamespace(frame_id='camera_init'))

    def test_organized_padded_big_endian_cloud_filters_nan_and_resets(self):
        for endian in (False, True):
            preview = MappingPreview()
            preview.add(self.message(endian))
            state = preview.snapshot()
            self.assertEqual(set(state['points']), {(1., 2.), (4., 5.)})
            self.assertEqual(state['last_scan_points'], 4)
            self.assertFalse(state['stale'])
            preview.reset()
            self.assertFalse(preview.snapshot()['available'])

    def test_truncated_and_wrong_field_types_are_rejected(self):
        preview = MappingPreview()
        message = self.message()
        message.data = message.data[:12]
        preview.add(message)
        self.assertFalse(preview.snapshot()['available'])
        message = self.message()
        message.fields[0].datatype = 2
        preview.add(message)
        self.assertFalse(preview.snapshot()['available'])

    def test_pcd_header_rejects_empty_and_incomplete_maps(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.pcd'
            path.write_bytes(PCD)
            self.assertEqual(pcd_info(path)['points'], 1)
            for value in (b'', PCD.replace(b'POINTS 1', b'POINTS 0'), PCD[:PCD.index(b'DATA')]):
                path.write_bytes(value)
                with self.assertRaises(ValueError):
                    pcd_info(path)

    def test_reused_zombie_leader_does_not_authorize_signaling_another_group(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for pid, state, token in [(100, 'Z', 200), (101, 'S', 201)]:
                fields = [state, '1', '100', '100'] + ['0'] * 16
                fields[19] = str(token)
                (root / str(pid)).mkdir()
                (root / str(pid) / 'stat').write_text(str(pid) + ' (fixture) ' + ' '.join(fields))
            self.assertEqual(group_state(100, 123, root), 'unverified')
            self.assertEqual(group_state(100, 200, root), 'running')


if __name__ == '__main__':
    unittest.main()
