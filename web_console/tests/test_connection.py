"""Connection diagnostics are bounded, authenticated and never record credentials."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

from g1_web.adapters.mock import MockAdapter
from g1_web.app import create_app
from g1_web.config import Settings
from g1_web.cloud import MappingPreview
from g1_web.runtime import group_state, process_observation
import g1_process


class ConnectionTests(unittest.TestCase):
    def test_shared_process_scan_preserves_ownership_and_does_not_leak_to_commands(self):
        record = dict(boot_id='boot', pid=10, pgid=10, sid=10, token='owned',
                      start='50', command_sha256='hash')
        identity = {key: value for key, value in record.items() if key != 'boot_id'}
        snapshot = ('boot', [identity], [(10, 10, 'owned')])
        with patch('g1_process.process_snapshot', return_value=snapshot) as scan:
            with process_observation():
                self.assertEqual(group_state(10, record), 'running')
                self.assertEqual(group_state(10, record), 'running')
                self.assertEqual(scan.call_count, 1)
        with patch('g1_process.members', return_value=[]) as direct:
            self.assertEqual(group_state(10, record), 'stopped')
            direct.assert_called_once_with(record)
        with self.assertRaises(RuntimeError):
            g1_process.members(record, snapshot=('boot', [identity], [(10, 10, 'owned'), (11, 10, 'foreign')]))
        with self.assertRaises(RuntimeError):
            g1_process.members(record, snapshot=('boot', [dict(identity, start='51')], [(10, 10, 'owned')]))
        self.assertEqual(g1_process.members(record, snapshot=('old-boot', [identity], [])), [])

    def test_preview_payload_is_bounded_without_discarding_stored_cells(self):
        preview = MappingPreview()
        preview.cells = {(i, 0): (float(i), 0.0) for i in range(6000)}
        self.assertLessEqual(len(preview.snapshot()['points']), 1000)
        self.assertEqual(len(preview.cells), 6000)

    def test_concurrent_readers_share_a_sample_but_completed_samples_are_not_reused(self):
        settings = Settings(mode='mock')
        robot = MockAdapter(settings)
        app = create_app(settings, adapter=robot)
        samples = []
        original = robot.snapshot

        async def snapshot():
            samples.append(1)
            await asyncio.sleep(0.1)
            return await original()

        robot.snapshot = snapshot
        with TestClient(app) as client:
            with client.websocket_connect('/ws/state') as first:
                with client.websocket_connect('/ws/state') as second:
                    a, b = first.receive_json(), second.receive_json()
                    self.assertEqual(a['sequence'], b['sequence'])
                    self.assertEqual(len(samples), 1)
            self.assertGreater(client.get('/api/v1/state').json()['sequence'], a['sequence'])
            self.assertGreater(len(samples), 1)

    def test_diagnostics_require_auth_and_reject_unknown_data(self):
        with tempfile.TemporaryDirectory() as root:
            settings = Settings(mode='local', workspace=root, token='diagnostic-test-token')
            with TestClient(create_app(settings, adapter=MockAdapter(settings))) as client:
                path = '/api/v1/diagnostics/connection'
                self.assertEqual(client.post(path, json={'event': 'state_stale'}).status_code, 401)
                client.post('/api/v1/session', json={'token': settings.token})
                for payload in ({'event': 'unknown'}, {'event': 'state_stale', 'lease_id': 'secret'},
                                {'event': 'state_stale', 'age_ms': -1},
                                {'event': 'state_stale', 'detail': 'x' * 241}):
                    self.assertEqual(client.post(path, json=payload).status_code, 422)
                self.assertEqual(client.post(path, json={'event': 'state_stale', 'age_ms': 2500}).status_code, 204)
                self.assertEqual(client.post(path, json={'event': 'state_failed', 'detail': 'TimeoutError'}).status_code, 204)

    def test_connection_logs_exclude_session_and_control_secrets(self):
        with tempfile.TemporaryDirectory() as root:
            settings = Settings(mode='local', workspace=root, token='diagnostic-test-token')
            with TestClient(create_app(settings, adapter=MockAdapter(settings))) as client:
                client.post('/api/v1/session', json={'token': settings.token})
                secret = client.post('/api/v1/control/acquire', json={'owner': 'test'}).json()['lease_id']
                self.assertEqual(client.post('/api/v1/control/renew', headers={'X-Control-Lease': secret}).status_code, 200)
                with client.websocket_connect('/ws/state') as socket:
                    self.assertEqual(socket.receive_json()['type'], 'state')
                client.post('/api/v1/diagnostics/connection', json={'event': 'ws_close', 'code': 1006})
                content = (Path(root) / 'runtime/web_console/connection.log').read_text()
                for value in (secret, settings.token, client.cookies['g1_session']):
                    self.assertNotIn(value, content)
                events = [json.loads(line) for line in content.splitlines()]
                self.assertTrue(any(e['event'] == 'http' and e['status'] == 200 for e in events))
                self.assertTrue(any(e['event'] == 'browser.ws_close' and e['code'] == 1006 for e in events))

    def test_independent_state_inspections_do_not_wait_for_each_other(self):
        settings = Settings(mode='mock')
        app = create_app(settings)
        with TestClient(app) as client:
            entered = set()
            released = asyncio.Event()

            def gate(name, original):
                async def snapshot():
                    entered.add(name)
                    if len(entered) == 3:
                        released.set()
                    await asyncio.wait_for(released.wait(), timeout=1)
                    return await original()
                return snapshot

            for name in ('mapping', 'navigation', 'cruise'):
                manager = getattr(app.state, name)
                manager.snapshot = gate(name, manager.snapshot)
            self.assertEqual(client.get('/api/v1/state').status_code, 200)
            self.assertEqual(entered, {'mapping', 'navigation', 'cruise'})


if __name__ == '__main__':
    unittest.main()
