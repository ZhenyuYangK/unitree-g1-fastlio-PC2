"""Navigation boundaries, durable admission, and telemetry without a robot."""
import asyncio
import fcntl
import math
from pathlib import Path
import subprocess
import tempfile
import time
from types import SimpleNamespace as N
import unittest
import uuid
from unittest.mock import AsyncMock, patch

from fastapi.testclient import TestClient
from PIL import Image
import yaml

from g1_web.app import create_app
from g1_web.adapters.mock import MockAdapter
from g1_web.adapters.ros_control import RosControlAdapter
from g1_web.config import Settings
from g1_web.control import GatewayError
from g1_web.journal import CommandJournal
from g1_web.mapping import process, write_json
from g1_web.navigation import NavigationManager
from g1_web.telemetry import path_view, planar_pose, fresh_stamp


def fixture(root):
    points = root / 'task_data/maps/laboratory_map_v1/waypoints.yaml'
    missions = root / 'task_data/missions/missions.yaml'
    map_yaml = root / 'maps/map_multilayer.yaml'
    for path in (points, missions, map_yaml):
        path.parent.mkdir(parents=True, exist_ok=True)
    points.write_text(yaml.safe_dump(dict(schema_version=1, map_id='test', frame_id='map', base_frame='nav_base',
        waypoints={'point_01': dict(name='门口', pose=dict(x=1.0,y=2.0,yaw=0.0))})))
    missions.write_text(yaml.safe_dump(dict(schema_version=1,map_id='test',missions={
        'five_point_tour':dict(name='测试巡航',steps=[dict(waypoint='point_01')])})))
    map_yaml.write_text(yaml.safe_dump(dict(image='floor.pgm', resolution=0.1,origin=[0.,0.,0.])))
    Image.new('L',(50,50),254).save(root / 'maps/floor.pgm')
    (root / 'script').mkdir()
    (root / 'script/g1_navigation.sh').write_text('# fixture runner only\n')


class TelemetryTests(unittest.TestCase):
    def test_rotated_pose_invalid_quaternion_and_nan(self):
        pose = planar_pose(N(x=1.,y=2.),N(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))
        self.assertAlmostEqual(pose['yaw'], math.pi/2)
        for q in (N(x=0.,y=0.,z=0.,w=0.), N(x=0.,y=0.,z=float('nan'),w=1.)):
            with self.assertRaises(ValueError):
                planar_pose(N(x=1.,y=2.),q)
        self.assertFalse(fresh_stamp(0, 1_000_000_000, 1))
        self.assertFalse(fresh_stamp(2_000_000_000, 1_000_000_000, 1))

    def test_path_frame_bounds_expiration_and_endpoints(self):
        header = N(frame_id='map',stamp=N(sec=10,nanosec=0))
        path = N(header=header,poses=[N(header=header,pose=N(position=N(x=float(i),y=0.))) for i in range(5000)])
        result = path_view(path,10_000_000_000,0)
        self.assertEqual(len(result['points']),800)
        self.assertEqual((result['points'][0][0], result['points'][-1][0]),(0.,4999.))
        self.assertTrue(path_view(path,14_000_000_000,0)['stale'])
        path.header.frame_id='odom'
        self.assertFalse(path_view(path,10_000_000_000,0)['available'])
        path.header.frame_id='map'; path.poses[0].pose.position.x=float('inf')
        self.assertFalse(path_view(path,10_000_000_000,0)['available'])

    def test_receipt_survives_restart_and_pending_is_never_reexecuted(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'commands.db'
            first = CommandJournal(path)
            self.assertIsNone(first.begin('a','start'))
            with self.assertRaises(GatewayError) as error:
                CommandJournal(path).begin('a','start')
            self.assertEqual(error.exception.code,'COMMAND_UNCONFIRMED')
            first.finish('a',dict(accepted=True,run_id='one'))
            self.assertEqual(CommandJournal(path).begin('a','start')['run_id'],'one')
            with self.assertRaises(GatewayError):
                first.begin('a','cancel')


class AsyncNavigationTests(unittest.IsolatedAsyncioTestCase):
    async def test_last_release_ack_cannot_overwrite_a_final_result(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter=RosControlAdapter(Settings(mode='ros_control',token='fixture',workspace=directory))
            adapter.run=dict(active=True,run_id='run',state='WAITING_FOR_CONTINUE',wait_id='wait',stale=False)
            adapter.snapshot=AsyncMock(return_value=dict(mission=dict(adapter.run)))
            adapter.continue_type=N(Request=lambda **kwargs:N(**kwargs))
            adapter.continue_client=N(call_async=lambda request:object())
            async def fast_completion(future):
                adapter.run.update(active=False,state='SUCCEEDED',gate_closed=True,goals_terminal=True)
                return N(accepted=True,message='released')
            with patch('g1_web.adapters.ros_control.ros_result',side_effect=fast_completion):
                await adapter.continue_mission('run','wait')
            self.assertFalse(adapter.run['active'])
            self.assertEqual(adapter.run['state'],'SUCCEEDED')


class NavigationGatewayTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory(prefix='g1-nav-api-')
        self.addCleanup(self.directory.cleanup)
        self.root=Path(self.directory.name); fixture(self.root)
        self.settings=Settings(mode='ros_control',token='fixture',workspace=str(self.root),mock_travel_seconds=2)
        self.robot=MockAdapter(self.settings)
        self.manager=NavigationManager(self.settings,self.robot,runner=self.runner)
        self.calls=[]
        self.app=create_app(self.settings,adapter=self.robot,navigation=self.manager)
        self.client=TestClient(self.app).__enter__()
        self.addCleanup(self.client.__exit__,None,None,None)
        self.client.post('/api/v1/session',json=dict(token='fixture'))
        self.acquire()

    def acquire(self):
        lease=self.client.post('/api/v1/control/acquire',json=dict(owner='fixture')).json()['lease_id']
        self.headers={'X-Control-Lease':lease}

    async def runner(self,record):
        self.calls.append(record['action'])
        await asyncio.sleep(.05)
        return 0

    def post(self,path,**payload):
        return self.client.post('/api/v1/'+path,headers=self.headers,json=dict(request_id=str(uuid.uuid4()),**payload))

    def revision(self):
        return self.client.get('/api/v1/catalog').json()['revision']

    def test_revision_and_real_mode_auth_are_enforced(self):
        self.assertEqual(self.post('missions/start',mission_id='five_point_tour').status_code,428)
        self.assertEqual(self.post('navigation/go-to',waypoint_id='point_01',revision='old').status_code,409)
        self.assertFalse(self.robot.mission['active'])
        bad=self.client.post('/api/v1/navigation/go-to',headers={**self.headers,'Origin':'https://foreign.example'},json=dict(request_id=str(uuid.uuid4()),waypoint_id='point_01'))
        self.assertEqual(bad.status_code,403)

    def test_persistent_start_receipt_across_new_gateway_and_controller(self):
        payload=dict(request_id=str(uuid.uuid4()),mission_id='five_point_tour',revision=self.revision())
        first=self.client.post('/api/v1/missions/start',headers=self.headers,json=payload)
        self.assertEqual(first.status_code,202,first.text)
        self.assertFalse(first.json()['simulated'])  # Mode boundary, injected adapter is fixture-only.
        with TestClient(create_app(self.settings,adapter=MockAdapter(self.settings))) as second:
            second.post('/api/v1/session',json=dict(token='fixture'))
            lease=second.post('/api/v1/control/acquire',json=dict(owner='new')).json()['lease_id']
            replay=second.post('/api/v1/missions/start',headers={'X-Control-Lease':lease},json=payload)
            self.assertEqual(first.json(),replay.json())

    def test_single_point_and_patrol_are_exclusive_and_old_cancel_is_rejected(self):
        response=self.post('navigation/go-to',waypoint_id='point_01',revision=self.revision())
        self.assertEqual(response.status_code,202,response.text)
        run_id=response.json()['run_id']
        self.assertEqual(self.post('missions/start',mission_id='five_point_tour',revision=self.revision()).status_code,409)
        self.assertEqual(self.post('missions/cancel',run_id='old').status_code,409)
        self.assertEqual(self.post('missions/cancel',run_id=run_id).status_code,202)

    def test_lifecycle_is_durable_and_rejects_custom_arguments(self):
        payload=dict(request_id=str(uuid.uuid4()),operation='navigation.start')
        first=self.client.post('/api/v1/operations',headers=self.headers,json=payload)
        self.assertEqual(first.status_code,202,first.text)
        second=self.client.post('/api/v1/operations',headers=self.headers,json=payload)
        self.assertEqual(first.json(),second.json())
        operation=first.json()['operation_id']
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            state=self.client.get('/api/v1/operations/'+operation).json()
            if state['state']=='SUCCEEDED': break
            time.sleep(.02)
        self.assertEqual(state['state'],'SUCCEEDED',state)
        self.assertEqual(self.calls,['navigation.start'])
        self.assertEqual(self.post('operations',operation='navigation.start',name='custom').status_code,422)
        self.assertEqual(self.client.get('/api/v1/operations/not-a-uuid').status_code,404)

    def test_failed_initialization_keeps_specific_fresh_localization_reason(self):
        original = self.robot.snapshot
        async def snapshot():
            data = await original()
            data['localization'].update(state='AMBIGUOUS', valid=False, status_stale=False,
                                        reason='Ranked candidates remain competing after 3 attempts')
            return data
        async def failure(record):
            return 1
        self.robot.snapshot = snapshot
        self.manager.runner = failure
        response = self.post('operations', operation='navigation.start')
        self.assertEqual(response.status_code, 202, response.text)
        operation = response.json()['operation_id']
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            state = self.client.get('/api/v1/operations/' + operation).json()
            if state['state'] == 'FAILED':
                break
            time.sleep(.02)
        self.assertEqual(state['state'], 'FAILED', state)
        self.assertIn('Ranked candidates remain competing', state['message'])

    def test_recovery_and_cli_lock_block_start(self):
        runtime=self.root/'runtime/missions'; runtime.mkdir(parents=True)
        (runtime/'recovery_required.json').write_text('{}')
        self.assertEqual(self.post('navigation/go-to',waypoint_id='point_01',revision=self.revision()).status_code,409)
        (runtime/'recovery_required.json').unlink()
        with (runtime/'mission.lock').open('w') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            self.assertEqual(self.post('operations',operation='navigation.start').status_code,409)
        self.assertEqual(self.calls,[])

    def test_reused_pid_does_not_allow_navigation_stop(self):
        child=subprocess.Popen(['sleep','30'],start_new_session=True)
        self.addCleanup(child.wait)
        self.addCleanup(child.terminate)
        runtime=self.root/'runtime/navigation'; runtime.mkdir(parents=True)
        token=process(child.pid)['token']
        (runtime/'navigation.pids').write_text(f'nav2 {child.pid} {token+1}\n')
        self.assertEqual(self.post('operations',operation='navigation.stop').status_code,409)
        self.assertIsNone(child.poll())

    def test_report_download_rejects_paths_outside_mission_reports(self):
        identifier=str(uuid.uuid4())
        records=self.root/'runtime/missions/actions'; records.mkdir(parents=True)
        private=self.root/'not-a-task-report.json'; private.write_text('{"private":true}')
        write_json(records/(identifier+'.json'),dict(goal_id=identifier,report_path=str(private)))
        self.assertEqual(self.client.get('/api/v1/navigation/report',params=dict(goal_id=identifier)).status_code,404)
        report=self.root/'runtime/missions/result.json'; report.write_text('{"state":"SUCCEEDED"}')
        write_json(records/(identifier+'.json'),dict(goal_id=identifier,report_path=str(report)))
        response=self.client.get('/api/v1/navigation/report',params=dict(goal_id=identifier))
        self.assertEqual(response.status_code,200)
        self.assertEqual(response.json()['state'],'SUCCEEDED')

    def test_stop_does_not_trust_removed_pid_file_while_process_is_alive(self):
        child=subprocess.Popen(['sleep','30'],start_new_session=True)
        self.addCleanup(child.wait); self.addCleanup(child.terminate)
        runtime=self.root/'runtime/navigation'; runtime.mkdir(parents=True)
        token=process(child.pid)['token']
        pid_file=runtime/'navigation.pids'; pid_file.write_text(f'nav2 {child.pid} {token}\n')
        async def false_stop(record):
            pid_file.unlink()
            return 0
        self.manager.runner=false_stop
        response=self.post('operations',operation='navigation.stop')
        self.assertEqual(response.status_code,202,response.text)
        operation=response.json()['operation_id']; deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            state=self.client.get('/api/v1/operations/'+operation).json()
            if state['state']=='FAILED': break
            time.sleep(.02)
        self.assertEqual(state['state'],'FAILED',state)
        self.assertTrue(self.client.get('/api/v1/navigation').json()['lifecycle']['active'])


if __name__=='__main__':
    unittest.main()
