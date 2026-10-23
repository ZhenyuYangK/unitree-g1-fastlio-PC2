"""PC2 layout, Python 3.8, ownership, and continuous-cruise boundaries."""
import asyncio
import contextvars
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.request import urlopen
import uuid
from unittest.mock import AsyncMock

import pytest
from fastapi.testclient import TestClient

from g1_web.app import create_app
from g1_web.adapters.mock import MockAdapter
from g1_web.adapters.ros_readonly import localization_view
from g1_web.compat import to_thread
from g1_web.config import Settings
from g1_web.control import GatewayError
from g1_web.cruise import CruiseManager
from g1_web.mapping import MappingManager
from g1_web.navigation import NavigationManager
from g1_web.runtime import processes
from g1_web.workspace import WorkspaceData
from g1_web.paths import SOURCE_WORKSPACE
from test_navigation import fixture


def test_relative_files_and_workspace_override(tmp_path, monkeypatch):
    fixture(tmp_path)
    monkeypatch.setenv('G1_WS', str(tmp_path))
    monkeypatch.setenv('G1_MAP_YAML', 'maps/map_multilayer.yaml')
    monkeypatch.setenv('G1_WAYPOINTS_FILE', 'task_data/maps/laboratory_map_v1/waypoints.yaml')
    settings = Settings.from_env()
    monkeypatch.chdir('/tmp')
    data = WorkspaceData(settings)
    assert data.root == tmp_path and data.map_yaml == tmp_path / 'maps/map_multilayer.yaml'
    assert data.catalog()['waypoints'][0]['id'] == 'point_01'


def test_python38_thread_preserves_context_and_event_loop():
    marker = contextvars.ContextVar('pc2-test')
    async def check():
        marker.set('pc2')
        assert await to_thread(marker.get) == 'pc2'
    asyncio.run(check())


def test_launcher_from_another_directory_uses_system_interpreter(tmp_path):
    with socket.socket() as port:
        port.bind(('127.0.0.1', 0))
        number = port.getsockname()[1]
    env = dict(os.environ, G1_WS=str(tmp_path), G1_ROS_DISTRO='foxy', G1_WEB_MODE='mock',
               G1_WEB_HOST='127.0.0.1', G1_WEB_PORT=str(number), G1_WEB_TOKEN='',
               G1_WEB_PYTHON=sys.executable, PYTHONPATH='')
    child = subprocess.Popen(['bash', str(SOURCE_WORKSPACE/'web_console/run.sh')],cwd='/tmp',env=env,
                             start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            try:
                with urlopen('http://127.0.0.1:{}/api/v1/health'.format(number),timeout=.2) as response:
                    assert json.load(response)['mode']=='mock'
                break
            except OSError:
                assert child.poll() is None
                time.sleep(.05)
        else:
            pytest.fail('PC2 launcher did not serve mock mode')
    finally:
        if child.poll() is None: os.killpg(child.pid,signal.SIGTERM)
        child.wait(timeout=5)


@pytest.mark.parametrize('changes,age', [({'stamp_ns':10**100},0), ({'stamp_ns':11_000_000_000},0),
    ({'last_success_stamp_ns':10_000_000_001},0), ({'valid':1},0), ({},float('nan')), ({},-1), ({},1.01)])
def test_web_localization_uses_pc2_health_contract(changes, age):
    raw = dict(stamp_ns=10_000_000_000,last_success_stamp_ns=10_000_000_000,state='TRACKING',valid=True)
    raw.update(changes)
    assert localization_view(raw,10_000_000_000,age)['valid'] is False


def test_pc2_json_ownership_and_session_identity(tmp_path, monkeypatch):
    import g1_process
    monkeypatch.setenv('G1_WS', str(tmp_path))
    path = tmp_path / 'runtime/mapping/mapping.pids'
    path.parent.mkdir(parents=True)
    g1_process.start(path,'fast_lio',tmp_path/'sleep.log',[sys.executable,'-c','import time;time.sleep(60)'])
    child = g1_process.started_processes[-1]
    try:
        assert processes(path)[0]['state'] == 'running'
        record = json.loads(path.read_text())
        nav = tmp_path / 'runtime/navigation/navigation.pids'
        nav.parent.mkdir(parents=True)
        nav.write_text(json.dumps(record))
        settings = Settings(mode='ros_control', token='fixture', workspace=str(tmp_path))
        assert MappingManager(settings)._navigation_running()
        assert NavigationManager(settings, MockAdapter(settings)).tracked()[0]['state'] == 'running'
        record['processes'][0]['command_sha256'] = 'wrong'
        path.write_text(json.dumps(record))
        assert processes(path)[0]['state'] == 'unverified'
        record['workspace'] = '/tmp/other-workspace'
        path.write_text(json.dumps(record))
        with pytest.raises(ValueError): processes(path)
        session = path.parent / 'session.json'
        session.write_text(json.dumps(dict(session_id='pc2-session',output=str(tmp_path/'maps/test.pcd'))))
        assert MappingManager(Settings(workspace=str(tmp_path))).session()['id'] == 'pc2-session'
    finally:
        os.killpg(child.pid,signal.SIGTERM)
        child.wait(timeout=5)


def test_cruise_socket_pause_resume_stop_and_stale_run(tmp_path):
    from g1_task_manager.cruise import Control
    manager = CruiseManager(Settings(mode='ros_control',token='fixture',workspace=str(tmp_path)))
    manager.runtime.mkdir(parents=True)
    control = Control(dict(route=[dict(waypoint='point_01')],laps=2))
    server = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    server.bind(str(manager.runtime/'control.sock')); server.listen(4); server.settimeout(.1)
    thread = threading.Thread(target=control.serve,args=(server,),daemon=True); thread.start()
    try:
        state = manager.inspect()
        assert state['live'] and state['active']
        assert manager.rpc('pause', 'previous-run')['ok'] is False
        assert control.snapshot()['state'] == 'STARTING'
        with pytest.raises(GatewayError): asyncio.run(manager.control('pause','old'))
        asyncio.run(manager.control('pause',state['run_id']))
        assert control.snapshot()['state'] == 'PAUSING'
        with pytest.raises(GatewayError): asyncio.run(manager.control('resume',state['run_id']))
        control.update(state='PAUSED',gate_closed=True,goals_terminal=True,dwell_remaining=1.5)
        assert manager.inspect()['dwell_remaining'] == 1.5
        asyncio.run(manager.control('resume',state['run_id']))
        assert control.snapshot()['state'] == 'RESUMING'
        asyncio.run(manager.control('stop',state['run_id']))
        assert control.snapshot()['state'] == 'STOPPING'
    finally:
        control.closed.set(); server.close(); thread.join(timeout=2)
    (manager.runtime/'status.json').write_text('{"state":"NAVIGATING"}')
    assert manager.inspect()['state'] == 'STOP_UNCONFIRMED'
    (manager.runtime/'status.json').write_text('invalid')
    assert manager.inspect()['state'] == 'STOP_UNCONFIRMED'


@pytest.mark.parametrize('mode', ['local','mock','ros_readonly'])
def test_cruise_control_mode_boundaries(tmp_path,mode):
    settings = Settings(mode=mode,token='fixture' if mode=='ros_readonly' else '',workspace=str(tmp_path))
    with TestClient(create_app(settings,adapter=MockAdapter(settings))) as client:
        if settings.token: client.post('/api/v1/session',json=dict(token='fixture'))
        headers={'X-Control-Lease':client.post('/api/v1/control/acquire',json=dict(owner='browser')).json()['lease_id']}
        result=client.post('/api/v1/cruise/pause',headers=headers,json=dict(request_id=str(uuid.uuid4()),run_id='old'))
        assert result.status_code==501


def test_cruise_start_revision_lease_replay_and_navigation_exclusion(tmp_path):
    fixture(tmp_path)
    settings=Settings(mode='ros_control',token='fixture',workspace=str(tmp_path))
    robot=MockAdapter(settings)
    app=create_app(settings,adapter=robot)
    with TestClient(app) as client:
        client.post('/api/v1/session',json=dict(token='fixture'))
        headers={'X-Control-Lease':client.post('/api/v1/control/acquire',json=dict(owner='browser')).json()['lease_id']}
        revision=client.get('/api/v1/catalog').json()['revision']
        app.state.cruise.start=AsyncMock(return_value=dict(message='fixture accepted'))
        def post(**updates):
            body=dict(request_id=str(uuid.uuid4()),revision=revision,points=['point_01'],laps=2)
            body.update(updates)
            return client.post('/api/v1/cruise/start',headers=headers,json=body)
        assert post(revision='old').status_code==409
        assert post(laps=True).status_code==422
        assert post(timeout_sec=True).status_code==422
        assert post(dwell_sec='2').status_code==422
        identifier=str(uuid.uuid4())
        assert post(request_id=identifier).status_code==202
        assert post(request_id=identifier).status_code==202
        assert app.state.cruise.start.await_count==1
        assert post(request_id=identifier,laps=3).status_code==409
        app.state.cruise.snapshot=AsyncMock(return_value=dict(active=True,live=True,state='PAUSED',run_id='1:2'))
        result=client.post('/api/v1/navigation/initial-pose',headers=headers,json=dict(request_id=str(uuid.uuid4()),x=0.,y=0.,yaw=0.))
        assert result.status_code==409 and result.json()['error']['code']=='CRUISE_BUSY'


def test_cruise_start_uses_fixed_cli_arguments(tmp_path):
    fixture(tmp_path)
    scripts=tmp_path/'script'; scripts.mkdir(exist_ok=True)
    (scripts/'g1_process.py').write_text("import json,sys,os\nfrom pathlib import Path\nPath(os.environ['G1_WS'],'args.json').write_text(json.dumps(sys.argv[1:]))\n")
    manager=CruiseManager(Settings(mode='ros_control',token='fixture',workspace=str(tmp_path)))
    from g1_web.app import CruiseStart
    asyncio.run(manager.start(CruiseStart(request_id=uuid.uuid4(),points=['point_01'],laps=3)))
    args=json.loads((tmp_path/'args.json').read_text())
    assert args[0]=='start' and '--execute' in args and args[args.index('--laps')+1]=='3'
    assert args[args.index('--file',args.index('--'))+1]==str(WorkspaceData(manager.settings).waypoints)
    asyncio.run(manager.start(CruiseStart(request_id=uuid.uuid4(),points=['point_01'],loop=True)))
    args=json.loads((tmp_path/'args.json').read_text())
    assert '--loop' in args and '--laps' not in args
