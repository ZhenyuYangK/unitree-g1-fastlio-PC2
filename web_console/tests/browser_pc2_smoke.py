"""Synthetic PC2 UI and existing edit/mock workflows; only temporary workspaces."""
from contextlib import contextmanager
import os
import sys
from pathlib import Path
import shutil
import socket
import tempfile
import threading
import time
from unittest.mock import AsyncMock

from PIL import Image
from playwright.sync_api import expect, sync_playwright
import uvicorn
import yaml

from g1_web.app import create_app
from g1_web.adapters.mock import MockAdapter
from g1_web.config import Settings
from test_navigation import fixture


@contextmanager
def serve(app):
    listener=socket.socket(); listener.bind(('127.0.0.1',0))
    url='http://127.0.0.1:'+str(listener.getsockname()[1])
    server=uvicorn.Server(uvicorn.Config(app,log_level='error'))
    thread=threading.Thread(target=server.run,kwargs={'sockets':[listener]},daemon=True)
    thread.start(); deadline=time.monotonic()+10
    while not server.started:
        if not thread.is_alive() or time.monotonic()>deadline: raise RuntimeError('fixture server failed')
        time.sleep(.01)
    try: yield url
    finally:
        server.should_exit=True; thread.join(timeout=10); listener.close()
        assert not thread.is_alive()


def main():
    with tempfile.TemporaryDirectory(prefix='g1-pc2-browser-') as directory:
        root=Path(directory); fixture(root)
        settings=Settings(mode='ros_control',workspace=directory,token='fixture')
        app=create_app(settings,adapter=MockAdapter(settings))
        state=dict(available=True,active=False,live=False,state='NOT_RUNNING',run_id='',laps=2,lap=1)
        app.state.cruise.snapshot=AsyncMock(side_effect=lambda:dict(state))
        calls=[]
        async def start(payload):
            calls.append(payload.model_dump())
            state.update(active=True,live=True,state='NAVIGATING',run_id='fixture-run',current_waypoint='point_01')
            return dict(message='fixture started')
        async def control(action,run_id):
            assert run_id=='fixture-run'
            state.update(state={'pause':'PAUSED','resume':'NAVIGATING','stop':'CANCELED'}[action],
                         active=action!='stop',gate_closed=action!='resume',goals_terminal=action!='resume')
            return dict(message='fixture '+action)
        app.state.cruise.start=start; app.state.cruise.control=control
        with serve(app) as url, sync_playwright() as p:
            browser=p.chromium.launch(executable_path=shutil.which('google-chrome'),headless=True,args=['--no-sandbox'])
            page=browser.new_page(viewport={'width':390,'height':844},is_mobile=True,has_touch=True)
            errors=[]; page.on('pageerror',lambda error:errors.append(str(error)))
            page.goto(url); page.locator('#tokenInput').fill('fixture'); page.locator('#loginForm button').click()
            expect(page.locator('#connection')).to_have_text('实时连接')
            page.locator('#controlButton').click()
            page.locator('[data-view="missions"]').click()
            page.locator('#cruiseLaps').fill('2'); page.locator('#cruiseDwell').fill('3')
            expect(page.locator('#cruiseStart')).to_be_enabled(); page.locator('#cruiseStart').click()
            expect(page.locator('#cruisePhase')).to_have_text('NAVIGATING')
            assert calls[0]['points']==['point_01'] and calls[0]['laps']==2 and calls[0]['dwell_sec']==3
            page.locator('#cruisePause').click(); expect(page.locator('#cruisePhase')).to_have_text('PAUSED')
            previews=Path(__file__).resolve().parents[1]/'.preview'; previews.mkdir(exist_ok=True)
            page.screenshot(path=str(previews/'pc2-cruise-mobile.png'),full_page=True)
            expect(page.locator('#cruiseResume')).to_be_enabled()
            page.reload(); expect(page.locator('#controlValue')).to_have_text('本设备控制')
            page.locator('[data-view="missions"]').click(); expect(page.locator('#cruisePhase')).to_have_text('PAUSED')
            page.locator('#cruiseResume').click(); expect(page.locator('#cruisePhase')).to_have_text('NAVIGATING')
            page.locator('#cruiseStop').click(); expect(page.locator('#cruisePhase')).to_have_text('CANCELED')
            expect(page.locator('#cruiseStart')).to_be_enabled()
            assert not errors,errors
            browser.close()
        print('PC2 cruise browser checks passed: route/laps/dwell, pause/resume/stop, mobile and reload recovery.')

    with tempfile.TemporaryDirectory(prefix='g1-p6-browser-mock-') as directory:
        with serve(create_app(Settings(mode='mock',workspace=directory))) as url:
            os.environ['G1_WEB_TEST_URL']=url
            from browser_smoke import main as mock_workflow
            mock_workflow()

    edit_main()


def edit_main():
    with tempfile.TemporaryDirectory(prefix='g1-p6-browser-edit-') as directory:
        root=Path(directory); fixture(root)
        points=root/'task_data/maps/laboratory_map_v1/waypoints.yaml'
        data=yaml.safe_load(points.read_text())
        data['waypoints']={f'point_{i:02}':dict(name=f'讲解点{i}',pose=dict(x=float(i*5),y=float(i*5),yaw=0.),recorded_at='2026-01-01T00:00:00Z') for i in range(1,6)}
        points.write_text(yaml.safe_dump(data,allow_unicode=True))
        missions=root/'task_data/missions/missions.yaml'
        missions.write_text(yaml.safe_dump(dict(schema_version=1,map_id='test',missions={'five_point_patrol':dict(name='五站巡航',steps=[dict(waypoint=f'point_{i:02}',advance_mode='manual',speech=dict(mode='tts',language='zh',content={'zh':dict(text='中文讲解',speaker_id=0,wait_sec=5)})) for i in range(1,6)])}),allow_unicode=True))
        Image.new('L',(500,500),254).save(root/'maps/floor.pgm')
        with serve(create_app(Settings(workspace=directory))) as url:
            os.environ.update(G1_WEB_TEST_URL=url,G1_WEB_TEST_WORKSPACE=directory)
            from browser_sync_smoke import main as edit_workflow
            edit_workflow()


if __name__=='__main__':
    edit_main() if '--edit-only' in sys.argv else main()
