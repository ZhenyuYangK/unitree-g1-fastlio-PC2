"""Self-contained browser QA using synthetic navigation and a disposable workspace."""
from pathlib import Path
import shutil
import socket
import tempfile
import threading
import time

from playwright.sync_api import sync_playwright, expect
import uvicorn

from g1_web.app import create_app
from g1_web.adapters.mock import MockAdapter
from g1_web.config import Settings
from g1_web.navigation import NavigationManager
from test_navigation import fixture


class NavigationFixture(MockAdapter):
    def __init__(self,settings):
        super().__init__(settings); self.initial=[]; self.progress=None
    async def snapshot(self):
        data=await super().snapshot()
        data['simulated']=False
        data['capabilities']['initial_pose']=True
        if self.progress:
            data['localization'].update(self.progress)
            data['navigation']['ready']=False
        return data
    async def initial_pose(self,x,y,yaw):
        self.initial.append((x,y,yaw))
        return dict(message='初始位姿已提交，等待定位确认')


def main():
    with tempfile.TemporaryDirectory(prefix='g1-nav-browser-') as directory:
        root=Path(directory); fixture(root)
        settings=Settings(mode='ros_control',token='fixture',workspace=str(root),mock_travel_seconds=4.)
        robot=NavigationFixture(settings)
        async def runner(record):
            return 0
        manager=NavigationManager(settings,robot,runner)
        app=create_app(settings,adapter=robot,navigation=manager)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
        server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=port,log_level='error'))
        thread=threading.Thread(target=server.run,daemon=True); thread.start()
        deadline=time.monotonic()+10
        while not server.started and time.monotonic()<deadline: time.sleep(.05)
        assert server.started
        errors=[]
        try:
            with sync_playwright() as playwright:
                browser=playwright.chromium.launch(executable_path=shutil.which('google-chrome'),headless=True,args=['--no-sandbox'])
                context=browser.new_context(viewport=dict(width=1440,height=1100))
                page=context.new_page(); page.on('pageerror',lambda error:errors.append(str(error)))
                page.goto(f'http://127.0.0.1:{port}')
                page.locator('#tokenInput').fill('fixture'); page.locator('#loginForm button').click()
                expect(page.locator('#connection')).to_have_text('实时连接')
                expect(page.locator('#goToButton')).to_be_disabled()
                page.locator('#controlButton').click()
                expect(page.locator('#goToButton')).to_be_enabled()
                page.locator('[data-view="integration"]').first.click()
                expect(page.locator('#navigationChecks li.is-ready')).to_have_count(7)
                robot.progress=dict(state='VERIFYING_CANDIDATES',valid=False,status_stale=False,
                                    attempt=2,max_attempts=3,candidates=[dict(candidate=1,score=.81,stable=True),
                                                                      dict(candidate=2,score=.78,stable=True)])
                expect(page.locator('#navigationPhase')).to_have_text('比较定位候选')
                expect(page.locator('#navigationOperationMessage')).to_contain_text('第 2/3 次')
                expect(page.locator('#navigationOperationMessage')).to_contain_text('候选 1 81.0 分')
                page.screenshot(path='/tmp/g1-localization-candidates.png',full_page=True)
                page.locator('[data-view="overview"]').first.click()
                expect(page.locator('#localizationValue')).to_have_text('比较定位候选')
                robot.progress.update(state='RETRYING',candidates=[])
                expect(page.locator('#localizationValue')).to_have_text('重新采集定位')
                page.screenshot(path='/tmp/g1-localization-retry.png',full_page=True)
                page.locator('[data-view="integration"]').first.click()
                robot.progress=None
                page.locator('#navigationStart').click()
                expect(page.locator('#navigationOperation')).to_have_text('已完成')
                page.locator('#navigationInitial').click()
                page.locator('#initialX').fill('1.25'); page.locator('#initialY').fill('2.50'); page.locator('#initialYaw').fill('90')
                page.locator('#submitInitialPose').click()
                expect(page.locator('#initialPoseDialog')).not_to_be_visible()
                assert robot.initial and abs(robot.initial[0][2]-1.5707963)<1e-5
                page.locator('#navigationInitial').click()
                page.locator('#pickInitialPose').click()
                expect(page.locator('#navigationPickHint')).to_be_visible()
                box=page.locator('#map').bounding_box()
                page.mouse.move(box['x']+box['width']*.5,box['y']+box['height']*.5)
                page.mouse.down(); page.mouse.move(box['x']+box['width']*.65,box['y']+box['height']*.5); page.mouse.up()
                expect(page.locator('#initialPoseDialog')).to_be_visible()
                assert abs(float(page.locator('#initialYaw').input_value()))<1
                page.locator('#closeInitialPose').click()
                expect(page.locator('#targetMarker')).to_be_visible()
                page.locator('#goToButton').click()
                expect(page.locator('#navigationConfirmDetail')).to_contain_text('门口')
                page.locator('#confirmGoTo').click()
                expect(page.locator('#missionValue')).to_have_text('前往站点')
                expect(page.locator('#planLine')).not_to_have_attribute('hidden','')
                expect(page.locator('#pathStatus')).to_have_text('规划路径（实线）')
                expect(page.locator('#goToButton')).to_be_disabled()
                expect(page.locator('#startButton')).not_to_be_visible()
                page.locator('#cancelButton').click()
                expect(page.locator('#missionValue')).to_have_text('已取消')
                expect(page.locator('#stopConfirmation')).to_have_text('本次任务停止收尾已确认')
                page.locator('#mapFollow').click()
                expect(page.locator('#mapFollow')).to_have_attribute('aria-pressed','true')
                page.locator('#mapFit').click()
                expect(page.locator('#mapFollow')).to_have_attribute('aria-pressed','false')
                page.locator('#themeButton').click()
                page.screenshot(path='/tmp/g1-navigation-desktop.png',full_page=True)
                page.set_viewport_size(dict(width=390,height=844))
                assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
                page.locator('[data-view="integration"]').first.click()
                assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
                page.screenshot(path='/tmp/g1-navigation-mobile.png',full_page=True)
                context.set_offline(True)
                expect(page.locator('#connection')).to_have_text('连接中 / 状态过期',timeout=7000)
                expect(page.locator('#navigationInitial')).to_be_disabled()
                context.set_offline(False)
                expect(page.locator('#connection')).to_have_text('实时连接',timeout=12000)
                assert not errors,errors
                browser.close()
            print('Browser navigation checks passed: ranked candidate scores, attempt count, retry progress, preparation, readiness, numeric and map-drag pose preview, single-point confirmation/path/cancel, follow/reset, mobile/dark mode, disconnect controls; no page errors.')
        finally:
            server.should_exit=True; thread.join(timeout=10)


if __name__=='__main__':
    main()
