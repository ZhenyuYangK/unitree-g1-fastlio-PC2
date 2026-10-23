"""Self-contained mapping browser QA. Only fixture sleep processes; no robot/ROS commands."""
from pathlib import Path
import shutil
import socket
import tempfile
import threading
import time

import uvicorn
from playwright.sync_api import sync_playwright, expect

from g1_web.adapters.local import LocalAdapter
from g1_web.app import create_app
from g1_web.config import Settings
from g1_web.mapping import MappingManager
from test_mapping import FakeRunner, PCD


class PreviewFixture(LocalAdapter):
    async def snapshot(self):
        result = await super().snapshot()
        points = []
        for i in range(240):
            t = i / 240
            points.extend([(12*t, 0), (12*t, 6), (0, 6*t), (12, 6*t)])
        result['mapping_preview'] = dict(available=True, stale=False, frame_id='camera_init', points=points,
                                         scans=27, last_scan_points=18000, revision=1, received_at=time.time())
        return result


def main():
    with tempfile.TemporaryDirectory(prefix='g1-mapping-browser-') as directory:
        root = Path(directory)
        site = Path(__file__).resolve().parents[2]
        for relative in ['maps/map_multilayer.yaml', 'maps/map_multilayer.pgm',
                         'task_data/maps/laboratory_map_v1/waypoints.yaml', 'task_data/missions/missions.yaml']:
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(site / relative, target)
        (root / 'script').mkdir()
        (root / 'script/g1_mapping.sh').write_text('# test placeholder; runner injected, never executed\n')
        settings = Settings(workspace=str(root))
        runner = FakeRunner(root)
        manager = MappingManager(settings, runner)
        app = create_app(settings, adapter=PreviewFixture(), mapping=manager)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        server = uvicorn.Server(uvicorn.Config(app, host='127.0.0.1', port=port, log_level='error'))
        thread = threading.Thread(target=server.run, daemon=True)
        thread.start()
        deadline = time.monotonic() + 10
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.05)
        assert server.started
        errors = []
        previews = Path(__file__).resolve().parents[1] / '.preview'
        previews.mkdir(exist_ok=True)
        try:
            with sync_playwright() as p:
                browser = p.chromium.launch(executable_path=shutil.which('google-chrome'), headless=True, args=['--no-sandbox'])
                first = browser.new_context(viewport={'width':1440,'height':1100})
                operator = first.new_page()
                operator.on('pageerror', lambda e: errors.append(str(e)))
                operator.goto(f'http://127.0.0.1:{port}')
                expect(operator.locator('#connection')).to_have_text('实时连接')
                operator.locator('[data-view="mapping"]').first.click()
                expect(operator.locator('#mappingStart')).to_be_disabled()
                operator.locator('#controlButton').click()
                expect(operator.locator('#mappingStart')).to_be_enabled()
                second = browser.new_context()
                observer = second.new_page()
                observer.on('pageerror', lambda e: errors.append(str(e)))
                observer.goto(f'http://127.0.0.1:{port}')
                expect(observer.locator('#connection')).to_have_text('实时连接')
                observer.locator('[data-view="mapping"]').first.click()
                expect(observer.locator('#mappingStart')).to_be_disabled()
                operator.locator('#mappingName').fill('建图界面验证')
                operator.locator('#mappingStart').click()
                expect(operator.locator('#mappingPhase')).to_have_text('采集中')
                expect(observer.locator('#mappingPhase')).to_have_text('采集中')
                expect(operator.locator('#mappingPreviewStatus')).to_have_text('实时点云')
                operator.evaluate('const actualNow = Date.now; Date.now = () => actualNow() + 3600000')
                operator.wait_for_timeout(700)
                expect(operator.locator('#mappingPreviewStatus')).to_have_text('实时点云')
                expect(operator.locator('#newWaypointButton')).to_be_disabled()
                runner.failure = 'mapping.save'
                operator.locator('#mappingSave').click()
                expect(operator.locator('#mappingOperation')).to_have_text('保存地图 · 未成功')
                expect(operator.locator('#mappingPhase')).to_have_text('采集中')
                runner.failure = ''
                operator.locator('#mappingSave').click()
                expect(operator.locator('#mappingOperation')).to_have_text('保存地图 · 已完成')
                expect(operator.locator('#mappingDownload')).to_be_visible()
                with operator.expect_download() as downloading:
                    operator.locator('#mappingDownload').click()
                assert Path(downloading.value.path()).read_bytes() == PCD
                expect(operator.locator('#mappingLogs')).to_contain_text('fixture cloud ready')
                operator.evaluate('document.fonts.ready')
                operator.evaluate('scrollTo(0, 0)')
                operator.screenshot(path=str(previews / 'mapping-light.png'), full_page=True)
                operator.locator('#themeButton').click()
                operator.screenshot(path=str(previews / 'mapping-dark.png'), full_page=True)
                operator.reload()
                expect(operator.locator('#connection')).to_have_text('实时连接')
                operator.locator('[data-view="mapping"]').first.click()
                expect(operator.locator('#mappingPhase')).to_have_text('采集中')
                assert runner.calls.count('mapping.start') == 1
                operator.locator('#mappingFinish').click()
                expect(operator.locator('#mappingPhase')).to_have_text('未运行')
                expect(operator.locator('#mappingOperation')).to_have_text('保存并结束 · 已完成')
                operator.locator('#mappingStart').click()
                expect(operator.locator('#mappingPhase')).to_have_text('采集中')
                operator.set_viewport_size({'width':390,'height':844})
                operator.emulate_media(reduced_motion='reduce')
                assert operator.evaluate('document.documentElement.scrollWidth <= innerWidth')
                operator.locator('#mappingSave').focus()
                operator.keyboard.press('Tab')
                assert operator.evaluate('getComputedStyle(document.activeElement).outlineWidth') == '3px'
                operator.evaluate('scrollTo(0, 0)')
                operator.screenshot(path=str(previews / 'mapping-mobile.png'), full_page=True)
                operator.locator('#mappingStop').click()
                expect(operator.locator('#mappingPhase')).to_have_text('未运行')
                expect(operator.locator('#mappingOutputDetail')).to_contain_text('尚未生成有效 PCD')
                first.set_offline(True)
                expect(operator.locator('#connection')).to_have_text('连接中 / 状态过期', timeout=7000)
                expect(operator.locator('#mappingStart')).to_be_disabled()
                first.set_offline(False)
                expect(operator.locator('#connection')).to_have_text('实时连接', timeout=15000)
                assert not errors, errors
                browser.close()
        finally:
            server.should_exit = True
            thread.join(timeout=8)
            runner.cleanup()
        assert not thread.is_alive()
    print('Mapping browser checks passed: start/save/failure/finish/stop/download, preview, multiple viewers, reload, mobile/themes/focus/offline; fixture processes only.')


if __name__ == '__main__':
    main()
