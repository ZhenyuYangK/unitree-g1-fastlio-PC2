"""Browser writes against an explicit disposable copy of a workspace.

Start a local gateway with G1_WS pointing at a copy under /tmp, then set
G1_WEB_TEST_URL and G1_WEB_TEST_WORKSPACE. Refuses to modify the real workspace.
"""
import json
import os
from pathlib import Path
import shutil
from urllib.request import urlopen

from playwright.sync_api import sync_playwright, expect
import yaml


def main():
    url = os.environ["G1_WEB_TEST_URL"]
    fixture = Path(os.environ["G1_WEB_TEST_WORKSPACE"]).resolve()
    assert Path("/tmp") in fixture.parents and fixture != Path("/tmp"), "Use a disposable workspace under /tmp"
    with urlopen(url + "/api/v1/catalog") as response:
        catalog = json.load(response)
    points = Path(catalog["files"]["waypoints"])
    missions = Path(catalog["files"]["missions"])
    assert fixture in points.parents and fixture in missions.parents, "Refusing site writes"
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=shutil.which("google-chrome"), headless=True, args=["--no-sandbox"])
        first = browser.new_context(viewport={"width": 1440, "height": 1100})
        operator = first.new_page()
        operator.on("pageerror", lambda e: errors.append(str(e)))
        operator.goto(url)
        expect(operator.locator("#mapMode")).to_have_text("后端地图")
        expect(operator.locator("#signStopName")).to_have_text("讲解点1")
        assert operator.locator("#demoMapLayer").is_hidden()
        assert operator.locator("#liveMapImage").is_visible()
        expect(operator.locator("#startButton")).to_be_disabled()
        operator.locator("#controlButton").click()
        expect(operator.locator("#newWaypointButton")).to_be_enabled()
        second = browser.new_context(viewport={"width": 1280, "height": 900})
        observer = second.new_page()
        observer.on("pageerror", lambda e: errors.append(str(e)))
        observer.goto(url)
        expect(observer.locator("#controlButton")).to_be_disabled()
        observer.locator('.nav-item[data-view="waypoints"]').click()

        # Edit through an actual map marker, persist to the original YAML and update another page.
        operator.locator('#waypointLayer [data-waypoint-id="point_01"]').click()
        expect(operator.locator("#waypointDialog")).to_be_visible()
        operator.locator('#waypointForm [name="name"]').fill("测试门口")
        operator.locator('#waypointForm [name="x"]').fill("9.5")
        operator.locator("#saveWaypointButton").click()
        expect(operator.locator("#waypointDialog")).not_to_be_visible()
        expect(observer.locator('#waypointTable tr[data-point-id="point_01"]')).to_contain_text("测试门口")
        saved = yaml.safe_load(points.read_text())["waypoints"]["point_01"]
        assert saved["name"] == "测试门口" and saved["pose"]["x"] == 9.5
        assert saved.get("recorded_at"), "Original sampling metadata lost"
        assert list((fixture / "backup/web_console").glob("*/waypoints.yaml"))

        # Existing task references prevent an unsafe point deletion.
        operator.locator('.nav-item[data-view="waypoints"]').click()
        operator.locator('#waypointTable tr[data-point-id="point_01"] button').click()
        operator.locator("#deleteWaypointButton").click()
        expect(operator.locator("#waypointError")).to_contain_text("仍被任务引用")
        operator.locator('[data-close="waypointDialog"]').click()

        # Reorder the real task and change its speech, not a browser-only draft.
        operator.locator('.nav-item[data-view="missions"]').click()
        operator.locator("#missionCatalog .data-edit").click()
        rows = operator.locator("#missionSteps .step-editor")
        expect(rows).to_have_count(5)
        rows.nth(0).locator('[data-field="zh_text"]').fill("网页修改后的中文讲解")
        rows.nth(1).locator('[data-step-action="up"]').click()
        operator.locator("#saveMissionButton").click()
        expect(operator.locator("#missionDialog")).not_to_be_visible()
        data = yaml.safe_load(missions.read_text())["missions"]["five_point_patrol"]
        assert data["steps"][0]["waypoint"] == "point_02"
        assert data["steps"][1]["speech"]["content"]["zh"]["text"] == "网页修改后的中文讲解"
        expect(observer.locator("#signStopName")).to_have_text("讲解点2")

        # External changes update both pages; an already open draft keeps its old revision.
        operator.locator('.nav-item[data-view="waypoints"]').click()
        operator.locator('#waypointTable tr[data-point-id="point_01"] button').click()
        operator.locator('#waypointForm [name="name"]').fill("不应覆盖")
        value = yaml.safe_load(points.read_text())
        value["waypoints"]["point_01"]["name"] = "外部工具更新"
        points.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
        expect(observer.locator('#waypointTable tr[data-point-id="point_01"]')).to_contain_text("外部工具更新")
        operator.locator("#saveWaypointButton").click()
        expect(operator.locator("#waypointError")).to_contain_text("后端配置已变化")
        assert yaml.safe_load(points.read_text())["waypoints"]["point_01"]["name"] == "外部工具更新"
        operator.locator("#reloadWaypointButton").click()
        expect(operator.locator('#waypointForm [name="name"]')).to_have_value("外部工具更新")
        operator.locator('[data-close="waypointDialog"]').click()

        # Create a point from map coordinates and then remove the unreferenced point.
        operator.locator('.nav-item[data-view="overview"]').click()
        operator.locator("#mapNewPoint").click()
        operator.locator("#map").click(position={"x": 260, "y": 260})
        expect(operator.locator("#waypointDialog")).to_be_visible()
        new_id = operator.locator('#waypointForm [name="id"]').input_value()
        operator.locator('#waypointForm [name="name"]').fill("地图选点测试")
        operator.locator("#saveWaypointButton").click()
        expect(operator.locator("#waypointDialog")).not_to_be_visible()
        expect(observer.locator("#waypointTable tr")).to_have_count(6)
        operator.locator('.nav-item[data-view="waypoints"]').click()
        operator.locator(f'#waypointTable tr[data-point-id="{new_id}"] button').click()
        operator.locator("#deleteWaypointButton").click()
        expect(operator.locator("#waypointDialog")).not_to_be_visible()
        expect(observer.locator("#waypointTable tr")).to_have_count(5)

        # New tasks save from the form; defaults in older sparse YAML keep their meaning.
        operator.locator('.nav-item[data-view="missions"]').click()
        operator.locator("#newMissionButton").click()
        mission_id = operator.locator('#missionForm [name="id"]').input_value()
        operator.locator('#missionForm [name="name"]').fill("页面新建测试")
        operator.locator("#saveMissionButton").click()
        expect(operator.locator("#missionDialog")).not_to_be_visible()
        assert mission_id in yaml.safe_load(missions.read_text())["missions"]
        operator.locator(".catalog-card").filter(has_text="页面新建测试").locator(".data-edit").click()
        operator.locator("#deleteMissionButton").click()
        expect(operator.locator("#missionDialog")).not_to_be_visible()
        assert mission_id not in yaml.safe_load(missions.read_text())["missions"]
        value = yaml.safe_load(missions.read_text())
        value["missions"]["sparse_tour"] = {"name": "旧配置默认值", "steps": [{"waypoint": "point_01", "speech": {
            "mode": "tts", "language": "zh", "content": {
                "zh": {"text": "中文", "speaker_id": 0, "wait_sec": 5},
                "en": {"text": "Keep this translation", "speaker_id": 1, "wait_sec": 6}}}}]}
        missions.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
        card = operator.locator(".catalog-card").filter(has_text="旧配置默认值")
        expect(card).to_be_visible()
        card.locator(".data-edit").click()
        expect(operator.locator('[data-field="advance_mode"]')).to_have_value("auto")
        expect(operator.locator('[data-field="timeout_sec"]')).to_have_value("60")
        expect(operator.locator('[data-field="dwell_sec"]')).to_have_value("2")
        operator.locator("#saveMissionButton").click()
        expect(operator.locator("#missionDialog")).not_to_be_visible()
        step = yaml.safe_load(missions.read_text())["missions"]["sparse_tour"]["steps"][0]
        assert step["speech"]["content"]["en"]["text"] == "Keep this translation"
        card.locator(".data-edit").click()
        operator.locator("#deleteMissionButton").click()
        expect(operator.locator("#missionDialog")).not_to_be_visible()

        # Forms remain usable on a narrow screen, including keyboard focus and reduced motion.
        operator.set_viewport_size({"width": 390, "height": 844})
        operator.locator('.nav-item[data-view="waypoints"]').click()
        operator.locator('#waypointTable tr[data-point-id="point_01"] button').click()
        assert operator.evaluate("document.documentElement.scrollWidth <= window.innerWidth")
        assert operator.locator("#waypointDialog").evaluate("el => el.scrollWidth <= el.clientWidth")
        operator.locator('#waypointForm [name="name"]').focus()
        operator.keyboard.press("Tab")
        assert operator.evaluate("getComputedStyle(document.activeElement).outlineWidth") == "3px"
        operator.locator('[data-close="waypointDialog"]').click()
        operator.locator('.nav-item[data-view="overview"]').click()
        operator.locator("#themeButton").click()
        operator.emulate_media(reduced_motion="reduce")
        assert operator.locator("#progressBar").evaluate("el => getComputedStyle(el).transitionDuration") == "0s"
        assert operator.evaluate("document.documentElement.scrollWidth <= window.innerWidth")
        operator.screenshot(path="/tmp/g1-console-sync-mobile.png", full_page=True)
        first.set_offline(True)
        expect(operator.locator("#connection")).to_have_text("连接中 / 状态过期", timeout=7000)
        expect(operator.locator("#newWaypointButton")).to_be_disabled()
        first.set_offline(False)
        expect(operator.locator("#connection")).to_have_text("实时连接", timeout=15000)
        assert not errors, errors
        browser.close()
    print("Browser sync checks passed: real map, point/speech/order persistence, backups, multi-page/external sync, reference/conflict rejection, map picking, mobile/focus and offline recovery.")


if __name__ == "__main__":
    main()
