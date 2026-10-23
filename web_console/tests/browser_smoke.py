"""Optional real-browser checks against a running mock gateway (no robot connection).

Install playwright in the test environment and use an existing Chrome, or install
Playwright Chromium separately. Example: G1_WEB_TEST_URL=http://127.0.0.1:8080 python tests/browser_smoke.py
"""
import os
import shutil

from playwright.sync_api import sync_playwright, expect


def main():
    url = os.getenv("G1_WEB_TEST_URL", "http://127.0.0.1:8080")
    chrome = os.getenv("G1_WEB_TEST_CHROME") or shutil.which("google-chrome")
    errors = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=chrome, headless=True,
                                             args=["--no-sandbox"])
        first = browser.new_context(viewport={"width": 1440, "height": 1100}, color_scheme="light", timezone_id="America/Los_Angeles")
        operator = first.new_page()
        operator.on("pageerror", lambda error: errors.append(str(error)))
        operator.goto(url)
        expect(operator.locator("#connection")).to_have_text("实时连接")
        operator.evaluate("document.fonts.ready")
        assert operator.evaluate('document.fonts.check(\'700 24px "Barlow Condensed"\')'), "stop font not loaded"
        assert operator.locator("#signBanner").evaluate("el => getComputedStyle(el).borderBottomWidth") == "6px"
        assert operator.locator(".panel").first.evaluate("el => getComputedStyle(el).boxShadow") == "none"
        operator.locator(".brand").focus()
        operator.keyboard.press("Tab")
        assert operator.evaluate("getComputedStyle(document.activeElement).outlineWidth") == "3px", "keyboard focus not visible"
        operator.locator("#controlButton").click()
        expect(operator.locator("#controlValue")).to_have_text("本设备控制")

        second = browser.new_context(viewport={"width": 1280, "height": 900})
        observer = second.new_page()
        observer.on("pageerror", lambda error: errors.append(str(error)))
        observer.goto(url)
        expect(observer.locator("#connection")).to_have_text("实时连接")
        expect(observer.locator("#controlButton")).to_be_disabled()

        operator.bring_to_front()
        expect(operator.locator("#controlValue")).to_have_text("本设备控制")
        operator.locator("#startButton").click()
        expect(operator.locator("#missionValue")).to_have_text("等待放行", timeout=12000)
        run_id = operator.locator("#runId").get_attribute("title")
        operator.reload()
        expect(operator.locator("#controlValue")).to_have_text("本设备控制")
        expect(operator.locator("#runId")).to_have_attribute("title", run_id)
        expect(observer.locator("#runId")).to_have_attribute("title", run_id)
        expect(operator.locator('#stopList [data-point-id="point_01"]')).to_have_class("stop-row is-current")
        expect(operator.locator("#signStopName")).to_have_text("接待区")
        expect(operator.locator("#continueButton")).to_be_enabled()
        operator.locator("#continueButton").click()
        expect(operator.locator("#stepDetail")).to_have_text("point_02")
        expect(operator.locator("#missionValue")).to_have_text("等待放行", timeout=12000)
        expect(operator.locator('#stopList [data-point-id="point_01"]')).to_have_class("stop-row is-complete")
        expect(operator.locator('#stopList [data-point-id="point_02"]')).to_have_class("stop-row is-current")
        expect(operator.locator("#signStopName")).to_have_text("研究展示区")
        expect(operator.locator("#toast")).to_be_hidden(timeout=6000)
        operator.evaluate("new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))")
        operator.screenshot(path="/tmp/g1-console-wayfinding-light.png", full_page=True, animations="disabled")
        operator.locator("#themeButton").click()
        expect(operator.locator("html")).to_have_attribute("data-theme", "dark")
        assert operator.locator("html").evaluate("el => getComputedStyle(el).backgroundColor") == "rgb(11, 23, 48)"
        operator.evaluate("new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))")
        operator.screenshot(path="/tmp/g1-console-wayfinding-dark.png", full_page=True, animations="disabled")
        operator.reload()
        expect(operator.locator("html")).to_have_attribute("data-theme", "dark")
        expect(operator.locator("#connection")).to_have_text("实时连接")
        assert operator.locator("#signBanner").evaluate("el => { el.classList.add('emergency-active'); const color = getComputedStyle(el).borderBottomColor; el.classList.remove('emergency-active'); return color; }") == "rgb(255, 107, 94)"
        expect(operator.locator("#emergencyButton")).to_have_count(0)
        assert operator.locator("#cancelButton").evaluate("el => getComputedStyle(el).backgroundColor") == "rgb(18, 38, 75)"
        operator.locator("#cancelButton").click()
        expect(operator.locator("#missionValue")).to_have_text("已取消")
        operator.locator("#controlButton").click()
        expect(observer.locator("#controlButton")).to_be_enabled()
        observer.locator("#controlButton").click()
        expect(observer.locator("#controlValue")).to_have_text("本设备控制")
        observer.locator('[data-view="waypoints"]').click()
        expect(observer.locator("#waypointTable tr")).to_have_count(5)
        observer.locator('[data-view="missions"]').click()
        expect(observer.locator(".catalog-card")).to_have_count(1)
        observer.locator('[data-view="integration"]').click()
        expect(observer.locator(".integration-grid button:disabled")).to_have_count(1)

        mobile = browser.new_context(viewport={"width": 390, "height": 844}, is_mobile=True,
                                     device_scale_factor=1, has_touch=True, color_scheme="dark", reduced_motion="reduce", timezone_id="America/Los_Angeles")
        phone = mobile.new_page()
        phone.on("pageerror", lambda error: errors.append(str(error)))
        phone.goto(url)
        expect(phone.locator("#connection")).to_have_text("实时连接")
        assert phone.evaluate("document.documentElement.scrollWidth <= window.innerWidth"), "mobile page overflow"
        assert phone.locator("#robotMarker").evaluate("el => getComputedStyle(el).transitionDuration") == "0s"
        assert phone.locator("#progressBar").evaluate("el => getComputedStyle(el).transitionDuration") == "0s"
        phone.screenshot(path="/tmp/g1-console-wayfinding-mobile.png", full_page=True)
        phone.set_viewport_size({"width": 320, "height": 844})
        assert phone.evaluate("document.documentElement.scrollWidth <= window.innerWidth"), "narrow mobile page overflow"
        first.set_offline(True)
        expect(operator.locator("#connection")).to_have_text("连接中 / 状态过期", timeout=7000)
        expect(operator.locator("#controlButton")).to_be_disabled()
        first.set_offline(False)
        expect(operator.locator("#connection")).to_have_text("实时连接", timeout=15000)
        assert not errors, errors
        print("Browser checks passed: wayfinding light/dark, offline fonts, keyboard focus, completed/current stops, mobile/reduced motion, controller transfer, reload and offline/reconnect; no page errors.")
        browser.close()


if __name__ == "__main__":
    main()
