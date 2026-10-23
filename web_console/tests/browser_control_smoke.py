"""Mobile control recovery against an isolated mock server; never uses the robot.

PYTHONPATH=web_console/backend web_console/.venv/bin/python web_console/tests/browser_control_smoke.py
"""
import json
import re
import shutil
import socket
import tempfile
import threading
import time

import uvicorn
from playwright.sync_api import expect, sync_playwright

from g1_web.app import create_app
from g1_web.config import Settings


def visibility(page, hidden):
    # Exercise the mobile visibility handler without relying on OS lock-screen APIs.
    page.evaluate("""hidden => {
      Object.defineProperty(document, 'hidden', {configurable:true, get:() => hidden});
      document.dispatchEvent(new Event('visibilitychange'));
    }""", hidden)


def main():
    with tempfile.TemporaryDirectory(prefix="g1-mobile-control-") as workspace:
        app = create_app(Settings(mode="mock", workspace=workspace))
        offset = [0.0]
        app.state.lease.clock = lambda: time.monotonic() + offset[0]
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        url = f"http://127.0.0.1:{listener.getsockname()[1]}"
        server = uvicorn.Server(uvicorn.Config(app, log_level="error"))
        thread = threading.Thread(target=server.run, kwargs={"sockets": [listener]}, daemon=True)
        thread.start()
        deadline = time.monotonic() + 10
        while not server.started:
            if not thread.is_alive() or time.monotonic() > deadline:
                raise RuntimeError("mock server did not start")
            time.sleep(0.01)
        try:
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(executable_path=shutil.which("google-chrome"),
                                                     headless=True, args=["--no-sandbox"])
                context = browser.new_context(viewport={"width": 390, "height": 844},
                                              is_mobile=True, has_touch=True)
                phone = context.new_page()
                errors, requests = [], []
                phone.on("pageerror", lambda error: errors.append(str(error)))
                phone.on("request", lambda request: requests.append(request.url))
                phone.goto(url)
                expect(phone.locator("#connection")).to_have_text("实时连接")
                phone.locator("#controlButton").click()
                expect(phone.locator("#controlValue")).to_have_text("本设备控制")
                expect(phone.locator("#controlTime")).to_have_text(re.compile(r"操作剩余时间：0[45]:\d{2}"))
                expect(phone.locator("#controlTime")).to_be_visible()
                expect(phone.locator("#controlDetail")).to_have_text(re.compile(r"操作剩余时间：0[45]:\d{2}"))
                expect(phone.locator("#emergencyButton")).to_have_count(0)
                expect(phone.locator("#emergencyNote")).to_have_count(0)
                secret = phone.evaluate("sessionStorage.getItem('g1_control_lease')")
                assert app.state.lease.seconds == 300.0
                acquire_count = requests.count(url + "/api/v1/control/acquire")

                # One failed HTTP request must preserve credentials and allow recovery.
                for failure in ("network", "server"):
                    def fail(route):
                        if failure == "network":
                            route.abort("failed")
                        else:
                            route.fulfill(status=503, content_type="application/json",
                                          body=json.dumps({"error": {"code": "TEMPORARY", "message": "retry"}}))
                    phone.route("**/api/v1/control/renew", fail)
                    phone.evaluate("renew()")
                    expect(phone.locator("#controlValue")).to_have_text("正在核验操作权")
                    expect(phone.locator("#startButton")).to_be_disabled()
                    assert phone.evaluate("sessionStorage.getItem('g1_control_lease')") == secret
                    phone.unroute("**/api/v1/control/renew", fail)
                    phone.evaluate("renew()")
                    expect(phone.locator("#controlValue")).to_have_text("本设备控制")
                    expect(phone.locator("#startButton")).to_be_enabled()

                # Network recovery renews the same credential without another acquire.
                context.set_offline(True)
                phone.evaluate("renew()")
                assert phone.evaluate("sessionStorage.getItem('g1_control_lease')") == secret
                context.set_offline(False)
                expect(phone.locator("#connection")).to_have_text("实时连接", timeout=15000)
                expect(phone.locator("#controlValue")).to_have_text("本设备控制", timeout=15000)

                # Returning from the background before five minutes keeps ownership.
                visibility(phone, True)
                offset[0] += 240.0
                visibility(phone, False)
                expect(phone.locator("#controlValue")).to_have_text("本设备控制")
                assert phone.evaluate("sessionStorage.getItem('g1_control_lease')") == secret
                assert requests.count(url + "/api/v1/control/acquire") == acquire_count

                # A request taking longer than the timer interval must not overlap.
                held = []
                phone.route("**/api/v1/control/renew", lambda route: held.append(route))
                phone.evaluate("() => { window.firstRenew = renew(); window.secondRenew = renew(); }")
                phone.wait_for_timeout(5500)
                assert len(held) == 1, f"overlapping renew requests: {len(held)}"
                held[0].fulfill(response=held[0].fetch())
                phone.evaluate("Promise.all([window.firstRenew, window.secondRenew])")
                phone.unroute("**/api/v1/control/renew")

                # A late error for a released credential must not clear a newly acquired one.
                held.clear()
                phone.route("**/api/v1/control/renew", lambda route: held.append(route))
                phone.evaluate("() => { window.oldRenew = renew(); }")
                phone.wait_for_timeout(200)
                assert len(held) == 1
                phone.locator("#controlButton").click()
                expect(phone.locator("#controlValue")).to_have_text("暂无控制设备")
                phone.locator("#controlButton").click()
                expect(phone.locator("#controlValue")).to_have_text("本设备控制")
                new_secret = phone.evaluate("sessionStorage.getItem('g1_control_lease')")
                assert new_secret != secret
                held[0].fulfill(status=409, content_type="application/json",
                                body=json.dumps({"error": {"code": "CONTROL_REQUIRED", "message": "expired"}}))
                phone.evaluate("window.oldRenew")
                phone.unroute("**/api/v1/control/renew")
                assert phone.evaluate("sessionStorage.getItem('g1_control_lease')") == new_secret
                expect(phone.locator("#controlValue")).to_have_text("本设备控制")

                # An actual expiry permits another controller; the phone cannot reclaim it.
                visibility(phone, True)
                offset[0] += 301.0
                response = context.request.post(url + "/api/v1/control/acquire", data={"owner": "other device"})
                assert response.status == 200
                visibility(phone, False)
                expect(phone.locator("#controlValue")).to_have_text("other device")
                expect(phone.locator("#controlButton")).to_be_disabled()
                assert phone.evaluate("sessionStorage.getItem('g1_control_lease')") is None
                assert not errors, errors
                for width in (390, 320):
                    phone.set_viewport_size({"width": width, "height": 844})
                    assert phone.evaluate("document.documentElement.scrollWidth <= innerWidth"), "mobile overflow"
                phone.screenshot(path="/tmp/g1-mobile-control-recovery.png", full_page=True)
                print("Mobile control checks passed: 300-second validity, transient failure, offline/online, foreground recovery, serialized renewal, stale response, expiry/transfer, countdown and removed placeholder; no page errors.")
                browser.close()
        finally:
            server.should_exit = True
            thread.join(timeout=10)
            listener.close()


if __name__ == "__main__":
    main()
