import asyncio
import time
import unittest
import uuid

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from g1_web.app import create_app
from g1_web.adapters.mock import MockAdapter
from g1_web.adapters.ros_readonly import localization_view
from g1_web.config import Settings
from g1_web.control import ControlLease, GatewayError


def request_id():
    return str(uuid.uuid4())


class GatewayFlowTests(unittest.TestCase):
    def setUp(self):
        self.settings = Settings(mode="mock", mock_travel_seconds=0.02, mock_speech_seconds=0.01)
        self.app = create_app(self.settings)
        self.client = TestClient(self.app)
        self.client.__enter__()
        self.headers = {}

    def tearDown(self):
        self.client.__exit__(None, None, None)

    def state(self):
        return self.client.get("/api/v1/state").json()["data"]

    def acquire(self):
        response = self.client.post("/api/v1/control/acquire", json={"owner": "test browser"})
        self.assertEqual(response.status_code, 200)
        self.headers = {"X-Control-Lease": response.json()["lease_id"]}
        return response.json()

    def post(self, action, **payload):
        return self.client.post("/api/v1/missions/" + action, headers=self.headers,
                                json=dict(request_id=request_id(), **payload))

    def start(self):
        self.acquire()
        response = self.post("start", mission_id="five_point_tour")
        self.assertEqual(response.status_code, 202)
        return response.json()["run_id"]

    def wait(self, predicate):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            mission = self.state()["mission"]
            if predicate(mission):
                return mission
            time.sleep(0.005)
        self.fail("task did not reach expected state: " + str(self.state()["mission"]))

    def test_static_page_and_mock_health(self):
        self.assertEqual(self.client.get("/api/v1/health").json()["mode"], "mock")
        self.assertIn('id="modeText"', self.client.get("/").text)
        self.assertEqual(self.client.get("/app.js").status_code, 200)
        self.assertEqual(self.client.get("/styles.css").status_code, 200)
        self.assertEqual(len(self.client.get("/api/v1/catalog").json()["waypoints"]), 5)

    def test_only_one_controller_and_no_lease_secret_broadcast(self):
        clock = [100.0]
        self.app.state.lease.clock = lambda: clock[0]
        lease = self.acquire()
        self.assertEqual(lease["expires_in"], 300.0)
        self.assertEqual(self.client.post("/api/v1/control/acquire", json={"owner": "another browser"}).status_code, 409)
        self.assertNotIn(lease["lease_id"], self.client.get("/api/v1/state").text)
        clock[0] += 299.0
        renewed = self.client.post("/api/v1/control/renew", headers=self.headers)
        self.assertEqual(renewed.status_code, 200)
        self.assertEqual(renewed.json()["remaining_sec"], 300.0)
        clock[0] += 299.0
        self.assertEqual(self.client.post("/api/v1/control/acquire", json={"owner": "another browser"}).status_code, 409)
        self.assertEqual(self.client.post("/api/v1/control/release", headers=self.headers).status_code, 200)
        self.assertEqual(self.post("start", mission_id="five_point_tour").status_code, 409)

    def test_commands_require_controller_and_reject_extra_fields(self):
        self.assertEqual(self.post("start", mission_id="five_point_tour").status_code, 409)
        self.acquire()
        self.assertEqual(self.post("start", mission_id="five_point_tour", execute_real=True).status_code, 422)

    def test_duplicate_start_returns_same_result_and_conflicting_id_rejected(self):
        self.acquire()
        payload = {"request_id": request_id(), "mission_id": "five_point_tour"}
        first = self.client.post("/api/v1/missions/start", headers=self.headers, json=payload)
        second = self.client.post("/api/v1/missions/start", headers=self.headers, json=payload)
        self.assertEqual(first.status_code, 202)
        self.assertEqual(first.json(), second.json())
        self.assertEqual(self.post("start", mission_id="five_point_tour").status_code, 409)
        payload["mission_id"] = "other_mission"
        conflict = self.client.post("/api/v1/missions/start", headers=self.headers, json=payload)
        self.assertEqual(conflict.json()["error"]["code"], "REQUEST_ID_CONFLICT")

    def test_all_five_stations_require_unique_release_including_last(self):
        run_id = self.start()
        previous_wait = "expired"
        for index in range(1, 6):
            current = self.wait(lambda m: m["state"] == "WAITING_FOR_CONTINUE" and m["current_step"] == index)
            self.assertEqual(current["run_id"], run_id)
            self.assertTrue(current["active"])
            self.assertTrue(current["gate_closed"])
            self.assertEqual(self.post("continue", run_id=run_id, wait_id=previous_wait).status_code, 409)
            payload = {"request_id": request_id(), "run_id": run_id, "wait_id": current["wait_id"]}
            response = self.client.post("/api/v1/missions/continue", headers=self.headers, json=payload)
            self.assertEqual(response.status_code, 202)
            replay = self.client.post("/api/v1/missions/continue", headers=self.headers, json=payload)
            self.assertEqual(response.json(), replay.json())
            previous_wait = current["wait_id"]
        final = self.wait(lambda m: not m["active"])
        self.assertEqual(final["state"], "SUCCEEDED")
        self.assertTrue(final["gate_closed"] and final["goals_terminal"])

    def test_cancel_binds_to_run_and_returns_terminal_confirmation(self):
        run_id = self.start()
        self.assertEqual(self.post("cancel", run_id="old-run").status_code, 409)
        self.assertEqual(self.post("cancel", run_id=run_id).status_code, 202)
        stopped = self.wait(lambda m: m["state"] == "CANCELED")
        self.assertFalse(stopped["active"])
        self.assertTrue(stopped["gate_closed"] and stopped["goals_terminal"])
        self.assertEqual(self.post("start", mission_id="five_point_tour").status_code, 202)
        self.assertEqual(self.post("cancel", run_id=run_id).status_code, 409)

    def test_expired_lease_does_not_end_autonomous_mock_mission(self):
        clock = [100.0]
        self.app.state.lease.clock = lambda: clock[0]
        self.start()
        clock[0] += 299.0
        self.assertTrue(self.state()["control"]["held"])
        clock[0] += 1.1
        self.assertTrue(self.state()["mission"]["active"])
        self.assertFalse(self.state()["control"]["held"])
        self.assertEqual(self.post("cancel", run_id=self.state()["mission"]["run_id"]).status_code, 409)
        self.acquire()
        self.assertTrue(self.state()["control"]["held"])

    def test_websocket_reconnect_keeps_backend_run(self):
        run_id = self.start()
        with self.client.websocket_connect("/ws/state") as socket:
            first = socket.receive_json()
            socket.send_text("ping")
            second = socket.receive_json()
            self.assertGreater(second["sequence"], first["sequence"])
        with self.client.websocket_connect("/ws/state") as socket:
            reconnected = socket.receive_json()
            self.assertEqual(reconnected["instance_id"], first["instance_id"])
            self.assertEqual(reconnected["data"]["mission"]["run_id"], run_id)

    def test_process_control_is_explicitly_unimplemented(self):
        response = self.client.post("/api/v1/operations", json={"request_id": request_id(), "operation": "mapping.start"})
        self.assertEqual(response.status_code, 501)


class AuthenticationTests(unittest.TestCase):
    def test_login_http_websocket_origin_and_logout(self):
        with TestClient(create_app(Settings(mode="mock", token="test-fixture-only"))) as client:
            self.assertEqual(client.get("/api/v1/state").status_code, 401)
            with self.assertRaises(WebSocketDisconnect) as rejected:
                with client.websocket_connect("/ws/state"):
                    pass
            self.assertEqual(rejected.exception.code, 4401)
            self.assertEqual(client.post("/api/v1/session", json={"token": "wrong"}).status_code, 401)
            response = client.post("/api/v1/session", json={"token": "test-fixture-only"})
            self.assertIn("HttpOnly", response.headers["set-cookie"])
            self.assertEqual(client.get("/api/v1/state").status_code, 200)
            self.assertEqual(client.get("/api/v1/openapi.json").status_code, 200)
            self.assertEqual(client.post("/api/v1/control/acquire", headers={"Origin": "https://foreign.example"}, json={"owner": "x"}).status_code, 403)
            with self.assertRaises(WebSocketDisconnect) as rejected:
                with client.websocket_connect("/ws/state", headers={"Origin": "https://foreign.example"}):
                    pass
            self.assertEqual(rejected.exception.code, 4403)
            with client.websocket_connect("/ws/state", headers={"Origin": "http://testserver"}) as socket:
                self.assertTrue(socket.receive_json()["data"]["simulated"])
            self.assertEqual(client.delete("/api/v1/session").status_code, 200)
            self.assertEqual(client.get("/api/v1/state").status_code, 401)

    def test_readonly_mode_cannot_enable_writes_even_with_injected_capable_adapter(self):
        mock = MockAdapter(Settings())
        with TestClient(create_app(Settings(mode="ros_readonly", token="test-fixture"), adapter=mock)) as client:
            client.post("/api/v1/session", json={"token": "test-fixture"})
            self.assertEqual(client.post("/api/v1/control/acquire", json={"owner": "operator"}).status_code, 200)
            self.assertEqual(client.post("/api/v1/missions/start", json={"request_id": request_id(), "mission_id": "five_point_tour"}).status_code, 501)
            self.assertIsNone(mock.task)


class DomainTests(unittest.TestCase):
    def test_modes_and_lan_require_explicit_configuration(self):
        for kwargs in ({"mode": "ros"}, {"mode": "ros_readonly"}, {"host": "0.0.0.0"}):
            with self.assertRaises(ValueError):
                Settings(**kwargs)
        self.assertNotIn("test-secret-value", repr(Settings(token="test-secret-value")))

    def test_lease_rejects_stale_credential_after_transfer(self):
        clock = [0.0]
        lease = ControlLease(seconds=15.0, clock=lambda: clock[0])
        first = lease.acquire("A")
        clock[0] = 16.0
        second = lease.acquire("B")
        with self.assertRaises(GatewayError):
            lease.renew(first["lease_id"])
        lease.require(second["lease_id"])

    def test_localization_rejects_retained_old_status_and_old_scan(self):
        now = 10_000_000_000
        valid = {"state": "TRACKING", "valid": True, "stamp_ns": now, "last_success_stamp_ns": now}
        self.assertTrue(localization_view(valid, now, 0.0)["valid"])
        for changed, receive_age in (({"stamp_ns": now - 2_000_000_000}, 0.0),
                                     ({"last_success_stamp_ns": now - 4_000_000_000}, 0.0),
                                     ({"state": "LOST"}, 0.0), ({"stamp_ns": True}, 0.0), ({}, 1.1)):
            self.assertFalse(localization_view(dict(valid, **changed), now, receive_age)["valid"])

    def test_fresh_initialization_progress_is_visible_without_a_confirmed_pose(self):
        now = 10_000_000_000
        for state in ['COLLECTING', 'VERIFYING_CANDIDATES', 'RETRYING', 'AMBIGUOUS']:
            raw = dict(state=state, valid=False, stamp_ns=now, last_success_stamp_ns=0,
                       attempt=2, candidates=[dict(candidate=1, score=.8)])
            data = localization_view(raw, now, 0.0)
            self.assertFalse(data['status_stale'])
            self.assertTrue(data['stale'])
            self.assertFalse(data['valid'])
            self.assertEqual(data['candidates'], raw['candidates'])
            self.assertTrue(localization_view(raw, now, 1.1)['status_stale'])


class AsyncMockTests(unittest.IsolatedAsyncioTestCase):
    async def test_immediate_cancel_before_coroutine_first_runs(self):
        adapter = MockAdapter(Settings())
        result = await adapter.start_mission("five_point_tour")
        await adapter.cancel_mission(result["run_id"])
        await adapter.close()
        self.assertEqual((await adapter.snapshot())["mission"]["state"], "CANCELED")


if __name__ == "__main__":
    unittest.main()
