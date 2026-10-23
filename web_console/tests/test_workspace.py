"""Exercise real file writes in disposable workspaces, never the site configuration."""
import fcntl
import io
from pathlib import Path
import tempfile
import unittest

from fastapi.testclient import TestClient
from PIL import Image
import yaml

from g1_web.app import create_app
from g1_web.config import Settings


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="g1-web-test-")
        self.root = Path(self.temp.name)
        self.points = self.root / "task_data/maps/laboratory_map_v1/waypoints.yaml"
        self.missions = self.root / "task_data/missions/missions.yaml"
        self.map_yaml = self.root / "maps/map_multilayer.yaml"
        for path in (self.points, self.missions, self.map_yaml):
            path.parent.mkdir(parents=True, exist_ok=True)
        self.dump(self.points, {"schema_version": 1, "map_id": "test_lab", "frame_id": "map", "base_frame": "nav_base",
                               "waypoints": {"point_01": {"name": "现场门口", "pose": {"x": 1.5, "y": -2.0, "yaw": 0.0},
                                                          "recorded_at": "2026-01-01T00:00:00Z", "note": "保留扩展元数据"}}})
        self.dump(self.missions, {"schema_version": 1, "map_id": "test_lab", "missions": {
            "lab_tour": {"name": "现场巡航", "steps": [{"waypoint": "point_01", "advance_mode": "manual",
                "speech": {"mode": "tts", "language": "zh", "content": {"zh": {
                    "speaker_id": 0, "text": "现场讲解", "wait_sec": 5}}, "failure_policy": "abort"}}]}}})
        self.dump(self.map_yaml, {"image": "floor.pgm", "resolution": 0.05, "origin": [-8.5, -26.5, 0.0]})
        image = Image.new("L", (12, 18), 254)
        image.putpixel((3, 4), 0)
        image.save(self.root / "maps/floor.pgm")
        self.app = create_app(Settings(workspace=str(self.root)))
        self.client = TestClient(self.app).__enter__()
        self.lease = self.client.post("/api/v1/control/acquire", json={"owner": "fixture editor"}).json()["lease_id"]

    @staticmethod
    def dump(path, value):
        path.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8")

    def tearDown(self):
        self.client.__exit__(None, None, None)
        self.temp.cleanup()

    def catalog(self):
        response = self.client.get("/api/v1/catalog")
        self.assertEqual(response.status_code, 200)
        return response.json()

    def headers(self, revision=None):
        return {"X-Control-Lease": self.lease, "If-Match": '"' + (revision or self.catalog()["revision"]) + '"'}

    def point_payload(self, **changes):
        return dict(name="更改后的现场点", x=2.25, y=-1.75, yaw=0.5, **changes)

    def test_catalog_and_raster_come_from_backend_without_demo_fallback(self):
        catalog = self.catalog()
        self.assertFalse(catalog["simulated"])
        self.assertEqual(catalog["waypoints"][0]["name"], "现场门口")
        self.assertEqual(catalog["missions"][0]["steps"][0]["speech"]["content"]["zh"]["text"], "现场讲解")
        metadata = self.client.get("/api/v1/map").json()
        self.assertEqual((metadata["width"], metadata["height"]), (12, 18))
        self.assertEqual(metadata["origin"], [-8.5, -26.5, 0.0])
        response = self.client.get(metadata["image_url"])
        image = Image.open(io.BytesIO(response.content))
        self.assertEqual(image.getpixel((3, 4)), 0)
        self.assertEqual(self.client.get(metadata["image_url"], headers={"If-None-Match": response.headers["etag"]}).status_code, 304)
        self.assertEqual(self.client.get("/api/v1/map/image?version=old").status_code, 409)
        state = self.client.get("/api/v1/state").json()["data"]
        self.assertEqual(state["mode"], "local")
        self.assertTrue(state["configuration"]["available"])
        self.assertTrue(state["capabilities"]["waypoint_write"])
        self.assertIsNone(state["pose"])
        self.map_yaml.unlink()
        self.points.unlink()
        state = self.client.get("/api/v1/state").json()["data"]
        self.assertFalse(state["map"]["available"])
        self.assertFalse(state["configuration"]["available"])
        self.assertEqual(self.client.get("/api/v1/catalog").status_code, 503)

    def test_save_writes_same_yaml_preserves_metadata_and_backs_up_both_files(self):
        original_points, original_missions = self.points.read_bytes(), self.missions.read_bytes()
        revision = self.catalog()["revision"]
        response = self.client.put("/api/v1/waypoints/point_01", headers=self.headers(revision), json=self.point_payload())
        self.assertEqual(response.status_code, 200, response.text)
        saved = response.json()
        self.assertNotEqual(saved["revision"], revision)
        value = yaml.safe_load(self.points.read_text())["waypoints"]["point_01"]
        self.assertEqual(value["pose"]["x"], 2.25)
        self.assertEqual(value["recorded_at"], "2026-01-01T00:00:00Z")
        self.assertEqual(value["note"], "保留扩展元数据")
        self.assertIn("edited_at", value)
        self.assertEqual((self.root / saved["backup"] / "waypoints.yaml").read_bytes(), original_points)
        self.assertEqual((self.root / saved["backup"] / "missions.yaml").read_bytes(), original_missions)
        self.assertEqual(self.missions.read_bytes(), original_missions)
        # A second page sees the new version on the state channel.
        with self.client.websocket_connect("/ws/state") as socket:
            self.assertEqual(socket.receive_json()["data"]["configuration"]["revision"], saved["revision"])

    def test_edit_requires_lease_and_revision_and_stale_drafts_do_not_overwrite(self):
        path = "/api/v1/waypoints/point_01"
        original = self.points.read_bytes()
        self.assertEqual(self.client.put(path, json=self.point_payload()).status_code, 409)
        self.assertEqual(self.client.put(path, headers={"X-Control-Lease": self.lease}, json=self.point_payload()).status_code, 428)
        revision = self.catalog()["revision"]
        self.points.write_text(self.points.read_text().replace("现场门口", "外部修改"))
        externally_changed = self.points.read_bytes()
        self.assertEqual(self.client.put(path, headers=self.headers(revision), json=self.point_payload()).status_code, 409)
        self.assertEqual(self.points.read_bytes(), externally_changed)
        self.assertNotEqual(original, externally_changed)
        self.assertEqual(self.catalog()["waypoints"][0]["name"], "外部修改")

    def test_create_delete_and_reference_checks(self):
        response = self.client.post("/api/v1/waypoints", headers=self.headers(), json=dict(id="point_02", **self.point_payload()))
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(len(self.catalog()["waypoints"]), 2)
        self.assertEqual(self.client.delete("/api/v1/waypoints/point_01", headers=self.headers()).status_code, 409)
        self.assertEqual(self.client.delete("/api/v1/waypoints/point_02", headers=self.headers()).status_code, 200)
        mission = {"id": "second_tour", "name": "新增路线", "steps": [{"waypoint": "point_01"}]}
        self.assertEqual(self.client.post("/api/v1/missions", headers=self.headers(), json=mission).status_code, 200)
        self.assertEqual(self.client.delete("/api/v1/missions/second_tour", headers=self.headers()).status_code, 200)

    def test_mission_speech_edit_and_invalid_cross_reference_is_atomic(self):
        mission = self.catalog()["missions"][0]
        payload = {"name": mission["name"], "steps": mission["steps"]}
        payload["steps"][0]["speech"]["content"]["zh"]["text"] = "浏览器写入的讲解"
        response = self.client.put("/api/v1/missions/lab_tour", headers=self.headers(), json=payload)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertIn("浏览器写入的讲解", self.missions.read_text())
        original = self.missions.read_bytes()
        payload["steps"][0]["waypoint"] = "missing_point"
        response = self.client.put("/api/v1/missions/lab_tour", headers=self.headers(), json=payload)
        self.assertEqual(response.status_code, 422, response.text)
        self.assertEqual(self.missions.read_bytes(), original)
        self.assertFalse(list(self.missions.parent.glob("*.web_*")))

    def test_task_and_cli_locks_and_recovery_prevent_config_changes(self):
        runtime = self.root / "runtime/missions"
        runtime.mkdir(parents=True, exist_ok=True)
        original = self.points.read_bytes()
        for path in (runtime / "mission.lock", self.points.with_name(".waypoints.yaml.g1_waypoints.lock")):
            with path.open("a") as handle:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                response = self.client.put("/api/v1/waypoints/point_01", headers=self.headers(), json=self.point_payload())
                self.assertEqual(response.status_code, 409, response.text)
        (runtime / "recovery_required.json").write_text("{}")
        response = self.client.put("/api/v1/waypoints/point_01", headers=self.headers(), json=self.point_payload())
        self.assertEqual(response.json()["error"]["code"], "RECOVERY_REQUIRED")
        self.assertEqual(self.points.read_bytes(), original)
        self.assertTrue(self.client.get("/api/v1/state").json()["data"]["configuration"]["write_blocker"])

    def test_map_and_mission_external_changes_update_revisions(self):
        first_map = self.client.get("/api/v1/map").json()
        image = Image.open(self.root / "maps/floor.pgm")
        image.putpixel((2, 2), 0)
        image.save(self.root / "maps/floor.pgm")
        self.assertNotEqual(self.client.get("/api/v1/map").json()["revision"], first_map["revision"])
        revision = self.catalog()["revision"]
        self.missions.write_text(self.missions.read_text().replace("现场巡航", "外部改名"))
        self.assertNotEqual(self.client.get("/api/v1/state").json()["data"]["configuration"]["revision"], revision)

    def test_strict_validation_rejects_invalid_numeric_and_unknown_fields(self):
        original = self.points.read_bytes()
        for changes in ({"x": True}, {"yaw": "nan"}, {"extra": 1}):
            payload = self.point_payload()
            payload.update(changes)
            response = self.client.put("/api/v1/waypoints/point_01", headers=self.headers(), json=payload)
            self.assertEqual(response.status_code, 422)
        self.assertEqual(self.points.read_bytes(), original)

    def test_unquoted_yaml_timestamp_metadata_is_readable_and_preserved(self):
        self.points.write_text(self.points.read_text().replace("'2026-01-01T00:00:00Z'", "2026-01-01T00:00:00Z"))
        self.assertIn("2026-01-01", self.catalog()["waypoints"][0]["recorded_at"])
        response = self.client.put("/api/v1/waypoints/point_01", headers=self.headers(), json=self.point_payload())
        self.assertEqual(response.status_code, 200, response.text)
        self.assertIn("2026-01-01", str(yaml.safe_load(self.points.read_text())["waypoints"]["point_01"]["recorded_at"]))


if __name__ == "__main__":
    unittest.main()
