"""Shared live files: validation, optimistic concurrency, CLI locks and backups."""
import copy
import fcntl
import hashlib
import io
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
from contextlib import ExitStack, contextmanager
from datetime import datetime, timezone

import yaml
from PIL import Image

from .control import GatewayError
from .paths import TASK_SOURCE, site_file

# Use the same source validators as g1_waypoints.sh, without needing a ROS overlay.
if str(TASK_SOURCE) not in sys.path:
    sys.path.insert(0, str(TASK_SOURCE))
from g1_task_manager.waypoint_store import UniqueKeyLoader, WaypointStore, WaypointConfigError
from g1_task_manager.mission_store import load_missions


def digest(data):
    return hashlib.sha256(data).hexdigest()


class WorkspaceData:
    def __init__(self, settings):
        self.root = Path(settings.workspace).expanduser().resolve()
        self.waypoints = site_file(self.root, settings.waypoints_file, "task_data/maps/laboratory_map_v1/waypoints.yaml")
        self.missions = site_file(self.root, settings.missions_file, "task_data/missions/missions.yaml")
        self.map_yaml = site_file(self.root, settings.map_yaml, "maps/map_multilayer.yaml")
        self.map_override = site_file(self.root, settings.map_pgm, "") if settings.map_pgm else None
        self.runtime = self.root / "runtime/missions"
        self.lock = threading.RLock()
        self.cached_catalog = self.catalog_signature = None
        self.cached_map = self.map_signature = self.png = None

    @staticmethod
    def signature(paths):
        return tuple((str(path), path.stat().st_mtime_ns, path.stat().st_size, path.stat().st_ino) for path in paths)

    def catalog(self):
        with self.lock:
            try:
                signature = self.signature((self.waypoints, self.missions))
                if signature == self.catalog_signature:
                    return copy.deepcopy(self.cached_catalog)
                original_points, original_missions = self.waypoints.read_bytes(), self.missions.read_bytes()
                store = WaypointStore(self.waypoints).load()
                normalized = load_missions(self.missions, store)
                raw = yaml.load(original_missions.decode("utf-8"), Loader=UniqueKeyLoader)
                if original_points != self.waypoints.read_bytes() or original_missions != self.missions.read_bytes():
                    raise GatewayError(409, "DATA_CHANGED", "配置正在被其他程序修改，请刷新后重试")
                revision = digest(original_points + b"\0" + original_missions)
                result = {"simulated": False, "source": "workspace", "map_id": store.map_id,
                          "frame_id": store.frame_id, "revision": revision,
                          "waypoints": [dict(value, id=key) for key, value in store.list_waypoints()],
                          "missions": [{"id": key, "name": mission["name"],
                                        "waypoints": [step["waypoint"] for step in mission["steps"]],
                                        "steps": copy.deepcopy(raw["missions"][key]["steps"]),
                                        "advance_mode": "manual" if all(step["advance_mode"] == "manual" for step in mission["steps"]) else "mixed"}
                                       for key, mission in normalized.items()],
                          "files": {"waypoints": str(self.waypoints), "missions": str(self.missions),
                                    "map_yaml": str(self.map_yaml)}}
                self.catalog_signature, self.cached_catalog = signature, result
                return copy.deepcopy(result)
            except GatewayError:
                raise
            except (OSError, ValueError, yaml.YAMLError, WaypointConfigError, TypeError) as exc:
                raise GatewayError(503, "CATALOG_UNAVAILABLE", "本地配置不可用：" + str(exc)) from exc

    def map(self):
        with self.lock:
            try:
                raw_yaml = self.map_yaml.read_bytes()
                data = yaml.load(raw_yaml.decode("utf-8"), Loader=UniqueKeyLoader)
                resolution = data["resolution"]
                origin = data["origin"]
                if (isinstance(resolution, bool) or not isinstance(resolution, (int, float))
                        or not math.isfinite(resolution) or resolution <= 0 or not isinstance(origin, list)
                        or len(origin) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                                or not math.isfinite(v) for v in origin)):
                    raise ValueError("地图 resolution / origin 无效")
                image_path = self.map_override or (self.map_yaml.parent / data["image"]).resolve()
                signature = self.signature((self.map_yaml, image_path))
                if signature != self.map_signature:
                    image_bytes = image_path.read_bytes()
                    with Image.open(io.BytesIO(image_bytes)) as image:
                        width, height = image.size
                        buffer = io.BytesIO()
                        image.convert("L").save(buffer, format="PNG")
                    revision = digest(raw_yaml + b"\0" + image_bytes)
                    self.png = buffer.getvalue()
                    self.cached_map = {"available": True, "simulated": False, "revision": revision,
                                       "width": width, "height": height, "resolution": float(resolution),
                                       "origin": [float(v) for v in origin], "frame_id": "map",
                                       "image_url": "/api/v1/map/image?version=" + revision,
                                       "yaml_name": self.map_yaml.name, "image_name": image_path.name}
                    self.map_signature = signature
                return copy.deepcopy(self.cached_map)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, WaypointConfigError) as exc:
                raise GatewayError(503, "MAP_UNAVAILABLE", "本地地图不可用：" + str(exc)) from exc

    def map_image(self, version):
        with self.lock:
            metadata = self.map()
            if version and version != metadata["revision"]:
                raise GatewayError(409, "MAP_CHANGED", "地图已更新，请刷新地图")
            return self.png, metadata["revision"]

    def blocker(self):
        if (self.runtime / "recovery_required.json").exists():
            return "存在未完成收尾的任务，请先按项目恢复流程确认停止"
        path = self.runtime / "mission.lock"
        if path.exists():
            try:
                with path.open("r") as handle:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return "任务运行中，暂不能编辑导航点或任务配置"
        return ""

    def status(self):
        status = {"source": "workspace", "available": False, "revision": "", "error": "", "write_blocker": ""}
        try:
            catalog = self.catalog()
            status.update(available=True, revision=catalog["revision"], map_id=catalog["map_id"],
                          write_blocker=self.blocker())
        except (GatewayError, OSError) as exc:
            status["error"] = exc.message if isinstance(exc, GatewayError) else str(exc)
        try:
            map_data = self.map()
        except GatewayError as exc:
            map_data = {"available": False, "simulated": False, "error": exc.message}
        return status, map_data

    @contextmanager
    def edit_lock(self):
        with self.lock, ExitStack() as stack:
            paths = [self.runtime / "mission.lock",
                     self.waypoints.with_name("." + self.waypoints.name + ".g1_waypoints.lock"),
                     self.missions.with_name("." + self.missions.name + ".g1_missions.lock")]
            try:
                self.runtime.mkdir(parents=True, exist_ok=True)
                for path in paths:
                    handle = stack.enter_context(path.open("a"))
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise GatewayError(409, "DATA_BUSY", "已有任务或配置写入操作，请等待完成") from exc
            except OSError as exc:
                raise GatewayError(503, "DATA_UNWRITABLE", "配置目录不可写：" + str(exc)) from exc
            if (self.runtime / "recovery_required.json").exists():
                raise GatewayError(409, "RECOVERY_REQUIRED", "存在停止未确认的任务，不能修改现场配置")
            yield

    def save(self, kind, operation, identifier, payload, expected_revision):
        if not expected_revision:
            raise GatewayError(428, "REVISION_REQUIRED", "保存需要 If-Match 配置版本")
        with self.edit_lock():
            current = self.catalog()
            if expected_revision.strip('"') != current["revision"]:
                raise GatewayError(409, "REVISION_CONFLICT", "后端配置已变化，未覆盖；请重新加载后编辑")
            original_points, original_missions = self.waypoints.read_bytes(), self.missions.read_bytes()
            # Match the exact bytes used by validation, including changes by external editors.
            if digest(original_points + b"\0" + original_missions) != current["revision"]:
                raise GatewayError(409, "REVISION_CONFLICT", "配置已被其他程序修改，请刷新")
            target = self.waypoints if kind == "waypoints" else self.missions
            data = yaml.load((original_points if kind == "waypoints" else original_missions).decode("utf-8"), Loader=UniqueKeyLoader)
            entries = data[kind]
            if operation == "create" and identifier in entries:
                raise GatewayError(409, "ID_EXISTS", "该 ID 已存在")
            if operation != "create" and identifier not in entries:
                raise GatewayError(404, "ID_NOT_FOUND", "该 ID 不存在")
            if operation == "delete":
                if kind == "waypoints":
                    refs = [mission["id"] for mission in current["missions"] if identifier in mission["waypoints"]]
                    if refs:
                        raise GatewayError(409, "WAYPOINT_REFERENCED", "导航点仍被任务引用：" + ", ".join(refs))
                del entries[identifier]
            elif kind == "waypoints":
                value = copy.deepcopy(entries.get(identifier, {}))
                value.update(name=payload["name"].strip(), pose={key: payload[key] for key in ("x", "y", "yaw")},
                             edited_at=datetime.now(timezone.utc).isoformat(timespec="seconds"))
                value["pose"]["yaw"] = math.atan2(math.sin(value["pose"]["yaw"]), math.cos(value["pose"]["yaw"]))
                entries[identifier] = value
            else:
                entries[identifier] = {"name": payload["name"].strip(), "steps": payload["steps"]}
            try:
                fd, name = tempfile.mkstemp(prefix="." + target.name + ".web_", dir=target.parent)
            except OSError as exc:
                raise GatewayError(503, "DATA_UNWRITABLE", "配置目录不可写：" + str(exc)) from exc
            temporary = Path(name)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                store = WaypointStore(temporary if kind == "waypoints" else self.waypoints).load()
                load_missions(temporary if kind == "missions" else self.missions, store)
                if original_points != self.waypoints.read_bytes() or original_missions != self.missions.read_bytes():
                    raise GatewayError(409, "REVISION_CONFLICT", "保存期间文件已变化，未覆盖")
                root = self.root / "backup/web_console"
                root.mkdir(parents=True, exist_ok=True)
                (self.root / "backup/COLCON_IGNORE").touch(exist_ok=True)
                backup = Path(tempfile.mkdtemp(prefix=datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_") + operation + "_", dir=root))
                shutil.copy2(self.waypoints, backup / "waypoints.yaml")
                shutil.copy2(self.missions, backup / "missions.yaml")
                os.chmod(temporary, target.stat().st_mode & 0o777)
                os.replace(temporary, target)
                directory = os.open(target.parent, os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
                self.catalog_signature = None
                catalog = self.catalog()
                return {"saved": True, "revision": catalog["revision"], "catalog": catalog,
                        "backup": str(backup.relative_to(self.root)), "message": "已保存到后端并备份原配置"}
            except GatewayError:
                raise
            except (WaypointConfigError, ValueError, yaml.YAMLError, TypeError) as exc:
                raise GatewayError(422, "CONFIG_INVALID", str(exc)) from exc
            except OSError as exc:
                raise GatewayError(503, "SAVE_FAILED", "保存失败：" + str(exc)) from exc
            finally:
                temporary.unlink(missing_ok=True)
