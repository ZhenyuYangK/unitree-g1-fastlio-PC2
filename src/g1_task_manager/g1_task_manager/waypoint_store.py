import math
import os
import re
import shutil
from datetime import datetime
from pathlib import Path

import yaml


WAYPOINT_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


class WaypointConfigError(RuntimeError):
    pass


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}

    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)

        if key in mapping:
            line = key_node.start_mark.line + 1
            raise WaypointConfigError(
                f"YAML第{line}行包含重复键：{key!r}"
            )

        value = loader.construct_object(value_node, deep=deep)
        mapping[key] = value

    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


class WaypointStore:
    def __init__(self, file_path):
        self.file_path = Path(file_path)
        self.map_id = ""
        self.frame_id = ""
        self.base_frame = ""
        self._waypoints = {}
        self._data = {}

    def load(self):
        if not self.file_path.is_file():
            raise WaypointConfigError(
                f"导航点文件不存在：{self.file_path}"
            )

        try:
            text = self.file_path.read_text(encoding="utf-8")
            data = yaml.load(text, Loader=UniqueKeyLoader)
        except UnicodeDecodeError as exc:
            raise WaypointConfigError(
                f"导航点文件不是有效UTF-8：{exc}"
            ) from exc
        except yaml.YAMLError as exc:
            raise WaypointConfigError(
                f"YAML语法错误：{exc}"
            ) from exc

        if not isinstance(data, dict):
            raise WaypointConfigError(
                "YAML顶层必须是字典"
            )

        if data.get("schema_version") != 1:
            raise WaypointConfigError(
                "schema_version必须为1"
            )

        self.map_id = self._require_nonempty_string(
            data.get("map_id"),
            "map_id",
        )

        self.frame_id = self._require_nonempty_string(
            data.get("frame_id"),
            "frame_id",
        )

        self.base_frame = self._require_nonempty_string(
            data.get("base_frame"),
            "base_frame",
        )

        if self.frame_id != "map":
            raise WaypointConfigError(
                f"frame_id必须为'map'，当前为{self.frame_id!r}"
            )

        if self.base_frame != "nav_base":
            raise WaypointConfigError(
                "base_frame必须为'nav_base'，"
                f"当前为{self.base_frame!r}"
            )

        waypoints = data.get("waypoints")

        if not isinstance(waypoints, dict):
            raise WaypointConfigError(
                "waypoints必须是字典"
            )

        normalized = {}

        for waypoint_id, raw_waypoint in waypoints.items():
            self._validate_waypoint_id(waypoint_id)

            if not isinstance(raw_waypoint, dict):
                raise WaypointConfigError(
                    f"导航点{waypoint_id!r}的内容必须是字典"
                )

            name = raw_waypoint.get("name", waypoint_id)

            if not isinstance(name, str) or not name.strip():
                raise WaypointConfigError(
                    f"导航点{waypoint_id!r}的name必须是非空字符串"
                )

            pose = raw_waypoint.get("pose")

            if not isinstance(pose, dict):
                raise WaypointConfigError(
                    f"导航点{waypoint_id!r}缺少pose字典"
                )

            normalized_pose = {}

            for field_name in ("x", "y", "yaw"):
                value = pose.get(field_name)

                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                ):
                    raise WaypointConfigError(
                        f"导航点{waypoint_id!r}的"
                        f"{field_name}必须是数字"
                    )

                numeric_value = float(value)

                if not math.isfinite(numeric_value):
                    raise WaypointConfigError(
                        f"导航点{waypoint_id!r}的"
                        f"{field_name}必须是有限数字"
                    )

                normalized_pose[field_name] = numeric_value

            waypoint = dict(raw_waypoint)
            waypoint["name"] = name.strip()
            waypoint["pose"] = normalized_pose
            normalized[waypoint_id] = waypoint

        self._waypoints = normalized
        self._data = data
        return self

    @staticmethod
    def _require_nonempty_string(value, field_name):
        if not isinstance(value, str) or not value.strip():
            raise WaypointConfigError(
                f"{field_name}必须是非空字符串"
            )

        return value.strip()

    @staticmethod
    def _validate_waypoint_id(waypoint_id):
        if (
            not isinstance(waypoint_id, str)
            or WAYPOINT_ID_PATTERN.fullmatch(waypoint_id) is None
        ):
            raise WaypointConfigError(
                "导航点ID必须以小写字母开头，"
                "并且只能包含小写字母、数字和下划线："
                f"{waypoint_id!r}"
            )

    def has_waypoint(self, waypoint_id):
        self._validate_waypoint_id(waypoint_id)
        return waypoint_id in self._waypoints

    def record_waypoint(
        self,
        waypoint_id,
        name,
        x,
        y,
        yaw,
        overwrite=False,
    ):
        self._validate_waypoint_id(waypoint_id)

        exists = waypoint_id in self._waypoints

        if exists and not overwrite:
            raise WaypointConfigError(
                f"导航点已经存在：{waypoint_id}；"
                "如需更新请添加--overwrite"
            )

        if name is None:
            if exists:
                name = self._waypoints[waypoint_id]["name"]
            else:
                name = waypoint_id

        if not isinstance(name, str) or not name.strip():
            raise WaypointConfigError(
                "导航点name必须是非空字符串"
            )

        numeric_pose = {}

        for field_name, value in (
            ("x", x),
            ("y", y),
            ("yaw", yaw),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
            ):
                raise WaypointConfigError(
                    f"{field_name}必须是数字"
                )

            numeric_value = float(value)

            if not math.isfinite(numeric_value):
                raise WaypointConfigError(
                    f"{field_name}必须是有限数字"
                )

            numeric_pose[field_name] = numeric_value

        numeric_pose["yaw"] = math.atan2(
            math.sin(numeric_pose["yaw"]),
            math.cos(numeric_pose["yaw"]),
        )

        data = dict(self._data)
        waypoints = dict(data["waypoints"])

        waypoints[waypoint_id] = {
            "name": name.strip(),
            "pose": numeric_pose,
            "recorded_at": (
                datetime.now()
                .astimezone()
                .isoformat(timespec="seconds")
            ),
        }

        data["waypoints"] = waypoints

        backup_path = self.file_path.with_name(
            self.file_path.name
            + ".before_record_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
        )

        shutil.copy2(self.file_path, backup_path)

        temporary_path = self.file_path.with_name(
            "." + self.file_path.name + ".tmp"
        )

        try:
            with temporary_path.open(
                "w",
                encoding="utf-8",
            ) as handle:
                yaml.safe_dump(
                    data,
                    handle,
                    allow_unicode=True,
                    sort_keys=False,
                    default_flow_style=False,
                )
                handle.flush()
                os.fsync(handle.fileno())

            os.chmod(
                temporary_path,
                self.file_path.stat().st_mode,
            )

            os.replace(
                temporary_path,
                self.file_path,
            )
        finally:
            if temporary_path.exists():
                temporary_path.unlink()

        self.load()
        return backup_path

    def list_waypoints(self):
        return [
            (waypoint_id, self._waypoints[waypoint_id])
            for waypoint_id in sorted(self._waypoints)
        ]

    def get_waypoint(self, waypoint_id):
        self._validate_waypoint_id(waypoint_id)

        if waypoint_id not in self._waypoints:
            raise WaypointConfigError(
                f"导航点不存在：{waypoint_id}"
            )

        return self._waypoints[waypoint_id]
