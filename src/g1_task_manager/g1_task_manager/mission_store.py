"""严格校验整条路线；任何一步无效，都不能发送第一段目标。"""
import math
from pathlib import Path

import yaml

from .waypoint_store import UniqueKeyLoader, WaypointConfigError, WAYPOINT_ID_PATTERN


def _number(value, field, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointConfigError('{}必须是数字'.format(field))
    value = float(value)
    if not math.isfinite(value) or (value <= 0 if positive else value < 0):
        raise WaypointConfigError('{}必须是有限{}数'.format(field, '正' if positive else '非负'))
    return value


def _keys(data, allowed, field):
    if not isinstance(data, dict):
        raise WaypointConfigError('{}必须是字典'.format(field))
    unknown = set(data) - set(allowed)
    if unknown:
        raise WaypointConfigError('{}包含未知字段：{}'.format(field, repr(unknown)))


def load_missions(path, waypoints):
    try:
        data = yaml.load(Path(path).read_text(encoding='utf-8'), Loader=UniqueKeyLoader)
    except (yaml.YAMLError, UnicodeError, TypeError) as exc:
        raise WaypointConfigError('任务YAML读取失败：{}'.format(exc)) from exc
    _keys(data, ('schema_version', 'map_id', 'missions'), '任务文件')
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise WaypointConfigError('任务schema_version必须为整数1')
    if data.get('map_id') != waypoints.map_id:
        raise WaypointConfigError('任务map_id与导航点文件不一致')
    missions = data.get('missions')
    if not isinstance(missions, dict):
        raise WaypointConfigError('missions必须是字典')
    normalized = {}
    for mission_id, raw in missions.items():
        if not isinstance(mission_id, str) or not WAYPOINT_ID_PATTERN.fullmatch(mission_id):
            raise WaypointConfigError('任务ID格式无效：{!r}'.format(mission_id))
        _keys(raw, ('name', 'steps'), mission_id)
        name = raw.get('name', mission_id)
        if not isinstance(name, str) or not name.strip():
            raise WaypointConfigError('任务name必须是非空字符串')
        steps = raw.get('steps')
        if not isinstance(steps, list) or not steps:
            raise WaypointConfigError('任务steps必须是非空列表')
        normalized_steps = []
        for index, step in enumerate(steps, 1):
            field = '{}.steps[{}]'.format(mission_id, index)
            _keys(step, ('waypoint', 'timeout_sec', 'dwell_sec'), field)
            waypoint_id = step.get('waypoint')
            waypoint = waypoints.get_waypoint(waypoint_id)
            normalized_steps.append({
                'waypoint': waypoint_id,
                'goal': waypoint,
                'timeout_sec': _number(step.get('timeout_sec', 60.0), field + '.timeout_sec', True),
                'dwell_sec': _number(step.get('dwell_sec', 2.0), field + '.dwell_sec'),
            })
        normalized[mission_id] = {'id': mission_id, 'name': name.strip(), 'steps': normalized_steps}
    return normalized
