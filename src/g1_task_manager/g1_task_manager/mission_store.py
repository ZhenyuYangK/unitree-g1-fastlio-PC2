"""严格校验整条路线；任何一步无效，都不能发送第一段目标。"""
import math
from pathlib import Path

import yaml

from .waypoint_store import UniqueKeyLoader, WaypointConfigError, WAYPOINT_ID_PATTERN


def _number(value, field, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointConfigError('{}必须是数字'.format(field))
    try:
        value = float(value)
    except OverflowError as exc:
        raise WaypointConfigError('{}必须是有限数字'.format(field)) from exc
    if not math.isfinite(value) or (value <= 0 if positive else value < 0):
        raise WaypointConfigError('{}必须是有限{}数'.format(field, '正' if positive else '非负'))
    return value


def _keys(data, allowed, field):
    if not isinstance(data, dict):
        raise WaypointConfigError('{}必须是字典'.format(field))
    unknown = set(data) - set(allowed)
    if unknown:
        raise WaypointConfigError('{}包含未知字段：{}'.format(field, repr(unknown)))


def _speech(raw, field):
    _keys(raw, ('mode', 'language', 'content', 'failure_policy'), field)
    if raw.get('mode') != 'tts':
        raise WaypointConfigError('{}的mode只支持tts'.format(field))
    language = raw.get('language')
    if language not in ('zh', 'en', 'both'):
        raise WaypointConfigError('{}的language必须为zh、en或both'.format(field))
    if raw.get('failure_policy', 'abort') != 'abort':
        raise WaypointConfigError('{}的failure_policy只支持abort'.format(field))

    content = raw.get('content')
    _keys(content, ('zh', 'en'), field + '.content')
    selected = ('zh', 'en') if language == 'both' else (language,)
    for selected_language in selected:
        if selected_language not in content:
            raise WaypointConfigError('{}缺少选定语言{}'.format(field, selected_language))

    normalized_content = {}
    for language_id, entry in content.items():
        entry_field = '{}.content.{}'.format(field, language_id)
        _keys(entry, ('speaker_id', 'text', 'wait_sec'), entry_field)
        normalized_entry = {}
        required = language_id in selected
        if required or 'speaker_id' in entry:
            speaker_id = entry.get('speaker_id')
            # Unitree TtsMaker takes int32_t; bool must not pass as an integer.
            if type(speaker_id) is not int or not 0 <= speaker_id <= 2147483647:
                raise WaypointConfigError(
                    '{}.speaker_id必须是0到2147483647之间的整数'.format(entry_field))
            normalized_entry['speaker_id'] = speaker_id
        if required or 'text' in entry:
            text = entry.get('text')
            if not isinstance(text, str) or not text.strip():
                raise WaypointConfigError('{}.text必须是非空字符串'.format(entry_field))
            normalized_entry['text'] = text.strip()
        if required or 'wait_sec' in entry:
            normalized_entry['wait_sec'] = _number(
                entry.get('wait_sec'), entry_field + '.wait_sec', True)
        normalized_content[language_id] = normalized_entry
    return {
        'mode': 'tts',
        'language': language,
        'content': normalized_content,
        'failure_policy': 'abort',
    }


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
            _keys(step, ('waypoint', 'timeout_sec', 'dwell_sec', 'speech', 'advance_mode'), field)
            waypoint_id = step.get('waypoint')
            waypoint = waypoints.get_waypoint(waypoint_id)
            advance_mode = step.get('advance_mode', 'auto')
            if advance_mode not in ('auto', 'manual'):
                raise WaypointConfigError('{}.advance_mode必须为auto或manual'.format(field))
            normalized_steps.append({
                'waypoint': waypoint_id,
                'goal': waypoint,
                'timeout_sec': _number(step.get('timeout_sec', 60.0), field + '.timeout_sec', True),
                'dwell_sec': _number(step.get('dwell_sec', 2.0), field + '.dwell_sec'),
                'speech': _speech(step['speech'], field + '.speech') if 'speech' in step else None,
                'advance_mode': advance_mode,
            })
        normalized[mission_id] = {'id': mission_id, 'name': name.strip(), 'steps': normalized_steps}
    return normalized
