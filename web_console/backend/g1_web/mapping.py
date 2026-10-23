from .compat import to_thread
"""Versioned, asynchronous mapping operations over the existing fixed CLI."""
import asyncio
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import sys
import time
import uuid

from .control import GatewayError

from .paths import SCRIPT_SOURCE
if str(SCRIPT_SOURCE) not in sys.path:
    sys.path.insert(0, str(SCRIPT_SOURCE))
from .runtime import processes, process, group_state, pcd_info

ACTIONS = {'mapping.start': 'start', 'mapping.save': 'save', 'mapping.finish': 'finish', 'mapping.stop': 'stop'}
ACTIVE = {'QUEUED', 'RUNNING'}


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return default


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as handle:
        json.dump(value, handle, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def tail(path, limit=8000):
    try:
        with Path(path).open('rb') as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - limit))
            value = handle.read(limit).decode('utf-8', errors='replace')
        # Control sequences from CLI colors do not belong in product text.
        return re.sub(r'\x1b\[[0-9;]*m', '', value)
    except OSError:
        return ''


class MappingManager:
    def __init__(self, settings, runner=None):
        self.settings = settings
        self.root = Path(settings.workspace).resolve()
        self.runtime = self.root / 'runtime/mapping'
        self.records = self.runtime / 'web'
        self.script = self.root / 'script/g1_mapping.sh'
        self.enabled = settings.mode != 'mock' and self.script.is_file()
        self.runner = runner or self._run_script
        self.job = None
        self.child = None
        self.active_operation_id = None

    def operation(self, identifier):
        if self.settings.mode == 'mock':
            raise GatewayError(501, 'MAPPING_UNAVAILABLE', '模拟模式没有现场建图操作')
        try:
            identifier = str(uuid.UUID(identifier))
        except ValueError as exc:
            raise GatewayError(404, 'OPERATION_NOT_FOUND', '操作不存在') from exc
        record = read_json(self.records / (identifier + '.json'))
        if not record:
            raise GatewayError(404, 'OPERATION_NOT_FOUND', '操作不存在')
        if record['state'] in ACTIVE and not (self.job and not self.job.done() and identifier == self.active_operation_id):
            pid = record.get('command_pid')
            if not pid or group_state(pid, record.get('command_token')) != 'running':
                record = dict(record, state='INTERRUPTED', message='网关或命令已中断，结果未确认；请核对进程与文件后操作')
        return record

    def latest(self):
        identifier = read_json(self.records / 'latest.json', {}).get('operation_id')
        return self.operation(identifier) if identifier else None

    def _navigation_running(self):
        for filename in ('navigation.pids', 'real_bridge.pid', 'audio_bridge.pid', 'web/task_manager.pid'):
            path = self.root / 'runtime/navigation' / filename
            if not path.exists():
                continue
            if any(item['state'] != 'stopped' for item in processes(path)):
                return True
        return False

    def _blocker(self):
        if (self.root / 'runtime/missions/recovery_required.json').exists():
            return '存在停止未确认的任务，不能启动建图'
        for path, message in [(self.root / 'runtime/missions/mission.lock', '任务运行中'),
                              (self.root / 'runtime/stack.lock', '建图或导航正在切换')]:
            if path.exists():
                with path.open('r') as handle:
                    try:
                        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        return message
        if self._navigation_running():
            return '定位、导航或运动桥仍在运行，请先按原管理方式结束'
        return ''

    def session(self):
        value = read_json(self.runtime / 'session.json')
        if value:
            return dict(value, id=value.get('session_id') or value.get('id'))
        # Legacy sessions remain visible; do not source executable session.env.
        import shlex
        path = self.runtime / 'session.env'
        if path.exists():
            for line in path.read_text().splitlines():
                parts = shlex.split(line)
                if len(parts) == 1 and parts[0].startswith('MAP_OUTPUT='):
                    return {'id': 'legacy', 'output': parts[0].split('=', 1)[1], 'started_at': ''}
        return None

    def inspect(self):
        if self.settings.mode == 'mock':
            return {'available': False, 'phase': 'STOPPED', 'active': False, 'busy': False,
                    'can_start': False, 'can_save': False, 'can_finish': False, 'can_stop': False,
                    'session': None, 'operation': None, 'processes': [], 'blocker': '', 'error': ''}
        try:
            items = processes(self.runtime / 'mapping.pids')
            session = self.session()
            current = self.latest()
            known = {(p['pid'], str(p['token'])) for p in items}
            for item in (current or {}).get('tracked_processes', []):
                if (item['pid'], str(item['token'])) not in known:
                    state = group_state(item['pid'], item['token'])
                    if state != 'stopped':
                        items.append(dict(item, state=state))
            busy = bool(current and current['state'] in ACTIVE)
            active = any(p['state'] != 'stopped' for p in items)
            unverified = any(p['state'] == 'unverified' for p in items)
            missing_record = active and not (self.runtime / 'mapping.pids').exists()
            core = {p['name'] for p in items if p['state'] == 'running'}
            blocker = self._blocker() if not busy else ''
            if missing_record:
                blocker = '已知建图进程尚未退出，但 PID 记录缺失，请核对原进程'
                unverified = True
            healthy = {'livox', 'fast_lio'}.issubset(core) and not unverified
            phase = 'RUNNING' if healthy else 'DEGRADED' if active else 'STOPPED'
            output = None
            if session:
                output = {'name': Path(session['output']).name, 'session_id': session['id'], 'saved': False}
                try:
                    info = pcd_info(session['output'])
                    output.update(info, saved=True, saved_at=info['mtime_ns'] / 1e9,
                                  download_url='/api/v1/mapping/artifact?session_id=' + session['id'])
                except (OSError, ValueError):
                    pass
            return {'available': self.enabled, 'phase': phase, 'active': active, 'busy': busy,
                    'blocker': blocker, 'processes': items, 'session': output, 'operation': current,
                    'can_start': self.enabled and not active and not busy and not blocker,
                    'can_save': self.enabled and healthy and not busy,
                    'can_finish': self.enabled and healthy and not busy,
                    'can_stop': self.enabled and active and not unverified and not busy,
                    'message': '建图点云在独立坐标系中；保存生成 PCD，不自动替换导航地图', 'error': ''}
        except (OSError, ValueError, KeyError, TypeError, GatewayError) as exc:
            return {'available': self.enabled, 'phase': 'UNKNOWN', 'active': False, 'busy': False,
                    'can_start': False, 'can_save': False, 'can_finish': False, 'can_stop': False,
                    'session': None, 'operation': None, 'processes': [], 'blocker': '无法确认建图状态', 'error': str(exc)}

    async def snapshot(self):
        return await to_thread(self.inspect)

    def logs(self):
        if self.settings.mode == 'mock':
            return []
        current = self.latest()
        result = []
        for name in ['livox', 'fast_lio', 'rviz']:
            content = tail(self.runtime / 'logs' / (name + '.log'))
            if content:
                result.append({'name': name, 'text': content})
        if current:
            result.insert(0, {'name': '当前操作', 'text': tail(self.records / (current['operation_id'] + '.log'))})
        return result

    def artifact(self, identifier):
        if not self.enabled:
            raise GatewayError(501, 'MAPPING_UNAVAILABLE', '当前环境没有可用的建图成果下载')
        session = self.session()
        if not session or identifier != session['id']:
            raise GatewayError(404, 'MAP_ARTIFACT_NOT_FOUND', '该建图会话不存在或已切换，请刷新后下载')
        path = Path(session['output']).resolve()
        if (self.root / 'maps').resolve() not in path.parents:
            raise GatewayError(403, 'ARTIFACT_PATH_DENIED', '只允许下载工作空间 maps 内的建图成果')
        try:
            pcd_info(path)
        except (OSError, ValueError) as exc:
            raise GatewayError(409, 'MAP_NOT_SAVED', '尚未生成有效 PCD，请先保存') from exc
        if self.inspect()['busy']:
            raise GatewayError(409, 'MAPPING_BUSY', '操作进行中，请完成保存后下载')
        return path

    async def submit(self, identifier, action, name=''):
        if not self.enabled:
            raise GatewayError(501, 'MAPPING_UNAVAILABLE', '当前环境没有可用的建图脚本')
        if action not in ACTIONS:
            raise GatewayError(501, 'NOT_IMPLEMENTED', '该进程操作尚未接入')
        identifier = str(identifier)
        path = self.records / (identifier + '.json')
        existing = read_json(path)
        if existing:
            if existing['action'] != action or existing['name'] != name:
                raise GatewayError(409, 'REQUEST_ID_CONFLICT', '同一 request_id 不能用于不同建图操作')
            return self.accepted(existing)
        if name and action != 'mapping.start':
            raise GatewayError(422, 'MAPPING_NAME_UNUSED', '地图名称只用于开始建图')
        state = await self.snapshot()
        if state['busy']:
            raise GatewayError(409, 'MAPPING_BUSY', '已有建图操作进行中，请等待完成')
        permission = 'can_' + ACTIONS[action]
        if not state[permission]:
            raise GatewayError(409, 'MAPPING_STATE_CONFLICT', state.get('blocker') or '当前建图状态不能执行该操作')
        self.records.mkdir(parents=True, exist_ok=True)
        handle = ExitStack()
        try:
            mission_lock = self.root / 'runtime/missions/mission.lock'
            mission_lock.parent.mkdir(parents=True, exist_ok=True)
            for lock_path in (self.records / 'operation.lock', mission_lock):
                lock = handle.enter_context(lock_path.open('a'))
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.close()
            raise GatewayError(409, 'MAPPING_BUSY', '已有任务或另一个网关正在执行操作') from exc
        record = {'operation_id': identifier, 'request_id': identifier, 'action': action, 'name': name,
                  'state': 'QUEUED', 'started_at': time.time(), 'finished_at': None,
                  'message': '建图操作已接受，正在准备', 'exit_code': None}
        if action == 'mapping.start':
            session_id = str(uuid.uuid4())
            label = name or time.strftime('map_%Y%m%d_%H%M%S')
            record.update(session_id=session_id, output=str(self.root / 'maps/web_maps' / (label + '_' + session_id[:8] + '.pcd')))
        else:
            record.update(session_id=(self.session() or {}).get('id'), output=(self.session() or {}).get('output'))
        try:
            write_json(path, record)
            write_json(self.records / 'latest.json', {'operation_id': identifier})
            self.active_operation_id = identifier
            self.job = asyncio.create_task(self._work(record, handle))
        except BaseException:
            handle.close()
            raise
        return self.accepted(record)

    @staticmethod
    def accepted(record):
        return {'accepted': True, 'simulated': False, 'operation_id': record['operation_id'],
                'request_id': record['request_id'], 'message': '建图操作已接受，请查看执行结果'}

    async def _work(self, record, handle):
        path = self.records / (record['operation_id'] + '.json')
        try:
            before = None
            prior_processes = processes(self.runtime / 'mapping.pids')
            record['tracked_processes'] = prior_processes
            if record.get('output') and Path(record['output']).exists():
                stat = Path(record['output']).stat()
                before = (stat.st_mtime_ns, stat.st_size, stat.st_ino)
            record.update(state='RUNNING', message='正在' + {'start': '启动建图', 'save': '保存地图', 'finish': '保存并结束建图', 'stop': '停止建图'}[ACTIONS[record['action']]])
            write_json(path, record)
            code = await self.runner(record)
            record['exit_code'] = code
            if code != 0:
                raise RuntimeError('建图命令未成功，请查看操作日志；已有进程与地图保留')
            items = await to_thread(processes, self.runtime / 'mapping.pids')
            active = any(p['state'] != 'stopped' for p in items)
            if record['action'] == 'mapping.start' and not {'livox', 'fast_lio'}.issubset({p['name'] for p in items if p['state'] == 'running'}):
                raise RuntimeError('命令已退出，但雷达/FAST-LIO 进程未确认运行')
            if record['action'] == 'mapping.start':
                session = self.session()
                if not session or Path(session['output']).resolve() != Path(record['output']).resolve():
                    raise RuntimeError('建图会话输出与请求不一致')
                record['session_id'] = session['id']
            if record['action'] in {'mapping.stop', 'mapping.finish'} and (active or any(
                    group_state(p['pid'], p['token']) != 'stopped' for p in prior_processes)):
                raise RuntimeError('建图进程仍未确认退出')
            if record['action'] in {'mapping.save', 'mapping.finish'}:
                info = pcd_info(record['output'])
                stat = Path(record['output']).stat()
                if before == (stat.st_mtime_ns, stat.st_size, stat.st_ino):
                    raise RuntimeError('PCD 未更新，不能把旧文件作为保存成功依据')
                record['artifact'] = info
            record.update(state='SUCCEEDED', message={'mapping.start': '建图已启动，请用遥控器采集环境',
                          'mapping.save': 'PCD 地图已保存，建图继续运行', 'mapping.finish': 'PCD 已保存且建图进程已退出',
                          'mapping.stop': '建图进程已退出，未主动请求保存'}[record['action']])
        except asyncio.CancelledError:
            record.update(state='INTERRUPTED', message='网关关闭，操作结果未确认；建图子进程与已保存地图保留')
            if self.child and self.child.returncode is None:
                os.killpg(self.child.pid, signal.SIGTERM)
                try:
                    await asyncio.wait_for(self.child.wait(), 3)
                except asyncio.TimeoutError:
                    os.killpg(self.child.pid, signal.SIGKILL)
                    await self.child.wait()
        except Exception as exc:
            record.update(state='FAILED', message=str(exc))
        finally:
            self.child = None
            record['finished_at'] = time.time()
            write_json(path, record)
            handle.close()

    async def _run_script(self, record):
        env = dict(os.environ)
        env.pop('G1_WEB_TOKEN', None)
        env.update(G1_WS=str(self.root), G1_ENABLE_RVIZ='0')
        if record['action'] == 'mapping.start':
            env.update(G1_MAPPING_OUTPUT=record['output'], G1_MAPPING_SESSION_ID=record['session_id'])
        with (self.records / (record['operation_id'] + '.log')).open('ab') as stream:
            self.child = await asyncio.create_subprocess_exec('bash', str(self.script), ACTIONS[record['action']],
                env=env, stdout=stream, stderr=asyncio.subprocess.STDOUT, start_new_session=True)
            info = process(self.child.pid)
            record.update(command_pid=self.child.pid, command_token=info['token'] if info else None)
            write_json(self.records / (record['operation_id'] + '.json'), record)
            try:
                return await asyncio.wait_for(self.child.wait(), {'start': 600, 'save': 160, 'finish': 200, 'stop': 45}[ACTIONS[record['action']]])
            except asyncio.TimeoutError:
                os.killpg(self.child.pid, signal.SIGTERM)
                try:
                    await asyncio.wait_for(self.child.wait(), 3)
                except asyncio.TimeoutError:
                    os.killpg(self.child.pid, signal.SIGKILL)
                    await self.child.wait()
                raise RuntimeError('操作超时，结果未确认；请查看日志和当前进程，勿反复点击启动')

    async def close(self):
        if self.job and not self.job.done():
            self.job.cancel()
            await self.job
