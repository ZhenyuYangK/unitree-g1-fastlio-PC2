"""PC2 continuous cruise over the P5 CLI and its private local control socket."""
import asyncio
import json
import os
from pathlib import Path
import socket

from .compat import to_thread
from .control import GatewayError
from .mapping import read_json
from .runtime import process
from .workspace import WorkspaceData

TERMINAL = {'SUCCEEDED', 'CANCELED', 'FAILED', 'STOP_UNCONFIRMED', 'NOT_RUNNING'}


class CruiseManager:
    def __init__(self, settings):
        self.settings = settings
        self.root = Path(settings.workspace).resolve()
        self.runtime = self.root / 'runtime/cruise'
        self.enabled = settings.mode == 'ros_control'

    def rpc(self, command, run_id=''):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(2)
            conn.connect(str(self.runtime / 'control.sock'))
            request = json.dumps(dict(command=command, run_id=run_id)) if run_id else command
            conn.sendall((request + '\n').encode('ascii'))
            with conn.makefile('r', encoding='utf-8') as stream:
                return json.loads(stream.readline(65536))

    def inspect(self):
        if self.settings.mode in ('local', 'mock'):
            return dict(available=False, live=False, active=False, state='NOT_RUNNING', run_id='')
        try:
            data = self.rpc('status')
            info = process(data['pid'])
            if not info or info['state'] == 'Z':
                raise OSError('巡航进程不可达')
            return dict(data, available=self.enabled, active=data['state'] not in TERMINAL,
                        controllable=bool(data.get('run_id')), run_id=data.get('run_id', ''))
        except (OSError, ValueError, KeyError, TypeError):
            try:
                data = read_json(self.runtime / 'status.json', {})
                if not isinstance(data, dict):
                    raise ValueError('无效巡航记录')
            except (OSError, ValueError):
                data = dict(state='STOP_UNCONFIRMED', message='巡航记录不可读，请按 CLI 恢复流程核实停止')
            data.setdefault('state', 'NOT_RUNNING')
            state = data.get('state', 'NOT_RUNNING')
            if state not in TERMINAL:
                data.update(last_known_state=state, state='STOP_UNCONFIRMED', message='巡航不可达，请按 CLI 恢复流程核实停止')
            return dict(data, available=self.enabled, live=False, active=False, run_id='')

    async def snapshot(self):
        return await to_thread(self.inspect)

    async def start(self, payload):
        if not self.enabled:
            raise GatewayError(501, 'CRUISE_UNAVAILABLE', '连续巡航需要 ros_control 模式')
        current = await self.snapshot()
        if current.get('active') or current.get('state') == 'STOP_UNCONFIRMED':
            raise GatewayError(409, 'CRUISE_BUSY', '巡航仍在运行或停止未确认')
        data = WorkspaceData(self.settings)
        catalog = await to_thread(data.catalog)
        points = {p['id'] for p in catalog['waypoints']}
        if not payload.points or any(p not in points for p in payload.points):
            raise GatewayError(422, 'CRUISE_ROUTE_INVALID', '路线包含不存在的导航点')
        env = dict(os.environ, G1_WS=str(self.root))
        env.pop('G1_WEB_TOKEN', None)
        self.runtime.mkdir(parents=True, exist_ok=True)
        command = ['/usr/bin/python3', str(self.root / 'script/g1_process.py'), 'start',
                   '--file', str(self.runtime / 'web.pid'), '--name', 'cruise',
                   '--log', str(self.runtime / 'web.log'), '--',
                   'bash', str(self.root / 'script/g1_cruise.sh'), 'start', '--execute',
                   '--file', str(data.waypoints), '--points', *payload.points,
                   '--timeout', str(payload.timeout_sec), '--dwell', str(payload.dwell_sec)]
        command += ['--loop'] if payload.loop else ['--laps', str(payload.laps)]
        child = await asyncio.create_subprocess_exec(*command, env=env, stdout=asyncio.subprocess.PIPE,
                                                     stderr=asyncio.subprocess.STDOUT)
        output, _ = await child.communicate()
        if child.returncode:
            raise GatewayError(409, 'CRUISE_START_FAILED', '巡航启动失败，请查看巡航日志：' + output.decode(errors='replace')[-1000:])
        # Start acceptance is not proof of motion or completion. The P5 coordinator
        # holds mission.lock and records recovery before admitting any navigation.
        return dict(message='巡航启动请求已提交，请查看连续巡航状态')

    async def control(self, action, run_id):
        if not self.enabled:
            raise GatewayError(501, 'CRUISE_UNAVAILABLE', '连续巡航需要 ros_control 模式')
        current = await self.snapshot()
        if not current.get('live') or not current.get('controllable') or not run_id or current['run_id'] != run_id:
            raise GatewayError(409, 'STALE_CRUISE', '巡航运行标识已过期，请刷新状态')
        try:
            result = await to_thread(self.rpc, action, run_id)
        except (OSError, ValueError) as exc:
            raise GatewayError(503, 'CRUISE_UNCONFIRMED', '巡航请求结果未确认，请查询当前状态') from exc
        if not result.get('ok'):
            raise GatewayError(409, 'CRUISE_STATE_CONFLICT', result.get('message', '当前状态不能执行该操作'))
        return dict(message=result['message'], run_id=run_id)
