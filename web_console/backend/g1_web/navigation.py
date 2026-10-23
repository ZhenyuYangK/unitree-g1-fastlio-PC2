"""Navigation lifecycle over fixed workspace scripts, with durable operations."""
import asyncio
from contextlib import ExitStack
import fcntl
import os
from pathlib import Path
import signal
import time

from .control import GatewayError
from .mapping import MappingManager, read_json, write_json, tail, processes, process, group_state
from .workspace import WorkspaceData


ACTIONS = {'navigation.start': 'start-auto', 'navigation.start-manual': 'start',
           'navigation.continue': 'continue', 'navigation.prepare': 'prepare', 'navigation.stop': 'stop'}


class NavigationManager(MappingManager):
    def __init__(self, settings, robot, runner=None):
        super().__init__(settings, runner)
        self.robot = robot
        self.runtime = self.root / 'runtime/navigation'
        self.records = self.runtime / 'web/operations'
        self.script = self.root / 'script/g1_navigation.sh'
        self.enabled = settings.mode == 'ros_control' and self.script.is_file()
        self.runner = runner or self._run_navigation

    def tracked(self):
        items = processes(self.runtime / 'navigation.pids')
        for filename, name in [('real_bridge.pid', 'motion_bridge'), ('audio_bridge.pid', 'audio_bridge'), ('web/task_manager.pid', 'task_manager')]:
            path = self.runtime / filename
            if path.exists():
                items.extend(processes(path))
        return items

    def blocker(self):
        if (self.root / 'runtime/missions/recovery_required.json').exists():
            return '存在停止恢复标记，请先按恢复流程处理'
        for relative in ['runtime/missions/mission.lock', 'runtime/stack.lock']:
            path = self.root / relative
            if path.exists():
                with path.open() as handle:
                    try:
                        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        return '任务运行中或建图/导航正在切换'
        if any(p['state'] != 'stopped' for p in processes(self.root / 'runtime/mapping/mapping.pids')):
            return '建图仍在运行，请先结束建图'
        return ''

    def inspect(self):
        if self.settings.mode == 'mock':
            return dict(available=False, phase='STOPPED', active=False, busy=False, processes=[], operation=None,
                        blocker='', can_start=False, can_continue=False, can_prepare=False, can_stop=False, error='')
        try:
            items = self.tracked()
            op = self.latest() if self.records.exists() else None
            known = {(item['pid'], str(item['token'])) for item in items}
            for item in (op or {}).get('tracked_processes', []):
                if (item['pid'], str(item['token'])) not in known:
                    state = group_state(item['pid'], item['token'])
                    if state != 'stopped':
                        items.append(dict(item, state=state))
            busy = bool(op and op['state'] in {'QUEUED', 'RUNNING'})
            active = any(p['state'] != 'stopped' for p in items)
            unverified = any(p['state'] == 'unverified' for p in items)
            blocker = self.blocker() if not busy else ''
            base_active = any(p['name'] in ('livox', 'fast_lio', 'relocalization', 'map_server', 'nav2') and p['state'] != 'stopped' for p in items)
            return dict(available=self.enabled, phase='UNKNOWN' if unverified else 'RUNNING' if active else 'STOPPED',
                        active=active, busy=busy, processes=items, operation=op, blocker=blocker,
                        can_start=self.enabled and not active and not busy and not blocker,
                        can_continue=self.enabled and base_active and not busy and not unverified and not blocker,
                        can_prepare=self.enabled and not busy and not unverified and not blocker,
                        can_stop=self.enabled and active and not busy and not unverified,
                        error='')
        except (OSError, ValueError, TypeError, KeyError) as exc:
            return dict(available=self.enabled, phase='UNKNOWN', active=False, busy=False, processes=[], operation=None,
                        blocker='导航进程状态无法确认', can_start=False, can_continue=False, can_prepare=False, can_stop=False, error=str(exc))

    async def submit(self, identifier, action, name=''):
        if not self.enabled:
            raise GatewayError(501, 'NAVIGATION_UNAVAILABLE', '导航控制需要显式启动 ros_control 模式')
        if action not in ACTIONS or name:
            raise GatewayError(422, 'NAVIGATION_OPERATION_INVALID', '导航操作不接受名称或自定义参数')
        identifier = str(identifier)
        path = self.records / (identifier + '.json')
        existing = read_json(path)
        if existing:
            if existing['action'] != action:
                raise GatewayError(409, 'REQUEST_ID_CONFLICT', '同一 request_id 不能用于不同导航操作')
            return self.accepted(existing)
        state = await self.snapshot()
        permission = 'can_start' if action in ('navigation.start', 'navigation.start-manual') else 'can_' + ACTIONS[action]
        if not state[permission]:
            raise GatewayError(409, 'NAVIGATION_STATE_CONFLICT', state['blocker'] or '当前导航状态不能执行该操作')
        telemetry = await self.robot.snapshot()
        if telemetry['mission']['active'] or telemetry.get('navigation', {}).get('active_nav_goal'):
            raise GatewayError(409, 'MISSION_BUSY', '任务运行或收尾未确认，不能启停导航')
        if action in ('navigation.start', 'navigation.start-manual') and telemetry.get('stack_present'):
            raise GatewayError(409, 'EXTERNAL_STACK', '检测到现有定位/导航节点，请使用原管理方式处理，避免重复启动')
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
            raise GatewayError(409, 'NAVIGATION_BUSY', '另一个网关正在执行导航操作') from exc
        record = dict(operation_id=identifier, request_id=identifier, action=action, name='', state='QUEUED',
                      started_at=time.time(), finished_at=None, exit_code=None, message='导航操作已接受')
        write_json(path, record)
        write_json(self.records / 'latest.json', dict(operation_id=identifier))
        self.active_operation_id = identifier
        self.job = asyncio.create_task(self._work_navigation(record, handle))
        return self.accepted(record)

    @staticmethod
    def accepted(record):
        return dict(accepted=True, simulated=False, operation_id=record['operation_id'], request_id=record['request_id'], message='导航操作已接受，请查看阶段进度与结果')

    async def _work_navigation(self, record, handle):
        try:
            record.update(state='RUNNING', message='正在处理导航操作，可查看实时就绪状态和日志')
            write_json(self.records / (record['operation_id'] + '.json'), record)
            record['tracked_processes'] = self.tracked()
            code = await self.runner(record)
            record['exit_code'] = code
            if code:
                if record['action'] in ('navigation.start', 'navigation.start-manual', 'navigation.continue'):
                    localization = (await self.robot.snapshot()).get('localization', {})
                    reason = localization.get('reason') if localization.get('status_stale') is False else ''
                    if reason:
                        raise RuntimeError('导航命令未成功：' + str(reason) + '；基础进程可能保留，可设置初始位姿后继续')
                raise RuntimeError('导航命令未成功；定位基础进程可能保留，请查看日志并人工接续')
            action = record['action']
            data = await self.robot.snapshot()
            if action in ('navigation.start', 'navigation.start-manual', 'navigation.continue'):
                checks = data.get('navigation', {}).get('checks', {})
                if not data['localization'].get('valid') or not all(checks.get(k, {}).get('ready') for k in ('controller_server', 'planner_server', 'recoveries_server', 'bt_navigator')):
                    raise RuntimeError('命令已退出，但实时定位或 Nav2 激活尚未确认')
            if action == 'navigation.stop' and (any(p['state'] != 'stopped' for p in self.tracked()) or any(group_state(p['pid'], p['token']) != 'stopped' for p in record['tracked_processes'])):
                raise RuntimeError('仍有受管理进程未确认退出，请查看进程状态')
            record.update(state='SUCCEEDED', message='导航操作已完成' if action != 'navigation.prepare' else '运动/语音桥及任务执行器已准备，控制门保持关闭')
        except asyncio.CancelledError:
            record.update(state='INTERRUPTED', message='网关已关闭，操作结果未确认；请重新连接核对状态')
            await self._terminate_command()
        except Exception as exc:
            record.update(state='FAILED', message=str(exc))
        finally:
            self.child = None
            record['finished_at'] = time.time()
            write_json(self.records / (record['operation_id'] + '.json'), record)
            handle.close()

    async def _terminate_command(self):
        if self.child and self.child.returncode is None:
            os.killpg(self.child.pid, signal.SIGTERM)
            try:
                await asyncio.wait_for(self.child.wait(), 3)
            except asyncio.TimeoutError:
                os.killpg(self.child.pid, signal.SIGKILL)
                await self.child.wait()

    async def _script_command(self, record, script, argument, extra=(), interpreter='bash'):
        env = dict(os.environ, G1_WS=str(self.root), G1_ENABLE_RVIZ='0')
        env.pop('G1_WEB_TOKEN', None)
        data = WorkspaceData(self.settings)
        env.update(G1_MAP_YAML=str(data.map_yaml), G1_WAYPOINTS_FILE=str(data.waypoints), G1_MISSIONS_FILE=str(data.missions))
        if self.settings.map_pgm:
            env['G1_MAP_PGM'] = self.settings.map_pgm
        with (self.records / (record['operation_id'] + '.log')).open('ab') as stream:
            self.child = await asyncio.create_subprocess_exec(interpreter, str(script), argument, *extra, env=env,
                stdout=stream, stderr=asyncio.subprocess.STDOUT, start_new_session=True)
            info = process(self.child.pid)
            record.update(command_pid=self.child.pid, command_token=info['token'] if info else None)
            write_json(self.records / (record['operation_id'] + '.json'), record)
            try:
                return await asyncio.wait_for(self.child.wait(), 600 if argument.startswith('start') else 90)
            except asyncio.TimeoutError:
                await self._terminate_command()
                raise RuntimeError('导航操作超时，请核对进程及实时状态')

    async def _run_navigation(self, record):
        action = record['action']
        telemetry = await self.robot.snapshot()
        if telemetry['mission']['active'] or telemetry.get('navigation', {}).get('active_nav_goal'):
            raise RuntimeError('检测到活动任务或导航目标，拒绝启停底层进程')
        if action == 'navigation.prepare':
            code = await self._script_command(record, self.root / 'script/g1_navigation_real.sh', 'start')
            if code:
                return code
            telemetry = await self.robot.snapshot()
            if not telemetry.get('navigation', {}).get('checks', {}).get('task_manager', {}).get('ready'):
                names = self.robot.node.get_node_names_and_namespaces()
                if ('g1_task_manager', '/') in names:
                    raise RuntimeError('已有任务执行器配置不匹配，请由原启动者处理后再准备')
                data = WorkspaceData(self.settings)
                log_path = self.runtime / 'logs/task_manager.log'
                log_path.parent.mkdir(parents=True, exist_ok=True)
                env = dict(os.environ, G1_WS=str(self.root))
                env.pop('G1_WEB_TOKEN', None)
                code = await self._script_command(record, self.root / 'script/g1_process.py', 'start',
                    ('--file', str(self.runtime / 'web/task_manager.pid'), '--name', 'task_manager',
                     '--log', str(log_path), '--', 'ros2', 'run', 'g1_task_manager', 'g1_task_manager_node',
                     '--ros-args', '-p', 'execute_real:=true', '-p', 'waypoints_file:='+str(data.waypoints),
                     '-p', 'missions_file:='+str(data.missions)), interpreter='/usr/bin/python3')
                if code:
                    raise RuntimeError('任务执行器未成功创建')
            for _ in range(80):
                telemetry = await self.robot.snapshot()
                checks = telemetry.get('navigation', {}).get('checks', {})
                if all(checks.get(key, {}).get('ready') for key in ('motion_bridge', 'audio_bridge', 'task_manager')):
                    await self.robot.close_gate()
                    return 0
                await asyncio.sleep(0.25)
            raise RuntimeError('桥或任务执行器就绪超时，请查看日志')
        if action == 'navigation.stop':
            names = self.robot.node.get_node_names_and_namespaces()
            if ('g1_cmd_vel_dry_run', '/') in names:
                await self.robot.close_gate()
            # Only stop processes whose identities are still proven.
            items = self.tracked()
            for item in items:
                if item['state'] == 'unverified':
                    raise RuntimeError('进程身份未确认，拒绝停止')
            owned_task = next((p for p in items if p['name'] == 'task_manager' and p['state'] == 'running'), None)
            if owned_task:
                code = await self._script_command(record, self.root / 'script/g1_process.py', 'stop',
                    ('--file', str(self.runtime / 'web/task_manager.pid')), interpreter='/usr/bin/python3')
                if code:
                    raise RuntimeError('任务执行器尚未退出')
            if (self.runtime / 'real_bridge.pid').exists():
                code = await self._script_command(record, self.root / 'script/g1_navigation_real.sh', 'stop')
                if code:
                    return code
            return await self._script_command(record, self.script, 'stop')
        return await self._script_command(record, self.script, ACTIONS[action])

    def logs(self):
        if self.settings.mode == 'mock':
            return []
        result = []
        current = self.latest() if self.records.exists() else None
        if current:
            result.append(dict(name='当前操作', text=tail(self.records / (current['operation_id']+'.log'))))
        for name in ['relocalization', 'nav2', 'real_bridge', 'audio_bridge', 'task_manager']:
            content = tail(self.runtime / 'logs' / (name+'.log'))
            if content:
                result.append(dict(name=name, text=content))
        return result
