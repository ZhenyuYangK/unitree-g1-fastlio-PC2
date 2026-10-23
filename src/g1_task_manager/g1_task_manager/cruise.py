"""PC2 cruise coordinator. Python 3.8; shared mission lock and stop recovery."""
import argparse
import copy
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
import uuid
from datetime import datetime

from .paths import workspace

WS = workspace()
ROOT = WS / 'runtime/cruise'
SOCKET = ROOT / 'control.sock'
STATUS = ROOT / 'status.json'
TERMINAL = ('SUCCEEDED', 'CANCELED', 'FAILED', 'STOP_UNCONFIRMED')


def write_json(path, data):
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temp), str(path))


def parser():
    p = argparse.ArgumentParser(description='巡航已有导航点：plan/start/pause/resume/stop/status')
    sub = p.add_subparsers(dest='command', required=True)
    for name in ('plan', 'start'):
        q = sub.add_parser(name, help='预览路线' if name == 'plan' else '前台执行巡航，保持终端打开')
        q.add_argument('--file', default=os.environ.get('G1_WAYPOINTS_FILE', str(WS / 'task_data/maps/laboratory_map_v1/waypoints.yaml')))
        q.add_argument('--points', nargs='+', help='按给定顺序巡航；省略则按ID字母序访问全部点')
        mode = q.add_mutually_exclusive_group()
        mode.add_argument('--laps', type=int, default=1, help='圈数，默认1')
        mode.add_argument('--loop', action='store_true', help='持续循环，直到暂停/停止/失败')
        q.add_argument('--timeout', type=float, default=60., help='每次前往一个点的超时秒数')
        q.add_argument('--dwell', type=float, default=2., help='每个点到达后停留秒数')
        if name == 'start':
            q.add_argument('--execute', action='store_true', help='明确执行真实导航')
    for name in ('pause', 'resume', 'stop', 'status'):
        sub.add_parser(name)
    return p


def make_plan(args):
    from g1_task_manager.waypoint_store import WaypointStore
    store = WaypointStore(args.file).load()
    ids = args.points if args.points else [key for key, _ in store.list_waypoints()]
    if not ids:
        raise RuntimeError('没有导航点')
    if args.laps < 1:
        raise RuntimeError('--laps 必须至少为1')
    for key, zero in (('timeout', False), ('dwell', True)):
        v = getattr(args, key)
        if not math.isfinite(v) or (v < 0 if zero else v <= 0):
            raise RuntimeError('{} 参数无效'.format(key))
    steps = [dict(waypoint=key, goal=copy.deepcopy(store.get_waypoint(key)),
                  timeout_sec=args.timeout, dwell_sec=0.) for key in ids]
    return store, dict(route=steps, laps=None if args.loop else args.laps,
                       dwell_sec=args.dwell, map_id=store.map_id,
                       waypoints_file=str(store.file_path.resolve()))


class Control:
    def __init__(self, plan):
        self.lock = threading.RLock()
        self.desired = 'run'
        self.state = dict(state='STARTING', run_id=uuid.uuid4().hex, pid=os.getpid(), route=[s['waypoint'] for s in plan['route']],
                          laps=plan['laps'], lap=1, step=1, current_waypoint='',
                          completed_visits=0, gate_closed=False, goals_terminal=False,
                          errors=[], dwell_remaining=0.)
        self.closed = threading.Event()

    def snapshot(self):
        with self.lock:
            return dict(self.state, requested=self.desired, live=True)

    def update(self, **fields):
        with self.lock:
            self.state.update(fields)

    def intent(self):
        with self.lock:
            return self.desired

    def request(self, command, expected_run_id=None):
        with self.lock:
            if expected_run_id is not None and expected_run_id != self.state['run_id']:
                return dict(ok=False, message='巡航运行标识已过期', **self.snapshot())
            state = self.state['state']
            ok = True
            message = '已收到请求；以最终状态为准'
            if command == 'status':
                pass
            elif state in TERMINAL:
                ok, message = False, '巡航已结束'
            elif command == 'stop':
                self.desired = 'stop'
                self.state['state'] = 'STOPPING'
            elif command == 'pause' and self.desired != 'stop':
                self.desired = 'pause'
                if state != 'PAUSED':
                    self.state['state'] = 'PAUSING'
            elif command == 'resume' and state == 'PAUSED' and self.desired == 'pause':
                self.desired = 'run'
                self.state['state'] = 'RESUMING'
            else:
                ok, message = False, '只有确认 PAUSED 后才能 resume；停止请求不能撤销'
            return dict(ok=ok, message=message, **self.snapshot())

    def serve(self, server):
        while not self.closed.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with conn:
                conn.settimeout(1.)
                try:
                    data = conn.recv(512).decode('ascii').strip()
                    expected_run_id = None
                    if data.startswith('{'):
                        payload = json.loads(data)
                        data, expected_run_id = payload['command'], payload['run_id']
                        if not isinstance(expected_run_id, str) or not expected_run_id:
                            raise ValueError('无效巡航运行标识')
                    if data not in ('pause', 'resume', 'stop', 'status'):
                        raise ValueError('未知指令')
                    reply = self.request(data, expected_run_id)
                    conn.sendall((json.dumps(reply, ensure_ascii=False) + '\n').encode())
                except (OSError, ValueError, KeyError, TypeError):
                    continue


def rpc(command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
        conn.settimeout(2.)
        conn.connect(str(SOCKET))
        conn.sendall((command + '\n').encode())
        with conn.makefile('r', encoding='utf-8') as stream:
            return json.loads(stream.readline(65536))


def last_status():
    data = json.loads(STATUS.read_text()) if STATUS.exists() else dict(state='NOT_RUNNING')
    data['live'] = False
    if data['state'] not in TERMINAL + ('NOT_RUNNING',):
        data['last_known_state'] = data['state']
        data['state'] = 'STOP_UNCONFIRMED'
        data['message'] = '控制进程不可达；历史状态不是当前停止确认，检查安全门与恢复标记'
    return data


def control_command(command):
    try:
        reply = rpc(command)
    except OSError:
        reply = last_status()
        print(json.dumps(reply, ensure_ascii=False, indent=2))
        return 0 if command == 'status' and reply['state'] != 'STOP_UNCONFIRMED' else 2
    if command in ('pause', 'stop') and reply['ok']:
        target = 'PAUSED' if command == 'pause' else 'CANCELED'
        deadline = time.monotonic() + 40.
        while reply['state'] not in TERMINAL + ('PAUSED',):
            if time.monotonic() >= deadline:
                print('请求仍在处理，未确认暂停/停止，请执行 status；不要强制杀进程。')
                break
            time.sleep(.1)
            try:
                reply = rpc('status')
            except OSError:
                reply = last_status()
                break
        print(json.dumps(reply, ensure_ascii=False, indent=2))
        return 0 if reply['state'] in (target, 'SUCCEEDED') and reply.get('gate_closed') and reply.get('goals_terminal') else 2
    print(json.dumps(reply, ensure_ascii=False, indent=2))
    return 0 if reply.get('ok', False) else 2


def run_loop(control, plan, transport, execute, persist, spin):
    """A failed leg never resumes; a user pause retries only the unfinished leg."""
    visits = 0
    remaining = 0.
    route = plan['route']
    while True:
        intent = control.intent()
        if intent == 'stop':
            return 'CANCELED'
        if intent == 'pause':
            # Only enter this branch after confirmed cleanup or initial gate close.
            control.update(state='PAUSED', gate_closed=True, goals_terminal=True,
                           dwell_remaining=round(remaining, 3))
            persist()
            while control.intent() == 'pause':
                spin(.05)
            continue
        if remaining > 0:
            control.update(state='DWELLING', dwell_remaining=round(remaining, 3))
            before = time.monotonic()
            spin(min(.05, remaining))
            remaining = max(0., remaining - (time.monotonic() - before))
            continue
        if plan['laps'] is not None and visits >= plan['laps'] * len(route):
            return 'SUCCEEDED'
        index = visits % len(route)
        step = route[index]
        control.update(state='CHECKING', lap=visits // len(route) + 1,
                       step=index + 1, current_waypoint=step['waypoint'], dwell_remaining=0.)
        persist()
        # Previous goal has reached a terminal state before these references reset.
        transport.cleanup = False
        transport.monitoring = False
        transport.goal_id = transport.send_future = transport.handle = transport.result_future = None
        report = execute(dict(id='cruise_leg', name='巡航单段', steps=[step]), transport,
                         lambda text: print('[CRUISE] ' + text, flush=True))
        control.update(gate_closed=report['gate_closed'], goals_terminal=report['goals_terminal'],
                       last_leg=report)
        persist()
        if not report['gate_closed'] or not report['goals_terminal']:
            control.update(errors=report['errors'])
            return 'STOP_UNCONFIRMED'
        arrived_before_cancel = (
            report['state'] == 'CANCELED' and control.intent() in ('pause', 'stop')
            and report.get('steps') and report['steps'][-1].get(
                'navigation_state', report['steps'][-1]['state']) == 'SUCCEEDED')
        if report['state'] == 'SUCCEEDED' or arrived_before_cancel:
            visits += 1
            remaining = plan['dwell_sec']
            control.update(completed_visits=visits)
        elif report['state'] == 'CANCELED' and control.intent() in ('pause', 'stop'):
            # Cancel acknowledged: keep visits unchanged to retry current point.
            continue
        else:
            control.update(errors=report['errors'])
            return 'FAILED'


def start(args, store, plan):
    if not args.execute:
        raise RuntimeError('start 会产生真实运动；请预览 plan，然后添加 --execute')
    ROOT.mkdir(parents=True, exist_ok=True)
    runtime = WS / 'runtime/missions'
    runtime.mkdir(parents=True, exist_ok=True)
    blocker = runtime / 'recovery_required.json'
    lock = (runtime / 'mission.lock').open('a+')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError('已有任务/巡航占用工作空间；暂停期间也不会释放任务锁')
    if blocker.exists():
        lock.close()
        raise RuntimeError('存在 recovery_required.json，请按已有恢复流程确认停止')
    control = Control(plan)
    report_path = ROOT / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_cruise.json')
    def persist():
        data = dict(control.snapshot(), plan=plan, report_path=str(report_path),
                    updated_at=datetime.now().astimezone().isoformat())
        write_json(report_path, data)
        write_json(STATUS, data)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    transport = None
    ros = None
    thread = None
    handlers = {}
    owns_marker = False
    final = 'STOP_UNCONFIRMED'
    try:
        if len(str(SOCKET).encode()) >= 104:
            raise RuntimeError('工作空间路径过长，无法创建本地控制socket')
        # The shared mission lock proves there is no other cooperative cruise.
        if SOCKET.exists():
            SOCKET.unlink()
        server.bind(str(SOCKET))
        os.chmod(str(SOCKET), 0o600)
        server.settimeout(.2)
        server.listen(4)
        thread = threading.Thread(target=control.serve, args=(server,), daemon=True)
        thread.start()
        import rclpy
        from g1_task_manager.mission_ros import MissionTransport
        from g1_task_manager.nav2_client import Nav2WaypointClient, Nav2NavigationError
        from g1_task_manager.mission_engine import execute_mission, MissionCanceled, MissionError
        ros = rclpy
        ros.init(args=[])
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, lambda *_: control.request('stop'))

        class CruiseTransport(MissionTransport):
            def checkpoint(self):
                if not self.cleanup and control.intent() != 'run':
                    raise MissionCanceled('巡航用户请求：' + control.intent())

            def _spin(self, seconds):
                self.checkpoint()
                super()._spin(seconds)
                self.checkpoint()

            def ready(self):
                deadline = time.monotonic() + 30.
                reason = '等待Nav2与定位'
                while time.monotonic() < deadline:
                    self._spin(.05)
                    if not self.node.action_client.server_is_ready():
                        continue
                    try:
                        self.node.check_health(**self.health_limits)
                    except Nav2NavigationError as exc:
                        reason = str(exc)
                        continue
                    self._spin(.5)
                    if any(s in (1, 2, 3) for s in self.statuses.values()):
                        raise MissionError('Nav2已有活动目标，请先结束原任务')
                    self.checkpoint()
                    return
                raise MissionError('导航前检查超时：' + reason)

            def prepare_goal(self, step):
                self.checkpoint()
                control.update(state='NAVIGATING', goals_terminal=False)
                persist()
                super().prepare_goal(step)

            def open_gate(self):
                self.checkpoint()
                control.update(gate_closed=False)
                super().open_gate()
                self.checkpoint()

        # Keep a recovery marker for the entire session, including pauses.
        write_json(blocker, dict(state='CRUISE_RUNNING', report=str(report_path), pid=os.getpid()))
        owns_marker = True
        persist()
        node = Nav2WaypointClient(store.frame_id, store.base_frame)
        try:
            transport = CruiseTransport(node, max_tf_age_sec=.5, max_fitness=2., max_fitness_age_sec=15.)
        except BaseException:
            node.destroy_node()
            raise
        transport.begin_cleanup()
        transport.close_gate()
        control.update(gate_closed=True, goals_terminal=True)
        final = run_loop(control, plan, transport, execute_mission, persist,
                         lambda sec: transport._spin(sec))
    except BaseException as exc:
        control.update(errors=control.snapshot()['errors'] + [str(exc)])
        final = 'FAILED'
    finally:
        # Always try gate close + owned-goal cancellation, even on IO/ROS errors.
        gate, goals = False, False
        if transport is not None:
            transport.begin_cleanup()
            try:
                transport.close_gate()
                gate = True
            except Exception as exc:
                control.update(errors=control.snapshot()['errors'] + ['关门未确认：' + str(exc)])
            try:
                goals = bool(transport.cancel_pending())
            except Exception as exc:
                control.update(errors=control.snapshot()['errors'] + ['取消未确认：' + str(exc)])
        if owns_marker and not (gate and goals):
            final = 'STOP_UNCONFIRMED'
        control.update(state=final, gate_closed=gate, goals_terminal=goals)
        try:
            persist()
            if owns_marker and gate and goals:
                blocker.unlink()
            elif owns_marker:
                write_json(blocker, dict(state='STOP_UNCONFIRMED', report=str(report_path)))
        finally:
            control.closed.set()
            server.close()
            if thread is not None:
                thread.join(timeout=2.)
            if SOCKET.exists():
                SOCKET.unlink()
            if transport is not None:
                transport.destroy()
            if ros is not None and ros.ok():
                ros.shutdown()
            for sig, old in handlers.items():
                signal.signal(sig, old)
            lock.close()
        print(json.dumps(control.snapshot(), ensure_ascii=False, indent=2), flush=True)
        print('report=' + str(report_path), flush=True)
    return 0 if final == 'SUCCEEDED' else (130 if final == 'CANCELED' else 2)


def main():
    argv = sys.argv[1:] or ['--help']
    if argv == ['help']:
        argv = ['--help']
    args = parser().parse_args(argv)
    try:
        if args.command in ('pause', 'resume', 'stop', 'status'):
            return control_command(args.command)
        store, plan = make_plan(args)
        print('路线：' + ' -> '.join(s['waypoint'] for s in plan['route']), flush=True)
        print('圈数：{}；每点超时={}s；到达停留={}s'.format(
            plan['laps'] if plan['laps'] is not None else '持续循环', args.timeout, args.dwell), flush=True)
        for s in plan['route']:
            print('{}: {}'.format(s['waypoint'], s['goal']['pose']), flush=True)
        return 0 if args.command == 'plan' else start(args, store, plan)
    except Exception as exc:
        print('[ERROR] ' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
