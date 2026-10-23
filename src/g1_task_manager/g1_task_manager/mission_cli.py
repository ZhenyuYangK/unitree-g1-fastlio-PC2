"""任务CLI、互斥锁和结果记录。check-only不创建ROS节点。"""
import fcntl
import json
import os
from datetime import datetime
from pathlib import Path

from .task_events import emit_event
from .health import validate_limits
from .mission_store import load_missions
from .waypoint_store import WaypointConfigError


from .paths import default_missions_file, workspace


def add_parsers(subparsers):
    listing = subparsers.add_parser('missions', help='列出已校验的多点任务')
    listing.add_argument('--missions-file', default=default_missions_file())
    mission = subparsers.add_parser('mission', help='检查或执行多点任务')
    mission.add_argument('mission_id')
    mission.add_argument('--missions-file', default=default_missions_file())
    mission.add_argument('--gate-service', default='/g1_loco_bridge/enable_real_nav')
    mission.add_argument('--max-tf-age', type=float, default=0.5)
    mission.add_argument('--max-fitness', type=float, default=2.0)
    mission.add_argument('--max-fitness-age', type=float, default=5.0)
    mode = mission.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check-only', action='store_true', help='只检查全部步骤，不连接ROS')
    mode.add_argument('--execute', action='store_true', help='实际执行并自动管理真实控制安全门')


def _write_json(path, data):
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def run_command(parsed, store):
    missions = load_missions(parsed.missions_file, store)
    if parsed.command == 'missions':
        for mission_id, mission in missions.items():
            print('{}: {} [{}]'.format(mission_id, mission['name'],
                  ' -> '.join(step['waypoint'] for step in mission['steps'])))
        if not missions:
            print('[INFO] 尚未配置任务')
        return 0
    if parsed.mission_id not in missions:
        raise WaypointConfigError('任务不存在：{}'.format(parsed.mission_id))
    validate_limits(parsed.max_tf_age, parsed.max_fitness, parsed.max_fitness_age)
    mission = missions[parsed.mission_id]
    print('[INFO] 任务 {}：{}'.format(mission['id'], mission['name']), flush=True)
    for index, step in enumerate(mission['steps'], 1):
        pose = step['goal']['pose']
        print('  {}. {} x={:.3f} y={:.3f} yaw={:.3f} timeout={}s dwell={}s'.format(
            index, step['waypoint'], pose['x'], pose['y'], pose['yaw'],
            step['timeout_sec'], step['dwell_sec']), flush=True)
    if parsed.check_only:
        emit_event('result', state='CHECKED', gate_closed=False, goals_terminal=False,
                   report_path='', errors=[])
        print('[OK] 全部任务配置和导航点引用有效；未连接ROS，未开门，未发送目标。')
        return 0

    # 两个本程序实例不能同时操作同一工作空间。RViz/其他客户端不受此锁管理。
    runtime = workspace() / 'runtime/missions'
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime / 'mission.lock').open('a+') as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise WaypointConfigError('已有多点任务运行，请在原终端Ctrl+C停止') from exc
        blocker = runtime / 'recovery_required.json'
        if blocker.exists():
            raise WaypointConfigError('上次任务未完成收尾，请按README恢复流程处理：{}'.format(blocker))
        stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        report_path = runtime / ('{}_{}.json'.format(stamp, mission['id']))
        metadata = {'started_at': datetime.now().astimezone().isoformat(),
                    'map_id': store.map_id, 'waypoints_file': str(store.file_path.resolve()),
                    'missions_file': str(Path(parsed.missions_file).resolve()),
                    'mission_id': mission['id'], 'plan': mission['steps']}
        # 运行前写入标记；异常退出/断电也会使下次执行保守地停止。
        _write_json(blocker, {'state': 'RUNNING', 'report': str(report_path), 'pid': os.getpid()})
        _write_json(report_path, dict(metadata, state='STARTING'))
        return _run_ros(store, mission, metadata, report_path, blocker, parsed)


def _run_ros(store, mission, metadata, report_path, blocker, parsed):
    from .mission_engine import execute_mission
    import rclpy
    from .mission_ros import create_transport

    transport = None
    try:
        rclpy.init(args=[])
        transport = create_transport(store.frame_id, store.base_frame,
                                     gate_service=parsed.gate_service,
                                     max_tf_age_sec=parsed.max_tf_age,
                                     max_fitness=parsed.max_fitness,
                                     max_fitness_age_sec=parsed.max_fitness_age)
        transport.install_signals()
        report = execute_mission(mission, transport, lambda event: print('[MISSION] ' + event, flush=True))
        report.update(metadata)
        _write_json(report_path, report)
        if report['gate_closed'] and report['goals_terminal']:
            blocker.unlink()
        else:
            _write_json(blocker, {'state': report['state'], 'report': str(report_path)})
        emit_event('result', report_path=str(report_path), **report)
        print('[MISSION] {} elapsed={}s gate_closed={} goals_terminal={}'.format(
            report['state'], report['elapsed_sec'], report['gate_closed'], report['goals_terminal']), flush=True)
        for error in report['errors']:
            print('[ERROR] ' + error, flush=True)
        print('[INFO] 任务记录：{}'.format(report_path), flush=True)
        if report['state'] == 'STOP_UNCONFIRMED':
            print('[ERROR] 停止未确认，请用遥控器确认停止并按README恢复流程处理。', flush=True)
        return {'SUCCEEDED': 0, 'CANCELED': 130}.get(report['state'], 2)
    finally:
        if transport is not None:
            transport.begin_cleanup()
            try:
                transport.node.destroy_node()
            finally:
                transport.restore_signals()
        if rclpy.ok():
            rclpy.shutdown()
