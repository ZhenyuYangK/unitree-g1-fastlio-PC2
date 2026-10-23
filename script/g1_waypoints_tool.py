#!/usr/bin/env python3
"""导航点脚本辅助程序，兼容 Ubuntu 20.04 / Python 3.8。"""
import argparse
import copy
import fcntl
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from datetime import datetime

WS = Path(os.environ.get('G1_WS', str(Path(__file__).resolve().parent.parent))).resolve()
# 优先复用本工作空间源码，避免读取另一个 overlay 的旧版模块。
sys.path.insert(0, str(WS / 'src/g1_task_manager'))


def parser():
    p = argparse.ArgumentParser(description='G1 导航点管理；坐标单位 m，yaw 单位 rad。')
    p.add_argument('--file', default=os.environ.get('G1_WAYPOINTS_FILE', str(
        WS / 'task_data/maps/laboratory_map_v1/waypoints.yaml')))
    p.add_argument('--missions-file', default=os.environ.get('G1_MISSIONS_FILE', str(
        WS / 'task_data/missions/missions.yaml')))
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('list', help='查看所有点（离线）')
    show = sub.add_parser('show', help='查看一个点，包括记录时间')
    show.add_argument('id')
    sub.add_parser('validate', help='校验点文件和指定任务文件')
    sub.add_parser('path', help='显示当前工作空间和数据路径')
    sub.add_parser('backup', help='备份导航点文件')
    for command in ('record', 'add', 'update'):
        q = sub.add_parser(command, help='在机器人当前位置录点' if command != 'update' else '重新采集已有点')
        q.add_argument('id')
        q.add_argument('--name', default=None)
        q.add_argument('--duration', type=float, default=2.0)
        q.add_argument('--min-samples', type=int, default=10)
        q.add_argument('--wait-timeout', type=float, default=30.0)
        q.add_argument('--max-tf-age', type=float, default=0.5)
        q.add_argument('--max-fitness', type=float, default=2.0)
        q.add_argument('--max-fitness-age', type=float, default=15.0)
        q.add_argument('--max-position-spread', type=float, default=0.05)
        q.add_argument('--max-yaw-spread', type=float, default=0.05)
    delete = sub.add_parser('delete', help='删除未被指定任务文件引用的点；默认仅预览')
    delete.add_argument('id')
    delete.add_argument('--yes', action='store_true', help='执行删除并备份')
    from g1_task_manager.mission_cli import add_parsers
    from g1_task_manager.mission_control_cli import add_parsers as add_control_parsers
    add_parsers(sub, missions_default=argparse.SUPPRESS)
    add_control_parsers(sub)
    return p


def backup(path, operation):
    # 每次使用独立目录，避免连续操作覆盖同一秒内的备份。
    root = WS / 'backup/waypoints'
    root.mkdir(parents=True, exist_ok=True)
    (WS / 'backup/COLCON_IGNORE').touch(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d_%H%M%S_') + operation + '_', dir=str(root)))
    dest = folder / path.name
    shutil.copy2(str(path), str(dest))
    return dest


def commit(path, original, data, operation, yaml, Store):
    # 先生成并校验临时文件，再备份、原子替换，避免半写入 YAML。
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    tmp = Path(name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        Store(tmp).load()
        if path.read_bytes() != original:
            raise RuntimeError('采样期间点文件已被其他程序修改，未覆盖；请重新运行。')
        saved = backup(path, operation)
        os.chmod(str(tmp), path.stat().st_mode & 0o777)
        os.replace(str(tmp), str(path))
        print('[OK] 已写入：{}'.format(path))
        print('backup: {}'.format(saved))
    finally:
        if tmp.exists():
            tmp.unlink()


def run(args):
    if args.command in ('status', 'continue'):
        from g1_task_manager.mission_control_cli import run_command
        return run_command(args)
    path = Path(args.file).expanduser().resolve()
    missions_path = Path(args.missions_file).expanduser().resolve()
    if args.command == 'path':
        print('workspace: {}\nwaypoints: {}\nmissions: {}'.format(WS, path, missions_path))
        return 0
    try:
        import yaml
        from g1_task_manager.waypoint_store import WaypointStore, UniqueKeyLoader
        from g1_task_manager.mission_store import load_missions
    except ImportError as exc:
        raise RuntimeError('缺少项目源码或 python3-yaml：{}'.format(exc)) from exc

    # 所有本入口的写操作互斥；锁文件不能在释放后删除，否则可能产生锁竞争。
    lock = None
    if args.command in ('record', 'add', 'update', 'delete', 'backup'):
        lock = path.with_name('.' + path.name + '.g1_waypoints.lock').open('a')
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            raise RuntimeError('已有导航点写入/采样操作，请等待完成。')
    try:
        original = path.read_bytes()
        store = WaypointStore(path).load()
        if args.command in ('mission', 'missions'):
            from g1_task_manager.mission_cli import run_command
            return run_command(args, store)
        if args.command == 'list':
            print('file: {}\nmap_id: {}\nframe: {} -> {}'.format(path, store.map_id, store.frame_id, store.base_frame))
            print('ID\tNAME\tX(m)\tY(m)\tYAW(rad)')
            for key, item in store.list_waypoints():
                pose = item['pose']
                print('{}\t{}\t{:.6f}\t{:.6f}\t{:.6f}'.format(key, item['name'], pose['x'], pose['y'], pose['yaw']))
            print('waypoint_count: {}'.format(len(store.list_waypoints())))
        elif args.command == 'show':
            print(yaml.safe_dump({args.id: store.get_waypoint(args.id)}, allow_unicode=True, sort_keys=False), end='')
        elif args.command == 'validate':
            missions = load_missions(missions_path, store)
            print('[OK] 导航点与任务引用有效：{} 个点，{} 个任务'.format(len(store.list_waypoints()), len(missions)))
        elif args.command == 'backup':
            print('backup: {}'.format(backup(path, 'manual')))
        elif args.command == 'delete':
            store.get_waypoint(args.id)
            # 缺失或格式错误时直接失败，不跳过引用检查。
            missions = load_missions(missions_path, store)
            refs = [key for key, task in missions.items() if any(s['waypoint'] == args.id for s in task['steps'])]
            if refs:
                raise RuntimeError('导航点 {} 被任务引用：{}；先修改任务路线。'.format(args.id, ', '.join(refs)))
            if not args.yes:
                print('[PREVIEW] 可删除 {}；检查范围：{}；添加 --yes 执行。'.format(args.id, missions_path))
                return 0
            data = yaml.load(original.decode('utf-8'), Loader=UniqueKeyLoader)
            del data['waypoints'][args.id]
            commit(path, original, data, 'delete', yaml, WaypointStore)
        else:
            exists = store.has_waypoint(args.id)
            if args.command == 'update' and not exists:
                raise RuntimeError('导航点不存在：{}；新建请用 record。'.format(args.id))
            if args.command in ('record', 'add') and exists:
                raise RuntimeError('导航点已存在：{}；重新采集请用 update。'.format(args.id))
            if args.name is not None and not args.name.strip():
                raise RuntimeError('名称不能为空。')
            # 在调用 ROS 前拒绝 NaN/inf/非正阈值。
            for key in ('duration', 'wait_timeout', 'max_tf_age', 'max_fitness', 'max_fitness_age', 'max_position_spread', 'max_yaw_spread'):
                value = getattr(args, key)
                if not math.isfinite(value) or value <= 0:
                    raise RuntimeError('{} 必须是有限正数。'.format(key))
            if args.min_samples < 2:
                raise RuntimeError('min-samples 不能小于2。')
            from g1_task_manager.waypoint_recorder import capture_current_pose
            print('[INFO] 请保持机器人静止；等待定位后连续采样 {:.1f}s。'.format(args.duration), flush=True)
            result = capture_current_pose(
                target_frame=store.frame_id, source_frame=store.base_frame,
                duration_sec=args.duration, minimum_samples=args.min_samples,
                max_position_spread_m=args.max_position_spread,
                max_yaw_spread_rad=args.max_yaw_spread,
                wait_timeout_sec=args.wait_timeout, max_tf_age_sec=args.max_tf_age,
                max_fitness=args.max_fitness, max_fitness_age_sec=args.max_fitness_age)
            data = yaml.load(original.decode('utf-8'), Loader=UniqueKeyLoader)
            item = copy.deepcopy(data['waypoints'].get(args.id, {}))
            item.update(name=args.name.strip() if args.name is not None else item.get('name', args.id),
                        pose={k: result[k] for k in ('x', 'y', 'yaw')},
                        recorded_at=datetime.now().astimezone().isoformat(timespec='seconds'))
            data['waypoints'][args.id] = item
            commit(path, original, data, args.command, yaml, WaypointStore)
            print(yaml.safe_dump({args.id: item, 'sampling': result}, allow_unicode=True, sort_keys=False), end='')
        return 0
    finally:
        if lock is not None:
            lock.close()


def main():
    # help 别名支持直接 bash script/g1_waypoints.sh help。
    argv = sys.argv[1:]
    if not argv or argv == ['help']:
        argv = ['--help']
    args = parser().parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        print('\n[INFO] 已取消。', file=sys.stderr)
        return 130
    except Exception as exc:
        print('[ERROR] {}'.format(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
