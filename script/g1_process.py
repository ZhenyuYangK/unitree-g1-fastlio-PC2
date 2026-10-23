#!/usr/bin/env python3
"""Manage only processes whose Linux identity and launch ownership are verified."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from g1_livox_config import atomic_json


# 保留 Popen 引用，便于嵌入式调用方回收自己的子进程。
started_processes = []


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def identity(pid):
    try:
        proc = Path('/proc') / str(pid)
        stat = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        if stat[0] == 'Z':
            return None
        command = (proc / 'cmdline').read_bytes()
        environment = (proc / 'environ').read_bytes().split(b'\0')
        return dict(pid=int(pid), uid=proc.stat().st_uid, pgid=int(stat[2]),
                    sid=int(stat[3]), start=stat[19],
                    command_sha256=hashlib.sha256(command).hexdigest(),
                    command=command.replace(b'\0', b' ').decode(errors='replace'),
                    token=next((v.split(b'=', 1)[1].decode() for v in environment
                                if v.startswith(b'G1_PROCESS_TOKEN=')), ''))
    except (FileNotFoundError, ProcessLookupError):
        return None


def read_records(path):
    if not path.exists():
        return []
    try:
        data = json.loads(path.read_text())
        if not isinstance(data, dict):
            raise json.JSONDecodeError('legacy PID record', '', 0)
    except json.JSONDecodeError:
        # P0 两列/单 PID 文件没有创建时间和命令行证据，不允许据此杀进程。
        live = []
        for line in path.read_text().splitlines():
            value = line.split()[-1:]
            if value and value[0].isdigit() and identity(int(value[0])):
                live.append(value[0])
        if live:
            raise RuntimeError('旧 PID 文件无法证明归属，需人工核实：{} ({})'.format(path, ', '.join(live)))
        return []
    if data.get('version') != 1 or not isinstance(data.get('processes'), list):
        raise RuntimeError('进程记录格式无效：{}'.format(path))
    if data.get('workspace') != str(Path(os.environ['G1_WS']).resolve()):
        raise RuntimeError('进程记录属于另一个工作空间：{}'.format(path))
    return data['processes']


def members(record):
    if record['boot_id'] != boot_id():
        return []
    # 启动器及子节点可能各自 setsid；按唯一 token 找到所有后代，再逐组核验。
    found, snapshots = [], []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            stat = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z':
                continue
            pid, pgid, uid = int(path.name), int(stat[2]), path.stat().st_uid
            if uid != os.getuid():
                snapshots.append((pid, pgid, None))
                continue
            try:
                info = identity(pid)
            except PermissionError:
                snapshots.append((pid, pgid, None))
                continue
        except (FileNotFoundError, ProcessLookupError):
            continue
        if info:
            snapshots.append((info['pid'], info['pgid'], info['token']))
        if info and info['token'] == record['token']:
            found.append(info)
    groups = {item['pgid'] for item in found}
    groups.add(record['pgid'])
    for pid, pgid, token in snapshots:
        if pgid in groups and token != record['token']:
            raise RuntimeError('进程组内归属不匹配或无法核验，拒绝发送信号：{}'.format(pid))
    leader = next((item for item in found if item['pid'] == record['pid']), None)
    if leader and any(leader[key] != record[key] for key in ('start', 'command_sha256', 'sid', 'pgid')):
        raise RuntimeError('PID 已复用或命令行改变，拒绝停止：{}'.format(record['pid']))
    return found


def store(path, records):
    atomic_json(path, dict(version=1, workspace=str(Path(os.environ['G1_WS']).resolve()), processes=records))


def start(path, name, log, command):
    records = read_records(path)
    for record in records:
        if record['name'] == name and members(record):
            raise RuntimeError('{} 已由当前工作空间启动'.format(name))
    records = [record for record in records if members(record)]
    token = uuid.uuid4().hex
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    # 当前会话必须从空日志开始，避免旧 NDT success 被当成本次初始化成功。
    if log.exists():
        log.rename(log.with_name('{}.{}.{}.log'.format(log.stem, time.time_ns(), token[:8])))
    with log.open('wb', buffering=0) as output:
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output, stderr=output,
                                   start_new_session=True, close_fds=True,
                                   env=dict(os.environ, G1_PROCESS_TOKEN=token))
    try:
        time.sleep(0.1)
        info = identity(process.pid)
        if not info or info['token'] != token or info['pgid'] != process.pid or info['sid'] != process.pid:
            raise RuntimeError('{} 启动失败，请查看 {}'.format(name, log))
        info.update(name=name, boot_id=boot_id(), argv=command, log=str(log))
        records.append(info)
        store(path, records)
        started_processes.append(process)
    except BaseException:
        # 写入记录失败时清理本次 Popen 的子进程，不留下未记录节点。
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
        raise
    print('{} PID={} log={}'.format(name, process.pid, log))


def stop(path):
    records = read_records(path)
    # 所有记录先验明归属；任何异常都保留原文件。
    for record in records:
        members(record)
    for sig, seconds in ((signal.SIGINT, 3.0), (signal.SIGTERM, 5.0)):
        for record in reversed(records):
            for pgid in sorted({item['pgid'] for item in members(record)}, reverse=True):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not any(members(record) for record in records):
                path.unlink(missing_ok=True)
                return
            time.sleep(0.1)
    if any(members(record) for record in records):
        raise RuntimeError('仍有进程未退出，已保留 PID 记录；停止操作未完成')
    path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    parser.add_argument('--file', type=Path, required=True)
    parser.add_argument('--name')
    parser.add_argument('--log')
    # REMAINDER 要求启动命令使用 -- 明确分隔。
    raw = sys.argv[1:]
    command = []
    if '--' in raw:
        split = raw.index('--')
        command, raw = raw[split + 1:], raw[:split]
    args = parser.parse_args(raw)
    try:
        if args.action == 'start':
            if not args.name or not args.log or not command:
                parser.error('start 需要 --name、--log 和 -- command')
            start(args.file, args.name, args.log, command)
        elif args.action == 'stop':
            stop(args.file)
        else:
            running = False
            for record in read_records(args.file):
                if args.name and record['name'] != args.name:
                    continue
                live = bool(members(record))
                running = running or live
                print('{} {} {}'.format(record['name'], record['pid'], '运行中' if live else '已退出'))
            return 0 if running else 1
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print('[ERROR] {}'.format(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
