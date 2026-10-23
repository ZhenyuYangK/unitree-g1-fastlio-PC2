"""Read PC2 process ownership records without weakening the CLI's stop checks."""
import json
from pathlib import Path

from .paths import SCRIPT_SOURCE
import g1_process


def process(pid, proc=Path('/proc')):
    try:
        stat = (Path(proc) / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=int(pid), state=stat[0], pgid=int(stat[2]), token=int(stat[19]))
    except (FileNotFoundError, ProcessLookupError):
        return None


def group_state(pid, token=None, proc=Path('/proc')):
    if isinstance(token, dict):
        try:
            return 'running' if g1_process.members(token) else 'stopped'
        except (OSError, RuntimeError, ValueError, KeyError):
            return 'unverified'
    leader = process(pid, proc)
    children = [p for path in Path(proc).iterdir() if path.name.isdigit()
                for p in [process(int(path.name), proc)] if p and p['pgid'] == pid and p['state'] != 'Z']
    if not children:
        return 'stopped'
    if token is None or not leader or leader['token'] != token or leader['pgid'] != pid:
        return 'unverified'
    return 'running'


def processes(path):
    path = Path(path)
    if not path.exists():
        return []
    raw = path.read_text()
    try:
        data = json.loads(raw)
    except ValueError:
        # Historical text records are visible but never authorize stopping.
        result = []
        for line in raw.splitlines():
            parts = line.split()
            if len(parts) not in (2, 3) or not parts[1].isdigit():
                raise ValueError('进程记录格式无效')
            pid = int(parts[1])
            token = int(parts[2]) if len(parts) == 3 and parts[2].isdigit() else None
            result.append(dict(name=parts[0], pid=pid, token=token, state=group_state(pid, token)))
        return result
    root = next((p.parent for p in path.parents if p.name == 'runtime'), None)
    if not isinstance(data, dict) or data.get('version') != 1 or not isinstance(data.get('processes'), list):
        raise ValueError('PC2 进程记录格式无效')
    if root is None or data.get('workspace') != str(root.resolve()):
        raise ValueError('进程记录属于另一个工作空间')
    return [dict(name=r['name'], pid=r['pid'], token=r, state=group_state(r['pid'], r)) for r in data['processes']]


def pcd_info(path):
    path = Path(path)
    stat = path.stat()
    header = {}
    with path.open('rb') as stream:
        for _ in range(100):
            line = stream.readline(4096)
            if not line:
                break
            fields = line.decode('ascii').strip().split()
            if not fields or fields[0].startswith('#'):
                continue
            header[fields[0]] = fields[1:]
            if fields[0] == 'DATA':
                if (not {'x', 'y', 'z'}.issubset(header.get('FIELDS', []))
                        or int(header.get('POINTS', ['0'])[0]) <= 0 or stream.tell() >= stat.st_size):
                    break
                return dict(points=int(header['POINTS'][0]), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)
    raise ValueError('PCD 文件为空或头部/数据不完整')
