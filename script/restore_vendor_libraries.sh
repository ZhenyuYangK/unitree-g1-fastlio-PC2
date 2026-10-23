#!/usr/bin/env bash
# 仅恢复显式选择的原生 SDK 库；当前 ROS2 桥不需要运行本脚本。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/g1_env.sh"
exec python3 - "$@" <<'PY'
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import tarfile
import tempfile

p = argparse.ArgumentParser(description='校验/恢复可选 Unitree SDK2 静态库；不联网、不链接 DDS')
p.add_argument('--arch', choices=('aarch64', 'x86_64'), default=platform.machine())
p.add_argument('--check', action='store_true', help='只读校验，不解压')
a = p.parse_args()
root = Path(os.environ['G1_WS'])
try:
    item = json.loads((root / 'third_party/vendor_manifest.json').read_text())['optional_native_libraries'][a.arch]
    path = root / item['path']
    digest = lambda data: hashlib.sha256(data).hexdigest()
    if path.exists():
        if digest(path.read_bytes()) != item['sha256']:
            raise RuntimeError('已有 SDK 库校验失败；未覆盖：{}'.format(path))
    else:
        if a.check:
            raise RuntimeError('缺少可选 SDK 静态库：{}'.format(path))
        archive = path.with_name(path.name + '.tar.gz')
        if not archive.is_file():
            raise RuntimeError('本地未提供压缩库：{}；当前 ROS2 桥无需此库'.format(archive))
        with tarfile.open(str(archive), 'r:gz') as stream:
            entries = stream.getmembers()
            if len(entries) != 1 or not entries[0].isfile() or Path(entries[0].name).name != path.name or entries[0].size != item['bytes']:
                raise RuntimeError('压缩库包含非预期内容')
            data = stream.extractfile(entries[0]).read()
        if digest(data) != item['sha256']:
            raise RuntimeError('压缩库内容 SHA-256 不匹配')
        fd, name = tempfile.mkstemp(dir=str(path.parent), prefix='.' + path.name)
        try:
            with os.fdopen(fd, 'wb') as output:
                output.write(data)
                output.flush()
                os.fsync(output.fileno())
            os.replace(name, str(path))
        finally:
            if os.path.exists(name): os.unlink(name)
    print('[OK] {} {}'.format(a.arch, item['sha256']))
except (OSError, ValueError, KeyError, tarfile.TarError, RuntimeError) as error:
    p.exit(1, '[ERROR] {}\n'.format(error))
PY
