#!/usr/bin/env python3
"""Refuse reusing a build tree across workspace paths, architectures or underlays."""
import json
import os
from pathlib import Path
import platform
import sys

from g1_livox_config import atomic_json
from g1_platform import check_rmw, command


def main():
    root = Path(os.environ['G1_WS']).resolve()
    build = Path(os.environ['G1_BUILD_DIR']).resolve()
    install = Path(os.environ['G1_INSTALL_DIR']).resolve()
    log = Path(os.environ['G1_LOG_DIR']).resolve()
    for path in (build, install, log):
        if path == root or path in root.parents or any(
                path == root / name or root / name in path.parents
                for name in ('build', 'install', 'log', 'src', 'third_party', 'backup', 'maps', 'task_data')):
            raise RuntimeError('请选择独立的新 PC2 构建/安装/日志目录：{}'.format(path))
    if len({build, install, log}) != 3:
        raise RuntimeError('构建、安装和日志目录必须各不相同')
    stamp = build / '.g1_platform.json'
    rmw = check_rmw()
    state = dict(workspace=str(root), build=str(build), install=str(install),
                 architecture=platform.machine(), ros=os.environ.get('ROS_DISTRO'),
                 python='{}.{}'.format(*sys.version_info[:2]),
                 compiler=command([os.environ.get('CXX', 'c++'), '--version'])[1],
                 rmw={key: rmw[key] for key in ('implementation', 'library', 'sha256', 'version',
                                               'dds_library', 'dds_sha256') if key in rmw},
                 pcl=command(['pkg-config', '--modversion', 'pcl_common'])[1],
                 underlays={key: os.environ.get(key, '') for key in (
                     'G1_ROS_SETUP', 'G1_PCL_SETUP', 'G1_UNITREE_ROS2_SETUP', 'G1_RMW_SETUP', 'G1_DDS_LIB_DIR')})
    if stamp.exists():
        if json.loads(stamp.read_text()) != state:
            raise RuntimeError('路径/平台/underlay 已改变，请配置全新的构建和安装目录')
    elif any(build.iterdir()) or any(install.iterdir()):
        raise RuntimeError('目录已有未验证产物，请选择空目录；不会删除旧资料')
    atomic_json(stamp, state)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        sys.exit('[ERROR] {}'.format(exc))
