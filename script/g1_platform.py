#!/usr/bin/env python3
"""Read-only PC2 dependency/ABI inventory; never creates ROS nodes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


def sha256(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def command(argv):
    try:
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=15, check=False, env=dict(os.environ, LC_ALL='C'))
        return result.returncode, result.stdout.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return 1, str(exc)


def package_prefix(package):
    for value in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if value and (Path(value) / 'share/ament_index/resource_index/packages' / package).exists():
            return Path(value)
    raise RuntimeError('找不到 ament 包：{}'.format(package))


def check_rmw():
    name = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    prefix = package_prefix(name)
    library = prefix / 'lib' / ('lib' + name + '.so')
    if not library.is_file():
        raise RuntimeError('缺少 RMW 共享库：{}'.format(library))
    rc, linked = command(['ldd', str(library)])
    if rc or 'not found' in linked:
        raise RuntimeError('RMW 共享库依赖未满足：\n{}'.format(linked))
    report = dict(implementation=name, prefix=str(prefix), library=str(library.resolve()),
                  sha256=sha256(library), ldd=linked)
    manifest = prefix / 'share' / name / 'package.xml'
    if manifest.exists():
        report['version'] = ET.parse(str(manifest)).getroot().findtext('version')
    if name == 'rmw_cyclonedds_cpp':
        match = re.search(r'libddsc\.so[^\s]*\s+=>\s+(.+?)\s+\(0x[0-9a-fA-F]+\)', linked)
        if not match:
            raise RuntimeError('无法确认 rmw_cyclonedds_cpp 实际链接的 libddsc')
        dds = Path(match.group(1)).resolve()
        expected = os.environ.get('G1_DDS_LIB_DIR')
        if expected and dds.parent != Path(expected).resolve():
            raise RuntimeError('DDS/RMW 配对错误：ldd 使用 {}，配置为 {}'.format(dds, expected))
        machine = command(['readelf', '-h', str(dds)])[1]
        wanted = 'AArch64' if platform.machine() == 'aarch64' else 'X86-64'
        if platform.machine() in ('aarch64', 'x86_64') and wanted.lower() not in machine.lower():
            raise RuntimeError('DDS 库架构与运行主机不一致：{}'.format(dds))
        report.update(dds_library=str(dds), dds_sha256=sha256(dds), dds_elf=machine)
        if os.environ.get('G1_RMW_SETUP'):
            # local_setup 一般位于 overlay 根或隔离安装前缀上一级。
            setup_root = Path(os.environ['G1_RMW_SETUP']).resolve().parent
            try:
                prefix.resolve().relative_to(setup_root)
            except ValueError:
                raise RuntimeError('实际 RMW 不属于 G1_RMW_SETUP：{}'.format(prefix))
    return report


def check_vendors(root):
    manifest = json.loads((root / 'third_party/vendor_manifest.json').read_text())
    failures = []
    for name, expected in manifest['files'].items():
        path = root / name
        if not path.is_file() or sha256(path) != expected['sha256']:
            failures.append(name)
    if failures:
        raise RuntimeError('厂商文件缺失或校验失败：' + ', '.join(failures))
    return dict(checked_files=len(manifest['files']), components=manifest['components'])


def inventory(root, offline=False):
    result = dict(target='Ubuntu 20.04 / Foxy / aarch64 / Python 3.8',
                  host=dict(os_release=Path('/etc/os-release').read_text(),
                            architecture=platform.machine(), python=sys.version,
                            compiler=command(['c++', '--version'])[1],
                            pcl=command(['pkg-config', '--modversion', 'pcl_common'])[1]),
                  paths={key: value for key, value in os.environ.items()
                         if key in ('G1_WS', 'G1_BUILD_DIR', 'G1_INSTALL_DIR', 'G1_ROS_SETUP',
                                    'G1_PCL_SETUP', 'G1_UNITREE_ROS2_SETUP', 'G1_RMW_SETUP', 'G1_DDS_LIB_DIR')},
                  vendors=check_vendors(root), offline=offline)
    if not offline:
        if platform.machine() != 'aarch64' or sys.version_info[:2] != (3, 8):
            raise RuntimeError('PC2 验收要求 aarch64 / Python 3.8；本主机仅可做 --offline 检查')
        if os.environ.get('ROS_DISTRO') != 'foxy':
            raise RuntimeError('PC2 验收要求 ROS 2 Foxy')
        result['rmw'] = check_rmw()
        packages = ('rclpy', 'pcl_ros', 'pcl_conversions', 'unitree_api', 'nav2_map_server',
                    'nav2_controller', 'nav2_planner', 'nav2_recoveries', 'nav2_bt_navigator',
                    'nav2_lifecycle_manager', 'tf2_ros', 'rviz2')
        if os.environ.get('G1_ENABLE_RVIZ') == '0':
            packages = tuple(p for p in packages if p != 'rviz2')
        result['packages'] = {name: str(package_prefix(name)) for name in packages}
        if command(['pkg-config', '--modversion', 'pcl_common'])[0]:
            raise RuntimeError('缺少 PCL 的 pkg-config 版本信息')
        from g1_livox_config import network_values
        host, lidar, domain = network_values(os.environ)
        result['network'] = dict(interface=os.environ['G1_NETWORK_INTERFACE'],
                                 host_ip=host, lidar_ip=lidar, ros_domain_id=domain)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('rmw', 'check'))
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    root = Path(os.environ.get('G1_WS', str(Path(__file__).resolve().parent.parent)))
    try:
        result = check_rmw() if args.action == 'rmw' else inventory(root, args.offline)
        if args.action == 'rmw':
            print('RMW: {} {}'.format(result['implementation'], result['library']), file=sys.stderr)
        else:
            print(json.dumps(result, ensure_ascii=False, indent=2))
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, '[ERROR] {}\n'.format(exc))


if __name__ == '__main__':
    main()
