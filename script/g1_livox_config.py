#!/usr/bin/env python3
"""Generate a PC2 MID360 runtime config without modifying the vendor template."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import tempfile


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name, dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, str(path))
    finally:
        if os.path.exists(name):
            os.unlink(name)


def network_values(environ, addresses=None):
    interface = environ.get('G1_NETWORK_INTERFACE', '')
    if not interface or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-' for c in interface):
        raise ValueError('请配置 PC2 实际 G1_NETWORK_INTERFACE')
    host = ipaddress.IPv4Address(environ.get('G1_HOST_IP', ''))
    lidar = ipaddress.IPv4Address(environ.get('G1_LIDAR_IP', ''))
    if host == lidar or any(ip.is_unspecified or ip.is_multicast or ip.is_loopback or int(ip) == 0xffffffff for ip in (host, lidar)):
        raise ValueError('主机与雷达必须是不同的有效单播 IPv4 地址')
    domain = int(environ.get('ROS_DOMAIN_ID', '0'))
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID 必须在 0–232 范围内')
    if addresses is None:
        result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'dev', interface],
                                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        addresses = json.loads(result.stdout)
    matching = [item for link in addresses if link.get('ifname') == interface
                for item in link.get('addr_info', []) if item.get('local') == str(host)]
    if not matching:
        raise ValueError('G1_HOST_IP 未配置在指定网卡上：{} / {}'.format(interface, host))
    if not any(lidar in ipaddress.IPv4Network('{}/{}'.format(host, item['prefixlen']), strict=False)
               for item in matching):
        raise ValueError('雷达 IP 与指定主机网卡不在同一子网')
    return str(host), str(lidar), domain


def generate(template, host, lidar):
    data = json.loads(Path(template).read_text(encoding='utf-8'))
    info = data['MID360']['host_net_info']
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        info[key] = host
    # 不因旧模板是否启用日志而向旧机器发送数据。
    info['log_data_ip'] = host if info.get('log_data_ip') else ''
    if len(data['lidar_configs']) != 1:
        raise ValueError('当前 PC2 入口要求单个 MID360 配置')
    data['lidar_configs'][0]['ip'] = lidar
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只检查配置和网卡，不写文件')
    args = parser.parse_args()
    root = Path(os.environ.get('G1_WS', str(Path(__file__).resolve().parent.parent)))
    try:
        host, lidar, domain = network_values(os.environ)
        template = root / 'src/livox_ros_driver2/config/MID360_config.json'
        data = generate(template, host, lidar)
        if not args.check:
            output = Path(os.environ.get('G1_LIVOX_CONFIG', str(root / 'runtime/livox/MID360_config.json')))
            atomic_json(output, data)
            print('Livox config: {}'.format(output))
        print('Network: {} -> {}, ROS_DOMAIN_ID={}'.format(host, lidar, domain))
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as exc:
        parser.exit(1, '[ERROR] {}\n'.format(exc))


if __name__ == '__main__':
    main()
