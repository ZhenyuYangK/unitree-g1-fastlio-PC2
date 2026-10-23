#!/usr/bin/env python3
"""Verify the generated configuration, active Nav2, feedback and actual bridge."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import yaml


def load_expected(manifest, mode, profile):
    record = json.loads(Path(manifest).read_text(encoding='utf-8'))
    if record['schema_version'] != 1 or record['profile'] != profile:
        raise ValueError('导航配置版本/速度档不匹配；请停止后重新启动导航栈')
    if set(record['files']) != {'nav', 'dry', 'real'}:
        raise ValueError('导航配置不完整；请重新启动导航栈')
    for item in record['files'].values():
        if hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('运行时配置已改变；请重新启动导航栈: ' + item['path'])
    expected = record['expected']
    if mode != 'nav':
        document = yaml.safe_load(Path(record['files'][mode]['path']).read_text(encoding='utf-8'))
        bridge = document['g1_cmd_vel_dry_run']['ros__parameters']
        bridge.update(real_backend=mode == 'real', backend_name='unitree_ros2' if mode == 'real' else 'dry_run')
        expected['g1_cmd_vel_dry_run'] = bridge
    return expected


def parameter_matches(actual, expected):
    if isinstance(expected, float):
        return isinstance(actual, float) and math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)
    return type(actual) is type(expected) and actual == expected


def parameter_python(value):
    # ParameterValue is stable on Foxy; avoid newer rclpy conversion helpers.
    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
              5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
              8: 'double_array_value', 9: 'string_array_value'}
    if value.type not in fields:
        return None
    result = getattr(value, fields[value.type])
    return list(result) if value.type >= 5 else result


def feedback_valid(message, now_ns):
    stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
    values = [getattr(vector, axis) for vector in (message.twist.twist.linear,
              message.twist.twist.angular) for axis in ('x', 'y', 'z')]
    return (message.header.frame_id == 'camera_init' and message.child_frame_id == 'nav_base'
            and stamp > 0 and 0 <= now_ns - stamp <= 1000000000
            and all(math.isfinite(v) for v in values))


def verify_ros(expected, timeout, enable=False):
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from std_srvs.srv import SetBool

    rclpy.init()
    node = rclpy.create_node('g1_navigation_readiness')
    deadline = time.monotonic() + timeout
    feedback = []
    subscription = node.create_subscription(Odometry, '/Odometry_nav', feedback.append, qos_profile_sensor_data)

    def request(service_type, service_name, value):
        client = node.create_client(service_type, service_name)
        try:
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                raise ValueError('服务不可用: ' + service_name)
            future = client.call_async(value)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
            if not future.done() or future.result() is None:
                raise ValueError('服务无响应: ' + service_name)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        for name in ('controller_server', 'planner_server', 'recoveries_server', 'bt_navigator'):
            state = request(GetState, '/' + name + '/get_state', GetState.Request()).current_state
            if state.id != 3:
                raise ValueError(name + ' 不是 active')
        for name, values in expected.items():
            response = request(GetParameters, '/' + name + '/get_parameters',
                               GetParameters.Request(names=list(values)))
            actual = [parameter_python(value) for value in response.values]
            if len(actual) != len(values):
                raise ValueError(name + ' 参数响应不完整')
            for key, value in zip(values, actual):
                if not parameter_matches(value, values[key]):
                    raise ValueError('{} 参数 {} 不匹配: {!r} != {!r}'.format(name, key, value, values[key]))
        # Service discovery can precede node-name discovery. Wait for missing
        # names within the same deadline, while rejecting ambiguous names.
        missing = set(expected) | {'planner_server'}
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            names = node.get_node_names_and_namespaces()
            missing = set()
            for name in set(expected) | {'planner_server'}:
                count = sum(n == name and ns == '/' for n, ns in names)
                if count > 1:
                    raise ValueError('存在同名节点: /' + name)
                if count == 0:
                    missing.add(name)
            if not missing and feedback and feedback_valid(feedback[-1], node.get_clock().now().nanoseconds):
                if enable:
                    service = expected['g1_cmd_vel_dry_run']['enable_service_name']
                    response = request(SetBool, service, SetBool.Request(data=True))
                    if not response.success:
                        raise ValueError('安全门拒绝启用: ' + response.message)
                return
        if missing:
            raise ValueError('节点尚未发现: ' + ', '.join(sorted(missing)))
        raise ValueError('未收到新鲜的 /Odometry_nav 速度反馈')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--mode', choices=('nav', 'dry', 'real'), default='nav')
    parser.add_argument('--profile', choices=('safe', 'target'), default='safe')
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--enable', action='store_true', help='Enable the verified bridge after all checks')
    args = parser.parse_args()
    try:
        expected = load_expected(args.manifest, args.mode, args.profile)
        if args.enable and args.mode == 'nav':
            raise ValueError('--enable requires dry or real bridge mode')
        verify_ros(expected, args.timeout, args.enable)
    except (ValueError, KeyError, OSError, RuntimeError, yaml.YAMLError) as exc:
        parser.exit(1, '[ERROR] 导航就绪检查失败: ' + str(exc) + '\n')
    print('[OK] Nav2 active、速度反馈、运行时参数和后端模式一致 (' + args.mode + '/' + args.profile + ')')


if __name__ == '__main__':
    main()
