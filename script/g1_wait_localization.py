#!/usr/bin/env python3
"""Wait for confirmed, fresh localization; optional fresh TF check. No motion calls."""
import argparse
import math
from pathlib import Path
import sys
import time

# The workspace script and ROS task consumers use exactly the same policy.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/g1_task_manager'))
from g1_task_manager.health import (  # noqa: E402
    MAX_STAMP_NS, localization_status_reason, parse_localization_status, tf_reason,
)

STATUS_TOPIC = '/g1_relocalization/status'


def stamp_problem(status, field, now_ns, age_sec):
    stamp = status.get(field)
    if type(stamp) is not int or not 0 < stamp <= MAX_STAMP_NS:
        return '{} 无效'.format(field)
    age = (now_ns - stamp) / 1e9
    if age < 0 or age > age_sec:
        return '{} 已过期/来自未来'.format(field)
    return None


def assess_status(status, now_ns, status_age_sec=1.0, scan_age_sec=3.0):
    reason = localization_status_reason(status, 0.0, now_ns, scan_age_sec)
    if not reason:
        reason = stamp_problem(status, 'stamp_ns', now_ns, status_age_sec)
    return not reason, reason or '{}: 定位有效，最近成功扫描仍新鲜'.format(status['state'])


def terminal_failure(status, now_ns):
    return (isinstance(status, dict) and isinstance(status.get('state'), str)
            and status['state'] in ('FAILED', 'AMBIGUOUS')
            and type(now_ns) is int and 0 < now_ns <= MAX_STAMP_NS
            and stamp_problem(status, 'stamp_ns', now_ns, 1.0) is None)


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('必须是有限的正数')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=positive_seconds)
    parser.add_argument('--once', action='store_true', help='检查实时状态，默认最多等待5秒')
    parser.add_argument('--tf-frame', help='同时检查 map 到指定坐标系的最新TF')
    parser.add_argument('--global-frame', default='map')
    parser.add_argument('--max-tf-age', type=positive_seconds, default=0.5)
    args = parser.parse_args(argv)
    timeout = args.timeout if args.timeout is not None else (5.0 if args.once else 180.0)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String
        if args.tf_frame:
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformListener
            try:
                from tf2_ros import TransformException
            except ImportError:
                from tf2_py import TransformException
    except ImportError as exc:
        print('[ERROR] 无法加载ROS Python环境：{}'.format(exc), file=sys.stderr)
        return 2

    rclpy.init(args=[])
    node = Node('g1_navigation_wait_localization')
    latest = None
    received_at = None
    last_reason = '未收到 ' + STATUS_TOPIC
    last_report = None

    def on_status(message):
        nonlocal latest, received_at
        latest = parse_localization_status(message.data)
        received_at = time.monotonic()

    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(String, STATUS_TOPIC, on_status, qos)
    if args.tf_frame:
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline - time.monotonic())))
            now_ns = node.get_clock().now().nanoseconds
            received_age = None if received_at is None else time.monotonic() - received_at
            last_reason = localization_status_reason(latest, received_age, now_ns)
            if not last_reason and args.tf_frame:
                try:
                    transform = tf_buffer.lookup_transform(args.global_frame, args.tf_frame, Time())
                    stamp = transform.header.stamp
                    last_reason = tf_reason(int(stamp.sec) * 1000000000 + int(stamp.nanosec),
                                            now_ns, args.max_tf_age)
                except TransformException as exc:
                    last_reason = 'TF {}→{}不可用：{}'.format(args.global_frame, args.tf_frame, exc)
            if not last_reason:
                print('[G1-LOCALIZATION] {}: 定位及新鲜度检查通过'.format(latest['state']), flush=True)
                return 0
            if last_reason != last_report:
                print('[G1-LOCALIZATION] ' + last_reason, flush=True)
                last_report = last_reason
            if received_age is not None and received_age <= 1. and terminal_failure(latest, now_ns):
                return 1
        print('[ERROR] 等待定位超时：' + last_reason, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
