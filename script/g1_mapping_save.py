#!/usr/bin/env python3
"""Require a successful Trigger response and a newly written nonempty map."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

from g1_livox_config import atomic_json


def fingerprint(path):
    try:
        stat = Path(path).stat()
        return stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns
    except FileNotFoundError:
        return None


def verify_saved(path, before, success):
    if not success:
        raise RuntimeError('/map_save 返回 success=false')
    after = fingerprint(path)
    if after is None or after[1] <= 0 or after == before:
        raise RuntimeError('未生成新的非空地图文件：{}'.format(path))
    return after[1]


def save(output, timeout):
    import rclpy
    from std_srvs.srv import Trigger
    before = fingerprint(output)
    rclpy.init(args=[])
    node = rclpy.create_node('g1_mapping_save_client')
    try:
        client = node.create_client(Trigger, '/map_save')
        started = time.monotonic()
        if not client.wait_for_service(timeout_sec=min(timeout, 10.0)):
            raise RuntimeError('/map_save 不存在')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, timeout - (time.monotonic() - started)))
        if not future.done() or future.cancelled():
            raise RuntimeError('/map_save 等待回执超时；保留建图节点供核查')
        response = future.result()
        if response is None:
            raise RuntimeError('/map_save 没有返回结果')
        size = verify_saved(output, before, response.success)
        return dict(success=True, bytes=size, message=response.message,
                    saved_at=datetime.now(timezone.utc).isoformat())
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    state = {}
    try:
        if not 0 < args.timeout <= 600:
            raise ValueError('保存超时须在 (0, 600] 秒内')
        state = json.loads(args.session.read_text())
        result = save(Path(state['output']), args.timeout)
    except Exception as exc:
        atomic_json(args.receipt, dict(success=False, session_id=state.get('session_id'), error=str(exc)))
        parser.exit(1, '[ERROR] {}\n'.format(exc))
    atomic_json(args.receipt, dict(result, session_id=state['session_id'], output=state['output']))
    print('地图保存成功：{} ({} bytes)'.format(state['output'], result['bytes']))


if __name__ == '__main__':
    main()
