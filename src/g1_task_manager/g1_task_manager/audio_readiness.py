"""Read-only checks for the real speech action; never submit a speech goal."""
import argparse
import math
import time


class AudioReadinessError(RuntimeError):
    pass


class AudioAbsent(AudioReadinessError):
    pass


def wait_for_audio(node, audio, spin, timeout=5.0, allow_absent=False):
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.action.graph import get_action_server_names_and_types_by_node

    parameters = node.create_client(GetParameters, '/g1_audio_bridge/get_parameters')
    deadline = time.monotonic() + timeout
    pending = None
    verified = False
    seen = False
    reason = '未发现语音桥'
    try:
        while True:
            nodes = node.get_node_names_and_namespaces()
            count = nodes.count(('g1_audio_bridge', '/'))
            action_ready = audio.server_is_ready()
            # A server under another node name must not cause us to start a
            # competing server for the same action. The generic service graph
            # also lists clients, including our own ActionClient; inspect servers.
            owners = []
            for name, namespace in set(nodes):
                try:
                    actions = get_action_server_names_and_types_by_node(node, name, namespace)
                except RuntimeError:  # A node may disappear during discovery.
                    continue
                if any(action == '/g1_audio/speak' for action, _ in actions):
                    owners.append((name, namespace))
            seen = seen or count > 0 or action_ready or bool(owners)
            if count > 1:
                raise AudioReadinessError('发现多个 /g1_audio_bridge 节点，请先停止重复实例')
            if len(owners) > 1:
                raise AudioReadinessError('发现多个 /g1_audio/speak 服务端，请先停止重复实例')
            if pending is None and parameters.service_is_ready():
                pending = parameters.call_async(GetParameters.Request(names=['dry_run']))
            if pending is not None and pending.done() and not verified:
                response = pending.result()
                if (response is None or len(response.values) != 1
                        or response.values[0].type != ParameterType.PARAMETER_BOOL):
                    raise AudioReadinessError('无法确认语音桥 dry_run 参数')
                if response.values[0].bool_value:
                    raise AudioReadinessError(
                        '语音桥为 dry_run=true，请停止该实例后重新执行 g1_navigation_real.sh start')
                verified = True
            if (count == 1 and action_ready and verified
                    and owners == [('g1_audio_bridge', '/')]):
                return
            if count:
                reason = ('语音桥参数服务未就绪' if not verified else
                          '语音 Action /g1_audio/speak 未就绪')
            elif seen:
                reason = '语音服务存在，但未发现标准节点 /g1_audio_bridge'
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if allow_absent and not seen:
                    raise AudioAbsent(reason)
                raise AudioReadinessError(
                    '{}；请执行 g1_navigation_real.sh start，并检查网卡和 ROS_DOMAIN_ID'.format(reason))
            spin(min(0.05, remaining))
    finally:
        if pending is not None and not pending.done():
            parameters.remove_pending_request(pending)
        node.destroy_client(parameters)


def positive_seconds(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('必须是有限正数')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=positive_seconds, default=8.0)
    parser.add_argument('--probe', action='store_true',
                        help='完全未发现语音桥时返回 3，供启动脚本判断是否需要创建进程')
    args = parser.parse_args(argv)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from g1_task_interfaces.action import Speak

    rclpy.init(args=[])
    node = Node('g1_check_audio')
    audio = ActionClient(node, Speak, '/g1_audio/speak')
    try:
        wait_for_audio(node, audio, lambda seconds: rclpy.spin_once(node, timeout_sec=seconds),
                       timeout=args.timeout, allow_absent=args.probe)
        print('[OK] 真实语音桥就绪：/g1_audio/speak，dry_run=false', flush=True)
        return 0
    except AudioAbsent:
        return 3
    except AudioReadinessError as error:
        print('[ERROR] {}'.format(error), flush=True)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        audio.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
