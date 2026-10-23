"""Action client with acknowledged cancellation on Ctrl+C (Foxy)."""
import argparse
import signal
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from g1_task_interfaces.action import ExecuteMission


def main(args=None):
    parser = argparse.ArgumentParser(description='Submit a mission to the task Action server')
    parser.add_argument('mission_id')
    parsed = parser.parse_args(args)
    rclpy.init(args=[])
    stop = threading.Event()
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, lambda *_: stop.set())
    node = Node('g1_mission_action_client')
    client = ActionClient(node, ExecuteMission, '/g1_task/execute_mission')
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            print('Task Action server unavailable')
            return 2
        if stop.is_set():
            return 130
        send = client.send_goal_async(
            ExecuteMission.Goal(mission_id=parsed.mission_id),
            feedback_callback=lambda msg: print(
                '{} {}/{} {} distance={:.3f}'.format(
                    msg.feedback.state, msg.feedback.current_step,
                    msg.feedback.total_steps, msg.feedback.current_waypoint,
                    msg.feedback.distance_remaining), flush=True))
        # Keep waiting after Ctrl+C if goal acceptance is pending: cancellation
        # needs the accepted handle. Never assume a client exit stops motion.
        while not send.done() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('Mission rejected (busy / invalid / recovery required); check server log')
            return 2
        result = handle.get_result_async()
        cancellation = None
        checked_cancel = False
        next_notice = time.monotonic() + 10.0
        while not result.done() and rclpy.ok():
            if stop.is_set() and cancellation is None:
                cancellation = handle.cancel_goal_async()
                print('Cancel requested; waiting for confirmed cleanup...', flush=True)
            rclpy.spin_once(node, timeout_sec=0.05)
            if cancellation is not None and cancellation.done() and not checked_cancel:
                reply = cancellation.result()
                checked_cancel = True
                if reply is None or not reply.goals_canceling:
                    print('Cancel was not accepted; still waiting for actual mission result', flush=True)
            if stop.is_set() and time.monotonic() > next_notice:
                print('Still waiting. Client exit is NOT a stop confirmation.', flush=True)
                next_notice = time.monotonic() + 10.0
        if not result.done():
            print('ROS stopped without a mission result; stop is UNCONFIRMED')
            return 2
        value = result.result().result
        print('state={} gate_closed={} goals_terminal={}\n{}\nreport={}'.format(
            value.state, value.gate_closed, value.goals_terminal,
            value.message, value.report_path))
        return 0 if value.success else (130 if value.state == 'CANCELED' else 2)
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
