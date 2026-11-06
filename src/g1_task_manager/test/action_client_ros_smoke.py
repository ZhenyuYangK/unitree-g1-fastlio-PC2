"""Force Foxy responses before send bookkeeping; no robot interfaces."""
import os
os.environ['ROS_DOMAIN_ID'] = '185'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('CYCLONEDDS_URI', None)

import threading
import time
import unittest
from unittest.mock import patch

import rclpy
import rclpy.action.client as client_module
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from g1_task_manager.action_client import SynchronizedActionClient


def until(predicate, timeout=4.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('action response did not complete')


class Probe:
    async def execute(self, data):
        if any(key in data for key in ('goal', 'cancel', 'result')):
            self.response_seen.set()
        await super().execute(data)


class OriginalProbe(Probe, ActionClient):
    pass


class FixedProbe(Probe, SynchronizedActionClient):
    pass


class ClientRaceTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node('action_sync_test')
        self.group = ReentrantCallbackGroup()
        self.finish = False
        self.server = ActionServer(self.node, FollowPath, '/action_sync_test/follow',
            self.follow, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.original = OriginalProbe(self.node, FollowPath, '/action_sync_test/follow', callback_group=self.group)
        self.fixed = FixedProbe(self.node, FollowPath, '/action_sync_test/follow', callback_group=self.group)
        self.original.response_seen = threading.Event()
        self.fixed.response_seen = threading.Event()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        until(self.fixed.server_is_ready)

    def follow(self, handle):
        while not self.finish and not handle.is_cancel_requested:
            time.sleep(.005)
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return FollowPath.Result()

    def delayed(self, name, client):
        original = getattr(client_module._rclpy_action, name)
        def request(*args, **kwargs):
            sequence = original(*args, **kwargs)
            self.assertTrue(client.response_seen.wait(2.), 'response was not dispatched')
            # The unfixed implementation consumes and discards the reply here;
            # the fixed dispatcher is waiting for its request lock instead.
            time.sleep(.02)
            return sequence
        client.response_seen.clear()
        return patch.object(client_module._rclpy_action, name, request)

    def test_original_foxy_race_is_reproduced(self):
        with self.delayed('rclpy_action_send_goal_request', self.original):
            future = self.original.send_goal_async(FollowPath.Goal())
        time.sleep(.05)
        self.assertFalse(future.done(), 'control must reproduce the lost response')
        self.finish = True
        future.cancel()

    def test_fixed_goal_cancel_and_result_survive_forced_response_race(self):
        for _ in range(15):
            with self.delayed('rclpy_action_send_goal_request', self.fixed):
                accepted = self.fixed.send_goal_async(FollowPath.Goal())
            until(accepted.done)
            handle = accepted.result()
            self.assertTrue(handle.accepted)
            with self.delayed('rclpy_action_send_cancel_request', self.fixed):
                canceled = handle.cancel_goal_async()
            until(canceled.done)
            self.assertTrue(canceled.result().goals_canceling)
            until(lambda: handle.status == 5)
            with self.delayed('rclpy_action_send_result_request', self.fixed):
                result = handle.get_result_async()
            until(result.done)
            self.assertEqual(result.result().status, 5)

    def tearDown(self):
        self.finish = True
        time.sleep(.03)
        self.executor.shutdown()
        self.thread.join(timeout=3.)
        self.original.destroy()
        self.fixed.destroy()
        self.server.destroy()
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
