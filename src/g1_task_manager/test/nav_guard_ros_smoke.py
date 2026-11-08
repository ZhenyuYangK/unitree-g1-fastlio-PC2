"""Isolated DDS tests with synthetic sensors and mock planner/controller/audio.

Never starts a Unitree backend. The test domain is separate from real navigation.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '183'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('CYCLONEDDS_URI', None)

import json
import threading
import time
import unittest

import rclpy
from dwb_msgs.msg import CriticScore, LocalPlanEvaluation, TrajectoryScore
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from g1_task_interfaces.action import Speak
from g1_task_interfaces.srv import NavigationHold
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from g1_task_manager.nav_guard import NavigationGuard
from g1_task_manager.action_client import SynchronizedActionClient
from g1_task_manager.mission_ros import MissionTransport
from g1_task_manager.mission_engine import MissionError
from g1_task_manager.mission_engine import execute_mission
from g1_task_manager.nav2_client import Nav2WaypointClient, Nav2NavigationError
from types import SimpleNamespace


def until(predicate, timeout=8.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    raise AssertionError('condition timed out')


class World(Node):
    def __init__(self):
        super().__init__('guard_test_world')
        self.group = ReentrantCallbackGroup()
        self.blocked = False
        self.local_blocked = False
        self.detour = 0.
        self.sensor_enabled = True
        self.odom_enabled = True
        self.cloud_future = False
        self.auto_recover = True
        self.hold_future = None
        self.hold_command = None
        self.acked_episode = None
        self.manual_gate = False
        self.gate_events = []
        self.plan_count = 0
        self.abort_during_replan = False
        self.succeed_during_replan = False
        self.abort_current = False
        self.arrive = False
        self.speeches = []
        self.statuses = []
        self.follow_generation = 0
        self.follows = 0
        self.local_failure = False
        self.stationary_blocked = False
        self.cancel_delay = 0.
        self.goal_x = 5.
        self.active_follow_ids = set()
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.live = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', qos)
        self.static = self.create_publisher(OccupancyGrid, '/g1_reference/global_costmap/costmap', qos)
        self.status = self.create_publisher(String, '/g1_relocalization/status', qos)
        self.fitness = self.create_publisher(Float64, '/localization_fitness', 1)
        self.cloud = self.create_publisher(PointCloud2, '/cloud_registered_body', 1)
        self.odom = self.create_publisher(Odometry, '/Odometry_nav', 1)
        self.eval = self.create_publisher(LocalPlanEvaluation, '/evaluation', 1)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, '/g1_navigation/status',
                                 lambda m: self.statuses.append(json.loads(m.data)), 100,
                                 callback_group=self.group)
        self.servers = [
            ActionServer(self, ComputePathToPose, '/compute_path_to_pose', self.plan,
                         callback_group=self.group),
            ActionServer(self, ComputePathToPose, '/g1_reference/compute_path_to_pose', self.reference,
                         callback_group=self.group),
            ActionServer(self, FollowPath, '/follow_path', self.follow,
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group),
            ActionServer(self, Speak, '/g1_audio/speak', self.speak,
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)]
        self.client = SynchronizedActionClient(self, NavigateToPose, '/g1_navigation/execute', callback_group=self.group)
        self.hold_client = self.create_client(NavigationHold, '/g1_navigation/data_hold',
                                              callback_group=self.group)
        self.create_service(SetBool, '/test_gate', self.gate, callback_group=self.group)
        self.create_timer(.05, self.publish, callback_group=self.group)

    def gate(self, request, response):
        self.manual_gate = request.data
        self.gate_events.append((time.monotonic(), request.data))
        response.success = True
        response.message = 'mock gate'
        return response

    def acknowledge_hold(self):
        if not self.auto_recover or not self.statuses:
            return
        status = self.statuses[-1]
        if self.hold_future is not None:
            if not self.hold_future.done():
                return
            response = self.hold_future.result()
            if response.accepted and self.hold_command == 'ack':
                self.acked_episode = response.episode
            self.hold_future = None
        if not status.get('data_hold'):
            return
        self.manual_gate = False
        episode = status['hold_episode']
        resume = self.acked_episode == episode and status.get('hold_ready')
        if self.acked_episode == episode and not resume:
            return
        request = NavigationHold.Request(session=status['session'], episode=episode,
                                         resume=bool(resume), reason='' if resume else 'ACK')
        self.hold_command = 'release' if resume else 'ack'
        self.hold_future = self.hold_client.call_async(request)

    def path(self, detour=0.):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        points = [(i*self.goal_x/50, 0.) for i in range(51)] if detour == 0 else [
            (0., 0.), (0., detour), (self.goal_x, detour), (self.goal_x, 0.)]
        for x, y in points:
            pose = PoseStamped()
            pose.header = p.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.
            p.poses.append(pose)
        return p

    def reference(self, handle):
        handle.succeed()
        return ComputePathToPose.Result(path=self.path())

    def plan(self, handle):
        self.plan_count += 1
        if self.plan_count == 2 and (self.abort_during_replan or self.succeed_during_replan):
            self.abort_current = self.abort_during_replan
            self.arrive = self.succeed_during_replan
            time.sleep(.35)
        if self.blocked:
            handle.abort()
            return ComputePathToPose.Result()
        handle.succeed()
        return ComputePathToPose.Result(path=self.path(self.detour))

    def follow(self, handle):
        key = bytes(handle.goal_id.uuid)
        self.active_follow_ids.add(key)
        try:
            return self.follow_active(handle)
        finally:
            self.active_follow_ids.discard(key)

    def follow_active(self, handle):
        self.follow_generation += 1
        generation = self.follow_generation
        self.follows += 1
        cancel_at = None
        while rclpy.ok():
            if self.abort_current:
                self.abort_current = False
                handle.abort()
                return FollowPath.Result()
            if handle.is_cancel_requested:
                if cancel_at is None:
                    cancel_at = time.monotonic()
                if time.monotonic()-cancel_at >= self.cancel_delay:
                    handle.canceled()
                    return FollowPath.Result()
            if generation != self.follow_generation:
                handle.abort()
                return FollowPath.Result()
            evaluation = LocalPlanEvaluation()
            evaluation.header.frame_id = 'camera_init'
            evaluation.header.stamp = self.get_clock().now().to_msg()
            invalid = self.local_blocked or self.local_failure
            score = TrajectoryScore(total=-1. if invalid else 1.)
            if not invalid and not self.stationary_blocked:
                score.traj.velocity.x = .1
            if invalid:
                score.scores = [CriticScore(name='BaseObstacle' if self.local_blocked else 'Oscillation',
                                            raw_score=-1.)]
            evaluation.twists = [score]
            if self.stationary_blocked:
                evaluation.twists.append(TrajectoryScore(total=-1., scores=[
                    CriticScore(name='BaseObstacle', raw_score=-1.)]))
            self.eval.publish(evaluation)
            if invalid:
                time.sleep(.08)
                handle.abort()
                return FollowPath.Result()
            if self.arrive:
                handle.succeed()
                return FollowPath.Result()
            time.sleep(.05)

    def speak(self, handle):
        self.speeches.append(handle.request.text)
        handle.succeed()
        return Speak.Result(success=True, message='mock')

    def publish(self):
        now = self.get_clock().now()
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.header.stamp = now.to_msg()
        transform.child_frame_id = 'nav_base'
        transform.transform.rotation.w = 1.
        self.broadcaster.sendTransform(transform)
        status = dict(valid=True, state='TRACKING', stamp_ns=now.nanoseconds,
                      last_success_stamp_ns=now.nanoseconds)
        self.status.publish(String(data=json.dumps(status)))
        self.fitness.publish(Float64(data=.03))
        odometry = Odometry()
        odometry.header.frame_id = 'camera_init'
        odometry.child_frame_id = 'nav_base'
        odometry.header.stamp = now.to_msg()
        odometry.pose.pose.orientation.w = 1.
        if self.odom_enabled:
            self.odom.publish(odometry)
        if self.sensor_enabled:
            cloud = PointCloud2()
            cloud.header.frame_id = 'body'
            cloud.header.stamp = now.to_msg()
            if self.cloud_future:
                cloud.header.stamp.sec += 5
            self.cloud.publish(cloud)
        value = OccupancyGrid()
        value.header.frame_id = 'map'
        value.header.stamp = now.to_msg()
        value.info.resolution = .1
        value.info.width = value.info.height = 100
        value.info.origin.position.x = -1.
        value.info.origin.position.y = -5.
        value.info.origin.orientation.w = 1.
        value.data = [0]*10000
        self.static.publish(value)
        if self.blocked or self.detour > 0 or self.local_blocked or self.stationary_blocked:
            for col in range(29, 32):
                for row in range(49, 52):
                    value.data[row*100 + col] = 100
        self.live.publish(value)
        self.acknowledge_hold()

    def destroy_node(self):
        self.client.destroy()
        for server in self.servers:
            server.destroy()
        return super().destroy_node()


class GuardRosTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.guard = NavigationGuard()
        self.world = World()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.guard)
        self.executor.add_node(self.world)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.handle = None
        until(lambda: self.world.client.server_is_ready() and not self.guard.health())

    def tearDown(self):
        if self.handle and self.handle.status in (1, 2, 3):
            future = self.handle.cancel_goal_async()
            until(future.done)
            until(self.handle.get_result_async().done)
        self.executor.shutdown()
        self.thread.join(timeout=3.)
        self.guard.destroy_node()
        self.world.destroy_node()
        rclpy.shutdown()

    def start(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.world.goal_x
        goal.pose.pose.orientation.w = 1.
        future = self.world.client.send_goal_async(goal)
        until(future.done)
        self.handle = future.result()
        self.assertTrue(self.handle.accepted)
        return self.handle.get_result_async()

    def latest(self):
        return self.world.statuses[-1] if self.world.statuses else {}

    def test_navigates_detour_within_budget_without_speech(self):
        self.world.detour = 4.5  # 14 m vs original 5 m: +9 m.
        result = self.start()
        until(lambda: self.latest().get('permit'))
        self.assertEqual(self.world.speeches, [])
        self.world.arrive = True
        until(result.done)
        self.assertEqual(result.result().status, 4)

    def test_excessive_detour_waits_once_then_resumes_after_two_seconds(self):
        self.world.detour = 5.5  # 16 m vs original 5 m: +11 m.
        self.start()
        until(lambda: len(self.world.speeches) == 1)
        self.assertFalse(self.latest()['permit'])
        self.assertEqual(self.world.follows, 0)
        time.sleep(1.2)
        self.assertEqual(len(self.world.speeches), 1)
        before = time.monotonic()
        self.world.detour = 0.
        until(lambda: self.latest().get('permit'))
        self.assertGreaterEqual(time.monotonic() - before, 1.9)

    def test_local_dwb_blockade_stops_and_speaks_once(self):
        self.world.local_blocked = True
        self.start()
        until(lambda: len(self.world.speeches) == 1)
        self.assertFalse(self.latest()['permit'])
        self.world.local_blocked = False
        until(lambda: self.latest().get('permit'))
        self.assertEqual(len(self.world.speeches), 1)

    def test_cancel_during_wait_prevents_late_speech_and_motion(self):
        self.world.blocked = True
        result = self.start()
        until(lambda: self.latest().get('state') == 'BLOCKED')
        until(self.handle.cancel_goal_async().done)
        until(result.done)
        self.assertEqual(result.result().status, 5)
        self.world.blocked = False
        time.sleep(1.2)
        self.assertEqual(self.world.speeches, [])
        self.assertFalse(self.latest()['permit'])

    def test_sensor_failure_aborts_without_obstacle_speech(self):
        result = self.start()
        until(lambda: self.latest().get('permit'))
        self.world.sensor_enabled = False
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertFalse(self.latest()['permit'])
        self.assertEqual(self.world.speeches, [])

    def test_short_cloud_outage_stops_then_resumes_same_goal(self):
        result = self.start()
        until(lambda: self.latest().get('permit'))
        session = self.latest()['session']
        self.world.sensor_enabled = False
        until(lambda: self.latest().get('data_hold'))
        self.assertFalse(self.latest()['permit'])
        until(lambda: not self.world.active_follow_ids)
        budget = self.guard.budget
        self.world.sensor_enabled = True
        until(lambda: self.latest().get('permit'))
        self.assertIs(self.guard.budget, budget)
        self.assertEqual(self.latest()['session'], session)
        self.assertFalse(result.done())
        self.assertEqual(self.world.speeches, [])
        self.world.arrive = True
        until(result.done)
        self.assertEqual(result.result().status, 4)

    def test_missing_closed_gate_ack_never_resumes_motion(self):
        self.world.auto_recover = False
        result = self.start()
        until(lambda: self.latest().get('permit'))
        self.world.sensor_enabled = False
        until(lambda: self.latest().get('data_hold'))
        self.world.sensor_enabled = True
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertIn('DATA_RECOVERY_TIMEOUT', self.guard.last_fault)
        self.assertFalse(self.latest()['permit'])

    def test_cross_session_and_late_episode_ack_cannot_release_hold(self):
        self.world.auto_recover = False
        result = self.start()
        until(lambda: self.latest().get('permit'))
        self.world.sensor_enabled = False
        until(lambda: self.latest().get('data_hold'))
        session, episode = self.latest()['session'], self.latest()['hold_episode']
        for wrong_session, wrong_episode in [('f'*32, episode), (session, episode+1)]:
            request = NavigationHold.Request(session=wrong_session, episode=wrong_episode,
                                             resume=False, reason='ACK')
            future = self.world.hold_client.call_async(request)
            until(future.done)
            self.assertFalse(future.result().accepted)
        self.world.sensor_enabled = True
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertFalse(self.latest()['permit'])

    def test_repeated_short_outages_exhaust_per_goal_recovery_budget(self):
        result = self.start()
        until(lambda: self.latest().get('permit'))
        for episode in range(1, 4):
            self.world.sensor_enabled = False
            until(lambda: self.latest().get('data_hold'))
            self.assertEqual(self.latest()['hold_episode'], episode)
            self.world.sensor_enabled = True
            until(lambda: self.latest().get('permit'))
        self.world.sensor_enabled = False
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertIn('DATA_RECOVERY_LIMIT', self.guard.last_fault)

    def test_cancel_during_data_hold_does_not_resume(self):
        result = self.start()
        until(lambda: self.latest().get('permit'))
        self.world.odom_enabled = False
        until(lambda: self.latest().get('data_hold'))
        until(self.handle.cancel_goal_async().done)
        self.world.odom_enabled = True
        until(result.done)
        self.assertEqual(result.result().status, 5)
        self.assertFalse(self.latest()['permit'])

    def test_future_sensor_timestamp_is_hard_fault_not_recovery(self):
        result = self.start()
        until(lambda: self.latest().get('permit'))
        self.world.cloud_future = True
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertEqual(self.guard.last_fault, 'OBSTACLE_SENSOR_CLOCK_INVALID')
        self.assertFalse(any(s.get('data_hold') for s in self.world.statuses))

    def test_controller_failure_during_planning_is_not_replaced(self):
        self.world.abort_during_replan = True
        result = self.start()
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertEqual(self.world.follows, 1)
        self.assertIn('CONTROLLER_FAILED_WITHOUT_OBSTACLE:6', self.guard.last_fault)

    def test_controller_success_during_planning_does_not_send_another_goal(self):
        self.world.succeed_during_replan = True
        result = self.start()
        until(result.done)
        self.assertEqual(result.result().status, 4)
        self.assertEqual(self.world.follows, 1)

    def test_nonobstacle_controller_error_is_fault_not_wait(self):
        self.world.detour = 4.5  # An unrelated global obstacle cannot hide this fault.
        self.world.local_failure = True
        result = self.start()
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertEqual(self.world.speeches, [])

    def test_planner_abort_without_live_obstacle_is_fault_not_indefinite_wait(self):
        # Mock a planner failure while costmaps and DWB show no obstacle.
        self.world.blocked = True
        self.guard.blocked_evidence = lambda: False
        result = self.start()
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertEqual(self.world.speeches, [])

    def test_lost_acceptance_response_cancels_exact_uuid_and_allows_next_task(self):
        original = self.guard.controller.send_goal_async
        hidden = []
        def lose_response(*args, **kwargs):
            hidden.append(original(*args, **kwargs))
            return Future()  # Server receives the goal; coordinator loses ack.
        self.guard.controller.send_goal_async = lose_response
        result = self.start()
        until(lambda: self.world.active_follow_ids)
        until(result.done, 10.)
        self.assertEqual(result.result().status, 6)
        until(lambda: not self.world.active_follow_ids)
        until(lambda: not self.guard.fault_latched and not self.guard.busy)
        self.assertIn('ACTION_ACCEPTANCE_TIMEOUT', self.guard.last_fault)
        self.assertFalse(self.latest()['permit'])
        self.guard.controller.send_goal_async = original
        self.start()
        until(lambda: self.latest().get('permit'))

    def test_late_controller_stop_holds_admission_then_recovers_without_restart(self):
        self.world.cancel_delay = 6.
        result = self.start()
        until(lambda: self.latest().get('permit'))
        until(self.handle.cancel_goal_async().done)
        until(result.done, 8.)
        until(lambda: self.guard.fault_latched)
        self.assertFalse(self.latest()['permit'])
        self.assertTrue(self.latest()['pending_owned_goals'])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 5.
        goal.pose.pose.orientation.w = 1.
        attempt = self.world.client.send_goal_async(goal)
        until(attempt.done)
        self.assertFalse(attempt.result().accepted)
        until(lambda: not self.guard.fault_latched and not self.world.active_follow_ids)
        self.world.cancel_delay = 0.
        self.start()
        until(lambda: self.latest().get('permit'))

    def test_legal_zero_trajectory_with_obstacle_enters_wait_instead_of_progress_abort(self):
        self.world.stationary_blocked = True
        self.start()
        until(lambda: len(self.world.speeches) == 1)
        self.assertFalse(self.latest()['permit'])
        self.assertTrue(self.handle.is_active if hasattr(self.handle, 'is_active') else self.handle.status in (1, 2))
        self.world.stationary_blocked = False
        until(lambda: self.latest().get('permit'))
        self.assertEqual(len(self.world.speeches), 1)

    def test_stationary_obstacle_outside_arrival_tolerance_still_waits(self):
        self.world.goal_x = .3
        self.world.stationary_blocked = True
        self.start()
        until(lambda: len(self.world.speeches) == 1)
        self.assertFalse(self.latest()['permit'])
        self.world.stationary_blocked = False
        until(lambda: self.latest().get('permit'))

    def test_tf_failure_stops_without_speech_then_new_goal_can_run(self):
        result = self.start()
        until(lambda: self.latest().get('permit'))
        original = self.guard.tf.lookup_transform
        def missing(*args, **kwargs):
            raise RuntimeError('synthetic TF outage')
        self.guard.tf.lookup_transform = missing
        until(result.done)
        self.assertEqual(result.result().status, 6)
        self.assertEqual(self.world.speeches, [])
        self.assertFalse(self.world.active_follow_ids)
        self.guard.tf.lookup_transform = original
        self.start()
        until(lambda: self.latest().get('permit'))


class MissionRecoveryTests(unittest.TestCase):
    setUp = GuardRosTests.setUp
    tearDown = GuardRosTests.tearDown
    latest = GuardRosTests.latest

    def run_recovery(self, task_tf=False):
        self.world.auto_recover = False
        node = Nav2WaypointClient('map', 'nav_base')
        node.action_client.destroy()
        node.action_client = ActionClient(node, NavigateToPose, '/g1_navigation/execute')
        transport = MissionTransport(node, gate_service='/test_gate')
        transport.cancel = node.create_client(
            __import__('action_msgs.srv', fromlist=['CancelGoal']).CancelGoal,
            '/g1_navigation/execute/_action/cancel_goal')
        original = node.check_health
        tf_outage = threading.Event()
        def check(*args, **kwargs):
            if tf_outage.is_set():
                raise Nav2NavigationError('synthetic task TF lag', 'TASK_TF_STALE')
            return original(*args, **kwargs)
        node.check_health = check
        mission = dict(id='recovery', name='recovery', steps=[dict(
            waypoint='point_01', goal=dict(pose=dict(x=5., y=0., yaw=0.)),
            timeout_sec=15., dwell_sec=0., speech=None, advance_mode='auto')])
        reports = []
        worker = threading.Thread(target=lambda: reports.append(
            execute_mission(mission, transport, lambda _: None)), daemon=True)
        worker.start()
        try:
            until(lambda: self.latest().get('permit') and self.world.manual_gate)
            if task_tf:
                tf_outage.set()
            else:
                self.world.sensor_enabled = False
            until(lambda: self.latest().get('data_hold'))
            until(lambda: not self.world.manual_gate)
            self.assertFalse(self.latest()['permit'])
            until(lambda: not self.world.active_follow_ids)
            self.assertFalse(self.world.active_follow_ids)
            self.world.sensor_enabled = True
            tf_outage.clear()
            until(lambda: self.latest().get('permit') and self.world.manual_gate)
            self.world.arrive = True
            worker.join(6.)
            self.assertFalse(worker.is_alive())
            self.assertEqual(reports[0]['state'], 'SUCCEEDED', reports)
            self.assertTrue(reports[0]['gate_closed'])
            self.assertTrue(reports[0]['goals_terminal'])
            self.assertFalse(self.world.manual_gate)
            self.assertEqual(sum(opened for _, opened in self.world.gate_events), 2)
        finally:
            if worker.is_alive() and transport.handle:
                transport.handle.cancel_goal_async()
                worker.join(6.)
            transport.destroy()

    def test_guard_cloud_hold_closes_manual_gate_and_recovers_current_task(self):
        self.run_recovery()

    def test_task_tf_lag_uses_same_hold_protocol_instead_of_aborting_tour(self):
        self.run_recovery(task_tf=True)


class NavigationTimeoutTests(unittest.TestCase):
    def transport(self, blocked, finish_after):
        transport = MissionTransport.__new__(MissionTransport)
        started = time.monotonic()
        transport.node = SimpleNamespace(obstacle_waiting=lambda *_: blocked)
        transport.navigation_pose, transport.navigation_sent_ns = None, 1
        transport.result_future = SimpleNamespace(
            done=lambda: time.monotonic()-started >= finish_after,
            result=lambda: SimpleNamespace(status=4))
        transport._spin = lambda duration: time.sleep(duration)
        return transport

    def test_confirmed_wait_can_outlast_navigation_time_budget(self):
        self.transport(True, .3).wait_result(.1)

    def test_unconfirmed_wait_does_not_disable_normal_timeout(self):
        with self.assertRaises(MissionError):
            self.transport(False, 1.).wait_result(.1)


if __name__ == '__main__':
    unittest.main()
