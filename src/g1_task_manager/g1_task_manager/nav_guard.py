"""Navigation-only coordinator: bounded detours, stop/wait, and one utterance.

No Unitree motion or enable RPCs. DWB remains the controller; the existing
motion bridge consumes our expiring permit in addition to its manual gate.
"""
import json
import math
import struct
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from dwb_msgs.msg import LocalPlanEvaluation
from geometry_msgs.msg import PoseStamped
from g1_task_interfaces.action import Speak
from g1_task_interfaces.srv import NavigationHold
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from unique_identifier_msgs.msg import UUID

from .action_client import SynchronizedActionClient as ActionClient
from .health import localization_status_reason
from .navigation_recovery import DataHold, RECOVERABLE
from .nav_guard_policy import (TEXT, ObstacleWait, RouteBudget, dynamic_block_on_path,
                               evaluation_state, path_length)


class NavigationFault(RuntimeError):
    pass


class NavigationCanceled(Exception):
    pass


class NavigationPause(Exception):
    pass


class NavigationArrived(Exception):
    pass


TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


class Operation:
    def __init__(self, client, goal_id, started):
        self.client, self.goal_id, self.started = client, goal_id, started
        self.future = None
        self.handle = self.result = None
        self.invalid = False
        self.rejected = False
        self.terminal_status = None
        self.terminal_at = 0.0
        self.cancel_at = -math.inf


class NavigationGuard(Node):
    def __init__(self):
        super().__init__('g1_navigation_guard')
        defaults = dict(extra_distance_m=10.0, speech_delay_sec=1.0, clear_hold_sec=2.0,
                        replan_period_sec=1.0, sensor_timeout_sec=1.0,
                        evaluation_timeout_sec=.5, planner_timeout_sec=3.0,
                        cancel_timeout_sec=2.0, speech_wait_sec=4.0,
                        data_recovery_timeout_sec=3.0, data_recovery_stable_sec=.5)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        if any(not isinstance(v, float) or not math.isfinite(v) or v <= 0 for v in self.cfg.values()):
            raise ValueError('Navigation guard parameters must be positive finite doubles')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.current = None
        self.busy = False
        self.cancel_requested = False
        self.fault_latched = False
        self.mode = 'IDLE'
        self.reason = ''
        self.odom_fault = False
        self.reference_path = None
        self.reference_grid = self.live_grid = None
        self.live_at = self.cloud_at = self.odom_at = self.status_at = 0.0
        self.cloud_stamp = 0
        self.last_cloud_receipt_gap = 0.0
        self.data_hold = DataHold(self.cfg['data_recovery_timeout_sec'],
                                  self.cfg['data_recovery_stable_sec'])
        self.clock_fault = False
        self.previous_ros = None
        self.last_health_diagnostic = {}
        self.localization = None
        self.odom = None
        self.eval = None
        self.eval_at = 0.0
        self.wait = None
        self.budget = None
        self.operations = []
        self.owned = {}
        self.recovery_operations = []
        self.last_fault = ''
        self.last_goal = None
        self.follow = None
        self.last_position = None
        self.ref_index = 0
        self.local_obstacle_at = -math.inf
        self.path_ok = False
        self.feedback_at = 0.0
        self.distance_remaining = 0.0
        self.status_pub = self.create_publisher(String, '/g1_navigation/status', 1)
        self.path_pub = self.create_publisher(Path, '/g1_navigation/path', 1)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        grid_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)
        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, '/g1_navigation/goal', status_qos)
        self.navigation_goal = None
        self.goal_publish_at = -math.inf
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.live_map, grid_qos,
                                 callback_group=self.group)
        self.create_subscription(OccupancyGrid, '/g1_reference/global_costmap/costmap',
                                 self.reference_map, grid_qos, callback_group=self.group)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.cloud,
                                 qos_profile_sensor_data, callback_group=self.group, raw=True)
        self.create_subscription(Odometry, '/Odometry_nav', self.odometry,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(String, '/g1_relocalization/status', self.localization_status,
                                 status_qos, callback_group=self.group)
        self.create_subscription(LocalPlanEvaluation, '/evaluation', self.evaluation, 1,
                                 callback_group=self.group)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose', callback_group=self.group)
        self.reference = ActionClient(self, ComputePathToPose,
                                      '/g1_reference/compute_path_to_pose', callback_group=self.group)
        self.controller = ActionClient(self, FollowPath, '/follow_path', callback_group=self.group)
        self.audio = ActionClient(self, Speak, '/g1_audio/speak', callback_group=self.group)
        self.endpoints = {self.planner: '/compute_path_to_pose',
                          self.reference: '/g1_reference/compute_path_to_pose',
                          self.controller: '/follow_path', self.audio: '/g1_audio/speak'}
        self.cancel_clients = {}
        self.status_subscriptions = []
        for client, endpoint in self.endpoints.items():
            self.cancel_clients[client] = self.create_client(
                CancelGoal, endpoint + '/_action/cancel_goal', callback_group=self.group)
            self.status_subscriptions.append(self.create_subscription(
                GoalStatusArray, endpoint + '/_action/status',
                lambda message, owner=client: self.action_status(owner, message),
                status_qos, callback_group=self.group))
        self.server = ActionServer(self, NavigateToPose, '/g1_navigation/execute', self.execute,
                                   goal_callback=self.accept, cancel_callback=self.cancel,
                                   callback_group=self.group)
        self.hold_service = self.create_service(NavigationHold, '/g1_navigation/data_hold',
                                                self.hold_request, callback_group=self.group)
        self.timer = self.create_timer(.05, self.heartbeat, callback_group=self.group)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

    def live_map(self, message):
        with self.lock:
            self.live_grid, self.live_at = message, time.monotonic()

    def reference_map(self, message):
        with self.lock:
            self.reference_grid = message

    def cloud(self, message):
        # Point data is never inspected here. Avoid constructing Python objects
        # for the full cloud just to read its CDR Header (sec/nanosec).
        if len(message) < 12 or bytes(message[:2]) not in (b'\x00\x01', b'\x00\x00'):
            return
        sec, nsec = struct.unpack_from('<iI' if message[1] & 1 else '>iI', message, 4)
        if sec < 0 or nsec >= 1000000000:
            return
        with self.lock:
            now = time.monotonic()
            self.last_cloud_receipt_gap = now - self.cloud_at if self.cloud_at else 0.0
            self.cloud_at, self.cloud_stamp = now, sec * 1000000000 + nsec

    def odometry(self, message):
        with self.lock:
            now = time.monotonic()
            if message.header.frame_id != 'camera_init' or message.child_frame_id != 'nav_base':
                return
            p = message.pose.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y)):
                return
            position = (p.x, p.y)
            if self.current and self.budget and self.last_position:
                distance = math.hypot(p.x - self.last_position[0], p.y - self.last_position[1])
                # Ignore sub-centimeter stationary jitter; retain the anchor so
                # actual slow motion accumulates rather than being discarded.
                if distance >= .01:
                    elapsed = max(.02, now - self.odom_at)
                    if distance > max(.5, elapsed * 2.0):
                        self.odom_fault = True
                    elif self.mode == 'MOVING':
                        self.budget.advance(distance)
                    self.last_position = position
            else:
                self.last_position = position
            self.odom, self.odom_at = message, now

    def localization_status(self, message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            value = None
        with self.lock:
            self.localization, self.status_at = value, time.monotonic()

    def evaluation(self, message):
        with self.lock:
            self.eval, self.eval_at = message, time.monotonic()
            if (self.follow and self.stamp(message) >= self.follow.started and
                    evaluation_state(message) in ('OBSTACLE', 'STATIONARY_OBSTACLE')):
                self.local_obstacle_at = self.eval_at

    def health(self):
        # All values form one snapshot. Read the ROS clock after message fields,
        # so a concurrent callback cannot manufacture a negative data age.
        with self.lock:
            return self._health_locked()

    def _health_locked(self):
        now = time.monotonic()
        ros = self.get_clock().now().nanoseconds
        if self.previous_ros is not None and ros < self.previous_ros:
            self.clock_fault = True
        self.previous_ros = ros
        self.last_health_diagnostic = dict(
            now_ns=ros, cloud_stamp_ns=self.cloud_stamp,
            cloud_receipt_age_sec=now-self.cloud_at,
            cloud_header_age_sec=(ros-self.cloud_stamp)/1e9,
            cloud_last_gap_sec=self.last_cloud_receipt_gap,
            odom_receipt_age_sec=now-self.odom_at,
            odom_header_age_sec=(ros-self.stamp(self.odom))/1e9 if self.odom else None,
            controller_uuid=bytes(self.follow.goal_id.uuid).hex() if self.follow else '')
        if self.clock_fault or ros <= 0:
            return 'NAVIGATION_CLOCK_INVALID'
        if self.odom_fault:
            return 'ODOMETRY_JUMP'
        reason = localization_status_reason(self.localization, now - self.status_at, ros)
        if reason:
            return reason
        if self.cloud_stamp > ros:
            return 'OBSTACLE_SENSOR_CLOCK_INVALID'
        if (now - self.cloud_at > self.cfg['sensor_timeout_sec'] or
                not 0 <= ros - self.cloud_stamp <= int(self.cfg['sensor_timeout_sec'] * 1e9)):
            return 'OBSTACLE_SENSOR_STALE'
        if self.odom is not None and self.stamp(self.odom) > ros:
            return 'ODOMETRY_CLOCK_INVALID'
        if self.odom is None or now - self.odom_at > .5 or not 0 <= ros - self.stamp(self.odom) <= 500000000:
            return 'ODOMETRY_STALE'
        if (self.live_grid is None or self.reference_grid is None or
                now - self.live_at > 2.5 or
                (self.stamp(self.live_grid) != 0 and
                 not 0 <= ros - self.stamp(self.live_grid) <= 2500000000)):
            # Foxy's Costmap2DPublisher stamps OccupancyGrid with Time(0).
            # Volatile subscription + steady receipt age checks the ongoing
            # publication; nonzero stamps are additionally checked above.
            return 'COSTMAP_NOT_READY_OR_STALE'
        try:
            transform = self.tf.lookup_transform('map', 'nav_base', Time())
            stamp = transform.header.stamp.sec * 1000000000 + transform.header.stamp.nanosec
            self.last_health_diagnostic['tf_stamp_ns'] = stamp
            self.last_health_diagnostic['tf_age_sec'] = (ros-stamp)/1e9
            if stamp > ros:
                return 'NAVIGATION_TF_CLOCK_INVALID'
            if not 0 <= ros - stamp <= 500000000:
                return 'NAVIGATION_TF_STALE'
        except Exception:
            return 'NAVIGATION_TF_UNAVAILABLE'
        return ''

    def begin_data_hold(self, reason):
        with self.lock:
            was_active = self.data_hold.active
            self.data_hold.begin(time.monotonic(), reason)
            self.mode, self.reason = 'DATA_HOLD', self.data_hold.reason
            self.path_ok = False
            if not was_active:
                self.get_logger().warning('NAVIGATION_DATA_HOLD: ' + json.dumps(dict(
                    reason=reason, episode=self.data_hold.episode,
                    diagnostics=self.last_health_diagnostic), ensure_ascii=False))
        self.heartbeat()

    def hold_request(self, request, response):
        with self.lock:
            response.episode = self.data_hold.episode
            if (self.current is None or self.cancel_requested or self.current.is_cancel_requested or
                    request.session != bytes(self.current.goal_id.uuid).hex()):
                response.message = 'Navigation session is no longer active'
                return response
            if request.resume:
                if (request.episode != self.data_hold.episode or
                        not self.data_hold.ready(time.monotonic()) or self.health()):
                    response.message = 'Data hold is not ready for release'
                    return response
                self.data_hold.released = True
            else:
                if request.reason not in ('ACK', 'TASK_TF_STALE', 'TASK_TF_UNAVAILABLE'):
                    response.message = 'Invalid hold reason'
                    return response
                if request.episode not in (0, self.data_hold.episode):
                    response.message = 'Stale data hold episode'
                    return response
                if not self.data_hold.active:
                    if request.reason == 'ACK' or request.episode != 0:
                        response.message = 'No data hold to acknowledge'
                        return response
                    try:
                        self.begin_data_hold(request.reason)
                    except RuntimeError as error:
                        response.message = str(error)
                        return response
                self.data_hold.acknowledged = True
            response.accepted = True
            response.episode = self.data_hold.episode
            response.message = 'Data hold release accepted' if request.resume else 'Closed gate acknowledged'
        return response

    def accept(self, request):
        p, q = request.pose.pose.position, request.pose.pose.orientation
        valid = (request.pose.header.frame_id == 'map' and
                 all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and
                 abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < .01)
        with self.lock:
            if self.busy or self.fault_latched or not valid:
                reason = ('PREVIOUS_GOALS_STOP_UNCONFIRMED' if self.fault_latched else
                          'NAVIGATION_BUSY' if self.busy else 'INVALID_GOAL')
                self.get_logger().warning('NAVIGATION_REJECTED: ' + reason)
                return GoalResponse.REJECT
            self.busy = True
            self.cancel_requested = False
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        del handle
        with self.lock:
            self.cancel_requested = True
            self.mode = 'CANCELING'
        self.heartbeat()
        return CancelResponse.ACCEPT

    def action_status(self, client, message):
        with self.lock:
            for item in message.status_list:
                operation = self.owned.get(bytes(item.goal_info.goal_id.uuid))
                if operation and operation.client == client and item.status in TERMINAL:
                    operation.terminal_status = item.status

    def terminal(self, operation):
        if operation.rejected or operation.terminal_status in TERMINAL:
            if not operation.terminal_at:
                operation.terminal_at = time.monotonic()
            return True
        if operation.result is not None and operation.result.done():
            try:
                status = operation.result.result().status
                if status in TERMINAL:
                    operation.terminal_status = status
                    operation.terminal_at = time.monotonic()
                    return True
            except Exception:
                pass  # A broken result Future is not proof the server stopped.
        return False

    def cancel_owned(self, operation):
        operation.invalid = True
        if self.terminal(operation) or time.monotonic() - operation.cancel_at < .25:
            return
        service = self.cancel_clients[operation.client]
        if not service.service_is_ready():
            return
        operation.cancel_at = time.monotonic()
        request = CancelGoal.Request()
        request.goal_info.goal_id = operation.goal_id
        # Never use cancel-all: the handle may be lost, but the pre-registered
        # UUID still identifies precisely the goal owned by this coordinator.
        future = service.call_async(request)

        def response_ready(response):
            try:
                response.result()
                # Foxy can report a repeated cancel as GOAL_TERMINATED while
                # execute is still canceling. Only result/status 4/5/6 proves
                # termination; UNKNOWN cannot rule out an in-flight send either.
            except Exception as error:
                self.get_logger().warning('OWNED_CANCEL_RESPONSE: ' + str(error))
        future.add_done_callback(response_ready)

    def reconcile(self):
        with self.lock:
            recovering = self.current is None and self.fault_latched
            operations = list(self.recovery_operations) if recovering else []
        for operation in operations:
            self.cancel_owned(operation)
        if recovering and all(self.terminal(operation) for operation in operations):
            with self.lock:
                if self.current is None and self.fault_latched and not self.health():
                    self.fault_latched = False
                    self.recovery_operations = []
                    self.mode, self.reason = 'IDLE', 'PREVIOUS_GOALS_TERMINATED'
                    self.get_logger().info('NAVIGATION_ADMISSION_RESTORED: previous owned goals terminated')
        with self.lock:
            completed = [key for key, operation in self.owned.items()
                         if self.terminal(operation) and operation is not self.follow and
                         ((operation.future is not None and operation.future.done() and
                           (operation.rejected or (operation.result is not None and operation.result.done()))) or
                          (operation.invalid and time.monotonic() - operation.terminal_at > 3.0))]
            for key in completed:
                operation = self.owned.pop(key)
                # Status/terminal-cancel response can confirm termination even
                # when a transport response was lost; release abandoned Futures.
                for future in (operation.future, operation.result):
                    if future is not None and not future.done():
                        operation.client.retire_future(future)

    def checkpoint(self):
        if not rclpy.ok() or self.cancel_requested or (self.current and self.current.is_cancel_requested):
            raise NavigationCanceled()
        reason = self.health()
        if reason:
            if reason not in RECOVERABLE:
                raise NavigationFault(reason)
            self.begin_data_hold(reason)
        if self.data_hold.active:
            raise NavigationPause(self.data_hold.reason)

    def check_controller_result(self):
        """Consume the current result before planning can replace its handle."""
        if self.follow is None or not self.terminal(self.follow):
            return
        status = self.follow.terminal_status
        if status == GoalStatus.STATUS_SUCCEEDED:
            raise NavigationArrived()
        if self.evaluation_now() != 'OBSTACLE':
            raise NavigationFault('CONTROLLER_FAILED_WITHOUT_OBSTACLE:' + str(status))
        self.set_mode('BLOCKED', 'DWB_OBSTACLE')
        self.wait.observe(time.monotonic(), False, True)
        self.stop_follow()

    def recover_data(self):
        """Stop first, then await bounded stable data AND a closed-gate handshake."""
        self.stop_follow()
        pending = [o for o in self.operations if not self.terminal(o)]
        stop_deadline = time.monotonic() + self.cfg['cancel_timeout_sec']
        while any(not self.terminal(o) for o in pending):
            if self.cancel_requested or self.current.is_cancel_requested:
                raise NavigationCanceled()
            for operation in pending:
                if not self.terminal(operation):
                    self.cancel_owned(operation)
            if time.monotonic() >= stop_deadline:
                raise NavigationFault('DATA_HOLD_STOP_UNCONFIRMED')
            time.sleep(.01)
        with self.lock:
            self.data_hold.stopped = True
        while True:
            if not rclpy.ok() or self.cancel_requested or self.current.is_cancel_requested:
                raise NavigationCanceled()
            reason = self.health()
            if reason and reason not in RECOVERABLE:
                raise NavigationFault(reason)
            with self.lock:
                now = time.monotonic()
                if self.data_hold.expired(now):
                    raise NavigationFault('DATA_RECOVERY_TIMEOUT:' + self.data_hold.reason)
                ready = self.data_hold.observe(now, not reason)
                if ready and self.data_hold.released:
                    self.get_logger().info('NAVIGATION_DATA_RECOVERED: episode={}'.format(
                        self.data_hold.episode))
                    self.data_hold.finish()
                    # Do not reset the original reference, traveled distance,
                    # obstacle episode, or total recovery-attempt allowance.
                    self.wait.clear_since = None
                    self.mode, self.reason = 'PLANNING', ''
                    return
            self.set_mode('DATA_READY' if ready else 'DATA_HOLD', self.data_hold.reason)
            time.sleep(.02)

    def set_mode(self, state, reason=''):
        with self.lock:
            if (self.mode, self.reason) != (state, reason):
                self.get_logger().info('{}: {}'.format(state, reason))
            self.mode, self.reason = state, reason
        if state != 'MOVING':
            self.heartbeat()

    def evaluation_now(self):
        if (self.follow is None or self.eval is None or
                self.stamp(self.eval) < self.follow.started or
                time.monotonic() - self.eval_at > self.cfg['evaluation_timeout_sec'] or
                not 0 <= self.get_clock().now().nanoseconds - self.stamp(self.eval) <=
                int(self.cfg['evaluation_timeout_sec'] * 1e9)):
            return 'MISSING'
        state = evaluation_state(self.eval)
        if state == 'STATIONARY_OBSTACLE':
            # Near a goal, a zero command can precede the normal success result.
            # Matches the reviewed SimpleGoalChecker xy_goal_tolerance (0.25 m).
            return 'OBSTACLE' if self.distance_remaining > .25 else 'CLEAR'
        return state

    def heartbeat(self):
        self.reconcile()
        with self.lock:
            active = self.current is not None and not self.cancel_requested and not self.current.is_cancel_requested
            permit = bool(active and self.mode == 'MOVING' and self.path_ok and
                          not self.data_hold.active and
                          self.evaluation_now() == 'CLEAR' and not self.health())
            goal = None
            if self.current:
                pose = self.current.request.pose
                q = pose.pose.orientation
                goal = dict(frame=pose.header.frame_id, x=pose.pose.position.x,
                            y=pose.pose.position.y, qx=q.x, qy=q.y, qz=q.z, qw=q.w)
                if self.navigation_goal is not None and time.monotonic()-self.goal_publish_at >= .5:
                    self.goal_pub.publish(self.navigation_goal)
                    self.goal_publish_at = time.monotonic()
            record = dict(schema='G1_NAVIGATION_WAIT_V1', stamp_ns=self.get_clock().now().nanoseconds,
                          active=active, permit=permit, state=self.mode, reason=self.reason, goal=goal,
                          last_goal=self.last_goal, last_fault=self.last_fault,
                          admission_ready=not self.busy and not self.fault_latched,
                          pending_owned_goals=[dict(action=self.endpoints[o.client],
                                                   uuid=bytes(o.goal_id.uuid).hex())
                                               for o in self.recovery_operations if not self.terminal(o)],
                          data_hold=self.data_hold.active,
                          hold_episode=self.data_hold.episode,
                          hold_ready=self.data_hold.ready(time.monotonic()),
                          hold_remaining_sec=max(0., self.data_hold.timeout -
                                                (time.monotonic()-self.data_hold.started))
                                             if self.data_hold.active else 0.,
                          obstacle_wait=bool(active and self.wait and self.wait.waiting and self.wait.confirmed),
                          session=bytes(self.current.goal_id.uuid).hex() if self.current else '',
                          episode=self.wait.episode if self.wait else 0,
                          reference_m=self.budget.reference if self.budget else None,
                          travelled_m=self.budget.travelled if self.budget else None,
                          extra_limit_m=self.cfg['extra_distance_m'])
        self.status_pub.publish(String(data=json.dumps(record, ensure_ascii=False, allow_nan=False)))

    def send(self, client, goal):
        self.checkpoint()
        if not client.server_is_ready():
            raise NavigationFault('ACTION_SERVER_UNAVAILABLE:' + str(getattr(client, '_action_name', 'unknown')))
        operation = Operation(client, UUID(uuid=list(uuid.uuid4().bytes)),
                              self.get_clock().now().nanoseconds)
        with self.lock:
            self.operations.append(operation)
            self.owned[bytes(operation.goal_id.uuid)] = operation
        operation.future = client.send_goal_async(goal, goal_uuid=operation.goal_id)

        def accepted(future):
            try:
                if future.cancelled():
                    return
                operation.handle = future.result()
                operation.rejected = bool(operation.handle and not operation.handle.accepted)
                if operation.handle and operation.handle.accepted:
                    operation.result = operation.handle.get_result_async()
                    if operation.invalid or self.current is None or self.cancel_requested:
                        self.cancel_owned(operation)
            except Exception as error:
                self.get_logger().error('Action acceptance: ' + str(error))
        operation.future.add_done_callback(accepted)
        return operation

    def wait_accept(self, operation):
        deadline = time.monotonic() + self.cfg['planner_timeout_sec']
        while operation.handle is None:
            self.checkpoint()
            if operation.future.done() and operation.future.exception():
                raise NavigationFault('ACTION_ACCEPTANCE_ERROR')
            if time.monotonic() >= deadline:
                operation.invalid = True
                raise NavigationFault('ACTION_ACCEPTANCE_TIMEOUT:' + self.endpoints[operation.client])
            time.sleep(.01)
        if not operation.handle.accepted:
            raise NavigationFault('ACTION_REJECTED')

    def plan(self, client, pose):
        request = ComputePathToPose.Goal()
        # PC2 Foxy uses pose; support goal on newer interface builds as well.
        if hasattr(request, 'pose'):
            request.pose = pose
        else:
            request.goal = pose
        request.planner_id = 'GridBased'
        operation = self.send(client, request)
        self.wait_accept(operation)
        deadline = time.monotonic() + self.cfg['planner_timeout_sec']
        while operation.result is None or not operation.result.done():
            self.checkpoint()
            if time.monotonic() >= deadline:
                operation.invalid = True
                operation.handle.cancel_goal_async()
                raise NavigationFault('PLANNING_TIMEOUT')
            time.sleep(.01)
        result = operation.result.result()
        if result.status == GoalStatus.STATUS_ABORTED:
            return None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise NavigationFault('PLANNING_FAILED:' + str(result.status))
        path = result.result.path
        if path.header.frame_id != 'map':
            raise NavigationFault('PLANNER_PATH_FRAME_INVALID')
        path_length(path)  # Reject empty and nonfinite paths.
        return path

    def stop_follow(self):
        with self.lock:
            operation, self.follow = self.follow, None
        if operation is None:
            return
        deadline = time.monotonic() + self.cfg['cancel_timeout_sec']
        self.cancel_owned(operation)
        while not self.terminal(operation):
            self.cancel_owned(operation)
            if time.monotonic() >= deadline:
                raise NavigationFault('CONTROLLER_STOP_UNCONFIRMED')
            time.sleep(.01)

    def blocked_evidence(self):
        if time.monotonic() - self.local_obstacle_at <= 2.0:
            return True
        if not self.reference_path or not self.live_grid or not self.reference_grid:
            return False
        transform = self.tf.lookup_transform('map', 'nav_base', Time())
        x, y = transform.transform.translation.x, transform.transform.translation.y
        poses = self.reference_path.poses
        # The reference cursor never moves backwards to obstacles already passed.
        self.ref_index = min(range(self.ref_index, len(poses)),
                             key=lambda i: (poses[i].pose.position.x-x)**2 +
                             (poses[i].pose.position.y-y)**2)
        return dynamic_block_on_path(self.reference_path, self.live_grid,
                                     self.reference_grid, self.ref_index)

    def speak_once(self):
        if not self.wait.speech_due(time.monotonic()) or self.mode != 'BLOCKED':
            return
        # No delayed queue: a missing/busy bridge is logged as an attempted
        # reminder. It can never delay stopping or leak into an arrival speech.
        self.wait.spoken = True
        if not self.audio.server_is_ready():
            self.get_logger().warning('OBSTACLE_SPEECH_UNAVAILABLE')
            return
        operation = self.send(self.audio, Speak.Goal(text=TEXT, speaker_id=0,
                                                   wait_sec=self.cfg['speech_wait_sec']))
        self.get_logger().info('OBSTACLE_SPEECH_REQUEST: ' + TEXT)

        def result_ready(future):
            try:
                handle = future.result()
                if not handle or not handle.accepted:
                    self.get_logger().warning('OBSTACLE_SPEECH_REJECTED')
                    return
                if self.mode != 'BLOCKED' or self.cancel_requested or self.current is None:
                    operation.invalid = True
                    handle.cancel_goal_async()
                def spoken(result):
                    if result.cancelled():
                        return
                    try:
                        self.get_logger().info('OBSTACLE_SPEECH_RESULT: ' + str(result.result().status))
                    except Exception as error:
                        self.get_logger().warning('OBSTACLE_SPEECH_RESULT_ERROR: ' + str(error))
                handle.get_result_async().add_done_callback(spoken)
            except Exception as error:
                self.get_logger().warning('OBSTACLE_SPEECH_ERROR: ' + str(error))
        operation.future.add_done_callback(result_ready)

    def publish_feedback(self):
        if time.monotonic() - self.feedback_at < .2:
            return
        self.feedback_at = time.monotonic()
        message = NavigateToPose.Feedback()
        transform = self.tf.lookup_transform('map', 'nav_base', Time())
        message.current_pose.header = transform.header
        message.current_pose.pose.position.x = transform.transform.translation.x
        message.current_pose.pose.position.y = transform.transform.translation.y
        message.current_pose.pose.orientation = transform.transform.rotation
        message.distance_remaining = float(self.distance_remaining)
        self.current.publish_feedback(message)

    def cleanup(self):
        self.set_mode('STOPPING')
        self.path_ok = False
        self.heartbeat()
        try:
            self.stop_follow()
        except NavigationFault:
            pass
        # Exact owned goals only; never cancel another node's speech/goal.
        for operation in self.operations:
            self.cancel_owned(operation)
        deadline = time.monotonic() + self.cfg['cancel_timeout_sec']
        while time.monotonic() < deadline:
            pending = [o for o in self.operations if not self.terminal(o)]
            if not pending:
                break
            for operation in pending:
                self.cancel_owned(operation)
            time.sleep(.01)
        # Late controller acceptance must block subsequent navigation admission.
        with self.lock:
            self.recovery_operations = [o for o in self.operations
                                        if o.client in (self.controller, self.audio) and not self.terminal(o)]
            complete = not self.recovery_operations
            self.fault_latched = not complete
        if not complete:
            self.get_logger().error('PREVIOUS_GOALS_STOP_UNCONFIRMED: {} owned goals; admission withheld'.format(
                len(self.recovery_operations)))
        return complete

    def execute(self, handle):
        result = NavigateToPose.Result()
        outcome = 'ABORTED'
        with self.lock:
            self.current = handle
            self.operations = []
            self.wait = ObstacleWait(self.cfg['speech_delay_sec'], self.cfg['clear_hold_sec'])
            self.budget = None
            self.reference_path = None
            self.follow = None
            self.ref_index = 0
            self.local_obstacle_at = -math.inf
            self.path_ok = False
            self.reason = ''
            self.odom_fault = False
            self.last_fault = ''
            self.navigation_goal = PoseStamped()
            self.navigation_goal.header.frame_id = handle.request.pose.header.frame_id
            self.navigation_goal.header.stamp = self.get_clock().now().to_msg()
            self.navigation_goal.pose = handle.request.pose.pose
            self.goal_publish_at = -math.inf
            self.data_hold = DataHold(self.cfg['data_recovery_timeout_sec'],
                                      self.cfg['data_recovery_stable_sec'])
            q = handle.request.pose.pose.orientation
            self.last_goal = dict(frame=handle.request.pose.header.frame_id,
                                  x=handle.request.pose.pose.position.x, y=handle.request.pose.pose.position.y,
                                  qx=q.x, qy=q.y, qz=q.z, qw=q.w)
        self.set_mode('PLANNING')
        try:
            while True:
                try:
                    self.navigate_current_goal(handle)
                except NavigationPause:
                    self.recover_data()
        except NavigationArrived:
            outcome = 'SUCCEEDED'
        except NavigationCanceled:
            outcome = 'CANCELED'
        except Exception as error:
            self.last_fault = str(error)
            self.get_logger().error('NAVIGATION_FAULT_SNAPSHOT: ' + json.dumps(
                self.last_health_diagnostic, ensure_ascii=False))
            self.get_logger().error('NAVIGATION_FAULT: {}: {}'.format(type(error).__name__, error))
        finally:
            stopped = self.cleanup()
            with self.lock:
                self.current = None
                self.data_hold.finish()
                self.busy = False
                self.mode = 'IDLE' if stopped else 'STOP_UNCONFIRMED'
            self.heartbeat()
        if not stopped:
            handle.abort()
        elif outcome == 'SUCCEEDED' and not handle.is_cancel_requested:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return result

    def navigate_current_goal(self, handle):
        deadline = time.monotonic() + self.cfg['planner_timeout_sec']
        while self.health():
            if self.cancel_requested or handle.is_cancel_requested:
                raise NavigationCanceled()
            if time.monotonic() >= deadline:
                raise NavigationFault(self.health())
            time.sleep(.02)
        pose = handle.request.pose
        if self.reference_path is None:
            self.reference_path = self.plan(self.reference, pose)
            if self.reference_path is None:
                raise NavigationFault('STATIC_REFERENCE_UNREACHABLE')
            self.budget = RouteBudget(path_length(self.reference_path), self.cfg['extra_distance_m'])
        next_plan = 0.0
        candidate = None
        while True:
            self.checkpoint()
            self.check_controller_result()
            now = time.monotonic()
            # An indefinite wait must not accumulate completed RPCs.
            self.operations[:] = [o for o in self.operations if not self.terminal(o) or o is self.follow]
            if now >= next_plan:
                candidate = self.plan(self.planner, pose)
                self.check_controller_result()
                self.path_ok = candidate is not None and self.budget.allows(path_length(candidate))
                if not self.path_ok:
                    evidence = self.blocked_evidence()
                    if not evidence:
                        # Planner reads its internal grid before the 1 Hz
                        # public costmap reaches us. Stop while that single
                        # publication catches up; never pause task timeouts
                        # for an unconfirmed planner failure.
                        self.set_mode('CHECKING_OBSTACLE')
                        self.stop_follow()
                        deadline = time.monotonic() + 1.5
                        while not evidence and time.monotonic() < deadline:
                            self.checkpoint()
                            time.sleep(.02)
                            evidence = self.blocked_evidence()
                        if not evidence:
                            raise NavigationFault('NO_ALLOWED_ROUTE_WITHOUT_OBSTACLE')
                    self.set_mode('BLOCKED', 'NO_ROUTE' if candidate is None else 'DETOUR_LIMIT')
                    self.wait.observe(time.monotonic(), False, evidence)
                    self.stop_follow()
                else:
                    self.path_pub.publish(candidate)
                    self.distance_remaining = path_length(candidate)
                    # Controller preemption updates the path without resetting
                    # its progress checker. The old exact handle is retained
                    # for cleanup, but only the new result may finish this leg.
                    previous = self.follow
                    self.follow = self.send(self.controller, FollowPath.Goal(path=candidate, controller_id='FollowPath'))
                    if previous and self.mode in ('MOVING', 'PROBING'):
                        self.follow.started = previous.started
                    self.wait_accept(self.follow)
                next_plan = time.monotonic() + self.cfg['replan_period_sec']
            now = time.monotonic()
            if self.follow:
                state = self.evaluation_now()
                if self.terminal(self.follow):
                    status = self.follow.terminal_status
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        raise NavigationArrived()
                    if state != 'OBSTACLE':
                        raise NavigationFault('CONTROLLER_FAILED_WITHOUT_OBSTACLE:' + str(status))
                    state = 'OBSTACLE'
                if state == 'CLEAR':
                    permit = self.wait.observe(now, True)
                    self.set_mode('MOVING' if permit else 'PROBING')
                elif state == 'OBSTACLE':
                    self.set_mode('BLOCKED', 'DWB_OBSTACLE')
                    self.wait.observe(now, False, True)
                    self.stop_follow()
                elif state == 'FAULT':
                    raise NavigationFault('DWB_FAILED_WITHOUT_OBSTACLE')
                else:
                    # A gap in fresh controller evidence breaks the two
                    # continuous clear seconds; it cannot release a wait.
                    self.wait.clear_since = None
                    if self.get_clock().now().nanoseconds - self.follow.started > 1000000000:
                        raise NavigationFault('DWB_EVALUATION_TIMEOUT')
            else:
                self.wait.observe(now, False, self.blocked_evidence())
            self.speak_once()
            self.publish_feedback()
            time.sleep(.02)

    def destroy_node(self):
        # Foxy action entities must release their node handles before Node.
        for entity in (self.server, self.planner, self.reference, self.controller, self.audio):
            entity.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGuard()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_requested = True
        node.mode = 'STOPPING'
        if rclpy.ok():
            node.heartbeat()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
