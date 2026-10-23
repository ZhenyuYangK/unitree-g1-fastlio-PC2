"""Foxy NavigateToPose + 现有SetBool安全门；不调用运动模式API。"""
import inspect
import math
import signal
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
from g1_task_interfaces.action import Speak
from g1_task_interfaces.srv import GetMissionState, ContinueMission

from .nav2_client import Nav2WaypointClient, STATUS_NAMES
from .mission_engine import MissionError, MissionCanceled
from .mission_control import MissionControl, MissionProgress
from .audio_readiness import AudioReadinessError, wait_for_audio
from .task_events import safe_print as print


TERMINAL = (4, 5, 6)  # action_msgs: SUCCEEDED / CANCELED / ABORTED


class MissionTransport(MissionProgress):
    def __init__(self, node, gate_service="/g1_loco_bridge/enable_real_nav",
                 max_tf_age_sec=0.5, max_fitness=2.0, max_fitness_age_sec=5.0):
        from .health import validate_limits
        validate_limits(max_tf_age_sec, max_fitness, max_fitness_age_sec)
        self.health_limits = dict(max_tf_age_sec=max_tf_age_sec,
                                  max_fitness=max_fitness,
                                  max_fitness_age_sec=max_fitness_age_sec)
        self.monitoring = False
        self.node = node
        # 在任何开门前检查安装版本支持自定义UUID，以便取消延迟接受的目标。
        if 'goal_uuid' not in inspect.signature(node.action_client.send_goal_async).parameters:
            raise MissionError('当前rclpy缺少goal_uuid参数，请先核对Foxy安装版本')
        self.gate = node.create_client(SetBool, gate_service)
        self.cancel = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.statuses = {}
        self.subscription = node.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._status, qos)
        self.goal_id = None
        self.send_future = None
        self.handle = None
        self.result_future = None
        self.old_handlers = {}
        self.cleanup = False
        self._setup_control()

    def _setup_control(self, simulated=False):
        self.control = MissionControl(simulated=simulated)
        self.state_service = self.node.create_service(
            GetMissionState, '/g1_task/get_state', self._get_state)
        self.continue_service = self.node.create_service(
            ContinueMission, '/g1_task/continue', self._continue)
        self.audio = None if simulated else ActionClient(self.node, Speak, '/g1_audio/speak')
        self.audio_cancel = None if simulated else self.node.create_client(
            CancelGoal, '/g1_audio/speak/_action/cancel_goal')
        self.audio_goal_id = None
        self.audio_send = None
        self.audio_handle = None
        self.audio_result = None
        self.audio_cancel_futures = []

    def _get_state(self, request, response):
        del request
        for key, value in self.control.snapshot().items():
            setattr(response, key, value)
        return response

    def _continue(self, request, response):
        response.accepted, response.message = self.control.continue_wait(
            request.run_id, request.wait_id)
        return response

    def install_signals(self):
        # init之后安装Python处理器，使Ctrl+C先执行关门/取消，再关闭ROS context。
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            self.old_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, self._interrupt)

    def _interrupt(self, signum, frame):
        if not self.cleanup:
            raise MissionCanceled('收到信号{}'.format(signum))

    def begin_cleanup(self):
        if self.cleanup:
            return
        self.cleanup = True
        self.monitoring = False
        # No audio wait/RPC response may delay motion cleanup. A late goal
        # acceptance is canceled again by _audio_accepted while cleanup spins.
        if self.audio_goal_id is not None:
            request = CancelGoal.Request()
            request.goal_info.goal_id = self.audio_goal_id
            try:
                self.audio_cancel_futures.append(self.audio_cancel.call_async(request))
                if self.audio_handle is not None and self.audio_handle.accepted:
                    self.audio_cancel_futures.append(self.audio_handle.cancel_goal_async())
            except Exception as exc:
                self.node.get_logger().warning('语音停止请求未确认：{}'.format(exc))

    def restore_signals(self):
        for sig, previous in self.old_handlers.items():
            signal.signal(sig, previous)

    def _status(self, message):
        self.statuses = {bytes(item.goal_info.goal_id.uuid): item.status
                         for item in message.status_list}

    def _spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise MissionError('ROS context已关闭')
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0, deadline - time.monotonic())))
            if self.monitoring and not self.cleanup:
                self.node.check_health(**self.health_limits)

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise MissionError('ROS请求等待超时（{}秒）'.format(timeout))
            self._spin(min(0.05, max(0, deadline - time.monotonic())))
        return future.result()

    def _service(self, client, request, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise MissionError('服务不可用：{}'.format(client.srv_name))
            self._spin(0.05)
        return self._wait(client.call_async(request), timeout)

    def _gate(self, enabled):
        request = SetBool.Request()
        request.data = enabled
        response = self._service(self.gate, request)
        if response is None or not response.success:
            raise MissionError('安全门{}失败：{}'.format(
                '开启' if enabled else '关闭', getattr(response, 'message', '无响应')))
        print('[OK] GATE {}: {}'.format('OPEN' if enabled else 'CLOSED', response.message), flush=True)

    def close_gate(self):
        self.monitoring = False
        self._gate(False)

    def preflight(self, mission):
        if self.audio is not None and any(step.get('speech') for step in mission['steps']):
            print('[INFO] 出发前检查真实语音桥（不发送播报请求）', flush=True)
            try:
                wait_for_audio(self.node, self.audio, self._spin)
            except AudioReadinessError as error:
                raise MissionError('语音预检失败：{}'.format(error)) from error
            print('[OK] 语音预检通过', flush=True)

    def ready(self):
        self.node.wait_until_ready(**self.health_limits)
        self._spin(0.5)  # 接收transient-local的已有Action状态。
        if any(status in (1, 2, 3) for status in self.statuses.values()):
            raise MissionError('Nav2已有活动目标，请先结束原任务；本任务不会取消其他客户端的目标')

    def prepare_goal(self, step):
        self.node.check_health(**self.health_limits)
        pose = step['goal']['pose']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.node.global_frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose['x'])
        goal.pose.pose.position.y = float(pose['y'])
        goal.pose.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
        self.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.handle = None
        self.result_future = None
        self.send_future = self.node.action_client.send_goal_async(
            goal, feedback_callback=self.node._feedback_callback, goal_uuid=self.goal_id)
        self.handle = self._wait(self.send_future, 10.0)
        if self.handle is None:
            raise MissionError('Nav2目标响应为空')
        if not self.handle.accepted:
            raise MissionError('Nav2拒绝目标')
        self.result_future = self.handle.get_result_async()

    def open_gate(self):
        # 若已在关门期间结束（例如本来就在目标处），无需开启运动输出。
        if self.result_future is not None and self.result_future.done():
            return
        # Recheck after goal acceptance; monitor even while the open RPC is pending.
        self.node.check_health(**self.health_limits)
        self.monitoring = True
        self._gate(True)

    def wait_result(self, timeout):
        result = self._wait(self.result_future, timeout)
        if result is None:
            raise MissionError('Nav2结果为空')
        return STATUS_NAMES.get(result.status, 'UNKNOWN')

    def dwell(self, seconds):
        print('[INFO] 安全门关闭，停留{:.1f}秒'.format(seconds), flush=True)
        self._spin(seconds)

    def _audio_accepted(self, future):
        try:
            self.audio_handle = future.result()
            if self.cleanup and self.audio_handle is not None and self.audio_handle.accepted:
                self.audio_cancel_futures.append(self.audio_handle.cancel_goal_async())
        except Exception as exc:
            self.node.get_logger().warning('语音响应异常：{}'.format(exc))

    def speak(self, content):
        deadline = time.monotonic() + 5.0
        while not self.audio.server_is_ready():
            if time.monotonic() >= deadline:
                raise MissionError('语音 Action /g1_audio/speak 不可用')
            self._spin(0.05)
        self.audio_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.audio_handle = None
        self.audio_result = None
        goal = Speak.Goal(text=content['text'], speaker_id=content['speaker_id'],
                          wait_sec=content['wait_sec'])
        self.audio_send = self.audio.send_goal_async(goal, goal_uuid=self.audio_goal_id)
        self.audio_send.add_done_callback(self._audio_accepted)
        handle = self._wait(self.audio_send, 10.0)
        if handle is None or not handle.accepted:
            raise MissionError('语音请求被拒绝')
        self.audio_handle = handle
        self.audio_result = handle.get_result_async()
        result = self._wait(self.audio_result, content['wait_sec'] + 20.0)
        if result is None or result.status != 4 or not result.result.success:
            raise MissionError('语音失败：{}'.format(
                getattr(getattr(result, 'result', None), 'message', '无有效结果')))
        self.audio_goal_id = None

    def finish_cleanup(self):
        if self.audio_goal_id is None:
            return []
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.audio_send is not None and self.audio_send.done() and self.audio_handle is None:
                self._audio_accepted(self.audio_send)
            pending_send = self.audio_send is not None and not self.audio_send.done()
            pending_cancel = any(not future.done() for future in self.audio_cancel_futures)
            if not pending_send and not pending_cancel:
                # A terminal result or an accepted cancel is enough; never wait
                # for the SDK's PlayStop response or physical playback here.
                if self.audio_handle is None or not self.audio_handle.accepted:
                    return []
                if self.audio_result is not None and self.audio_result.done():
                    return []
                if any(future.result() is not None and future.result().goals_canceling
                       for future in self.audio_cancel_futures):
                    return []
                return ['语音停止请求未被接受；运动收尾已先行完成，需现场确认语音状态']
            self._spin(0.05)
        return ['语音接受/取消响应超时；运动收尾已先行完成，语音停止未确认']

    def destroy(self):
        if self.audio is not None:
            self.audio.destroy()
        self.node.destroy_node()

    def cancel_pending(self):
        if self.goal_id is None:
            return True
        if self.send_future is not None and self.send_future.done() and self.handle is None:
            self.handle = self.send_future.result()
        if self.handle is not None and not self.handle.accepted:
            return True
        if self.result_future is not None and self.result_future.done():
            result = self.result_future.result()
            return result is not None and result.status in TERMINAL
        # 即使send_goal响应还没回来，也只取消自己生成的UUID，不使用取消全部。
        request = CancelGoal.Request()
        request.goal_info.goal_id = self.goal_id
        canceled_after_accept = self.handle is not None
        self._service(self.cancel, request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.handle is None and self.send_future is not None and self.send_future.done():
                self.handle = self.send_future.result()
            if self.handle is not None:
                if not self.handle.accepted:
                    return True
                if not canceled_after_accept:
                    # 首次取消可能早于服务端接受目标，再按同一UUID取消一次。
                    self._service(self.cancel, request)
                    canceled_after_accept = True
                if self.result_future is None:
                    self.result_future = self.handle.get_result_async()
                if self.result_future.done():
                    result = self.result_future.result()
                    return result is not None and result.status in TERMINAL
            if self.statuses.get(bytes(self.goal_id.uuid)) in TERMINAL:
                return True
            self._spin(0.05)
        return False


class SimulatedMissionTransport(MissionTransport):
    """ROS control services with simulated navigation, gate and speech only."""
    def __init__(self, node, time_scale=1.0):
        self.node = node
        self.time_scale = time_scale
        self.old_handlers = {}
        self.cleanup = False
        self.monitoring = False
        self._setup_control(simulated=True)

    def close_gate(self):
        print('[SIM] GATE CLOSED（未调用运动接口）', flush=True)

    def open_gate(self):
        print('[SIM] GATE OPEN（未调用运动接口）', flush=True)

    def ready(self):
        print('[SIM] 模拟定位和 Nav2 检查', flush=True)
        self._spin(0.1 * self.time_scale)

    def prepare_goal(self, step):
        print('[SIM] 导航到 {}'.format(step['waypoint']), flush=True)

    def wait_result(self, timeout):
        self._spin(min(0.2, timeout) * self.time_scale)
        return 'SUCCEEDED'

    def speak(self, content):
        print('[SIM] TTS speaker={} wait={}s: {}'.format(
            content['speaker_id'], content['wait_sec'], content['text']), flush=True)
        self._spin(content['wait_sec'] * self.time_scale)

    def dwell(self, seconds):
        self._spin(seconds * self.time_scale)

    def cancel_pending(self):
        return True


def create_transport(global_frame, base_frame, simulated=False, time_scale=1.0, **options):
    if simulated:
        node = Node('g1_simulated_mission')
        try:
            return SimulatedMissionTransport(node, time_scale=time_scale)
        except BaseException:
            node.destroy_node()
            raise
    node = Nav2WaypointClient(global_frame, base_frame)
    try:
        return MissionTransport(node, **options)
    except BaseException:
        node.destroy_node()
        raise
