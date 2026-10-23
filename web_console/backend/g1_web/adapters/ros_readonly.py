"""Foxy telemetry adapter. No publishers, motion clients or process launches."""
import json
import math
import threading
import time
from pathlib import Path

from .base import RobotAdapter, empty_mission
from ..control import GatewayError
from ..cloud import MappingPreview
from ..telemetry import NAV2_NODES, stamp_ns, fresh_stamp, planar_pose, path_view, navigation_readiness
from ..paths import TASK_SOURCE
from g1_task_manager.health import localization_status_reason, MAX_STAMP_NS


def localization_view(raw, now_ns, receive_age):
    def fresh(key, limit):
        stamp = raw.get(key)
        return type(stamp) is int and 0 < stamp <= MAX_STAMP_NS and 0 <= (now_ns - stamp) / 1e9 <= limit

    status_stale = not math.isfinite(receive_age) or not 0 <= receive_age <= 1.0 or not fresh("stamp_ns", 1.0)
    stale = status_stale or not fresh("last_success_stamp_ns", 3.0)
    reason = localization_status_reason(raw, receive_age, now_ns)
    return dict(raw, stale=stale, status_stale=status_stale,
                valid=not bool(reason), health_reason=reason)


class RosReadOnlyAdapter(RobotAdapter):
    def __init__(self, settings=None):
        self.settings = settings
        self.lock = threading.RLock()
        self.localization = {"state": "UNKNOWN", "valid": False, "reason": "等待定位数据"}
        self.localization_at = self.mission_at = 0.0
        self.mission = empty_mission(False)
        self.pending = None
        self.started_at = time.time()
        self.mapping_preview = MappingPreview()
        self.path = None
        self.path_at = 0.0
        self.checks = {}
        self.reads = {}
        self.nav_goal_active = False

    async def start(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy as ReliabilityPolicy, QoSDurabilityPolicy as DurabilityPolicy
            from std_msgs.msg import String
            from sensor_msgs.msg import PointCloud2
            from g1_task_interfaces.srv import GetMissionState
            from nav_msgs.msg import Path as NavPath
            from lifecycle_msgs.srv import GetState
            from rcl_interfaces.srv import GetParameters
            from tf2_ros import Buffer, TransformListener, TransformException
            from action_msgs.msg import GoalStatusArray
        except ImportError as exc:
            raise RuntimeError("ROS 模式需要已编译并加载的 Foxy 工作空间及系统 rclpy") from exc
        self.rclpy, self.service_type = rclpy, GetMissionState
        self.transform_error = TransformException
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("g1_web_readonly", context=self.context)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.path_subscription = self.node.create_subscription(NavPath, '/plan', self._path, 1)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.nav_goal_subscription = self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._nav_goals, status_qos)
        self.lifecycle_type, self.parameter_type = GetState, GetParameters
        self.lifecycle_clients = {name: self.node.create_client(GetState, '/' + name + '/get_state') for name in NAV2_NODES}
        self.parameter_names = {
            'motion_bridge': ('g1_cmd_vel_dry_run', ['dry_run', 'real_backend', 'backend_name', 'cmd_vel_topic', 'enable_service_name', 'require_localization', 'allow_lateral', 'allow_reverse', 'max_linear_x', 'max_angular_z']),
            'audio_bridge': ('g1_audio_bridge', ['dry_run']),
            'task_manager': ('g1_task_manager', ['execute_real', 'waypoints_file', 'missions_file']),
        }
        self.parameter_clients = {key: self.node.create_client(GetParameters, '/' + name + '/get_parameters') for key, (name, _) in self.parameter_names.items()}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.node.create_subscription(
            String, "/g1_relocalization/status", self._localization, qos)
        self.client = self.node.create_client(GetMissionState, "/g1_task/get_state")
        self.cloud_subscription = self.node.create_subscription(
            PointCloud2, '/cloud_registered', self.mapping_preview.add,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.node.create_timer(0.5, self._poll)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name="g1-web-ros", daemon=True)
        self.thread.start()

    def _localization(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
            # Reject NaN / Infinity rather than breaking the JSON snapshot stream.
            json.dumps(data, allow_nan=False)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.localization, self.localization_at = data, time.monotonic()

    def _poll(self):
        self._poll_checks()
        if self.pending is not None and not self.pending.done():
            if time.monotonic() - self.pending_at < 2.0:
                return
            self.client.remove_pending_request(self.pending)
            self.pending.cancel()
        if self.client.service_is_ready():
            self.pending_at = time.monotonic()
            self.pending = self.client.call_async(self.service_type.Request())
            self.pending.add_done_callback(self._mission)

    def _path(self, message):
        # Discard oversized input before storing it in the gateway.
        value = path_view(message, self.node.get_clock().now().nanoseconds, 0)
        with self.lock:
            self.path, self.path_at = value, time.monotonic()

    def _nav_goals(self, message):
        with self.lock:
            self.nav_goal_active = any(item.status in (1, 2, 3) for item in message.status_list)

    def _poll_checks(self):
        for key, client in {**self.lifecycle_clients, **self.parameter_clients}.items():
            previous = self.reads.get(key)
            if previous and not previous[0].done():
                if time.monotonic() - previous[1] < 2:
                    continue
                client.remove_pending_request(previous[0])
                previous[0].cancel()
            if not client.service_is_ready():
                continue
            request = self.lifecycle_type.Request() if key in NAV2_NODES else self.parameter_type.Request(names=self.parameter_names[key][1])
            future = client.call_async(request)
            self.reads[key] = (future, time.monotonic())
            future.add_done_callback(lambda done, key=key: self._check_result(key, done))

    def _check_result(self, key, future):
        if future.cancelled() or future.exception() is not None:
            return
        response = future.result()
        if key in NAV2_NODES:
            owners = [name for name, namespace in self.node.get_node_names_and_namespaces() if name == key and namespace == '/']
            value = dict(ready=response.current_state.id == 3 and len(owners) == 1, state=response.current_state.label)
        else:
            names = self.parameter_names[key][1]
            values = {}
            for name, p in zip(names, response.values):
                values[name] = {1: p.bool_value, 2: p.integer_value, 3: p.double_value, 4: p.string_value}.get(p.type)
                if isinstance(values[name], float) and not math.isfinite(values[name]):
                    values[name] = None
            if key == 'motion_bridge':
                ready = (values.get('dry_run') is False and values.get('real_backend') is True
                         and values.get('backend_name') == 'unitree_ros2' and values.get('cmd_vel_topic') == '/cmd_vel_nav'
                         and values.get('enable_service_name') == '/g1_loco_bridge/enable_real_nav'
                         and values.get('require_localization') is True and values.get('allow_lateral') is False
                         and values.get('allow_reverse') is False)
                ready = ready and all(type(values.get(k)) in (float, int) and values[k] > 0 for k in ('max_linear_x', 'max_angular_z'))
                ready = ready and any(name == '/g1_loco_bridge/enable_real_nav' and 'std_srvs/srv/SetBool' in types
                                      for name, types in self.node.get_service_names_and_types())
            elif key == 'audio_bridge':
                ready = values.get('dry_run') is False
            else:
                ready = values.get('execute_real') is True
                if self.settings:
                    from ..workspace import WorkspaceData
                    workspace = WorkspaceData(self.settings)
                    ready = ready and all(isinstance(values.get(k), str) and Path(values[k]).resolve() == path
                                          for k, path in [('waypoints_file', workspace.waypoints), ('missions_file', workspace.missions)])
            owners = [name for name, namespace in self.node.get_node_names_and_namespaces()
                      if name == self.parameter_names[key][0] and namespace == '/']
            if key in ('audio_bridge', 'task_manager'):
                from rclpy.action import get_action_server_names_and_types_by_node
                expected = '/g1_audio/speak' if key == 'audio_bridge' else '/g1_task/execute_mission'
                action_owners = []
                for name, namespace in self.node.get_node_names_and_namespaces():
                    try:
                        if any(action == expected for action, _ in get_action_server_names_and_types_by_node(self.node, name, namespace)):
                            action_owners.append((name, namespace))
                    except RuntimeError:
                        pass
                ready = ready and action_owners == [(self.parameter_names[key][0], '/')]
            value = dict(ready=bool(ready and len(owners) == 1), parameters=values)
        with self.lock:
            self.checks[key] = (value, time.monotonic())

    def _mission(self, future):
        if future.cancelled() or future.exception() is not None:
            return
        response = future.result()
        fields = ("active", "simulated", "run_id", "mission_id", "mission_name", "state",
                  "current_step", "total_steps", "current_waypoint", "next_waypoint", "wait_id", "message")
        with self.lock:
            self.mission.update({key: getattr(response, key) for key in fields})
            self.mission_at = time.monotonic()

    async def snapshot(self):
        now = time.monotonic()
        now_ns = self.node.get_clock().now().nanoseconds
        pose = None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'nav_base', self.rclpy.time.Time())
            if fresh_stamp(stamp_ns(transform.header.stamp), now_ns, 0.5):
                pose = dict(planar_pose(transform.transform.translation, transform.transform.rotation), frame_id='map', stamp_ns=stamp_ns(transform.header.stamp))
        except (self.transform_error, ValueError):
            # A missing transform is normal while starting localization.
            pass
        with self.lock:
            localization = localization_view(self.localization, self.node.get_clock().now().nanoseconds,
                                             now - self.localization_at)
            mission = dict(self.mission, stale=now - self.mission_at > 2.0)
            path = dict(self.path, stale=now-self.path_at > 3 or not fresh_stamp(self.path['stamp_ns'], now_ns, 3)) if self.path and 'stamp_ns' in self.path else dict(available=False, stale=True, points=[])
            checks = {key: dict(value, ready=value['ready'] and now-at < 2, stale=now-at >= 2) for key, (value, at) in self.checks.items()}
        names = self.node.get_node_names_and_namespaces()
        stack_present = any(namespace == '/' and name in ('g1_relocalization', 'g1_relocalization_node', 'map_server', *NAV2_NODES) for name, namespace in names)
        navigation = navigation_readiness(localization, pose, checks)
        navigation['active_nav_goal'] = self.nav_goal_active
        if self.nav_goal_active:
            navigation['ready'] = False
            navigation['blockers'].append('Nav2 已有活动目标')
        return {"simulated": False, "localization": localization, "mission": mission, "pose": pose,
                'stack_present': stack_present,
                'path': path, 'navigation': navigation,
                "mapping_preview": self.mapping_preview.snapshot(),
                "logs": [{"time": self.started_at, "message": "ROS 位姿、路径与建图点云通道已连接；当前为任务只读模式"}],
                "capabilities": {"mission_control": False, "process_control": False,
                                 "waypoint_write": False, "teleop": False, "video": False},
                "map": {"available": False, "simulated": False, "id": None}}

    async def catalog(self):
        try:
            from g1_task_manager.paths import get_default_waypoints_file, default_missions_file
            from g1_task_manager.waypoint_store import WaypointStore
            from g1_task_manager.mission_store import load_missions
            store = WaypointStore(get_default_waypoints_file()).load()
            missions = load_missions(default_missions_file(), store)
            return {"simulated": False, "map_id": store.map_id, "frame_id": store.frame_id,
                    "waypoints": [dict(value, id=key) for key, value in store.list_waypoints()],
                    "missions": [{"id": key, "name": value["name"],
                                  "waypoints": [step["waypoint"] for step in value["steps"]]}
                                 for key, value in missions.items()]}
        except Exception as exc:
            raise GatewayError(503, "CATALOG_UNAVAILABLE", "现场导航点或任务配置不可用，请检查后端配置") from exc

    async def close(self):
        if hasattr(self, "executor"):
            self.executor.shutdown(timeout_sec=2.0)
            self.thread.join(timeout=2.0)
        if hasattr(self, "node"):
            self._close_extra()
            self.tf_listener.unregister()
            self.node.destroy_node()
        if hasattr(self, "context"):
            if self.context.ok():
                self.context.shutdown()

    def _close_extra(self):
        pass
