import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer
from tf2_ros import TransformListener

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_py import TransformException


class Nav2NavigationError(RuntimeError):
    pass


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


def angular_distance(first, second):
    return abs(
        math.atan2(
            math.sin(first - second),
            math.cos(first - second),
        )
    )


class Nav2WaypointClient(Node):
    def __init__(
        self,
        global_frame,
        base_frame,
    ):
        super().__init__("g1_named_waypoint_client")

        self.global_frame = global_frame
        self.base_frame = base_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.latest_fitness = None
        self.latest_fitness_time = None
        self.localization_status = None
        self.localization_status_time = None
        self.status_subscription = self.create_subscription(
            String, '/g1_relocalization/status', self._status_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.fitness_subscription = (
            self.create_subscription(
                Float64,
                "/localization_fitness",
                self._fitness_callback,
                10,
            )
        )

        self.active_goal_handle = None
        self.last_feedback_time = 0.0

    def _fitness_callback(self, message):
        self.latest_fitness = float(message.data)
        self.latest_fitness_time = time.monotonic()

    def _status_callback(self, message):
        from .health import parse_localization_status
        self.localization_status = parse_localization_status(message.data)
        self.localization_status_time = time.monotonic()

    def wait_until_ready(
        self,
        server_timeout_sec=10.0,
        localization_timeout_sec=20.0,
        max_tf_age_sec=0.5,
        max_fitness=2.0,
        max_fitness_age_sec=5.0,
    ):
        self.get_logger().info(
            "等待Nav2 /navigate_to_pose Action服务器"
        )

        # 等待Action服务器时，同时处理TF和fitness回调。
        server_deadline = time.monotonic() + server_timeout_sec
        while not self.action_client.server_is_ready():
            if not rclpy.ok():
                raise Nav2NavigationError("ROS已关闭")

            remaining = server_deadline - time.monotonic()
            if remaining <= 0.0:
                raise Nav2NavigationError(
                    "Nav2 /navigate_to_pose Action服务器不可用"
                )

            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, remaining),
            )

        self.get_logger().info(
            "Action服务器已就绪，开始检查定位和TF"
        )

        deadline = (
            time.monotonic()
            + localization_timeout_sec
        )

        last_reason = "等待定位数据"

        while time.monotonic() < deadline:
            # 每轮先连续处理一小批回调，再检查最新位姿。
            pump_deadline = min(
                deadline,
                time.monotonic() + 0.05,
            )
            while rclpy.ok():
                remaining = pump_deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                rclpy.spin_once(
                    self,
                    timeout_sec=min(0.01, remaining),
                )

            if not rclpy.ok():
                raise Nav2NavigationError("ROS已关闭")

            try:
                transform = self.check_health(max_tf_age_sec, max_fitness,
                                              max_fitness_age_sec)
            except Nav2NavigationError as exc:
                last_reason = str(exc)
                continue
            self.get_logger().info("Nav2和定位检查通过")
            return transform

        raise Nav2NavigationError(
            "导航前检查超时："
            + last_reason
        )

    def check_health(self, max_tf_age_sec=0.5, max_fitness=2.0,
                     max_fitness_age_sec=5.0):
        """Non-blocking snapshot; TF lookup deliberately has no wait timeout."""
        check_started = time.monotonic()
        check_gap = check_started - getattr(
            self, "_previous_health_check", check_started)
        self._previous_health_check = check_started
        from .health import health_reason, localization_status_reason
        status_age = (None if self.localization_status_time is None else
                      time.monotonic() - self.localization_status_time)
        reason = localization_status_reason(self.localization_status, status_age,
                                           self.get_clock().now().nanoseconds,
                                           max_fitness_age_sec)
        if reason:
            raise Nav2NavigationError(reason)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
        except TransformException as exc:
            raise Nav2NavigationError('无法获取{}→{}：{}'.format(
                self.global_frame, self.base_frame, exc)) from exc
        stamp = transform.header.stamp
        age = (None if self.latest_fitness_time is None else
               time.monotonic() - self.latest_fitness_time)
        reason = health_reason(self.latest_fitness, age,
                               int(stamp.sec) * 1000000000 + int(stamp.nanosec),
                               self.get_clock().now().nanoseconds,
                               max_tf_age_sec, max_fitness, max_fitness_age_sec)
        if reason:
            if reason.startswith("TF"):
                diagnostic_now = self.get_clock().now().nanoseconds
                details = [
                    "check_gap={:.3f}s".format(check_gap),
                    "now_ns={}".format(diagnostic_now),
                    "chain_stamp_ns={}".format(
                        int(stamp.sec) * 1000000000 + int(stamp.nanosec)),
                ]
                for parent, child in (
                    ("map", "camera_init"),
                    ("camera_init", "body"),
                ):
                    try:
                        edge = self.tf_buffer.lookup_transform(
                            parent, child, Time())
                        edge_stamp = edge.header.stamp
                        edge_ns = (
                            int(edge_stamp.sec) * 1000000000
                            + int(edge_stamp.nanosec))
                        details.append(
                            "{}->{} age={:.3f}s".format(
                                parent, child,
                                (diagnostic_now - edge_ns) / 1e9))
                    except TransformException as exc:
                        details.append(
                            "{}->{} unavailable: {}".format(
                                parent, child, exc))
                reason += " [TF_DIAG] " + "; ".join(details)
            raise Nav2NavigationError(reason)
        return transform

    def _feedback_callback(self, feedback_message):
        now = time.monotonic()

        if now - self.last_feedback_time < 1.0:
            return

        self.last_feedback_time = now
        feedback = feedback_message.feedback

        distance = getattr(
            feedback,
            "distance_remaining",
            float("nan"),
        )

        from .task_events import emit_event
        emit_event('feedback', distance_remaining=(float(distance)
                   if math.isfinite(distance) else -1.0))

        recoveries = getattr(
            feedback,
            "number_of_recoveries",
            0,
        )

        self.get_logger().info(
            "导航反馈："
            f"distance_remaining={distance:.3f}m, "
            f"recoveries={recoveries}"
        )

    def cancel_active_goal(self):
        if self.active_goal_handle is None:
            return

        self.get_logger().warning(
            "正在取消当前Nav2目标"
        )

        cancel_future = (
            self.active_goal_handle
            .cancel_goal_async()
        )

        rclpy.spin_until_future_complete(
            self,
            cancel_future,
            timeout_sec=3.0,
        )

        self.active_goal_handle = None

    def navigate(
        self,
        waypoint_id,
        waypoint,
        current_transform,
        navigation_timeout_sec,
    ):
        pose = waypoint["pose"]

        current_translation = (
            current_transform.transform.translation
        )

        current_rotation = (
            current_transform.transform.rotation
        )

        current_yaw = yaw_from_quaternion(
            current_rotation
        )

        initial_distance = math.hypot(
            pose["x"] - current_translation.x,
            pose["y"] - current_translation.y,
        )

        initial_yaw_error = angular_distance(
            pose["yaw"],
            current_yaw,
        )

        self.get_logger().info(
            f"目标导航点：{waypoint_id}, "
            f"x={pose['x']:.3f}, "
            f"y={pose['y']:.3f}, "
            f"yaw={pose['yaw']:.3f}"
        )

        self.get_logger().info(
            "当前与目标的初始差值："
            f"distance={initial_distance:.3f}m, "
            f"yaw_error={initial_yaw_error:.3f}rad"
        )

        goal = NavigateToPose.Goal()

        self.check_health()

        goal.pose.header.frame_id = (
            self.global_frame
        )

        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = float(
            pose["x"]
        )

        goal.pose.pose.position.y = float(
            pose["y"]
        )

        goal.pose.pose.position.z = 0.0

        half_yaw = float(pose["yaw"]) / 2.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(
            half_yaw
        )
        goal.pose.pose.orientation.w = math.cos(
            half_yaw
        )

        send_future = (
            self.action_client.send_goal_async(
                goal,
                feedback_callback=(
                    self._feedback_callback
                ),
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            raise Nav2NavigationError(
                "发送Nav2目标超时"
            )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            return False, "REJECTED"

        self.active_goal_handle = goal_handle

        self.get_logger().info(
            "Nav2已经接受目标"
        )

        result_future = (
            goal_handle.get_result_async()
        )

        start_time = time.monotonic()

        while (
            rclpy.ok()
            and not result_future.done()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            try:
                self.check_health()
            except Nav2NavigationError:
                self.cancel_active_goal()
                raise

            elapsed = (
                time.monotonic() - start_time
            )

            if elapsed > navigation_timeout_sec:
                self.get_logger().error(
                    "导航任务超时"
                )
                self.cancel_active_goal()
                return False, "TIMEOUT"

        if not result_future.done():
            self.cancel_active_goal()
            return False, "INTERRUPTED"

        wrapped_result = result_future.result()
        status = wrapped_result.status

        self.active_goal_handle = None

        status_name = STATUS_NAMES.get(
            status,
            f"STATUS_{status}",
        )

        return (
            status == GoalStatus.STATUS_SUCCEEDED,
            status_name,
        )

    def destroy_node(self):
        """先释放 Action 客户端，再释放所属节点。"""
        client = getattr(self, 'action_client', None)
        if client is not None:
            client.destroy()
        return super().destroy_node()



def navigate_to_waypoint(
    global_frame,
    base_frame,
    waypoint_id,
    waypoint,
    navigation_timeout_sec=180.0,
):
    node = None
    rclpy.init(args=[])

    try:
        node = Nav2WaypointClient(
            global_frame,
            base_frame,
        )

        current_transform = (
            node.wait_until_ready()
        )

        return node.navigate(
            waypoint_id,
            waypoint,
            current_transform,
            navigation_timeout_sec,
        )

    except KeyboardInterrupt:
        if node is not None:
            node.cancel_active_goal()

        return False, "CANCELED_BY_USER"

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
