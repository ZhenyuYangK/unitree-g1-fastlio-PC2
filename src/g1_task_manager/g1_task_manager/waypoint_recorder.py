import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer
from tf2_ros import TransformListener

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_py import TransformException


class WaypointRecordError(RuntimeError):
    pass


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


def _angular_distance(first, second):
    return abs(
        math.atan2(
            math.sin(first - second),
            math.cos(first - second),
        )
    )


class WaypointRecorderNode(Node):
    def __init__(self, target_frame, source_frame):
        super().__init__("g1_waypoint_recorder")

        self.target_frame = target_frame
        self.source_frame = source_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.latest_fitness = None
        self.latest_fitness_time = None

        self.fitness_subscription = (
            self.create_subscription(
                Float64,
                "/localization_fitness",
                self._fitness_callback,
                10,
            )
        )

    def _fitness_callback(self, message):
        self.latest_fitness = float(message.data)
        self.latest_fitness_time = time.monotonic()

    def _check_fitness(
        self,
        max_fitness,
        max_fitness_age_sec,
    ):
        if (
            self.latest_fitness is None
            or self.latest_fitness_time is None
        ):
            raise WaypointRecordError(
                "尚未收到/localization_fitness"
            )

        age = (
            time.monotonic()
            - self.latest_fitness_time
        )

        if age > max_fitness_age_sec:
            raise WaypointRecordError(
                "定位质量数据已过期："
                f"age={age:.3f}s"
            )

        if not math.isfinite(self.latest_fitness):
            raise WaypointRecordError(
                "定位fitness不是有限数字"
            )

        if self.latest_fitness > max_fitness:
            raise WaypointRecordError(
                "定位质量不满足录入条件："
                f"fitness={self.latest_fitness:.6f}，"
                f"阈值={max_fitness:.6f}"
            )

    def _lookup_latest_transform(
        self,
        max_tf_age_sec,
    ):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.source_frame,
            Time(),
        )

        stamp = transform.header.stamp

        stamp_nanoseconds = (
            int(stamp.sec) * 1_000_000_000
            + int(stamp.nanosec)
        )

        if stamp_nanoseconds <= 0:
            raise WaypointRecordError(
                "map到nav_base的TF时间戳无效"
            )

        now_nanoseconds = (
            self.get_clock().now().nanoseconds
        )

        age = (
            now_nanoseconds - stamp_nanoseconds
        ) / 1_000_000_000.0

        if age < -0.1:
            raise WaypointRecordError(
                f"TF时间戳位于未来：age={age:.3f}s"
            )

        if age > max_tf_age_sec:
            raise WaypointRecordError(
                f"TF数据已过期：age={age:.3f}s"
            )

        stamp_key = (
            int(stamp.sec),
            int(stamp.nanosec),
        )

        return transform, stamp_key

    def wait_until_ready(
        self,
        wait_timeout_sec,
        max_tf_age_sec,
        max_fitness,
        max_fitness_age_sec,
    ):
        deadline = (
            time.monotonic() + wait_timeout_sec
        )
        last_error = "等待ROS 2数据"

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            try:
                self._check_fitness(
                    max_fitness,
                    max_fitness_age_sec,
                )

                self._lookup_latest_transform(
                    max_tf_age_sec
                )

                return
            except (
                WaypointRecordError,
                TransformException,
            ) as exc:
                last_error = str(exc)

        raise WaypointRecordError(
            "等待定位数据超时："
            + last_error
        )

    def capture(
        self,
        duration_sec,
        minimum_samples,
        max_position_spread_m,
        max_yaw_spread_rad,
        max_tf_age_sec,
        max_fitness,
        max_fitness_age_sec,
    ):
        samples = []
        seen_stamps = set()
        last_tf_error = ""

        deadline = time.monotonic() + duration_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            self._check_fitness(
                max_fitness,
                max_fitness_age_sec,
            )

            try:
                transform, stamp_key = (
                    self._lookup_latest_transform(
                        max_tf_age_sec
                    )
                )
            except (
                WaypointRecordError,
                TransformException,
            ) as exc:
                last_tf_error = str(exc)
                continue

            if stamp_key in seen_stamps:
                continue

            seen_stamps.add(stamp_key)

            translation = (
                transform.transform.translation
            )
            rotation = transform.transform.rotation

            samples.append(
                (
                    float(translation.x),
                    float(translation.y),
                    _yaw_from_quaternion(rotation),
                )
            )

        if len(samples) < minimum_samples:
            detail = (
                f"，最后的TF错误：{last_tf_error}"
                if last_tf_error
                else ""
            )

            raise WaypointRecordError(
                "有效TF样本不足："
                f"{len(samples)}/{minimum_samples}"
                + detail
            )

        x_values = [item[0] for item in samples]
        y_values = [item[1] for item in samples]
        yaw_values = [item[2] for item in samples]

        mean_x = sum(x_values) / len(x_values)
        mean_y = sum(y_values) / len(y_values)

        sin_sum = sum(
            math.sin(value)
            for value in yaw_values
        )
        cos_sum = sum(
            math.cos(value)
            for value in yaw_values
        )

        if math.hypot(sin_sum, cos_sum) < 1e-9:
            raise WaypointRecordError(
                "无法计算稳定的平均yaw"
            )

        mean_yaw = math.atan2(
            sin_sum,
            cos_sum,
        )

        position_spread = math.hypot(
            max(x_values) - min(x_values),
            max(y_values) - min(y_values),
        )

        yaw_spread = 0.0

        for index, first in enumerate(yaw_values):
            for second in yaw_values[index + 1:]:
                yaw_spread = max(
                    yaw_spread,
                    _angular_distance(first, second),
                )

        if position_spread > max_position_spread_m:
            raise WaypointRecordError(
                "机器人位置或定位结果不稳定："
                f"position_spread={position_spread:.6f}m，"
                f"阈值={max_position_spread_m:.6f}m"
            )

        if yaw_spread > max_yaw_spread_rad:
            raise WaypointRecordError(
                "机器人朝向或定位结果不稳定："
                f"yaw_spread={yaw_spread:.6f}rad，"
                f"阈值={max_yaw_spread_rad:.6f}rad"
            )

        self._check_fitness(
            max_fitness,
            max_fitness_age_sec,
        )

        return {
            "x": mean_x,
            "y": mean_y,
            "yaw": mean_yaw,
            "sample_count": len(samples),
            "position_spread": position_spread,
            "yaw_spread": yaw_spread,
            "fitness": self.latest_fitness,
        }


def capture_current_pose(
    target_frame,
    source_frame,
    duration_sec=2.0,
    minimum_samples=10,
    max_position_spread_m=0.05,
    max_yaw_spread_rad=0.05,
    wait_timeout_sec=10.0,
    max_tf_age_sec=0.5,
    max_fitness=2.0,
    max_fitness_age_sec=5.0,
):
    if duration_sec <= 0.0:
        raise WaypointRecordError(
            "采样时长必须大于0"
        )

    if minimum_samples < 2:
        raise WaypointRecordError(
            "最少样本数不能小于2"
        )

    if max_position_spread_m <= 0.0:
        raise WaypointRecordError(
            "位置稳定阈值必须大于0"
        )

    if max_yaw_spread_rad <= 0.0:
        raise WaypointRecordError(
            "角度稳定阈值必须大于0"
        )

    if wait_timeout_sec <= 0.0:
        raise WaypointRecordError(
            "等待超时必须大于0"
        )

    if max_tf_age_sec <= 0.0:
        raise WaypointRecordError(
            "TF最大年龄必须大于0"
        )

    if max_fitness <= 0.0:
        raise WaypointRecordError(
            "fitness阈值必须大于0"
        )

    if max_fitness_age_sec <= 0.0:
        raise WaypointRecordError(
            "fitness最大年龄必须大于0"
        )

    node = None
    rclpy.init(args=[])

    try:
        node = WaypointRecorderNode(
            target_frame,
            source_frame,
        )

        node.wait_until_ready(
            wait_timeout_sec,
            max_tf_age_sec,
            max_fitness,
            max_fitness_age_sec,
        )

        return node.capture(
            duration_sec,
            minimum_samples,
            max_position_spread_m,
            max_yaw_spread_rad,
            max_tf_age_sec,
            max_fitness,
            max_fitness_age_sec,
        )
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
