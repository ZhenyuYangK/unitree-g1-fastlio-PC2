import argparse
import sys

from g1_task_manager.waypoint_recorder import (
    WaypointRecordError,
    capture_current_pose,
)
from g1_task_manager.waypoint_store import (
    WaypointConfigError,
    WaypointStore,
)

from .paths import default_waypoints_file


def get_default_waypoints_file():
    return default_waypoints_file()


def build_parser():
    parser = argparse.ArgumentParser(
        prog="g1_waypoint",
        description="G1固定导航点管理工具",
    )

    parser.add_argument(
        "--file",
        default=str(get_default_waypoints_file()),
        help="waypoints.yaml路径",
    )

    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
    )

    subparsers.add_parser(
        "validate",
        help="校验导航点文件",
    )

    subparsers.add_parser(
        "list",
        help="列出全部导航点",
    )

    show_parser = subparsers.add_parser(
        "show",
        help="查看指定导航点",
    )

    show_parser.add_argument(
        "waypoint_id",
        help="导航点ID，例如home或lab_a",
    )

    record_parser = subparsers.add_parser(
        "record",
        help="采集map到nav_base并保存导航点",
    )

    record_parser.add_argument(
        "waypoint_id",
        help="要保存的导航点ID",
    )

    record_parser.add_argument(
        "--name",
        default=None,
        help="导航点中文名称",
    )

    record_parser.add_argument(
        "--overwrite",
        action="store_true",
        help="允许更新已经存在的导航点",
    )

    record_parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="连续采样时长，默认2秒",
    )

    record_parser.add_argument(
        "--min-samples",
        type=int,
        default=10,
        help="最少有效TF样本数，默认10",
    )

    record_parser.add_argument(
        "--max-position-spread",
        type=float,
        default=0.05,
        help="最大位置波动，默认0.05米",
    )

    record_parser.add_argument(
        "--max-yaw-spread",
        type=float,
        default=0.05,
        help="最大朝向波动，默认0.05弧度",
    )

    record_parser.add_argument(
        "--wait-timeout",
        type=float,
        default=10.0,
        help="等待定位数据的超时秒数",
    )

    record_parser.add_argument(
        "--max-tf-age",
        type=float,
        default=0.5,
        help="允许的TF最大年龄",
    )

    record_parser.add_argument(
        "--max-fitness",
        type=float,
        default=2.0,
        help="允许的NDT fitness上限",
    )

    record_parser.add_argument(
        "--max-fitness-age",
        type=float,
        default=5.0,
        help="允许的fitness最大年龄，默认5秒",
    )

    return parser


def print_waypoint(waypoint_id, waypoint):
    pose = waypoint["pose"]

    print(f"id: {waypoint_id}")
    print(f"name: {waypoint['name']}")
    print(f"x: {pose['x']:.6f}")
    print(f"y: {pose['y']:.6f}")
    print(f"yaw: {pose['yaw']:.6f}")


def main(args=None):
    parser = build_parser()
    parsed = parser.parse_args(args=args)

    try:
        store = WaypointStore(parsed.file).load()

        if parsed.command == "validate":
            print("[OK] 导航点文件有效")
            print(f"file: {store.file_path}")
            print(f"map_id: {store.map_id}")
            print(f"frame_id: {store.frame_id}")
            print(f"base_frame: {store.base_frame}")
            print(
                "waypoint_count: "
                f"{len(store.list_waypoints())}"
            )
            return 0

        if parsed.command == "list":
            waypoints = store.list_waypoints()

            print(f"map_id: {store.map_id}")
            print(f"waypoint_count: {len(waypoints)}")

            if not waypoints:
                print(
                    "[INFO] 当前还没有保存导航点"
                )
                return 0

            print(
                f"{'ID':<24}"
                f"{'NAME':<24}"
                f"{'X':>12}"
                f"{'Y':>12}"
                f"{'YAW':>12}"
            )

            for waypoint_id, waypoint in waypoints:
                pose = waypoint["pose"]

                print(
                    f"{waypoint_id:<24}"
                    f"{waypoint['name']:<24}"
                    f"{pose['x']:>12.3f}"
                    f"{pose['y']:>12.3f}"
                    f"{pose['yaw']:>12.3f}"
                )

            return 0

        if parsed.command == "show":
            waypoint = store.get_waypoint(
                parsed.waypoint_id
            )

            print_waypoint(
                parsed.waypoint_id,
                waypoint,
            )
            return 0

        if parsed.command == "record":
            if (
                store.has_waypoint(
                    parsed.waypoint_id
                )
                and not parsed.overwrite
            ):
                raise WaypointConfigError(
                    "导航点已经存在："
                    f"{parsed.waypoint_id}；"
                    "如需更新请添加--overwrite"
                )

            print(
                "[INFO] 等待NDT定位和"
                f"{store.frame_id}→{store.base_frame} TF"
            )

            print(
                "[INFO] 请保持机器人静止，"
                f"连续采样{parsed.duration:.1f}秒"
            )

            result = capture_current_pose(
                target_frame=store.frame_id,
                source_frame=store.base_frame,
                duration_sec=parsed.duration,
                minimum_samples=parsed.min_samples,
                max_position_spread_m=(
                    parsed.max_position_spread
                ),
                max_yaw_spread_rad=(
                    parsed.max_yaw_spread
                ),
                wait_timeout_sec=(
                    parsed.wait_timeout
                ),
                max_tf_age_sec=(
                    parsed.max_tf_age
                ),
                max_fitness=(
                    parsed.max_fitness
                ),
                max_fitness_age_sec=(
                    parsed.max_fitness_age
                ),
            )

            backup = store.record_waypoint(
                waypoint_id=parsed.waypoint_id,
                name=parsed.name,
                x=result["x"],
                y=result["y"],
                yaw=result["yaw"],
                overwrite=parsed.overwrite,
            )

            print("[OK] 导航点保存成功")
            print(
                f"id: {parsed.waypoint_id}"
            )
            print(f"x: {result['x']:.6f}")
            print(f"y: {result['y']:.6f}")
            print(f"yaw: {result['yaw']:.6f}")
            print(
                "sample_count: "
                f"{result['sample_count']}"
            )
            print(
                "position_spread: "
                f"{result['position_spread']:.6f} m"
            )
            print(
                "yaw_spread: "
                f"{result['yaw_spread']:.6f} rad"
            )
            print(
                "localization_fitness: "
                f"{result['fitness']:.6f}"
            )
            print(f"backup: {backup}")
            return 0

        parser.error("未知命令")
        return 2

    except (
        WaypointConfigError,
        WaypointRecordError,
        OSError,
    ) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
