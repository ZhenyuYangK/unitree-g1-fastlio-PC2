import argparse
import math
import os
import sys
from pathlib import Path

from g1_task_manager.mission_cli import add_parsers, run_command
from g1_task_manager.waypoint_store import (
    WaypointConfigError,
    WaypointStore,
)


DEFAULT_WS = Path(
    "/home/g1test/Desktop/irongroup/yzy/"
    "g1_slam_ws_source/g1_slam_ws"
)


def get_default_waypoints_file():
    explicit_file = os.environ.get(
        "G1_WAYPOINTS_FILE"
    )

    if explicit_file:
        return Path(explicit_file)

    workspace = Path(
        os.environ.get("G1_WS", os.environ.get("WS", str(DEFAULT_WS)))
    )

    return (
        workspace
        / "task_data/maps/laboratory_map_v1/waypoints.yaml"
    )


def build_parser():
    parser = argparse.ArgumentParser(
        prog="g1_taskctl",
        description="G1固定导航点任务控制工具",
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

    goto_parser = subparsers.add_parser(
        "goto",
        help="导航到指定导航点",
    )

    goto_parser.add_argument(
        "waypoint_id",
        help="目标导航点ID",
    )

    goto_parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="导航超时秒数，默认180秒",
    )

    goto_parser.add_argument(
        "--check-only",
        action="store_true",
        help="只检查并显示目标，不发送给Nav2",
    )

    add_parsers(subparsers)
    return parser


def print_goal(
    frame_id,
    waypoint_id,
    waypoint,
):
    pose = waypoint["pose"]
    half_yaw = pose["yaw"] / 2.0

    print(f"id: {waypoint_id}")
    print(f"name: {waypoint['name']}")
    print(f"frame_id: {frame_id}")
    print(f"x: {pose['x']:.6f}")
    print(f"y: {pose['y']:.6f}")
    print(f"yaw: {pose['yaw']:.6f}")
    print("quaternion_x: 0.000000")
    print("quaternion_y: 0.000000")
    print(
        "quaternion_z: "
        f"{math.sin(half_yaw):.6f}"
    )
    print(
        "quaternion_w: "
        f"{math.cos(half_yaw):.6f}"
    )


def main(args=None):
    parser = build_parser()
    parsed = parser.parse_args(args=args)

    try:
        if parsed.command == "goto" and (
            not math.isfinite(parsed.timeout) or parsed.timeout <= 0.0
        ):
            raise WaypointConfigError(
                "导航超时时间必须大于0"
            )

        store = WaypointStore(
            parsed.file
        ).load()

        if parsed.command in ("mission", "missions"):
            return run_command(parsed, store)

        if parsed.command == "goto":
            waypoint = store.get_waypoint(
                parsed.waypoint_id
            )

            print("[INFO] 已读取固定导航点")
            print_goal(
                store.frame_id,
                parsed.waypoint_id,
                waypoint,
            )

            if parsed.check_only:
                print(
                    "[OK] check-only完成，"
                    "没有向Nav2发送目标"
                )
                return 0

            from g1_task_manager.nav2_client import navigate_to_waypoint

            success, status = (
                navigate_to_waypoint(
                    global_frame=store.frame_id,
                    base_frame=store.base_frame,
                    waypoint_id=(
                        parsed.waypoint_id
                    ),
                    waypoint=waypoint,
                    navigation_timeout_sec=(
                        parsed.timeout
                    ),
                )
            )

            if success:
                print(
                    "[OK] 已到达导航点："
                    f"{parsed.waypoint_id}"
                )
                return 0

            print(
                "[ERROR] 导航未成功："
                f"{status}",
                file=sys.stderr,
            )
            return 2

        parser.error("未知命令")
        return 2

    except (
        WaypointConfigError,
        RuntimeError,
        ValueError,
        OSError,
    ) as exc:
        print(
            f"[ERROR] {exc}",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
