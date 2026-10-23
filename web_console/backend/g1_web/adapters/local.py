import time

from .base import RobotAdapter, empty_mission


class LocalAdapter(RobotAdapter):
    """File-backed configuration mode, independent of the robot or ROS runtime."""

    def __init__(self):
        self.started_at = time.time()

    async def snapshot(self):
        return {"simulated": False, "pose": None,
                "localization": {"state": "UNKNOWN", "valid": False, "stale": True},
                "mission": dict(empty_mission(False), stale=True),
                "capabilities": {"mission_control": False, "process_control": False,
                                 "waypoint_write": True, "mission_write": True, "teleop": False, "video": False},
                "map": {"available": False, "simulated": False},
                "logs": [{"time": self.started_at, "message": "已连接本地工作空间；机器人实时状态未连接"}]}
