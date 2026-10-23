import os
from dataclasses import dataclass, field
from pathlib import Path
from .paths import workspace


@dataclass(frozen=True)
class Settings:
    mode: str = "local"
    host: str = "127.0.0.1"
    port: int = 8080
    token: str = field(default="", repr=False)
    cookie_secure: bool = False
    lease_seconds: float = 300.0
    mock_travel_seconds: float = 2.0
    mock_speech_seconds: float = 0.8
    workspace: str = field(default_factory=lambda: str(workspace()))
    waypoints_file: str = ""
    missions_file: str = ""
    map_yaml: str = ""
    map_pgm: str = ""

    def __post_init__(self):
        if self.mode not in {"local", "mock", "ros_readonly", "ros_control"}:
            raise ValueError("G1_WEB_MODE 只支持 local / mock / ros_readonly / ros_control")
        if not 1 <= self.port <= 65535:
            raise ValueError("端口必须在 1..65535 之间")
        if self.lease_seconds <= 0 or self.mock_travel_seconds <= 0 or self.mock_speech_seconds <= 0:
            raise ValueError("租约和模拟阶段时长必须为正数")
        if (self.mode in {"ros_readonly", "ros_control"} or self.host not in {"127.0.0.1", "localhost", "::1"}) and not self.token:
            raise ValueError("局域网访问及 ROS 只读模式需要设置 G1_WEB_TOKEN")

    @classmethod
    def from_env(cls):
        return cls(
            mode=os.getenv("G1_WEB_MODE", "local"),
            host=os.getenv("G1_WEB_HOST", "127.0.0.1"),
            port=int(os.getenv("G1_WEB_PORT", "8080")),
            token=os.getenv("G1_WEB_TOKEN", ""),
            cookie_secure=os.getenv("G1_WEB_COOKIE_SECURE", "0") == "1",
            workspace=str(workspace()),
            waypoints_file=os.getenv("G1_WAYPOINTS_FILE", ""),
            missions_file=os.getenv("G1_MISSIONS_FILE", ""),
            map_yaml=os.getenv("G1_MAP_YAML", ""),
            map_pgm=os.getenv("G1_MAP_PGM", ""),
        )
