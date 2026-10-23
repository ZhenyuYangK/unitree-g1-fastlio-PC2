import asyncio
import copy
import time
import uuid
from collections import deque

from .base import RobotAdapter, empty_mission
from ..control import GatewayError


WAYPOINTS = [
    {"id": "point_01", "name": "接待区", "pose": {"x": 2.0, "y": 2.0, "yaw": 0.0}},
    {"id": "point_02", "name": "研究展示区", "pose": {"x": 6.0, "y": 2.0, "yaw": 1.57}},
    {"id": "point_03", "name": "设备实验区", "pose": {"x": 9.0, "y": 5.0, "yaw": 1.57}},
    {"id": "point_04", "name": "交流区", "pose": {"x": 6.0, "y": 8.0, "yaw": 3.14}},
    {"id": "point_05", "name": "参观终点", "pose": {"x": 2.0, "y": 8.0, "yaw": 3.14}},
]


class MockAdapter(RobotAdapter):
    """Deterministic browser demonstration; never imports ROS or starts a process."""

    def __init__(self, settings):
        self.settings = settings
        self.mission = empty_mission(True)
        self.pose = {"x": 1.0, "y": 2.0, "yaw": 0.0}
        self.logs = deque(maxlen=60)
        self.task = None
        self.route = WAYPOINTS
        self.single = False
        self.release = None
        self.log("模拟环境已就绪，所有坐标和反馈均为演示数据")

    async def start(self):
        self.release = asyncio.Event()

    def log(self, message):
        self.logs.append({"time": time.time(), "message": message})

    async def catalog(self):
        return {"simulated": True, "map_id": "demo_map", "frame_id": "map",
                "waypoints": copy.deepcopy(WAYPOINTS),
                "missions": [{"id": "five_point_tour", "name": "五站双语讲解巡航",
                              "waypoints": [w["id"] for w in WAYPOINTS],
                              "language": "both", "advance_mode": "manual"}]}

    async def snapshot(self):
        return {"simulated": True,
                "localization": {"state": "TRACKING", "valid": True, "stale": False,
                                 "fitness": 0.18, "reason": "模拟定位"},
                "pose": dict(self.pose), "mission": copy.deepcopy(self.mission),
                'path': dict(available=self.mission['state'] == 'NAVIGATING', stale=False, frame_id='map',
                             points=[[self.pose['x'], self.pose['y']]] + [[p['pose']['x'], p['pose']['y']] for p in self.route[max(0, self.mission['current_step']-1):self.mission['current_step']]]),
                'navigation': dict(ready=True, blockers=[], checks={key: dict(ready=True, simulated=True) for key in ('controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'motion_bridge', 'audio_bridge', 'task_manager')}),
                "logs": list(self.logs),
                "capabilities": {"mission_control": True, "process_control": False,
                                 "waypoint_write": False, "single_goal": True, "teleop": False, "video": False},
                "map": {"available": True, "simulated": True, "id": "demo_map"}}

    async def start_mission(self, mission_id):
        if self.mission["active"]:
            raise GatewayError(409, "MISSION_BUSY", "当前已有运行中的任务")
        if mission_id != "five_point_tour":
            raise GatewayError(404, "MISSION_NOT_FOUND", "任务不存在")
        self.route, self.single = WAYPOINTS, False
        return self._begin(mission_id, '五站双语讲解巡航')

    async def go_to(self, waypoint_id):
        if self.mission['active']:
            raise GatewayError(409, 'MISSION_BUSY', '已有导航任务运行中')
        point = next((p for p in WAYPOINTS if p['id'] == waypoint_id), None)
        if point is None:
            raise GatewayError(404, 'WAYPOINT_NOT_FOUND', '导航点不存在')
        self.route, self.single = [point], True
        return self._begin('waypoint:' + waypoint_id, '前往' + point['name'])

    def _begin(self, mission_id, name):
        self.mission = dict(empty_mission(True), active=True, run_id=uuid.uuid4().hex,
                            mission_id=mission_id, mission_name=name,
                            state="STARTING", total_steps=len(self.route), next_waypoint=self.route[0]['id'])
        self.task = asyncio.create_task(self._run(), name="mock-mission")
        run_id = self.mission["run_id"]
        self.task.add_done_callback(lambda task: self._finished(task, run_id))
        return {"run_id": self.mission["run_id"], "message": "模拟任务已接受"}

    def _finished(self, task, run_id):
        # Also handles cancellation before _run has entered its try block.
        if self.mission["run_id"] != run_id:
            return
        if task.cancelled() and self.mission["active"]:
            self.mission.update(active=False, state="CANCELED", wait_id="",
                                message="模拟取消已完成", gate_closed=True, goals_terminal=True)
        elif not task.cancelled() and task.exception() is not None:
            self.mission.update(active=False, state="FAILED", wait_id="",
                                message="模拟任务异常结束", gate_closed=True, goals_terminal=True)

    def phase(self, state, index, message):
        self.mission.update(state=state, current_step=index,
                            current_waypoint=self.route[index - 1]["id"],
                            next_waypoint=self.route[index]["id"] if index < len(self.route) else "",
                            wait_id="", message=message,
                            gate_closed=state != "NAVIGATING", goals_terminal=state != "NAVIGATING")
        self.log(message)

    async def _run(self):
        try:
            for index, waypoint in enumerate(self.route, 1):
                self.phase("NAVIGATING", index, "正在前往" + waypoint["name"])
                origin = dict(self.pose)
                ticks = max(1, int(self.settings.mock_travel_seconds / 0.1))
                for tick in range(1, ticks + 1):
                    await asyncio.sleep(self.settings.mock_travel_seconds / ticks)
                    self.pose = {k: origin[k] + (waypoint["pose"][k] - origin[k]) * tick / ticks
                                 for k in origin}
                if self.single:
                    continue
                for language, text in (("ZH", "中文讲解"), ("EN", "英文讲解")):
                    self.phase("SPEAKING_" + language, index, waypoint["name"] + " · " + text)
                    await asyncio.sleep(self.settings.mock_speech_seconds)
                self.release.clear()
                self.phase("WAITING_FOR_CONTINUE", index,
                           "最后一站，请放行以完成任务" if index == len(WAYPOINTS) else "讲解结束，等待负责人放行")
                self.mission["wait_id"] = uuid.uuid4().hex
                await self.release.wait()
            self.mission.update(active=False, state="SUCCEEDED", message="模拟巡航已完成",
                                wait_id="", gate_closed=True, goals_terminal=True)
            self.log("模拟巡航已完成")
        except asyncio.CancelledError:
            self.mission.update(active=False, state="CANCELED", wait_id="",
                                message="模拟取消已完成", gate_closed=True, goals_terminal=True)
            self.log("模拟取消已完成")
            raise

    async def continue_mission(self, run_id, wait_id):
        if (not self.mission["active"] or self.mission["state"] != "WAITING_FOR_CONTINUE"
                or run_id != self.mission["run_id"] or wait_id != self.mission["wait_id"]):
            raise GatewayError(409, "STALE_WAIT", "任务或等待标识已过期，请重新查询状态")
        self.mission.update(state="CONTINUING", wait_id="", message="放行请求已接受")
        self.release.set()
        return {"run_id": run_id, "message": "放行请求已接受，请查看后续状态"}

    async def cancel_mission(self, run_id):
        if not self.mission["active"] or run_id != self.mission["run_id"]:
            raise GatewayError(409, "STALE_RUN", "运行已结束或标识已过期")
        self.mission.update(state="CANCELING", wait_id="", message="取消请求已接受")
        self.task.cancel()
        return {"run_id": run_id, "message": "取消请求已接受，请等待终态确认"}

    async def close(self):
        if self.task and not self.task.done():
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
            self._finished(self.task, self.mission["run_id"])
