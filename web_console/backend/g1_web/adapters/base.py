from ..control import GatewayError


def empty_mission(simulated):
    return dict(active=False, simulated=simulated, run_id="", mission_id="", mission_name="",
                state="IDLE" if simulated else "UNKNOWN", current_step=0, total_steps=0,
                current_waypoint="", next_waypoint="", wait_id="", message="",
                gate_closed=True if simulated else None,
                goals_terminal=True if simulated else None, distance_remaining=None)


class RobotAdapter:
    async def start(self):
        pass

    async def close(self):
        pass

    async def snapshot(self):
        raise NotImplementedError

    async def catalog(self):
        raise NotImplementedError

    async def start_mission(self, mission_id):
        self.unsupported()

    async def continue_mission(self, run_id, wait_id):
        self.unsupported()

    async def cancel_mission(self, run_id):
        self.unsupported()

    async def go_to(self, waypoint_id):
        self.unsupported()

    async def initial_pose(self, x, y, yaw):
        self.unsupported()

    @staticmethod
    def unsupported():
        raise GatewayError(501, "NOT_IMPLEMENTED", "此接口尚未接入真机；当前 ROS 适配器只读")
