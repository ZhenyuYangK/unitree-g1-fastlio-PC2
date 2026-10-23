from ..compat import to_thread
"""Explicit high-level control; the existing robot-side executor owns all motion."""
import asyncio
import math
from pathlib import Path
import time
import uuid

from .base import empty_mission
from .ros_readonly import RosReadOnlyAdapter
from ..control import GatewayError
from ..mapping import read_json, write_json


TERMINAL = {'SUCCEEDED', 'CANCELED', 'FAILED', 'ABORTED', 'CHECKED', 'STOP_UNCONFIRMED'}


async def ros_result(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            # Do not cancel transport futures: their late result still matters.
            raise GatewayError(503, 'ROS_TIMEOUT', 'ROS 请求结果未确认，请查看当前状态，不要重复操作')
        await asyncio.sleep(0.02)
    if future.cancelled() or future.exception() is not None:
        raise GatewayError(503, 'ROS_UNAVAILABLE', 'ROS 请求失败，结果未确认')
    return future.result()


class RosControlAdapter(RosReadOnlyAdapter):
    def __init__(self, settings):
        super().__init__(settings)
        self.record = Path(settings.workspace) / 'runtime/navigation/web/run.json'
        self.run = read_json(self.record)
        self.handle = None
        self.recovery_future = None
        self.last_recovery = 0

    async def start(self):
        await super().start()
        from rclpy.action import ActionClient
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from g1_task_interfaces.action import ExecuteMission
        from g1_task_interfaces.srv import ContinueMission
        from std_srvs.srv import SetBool
        self.action_type = ExecuteMission
        self.pose_type = PoseWithCovarianceStamped
        self.action = ActionClient(self.node, ExecuteMission, '/g1_task/execute_mission')
        self.continue_client = self.node.create_client(ContinueMission, '/g1_task/continue')
        self.cancel_type, self.continue_type = CancelGoal, ContinueMission
        self.cancel_client = self.node.create_client(CancelGoal, '/g1_task/execute_mission/_action/cancel_goal')
        self.result_client = self.node.create_client(ExecuteMission.Impl.GetResultService, '/g1_task/execute_mission/_action/get_result')
        self.gate_type = SetBool
        self.gate_client = self.node.create_client(SetBool, '/g1_loco_bridge/enable_real_nav')
        self.initial_pose_publisher = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    def _save(self):
        write_json(self.record, self.run)

    def _event(self, message):
        events = self.run.setdefault('events', [])
        events.append(dict(time=time.time(), message=message))
        del events[:-60]

    async def snapshot(self):
        data = await super().snapshot()
        with self.lock:
            if self.run:
                if self.run['active']:
                    stored = read_json(Path(self.settings.workspace) / 'runtime/missions/actions' / (self.run['goal_id'] + '.json'))
                    if stored and stored.get('goal_id') == self.run['goal_id'] and stored.get('mission_id') == self.run['mission_id']:
                        state = stored['state'] if stored.get('gate_closed') is True and stored.get('goals_terminal') is True else 'STOP_UNCONFIRMED'
                        self.run.update({key: stored[key] for key in ('run_id', 'gate_closed', 'goals_terminal', 'report_path', 'message')})
                        self.run.update(active=False, stale=False, state=state, wait_id='', updated_at=time.time())
                        self._save()
                # Query the exact Action UUID after a gateway restart. Unknown is
                # conservative: an absent state service does not mean stopped.
                if self.run['active'] and self.handle is None and time.monotonic()-self.last_recovery > 2:
                    if self.result_client.service_is_ready() and (not self.recovery_future or self.recovery_future.done()):
                        self.last_recovery = time.monotonic()
                        request = self.action_type.Impl.GetResultService.Request()
                        request.goal_id.uuid = list(uuid.UUID(self.run['goal_id']).bytes)
                        self.recovery_future = self.result_client.call_async(request)
                        goal_id = self.run['goal_id']
                        self.recovery_future.add_done_callback(lambda f: self._result(f, goal_id))
                mission = dict(self.run)
                observed = data['mission']
                matches = bool(mission.get('run_id') and observed.get('run_id') == mission['run_id'] and not observed.get('stale'))
                if mission['active'] and matches:
                    for key in ('current_step', 'total_steps', 'current_waypoint', 'next_waypoint', 'wait_id', 'message', 'state'):
                        mission[key] = observed[key]
                    if mission['state'] in TERMINAL:
                        mission.update(state='CLEANING_UP', wait_id='')
                    mission['stale'] = False
                elif mission['active']:
                    mission['stale'] = time.time() - mission.get('updated_at', 0) > 2
                # A different CLI/client run is observable, but never adopted
                # as a web-owned cancel target.
                if observed.get('active') and observed.get('run_id') != mission.get('run_id'):
                    if not mission['active']:
                        mission = dict(observed, controllable=False)
                else:
                    mission['controllable'] = True
                data['mission'] = mission
            data['capabilities'].update(mission_control=True, navigation_control=True, initial_pose=True, single_goal=True)
            data['logs'] = [dict(time=self.started_at, message='ROS 导航控制已连接，运动由任务执行器管理')] + (self.run.get('events', []) if self.run else [])
            if self.run and self.run.get('report_path') and not self.run['active']:
                data['mission']['report_url'] = '/api/v1/navigation/report?goal_id=' + self.run['goal_id']
            if self.run and self.run['state'] == 'STOP_UNCONFIRMED':
                data['navigation']['ready'] = False
                data['navigation']['blockers'].append('本次停止未确认，请完成现场恢复并归档网页运行记录')
            data['navigation']['action_ready'] = self.action.server_is_ready()
            if not data['navigation']['action_ready']:
                data['navigation']['ready'] = False
                data['navigation']['blockers'].append('任务 Action 尚未就绪')
        return data

    async def _start(self, identifier, label):
        data = await self.snapshot()
        if data['mission']['active']:
            raise GatewayError(409, 'MISSION_BUSY', '已有任务或未确认的运行，请先核对当前任务')
        if not data['navigation']['ready']:
            raise GatewayError(409, 'NAVIGATION_NOT_READY', '；'.join(data['navigation']['blockers']))
        goal_id = str(uuid.uuid4())
        with self.lock:
            self.run = dict(empty_mission(False), active=True, state='STARTING', mission_id=identifier,
                            mission_name=label, goal_id=goal_id, updated_at=time.time(), stale=False,
                            message='请求已登记，等待机器人侧接受')
            self._event('任务启动请求已登记：' + label)
            self._save()  # Write UUID before sending, including the ambiguous-ack window.
        from unique_identifier_msgs.msg import UUID
        future = self.action.send_goal_async(self.action_type.Goal(mission_id=identifier),
            goal_uuid=UUID(uuid=list(uuid.UUID(goal_id).bytes)),
            feedback_callback=lambda message: self._feedback(message, goal_id))
        future.add_done_callback(lambda done: self._accepted(done, goal_id))
        try:
            handle = await ros_result(future)
        except GatewayError:
            return dict(run_id='', message='启动请求已发送，接受结果未确认；请查看任务状态')
        if not handle.accepted:
            raise GatewayError(409, 'GOAL_REJECTED', '任务执行器拒绝了本次请求，请检查就绪状态和恢复标记')
        return dict(run_id=self.run.get('run_id', ''), message='任务请求已接受，请查看执行状态')

    async def start_mission(self, mission_id):
        from ..workspace import WorkspaceData
        catalog = await to_thread(WorkspaceData(self.settings).catalog)
        selected = next((m for m in catalog['missions'] if m['id'] == mission_id), None)
        if selected is None:
            raise GatewayError(404, 'MISSION_NOT_FOUND', '任务不存在')
        if any(step.get('speech') for step in selected['steps']):
            data = await self.snapshot()
            if not data['navigation']['checks'].get('audio_bridge', {}).get('ready'):
                raise GatewayError(409, 'AUDIO_NOT_READY', '任务需要真实语音桥，请先准备运动与语音桥')
        return await self._start(mission_id, selected['name'])

    async def go_to(self, waypoint_id):
        from ..workspace import WorkspaceData
        catalog = await to_thread(WorkspaceData(self.settings).catalog)
        point = next((p for p in catalog['waypoints'] if p['id'] == waypoint_id), None)
        if point is None:
            raise GatewayError(404, 'WAYPOINT_NOT_FOUND', '导航点不存在')
        return await self._start('waypoint:' + waypoint_id, '前往' + point['name'])

    def _accepted(self, future, goal_id):
        with self.lock:
            if not self.run or self.run['goal_id'] != goal_id or future.cancelled() or future.exception():
                return
            handle = future.result()
            if not handle.accepted:
                self.run.update(active=False, state='ABORTED', message='执行器拒绝请求，未创建任务', updated_at=time.time())
                self._save()
                return
            self.handle = handle
            self._event('机器人侧已接受任务')
            self._save()
            handle.get_result_async().add_done_callback(lambda done: self._result(done, goal_id))

    def _feedback(self, message, goal_id):
        with self.lock:
            if not self.run or self.run['goal_id'] != goal_id or not self.run['active']:
                return
            feedback = message.feedback
            if feedback.state != self.run['state']:
                self._event(feedback.message or feedback.state)
            for key in ('run_id', 'wait_id', 'current_step', 'total_steps', 'current_waypoint', 'next_waypoint', 'message', 'state', 'simulated'):
                self.run[key] = getattr(feedback, key)
            self.run['distance_remaining'] = feedback.distance_remaining if feedback.distance_remaining >= 0 else None
            if self.run['state'] in TERMINAL:
                self.run.update(state='CLEANING_UP', wait_id='')
            self.run.update(updated_at=time.time(), stale=False)
            self._save()

    def _result(self, future, goal_id):
        with self.lock:
            if not self.run or self.run['goal_id'] != goal_id or not self.run['active']:
                return
            if future.cancelled() or future.exception() is not None:
                return
            response = future.result()
            if response.status not in (4, 5, 6):
                # UNKNOWN may mean the server restarted or evicted its result.
                self.run.update(stale=True, message='执行结果未知，请核对机器人和任务记录，禁止重复启动')
                self._save()
                return
            result = response.result
            state = result.state
            if not (result.gate_closed and result.goals_terminal):
                state = 'STOP_UNCONFIRMED'
            self.run.update(active=False, state=state, wait_id='', stale=False,
                            gate_closed=result.gate_closed, goals_terminal=result.goals_terminal,
                            message=result.message, report_path=result.report_path, updated_at=time.time())
            self._event(result.message or state)
            self._save()

    async def continue_mission(self, run_id, wait_id):
        mission = (await self.snapshot())['mission']
        if (not mission['active'] or mission.get('stale') or mission['run_id'] != run_id
                or mission['state'] != 'WAITING_FOR_CONTINUE' or mission['wait_id'] != wait_id):
            raise GatewayError(409, 'STALE_WAIT', '任务或等待标识已过期，请重新查询状态')
        response = await ros_result(self.continue_client.call_async(self.continue_type.Request(run_id=run_id, wait_id=wait_id)))
        if not response.accepted:
            raise GatewayError(409, 'STALE_WAIT', response.message)
        with self.lock:
            # The last station can finish before this RPC's acknowledgement is
            # delivered. Never overwrite an already-confirmed terminal result.
            if self.run['active'] and self.run['run_id'] == run_id:
                self.run.update(state='CONTINUING', wait_id='', updated_at=time.time())
                self._save()
        return dict(run_id=run_id, message=response.message)

    async def cancel_mission(self, run_id):
        with self.lock:
            if not self.run or not self.run['active'] or run_id != (self.run['run_id'] or self.run['goal_id']):
                raise GatewayError(409, 'STALE_RUN', '运行已结束或不属于当前网页任务')
            goal_id = self.run['goal_id']
        request = self.cancel_type.Request()
        request.goal_info.goal_id.uuid = list(uuid.UUID(goal_id).bytes)
        response = await ros_result(self.cancel_client.call_async(request))
        if not any(bytes(goal.goal_id.uuid) == uuid.UUID(goal_id).bytes for goal in response.goals_canceling):
            raise GatewayError(409, 'CANCEL_UNCONFIRMED', '取消未被接受，请查询当前任务结果')
        with self.lock:
            if self.run['active']:
                self.run.update(state='CANCELING', wait_id='', message='正在取消并等待停止收尾', updated_at=time.time())
                self._save()
        return dict(run_id=run_id, message='取消已接受，停止结果仍需确认')

    async def close_gate(self):
        if not self.gate_client.service_is_ready():
            raise GatewayError(409, 'GATE_UNKNOWN', '控制门服务不可用，不能确认关闭')
        response = await ros_result(self.gate_client.call_async(self.gate_type.Request(data=False)))
        if not response.success:
            raise GatewayError(409, 'GATE_UNCONFIRMED', response.message)
        return dict(message='控制门关闭已确认；任务停止状态请查看任务结果')

    async def initial_pose(self, x, y, yaw):
        data = await self.snapshot()
        if data['mission']['active'] or data['navigation'].get('active_nav_goal'):
            raise GatewayError(409, 'MISSION_BUSY', '任务运行中不能重新设置初始位姿')
        if ('g1_cmd_vel_dry_run', '/') in self.node.get_node_names_and_namespaces():
            await self.close_gate()
        if self.initial_pose_publisher.get_subscription_count() != 1:
            raise GatewayError(409, 'LOCALIZATION_NOT_READY', '需要唯一的定位初始化接收节点')
        message = self.pose_type()
        message.header.frame_id = 'map'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        message.pose.covariance[0] = message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = (math.pi/12)**2
        self.initial_pose_publisher.publish(message)
        return dict(message='初始位姿已提交，等待定位节点确认')

    async def close(self):
        # Losing a browser or gateway never cancels a robot-owned autonomous task.
        await super().close()

    def _close_extra(self):
        if hasattr(self, 'action'):
            self.action.destroy()
