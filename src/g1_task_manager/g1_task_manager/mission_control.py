"""One-use station release tokens, independent of ROS and wall-clock time."""
import threading
import uuid

from .task_events import emit_event, safe_print as print


class MissionControl:
    def __init__(self, simulated=False):
        self._lock = threading.Lock()
        self._mission = None
        self._released = False
        self._state = dict(active=False, simulated=simulated, run_id='', mission_id='',
                           mission_name='', state='IDLE', current_step=0, total_steps=0,
                           current_waypoint='', next_waypoint='', wait_id='', message='')

    def start(self, mission):
        with self._lock:
            self._mission = mission
            self._released = False
            self._state.update(active=True, run_id=uuid.uuid4().hex,
                               mission_id=mission['id'], mission_name=mission['name'],
                               state='STARTING', current_step=0, total_steps=len(mission['steps']),
                               current_waypoint='', next_waypoint=mission['steps'][0]['waypoint'],
                               wait_id='', message='')

    def phase(self, state, index, message=''):
        with self._lock:
            steps = self._mission['steps']
            self._state.update(state=state, current_step=index,
                               current_waypoint=steps[index - 1]['waypoint'] if index else '',
                               next_waypoint=steps[index]['waypoint'] if index < len(steps) else '',
                               message=message, wait_id='')
            self._released = False
            if state == 'WAITING_FOR_CONTINUE':
                self._state['wait_id'] = uuid.uuid4().hex
            return dict(self._state)

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def continue_wait(self, run_id, wait_id):
        with self._lock:
            if not self._state['active']:
                return False, '没有运行中的任务'
            if self._state['state'] != 'WAITING_FOR_CONTINUE':
                return False, '当前状态 {}，不能放行'.format(self._state['state'])
            if (not run_id or not wait_id or run_id != self._state['run_id']
                    or wait_id != self._state['wait_id']):
                return False, '任务或等待标识已过期，请重新查询状态'
            self._released = True
            self._state.update(state='CONTINUING', wait_id='', message='放行已接受')
            return True, '放行已接受；请通过 status 查看实际进展'

    def released(self):
        with self._lock:
            return self._released

    def finish(self, state):
        with self._lock:
            self._state.update(active=False, state=state, wait_id='', message=state)
            self._released = False


class MissionProgress:
    """Shared progress handling for real and simulated transports."""
    def start_mission(self, mission):
        self.control.start(mission)

    def preflight(self, mission):
        """Simulation has no hardware dependencies."""

    def phase(self, state, index, message=''):
        snapshot = self.control.phase(state, index, message)
        emit_event('progress', **snapshot)

    def finish_mission(self, state):
        self.control.finish(state)
        emit_event('progress', **self.control.snapshot())

    def finish_cleanup(self):
        return []

    def wait_for_continue(self, index):
        last = index == self.control.snapshot()['total_steps']
        message = '最后一站，继续以完成任务' if last else '等待负责人放行'
        self.phase('WAITING_FOR_CONTINUE', index, message)
        print('[MISSION] {}；安全门保持关闭。请在另一终端执行 g1_waypoints.sh continue'.format(
            message), flush=True)
        while not self.control.released():
            self._spin(0.05)
        self.phase('CONTINUING', index, '放行已接受')
