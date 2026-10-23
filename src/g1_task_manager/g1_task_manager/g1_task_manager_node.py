#!/usr/bin/env python3
"""Foxy Action front end for the proven, isolated CLI mission executor."""
import json
import os
import signal
import threading
import uuid

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_prefix
from g1_task_interfaces.action import ExecuteMission

from .health import validate_limits
from .mission_cli import default_missions_file, workspace, _write_json
from .task_cli import get_default_waypoints_file
from .worker import run_worker
from .action_plan import validate_request, action_plan


class G1TaskManagerNode(Node):
    def __init__(self):
        super().__init__('g1_task_manager')
        defaults = {
            'execute_real': False,
            'taskctl_executable': os.path.join(get_package_prefix('g1_task_manager'),
                                              'lib/g1_task_manager/g1_taskctl'),
            'waypoints_file': str(get_default_waypoints_file()),
            'missions_file': default_missions_file(),
            'gate_service': '/g1_loco_bridge/enable_real_nav',
            'max_tf_age_sec': 0.5, 'max_fitness': 2.0, 'max_fitness_age_sec': 5.0,
        }
        for key, default in defaults.items():
            self.declare_parameter(key, default, ParameterDescriptor(read_only=True))
        # These settings are intentionally latched at startup. Restart to change.
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        validate_limits(self.settings['max_tf_age_sec'], self.settings['max_fitness'],
                        self.settings['max_fitness_age_sec'])
        self.lock = threading.Lock()
        self.busy = False
        self.stopping = False
        self.blocked = False
        self.current_mission = ''
        self.cancel_event = threading.Event()
        self.group = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, '/g1_task/status', 10)
        self.server = ActionServer(
            self, ExecuteMission, '/g1_task/execute_mission', self.execute,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            callback_group=self.group, result_timeout=86400)
        # Keep the existing topic as an adapter to the very same Action admission.
        self.legacy_client = ActionClient(self, ExecuteMission,
                                         '/g1_task/execute_mission',
                                         callback_group=self.group)
        self.command_sub = self.create_subscription(
            String, '/g1_task/mission_request', self.legacy_request, 10,
            callback_group=self.group)
        self.cancel_srv = self.create_service(
            Trigger, '/g1_task/cancel', self.cancel_service, callback_group=self.group)
        self.publish_status('IDLE', '', 'EXECUTE' if self.settings['execute_real'] else 'PREVIEW')
        self.get_logger().info('Action ready: /g1_task/execute_mission; mode=' +
                               ('EXECUTE' if self.settings['execute_real'] else 'PREVIEW'))

    def publish_status(self, state, mission, message='', **fields):
        msg = String()
        msg.data = json.dumps(dict(state=state, mission=mission, message=message,
                                   mode='EXECUTE' if self.settings['execute_real'] else 'PREVIEW',
                                   **fields), ensure_ascii=False)
        self.status_pub.publish(msg)

    def goal_callback(self, request):
        with self.lock:
            if self.busy or self.stopping or self.blocked:
                self.get_logger().warning('Rejected: BUSY, shutting down, or STOP_UNCONFIRMED')
                return GoalResponse.REJECT
            try:
                validate_request(request.mission_id, self.settings['waypoints_file'], self.settings['missions_file'])
                if (workspace() / 'runtime/missions/recovery_required.json').exists():
                    raise ValueError('recovery_required.json exists; verify safe stop first')
            except Exception as exc:
                self.get_logger().error('Rejected: ' + str(exc))
                return GoalResponse.REJECT
            # Reserve NOW, before execute callback or subprocess creation.
            self.busy = True
            self.current_mission = request.mission_id
            self.cancel_event.clear()
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        with self.lock:
            if not self.busy:
                return CancelResponse.REJECT
            # The execution loop observes is_cancel_requested AFTER rclpy has
            # transitioned the Action to CANCELING, avoiding a terminal race.
            return CancelResponse.ACCEPT

    def cancel_service(self, request, response):
        del request
        with self.lock:
            response.success = self.busy
            if self.busy:
                self.cancel_event.set()
        response.message = ('Cancellation requested; wait for final state' if response.success
                            else 'No active mission')
        return response

    def request_stop(self):
        with self.lock:
            self.stopping = True
            self.cancel_event.set()

    def legacy_request(self, message):
        if not self.legacy_client.server_is_ready():
            self.get_logger().error('Task Action is not ready')
            return
        future = self.legacy_client.send_goal_async(
            ExecuteMission.Goal(mission_id=message.data.strip()))
        future.add_done_callback(self.legacy_accepted)

    def legacy_accepted(self, future):
        try:
            if not future.result().accepted:
                self.get_logger().warning('Topic request rejected by Action admission')
        except Exception as exc:
            self.get_logger().error(str(exc))

    def execute(self, goal_handle):
        mission = goal_handle.request.mission_id
        progress = dict(state='STARTING', current_waypoint='', current_step=0,
                        total_steps=0, distance_remaining=-1.0, message='', run_id='',
                        wait_id='', next_waypoint='', simulated=False)

        class Cancellation:
            def is_set(inner):
                return self.cancel_event.is_set() or goal_handle.is_cancel_requested

        def on_event(event):
            if event['kind'] not in ('progress', 'feedback'):
                return
            if event['kind'] == 'progress':
                progress['distance_remaining'] = -1.0
            for key in progress:
                if key in event:
                    progress[key] = event[key]
            # A disconnected feedback subscriber must not abandon the worker.
            try:
                goal_handle.publish_feedback(ExecuteMission.Feedback(**progress))
                self.publish_status(progress['state'], mission, progress['message'],
                                    current_step=progress['current_step'],
                                    total_steps=progress['total_steps'],
                                    current_waypoint=progress['current_waypoint'],
                                    distance_remaining=progress['distance_remaining'],
                                    run_id=progress['run_id'], wait_id=progress['wait_id'],
                                    next_waypoint=progress['next_waypoint'],
                                    simulated=progress['simulated'])
            except Exception as exc:
                self.get_logger().warning('Feedback publish failed: ' + str(exc))

        opts = self.settings
        command = [opts['taskctl_executable'], '--file', opts['waypoints_file'],
                   'mission', mission, '--missions-file', opts['missions_file'],
                   '--execute' if opts['execute_real'] else '--check-only',
                   '--gate-service', opts['gate_service'],
                   '--max-tf-age', str(opts['max_tf_age_sec']),
                   '--max-fitness', str(opts['max_fitness']),
                   '--max-fitness-age', str(opts['max_fitness_age_sec'])]
        report = dict(state='STOP_UNCONFIRMED', gate_closed=False,
                      goals_terminal=False, report_path='', errors=[])
        try:
            self.publish_status('STARTING', mission)
            with action_plan(mission, opts['waypoints_file'], opts['missions_file'],
                             workspace() / 'runtime/missions/plans') as (resolved_id, resolved_file):
                command[4] = resolved_id
                command[6] = resolved_file
                report = run_worker(command, Cancellation(), on_event,
                                    lambda line: self.get_logger().info('[MISSION] ' + line))
        except Exception as exc:
            report['errors'] = [str(exc)]
        # Do not announce SUCCEEDED/CANCELED when cleanup is unconfirmed.
        if opts['execute_real'] and report['state'] in ('SUCCEEDED', 'CANCELED'):
            if not (report.get('gate_closed') and report.get('goals_terminal')):
                report['state'] = 'STOP_UNCONFIRMED'
        state = report['state']
        result = ExecuteMission.Result(
            success=state in ('CHECKED', 'SUCCEEDED'), state=state,
            message='; '.join(report.get('errors', [])) or state,
            gate_closed=bool(report.get('gate_closed', False)),
            goals_terminal=bool(report.get('goals_terminal', False)),
            report_path=report.get('report_path', ''))
        try:
            # Preserve the result by exact Action UUID even if the server or
            # gateway restarts after motion cleanup but before client receipt.
            identifier = str(uuid.UUID(bytes=bytes(goal_handle.goal_id.uuid)))
            try:
                result_path = workspace() / 'runtime/missions/actions' / (identifier + '.json')
                result_path.parent.mkdir(parents=True, exist_ok=True)
                _write_json(result_path, dict(goal_id=identifier, run_id=progress['run_id'], mission_id=mission,
                            state=state, gate_closed=result.gate_closed, goals_terminal=result.goals_terminal,
                            report_path=result.report_path, message=result.message))
            except OSError as exc:
                self.get_logger().error('Cannot persist Action result: ' + str(exc))
            if state in ('CHECKED', 'SUCCEEDED'):
                goal_handle.succeed()
            elif state == 'CANCELED' and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                # A Trigger/SIGINT cancel is not an Action cancel handshake.
                # Report CANCELED in the payload, use ABORTED at protocol level.
                goal_handle.abort()
            self.publish_status(state, mission, result.message,
                                run_id=progress['run_id'],
                                gate_closed=result.gate_closed,
                                goals_terminal=result.goals_terminal,
                                report_path=result.report_path)
            return result
        finally:
            with self.lock:
                self.blocked = state == 'STOP_UNCONFIRMED'
                self.busy = False
                self.current_mission = ''


def main(args=None):
    rclpy.init(args=args)
    stop = threading.Event()
    previous = {}
    # Keep ROS alive until worker has closed the gate and canceled its goal.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, lambda *_: stop.set())
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = G1TaskManagerNode()
        executor.add_node(node)
        while rclpy.ok():
            if stop.is_set():
                node.request_stop()
                with node.lock:
                    if not node.busy:
                        break
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        if node is not None:
            node.legacy_client.destroy()
            node.server.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
