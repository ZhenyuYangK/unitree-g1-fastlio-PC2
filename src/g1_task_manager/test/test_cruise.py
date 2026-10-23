"""Preserve PC2 cruise semantics across the P5 mission executor upgrade."""
from contextlib import redirect_stdout
from io import StringIO
import unittest
from unittest.mock import patch

from g1_task_manager.cruise import Control, run_loop
from g1_task_manager.mission_control import MissionControl, MissionProgress
from g1_task_manager.mission_engine import execute_mission


class Transport(MissionProgress):
    def __init__(self):
        self.control = MissionControl()
        self.goals = []
        self.opened = False

    def close_gate(self):
        self.opened = False

    def open_gate(self):
        self.opened = True

    def ready(self):
        assert not self.opened

    def prepare_goal(self, step):
        self.goals.append(step['waypoint'])

    def wait_result(self, timeout):
        return 'SUCCEEDED'

    def dwell(self, seconds):
        assert not self.opened

    def begin_cleanup(self):
        self.cleanup = True

    def cancel_pending(self):
        return True


class CruiseTest(unittest.TestCase):
    def plan(self, laps=2, dwell=0.):
        return dict(route=[dict(waypoint=p, timeout_sec=1., dwell_sec=0.)
                           for p in ('a', 'b')], laps=laps, dwell_sec=dwell)

    def run_cruise(self, plan, execute=execute_mission, on_spin=None):
        control, transport = Control(plan), Transport()
        clock = [0.]
        states = []

        def persist():
            states.append(control.snapshot())

        def spin(seconds):
            clock[0] += max(seconds, 1e-6)
            if on_spin:
                on_spin(control)

        with redirect_stdout(StringIO()), patch(
                'g1_task_manager.cruise.time.monotonic', side_effect=lambda: clock[0]):
            result = run_loop(control, plan, transport, execute, persist, spin)
        return result, transport, control, states

    def test_two_laps_use_real_upgraded_executor(self):
        result, transport, control, _ = self.run_cruise(self.plan())
        self.assertEqual(result, 'SUCCEEDED')
        self.assertEqual(transport.goals, ['a', 'b', 'a', 'b'])
        self.assertEqual(control.snapshot()['completed_visits'], 4)
        self.assertFalse(transport.opened)

    def test_enroute_pause_retries_only_unfinished_leg_after_resume(self):
        calls = []

        def execute(mission, transport, emit):
            calls.append(mission['steps'][0]['waypoint'])
            if len(calls) == 1:
                control.request('pause')
                return dict(state='CANCELED', gate_closed=True, goals_terminal=True,
                            steps=[dict(state='CANCELED')], errors=[])
            return execute_mission(mission, transport, emit)

        plan = self.plan(laps=1)
        control, transport = Control(plan), Transport()
        paused = []

        def spin(_):
            paused.append(control.snapshot())
            self.assertEqual(control.snapshot()['state'], 'PAUSED')
            self.assertTrue(control.request('resume')['ok'])

        with redirect_stdout(StringIO()):
            result = run_loop(control, plan, transport, execute, lambda: None, spin)
        self.assertEqual(result, 'SUCCEEDED')
        self.assertEqual(calls, ['a', 'a', 'b'])
        self.assertTrue(paused[0]['gate_closed'] and paused[0]['goals_terminal'])

    def test_dwell_pause_preserves_remaining_time_and_does_not_repeat_goal(self):
        paused = []
        requested = [False]

        def spin(control):
            if control.snapshot()['state'] == 'DWELLING' and not requested[0]:
                requested[0] = True
                control.request('pause')
            elif control.snapshot()['state'] == 'PAUSED':
                paused.append(control.snapshot()['dwell_remaining'])
                control.request('resume')

        result, transport, _, _ = self.run_cruise(self.plan(laps=1, dwell=.2), on_spin=spin)
        self.assertEqual(result, 'SUCCEEDED')
        self.assertEqual(transport.goals, ['a', 'b'])
        self.assertGreater(paused[0], 0.)
        self.assertLess(paused[0], .2)

    def test_loop_continues_until_stop(self):
        def spin(control):
            if control.snapshot()['completed_visits'] >= 3:
                control.request('stop')
        result, transport, _, _ = self.run_cruise(self.plan(laps=None, dwell=.1), on_spin=spin)
        self.assertEqual(result, 'CANCELED')
        self.assertEqual(transport.goals, ['a', 'b', 'a'])

    def test_unconfirmed_stop_cannot_be_resumed(self):
        def execute(*_):
            return dict(state='STOP_UNCONFIRMED', gate_closed=True, goals_terminal=False,
                        steps=[], errors=['cancel timed out'])
        result, transport, _, _ = self.run_cruise(self.plan(), execute=execute)
        self.assertEqual(result, 'STOP_UNCONFIRMED')
        self.assertEqual(transport.goals, [])

    def test_navigation_failure_does_not_retry_or_start_next_point(self):
        calls = []
        def execute(mission, *_):
            calls.append(mission['steps'][0]['waypoint'])
            return dict(state='FAILED', gate_closed=True, goals_terminal=True,
                        steps=[], errors=['LOST'])
        result, _, _, _ = self.run_cruise(self.plan(), execute=execute)
        self.assertEqual(result, 'FAILED')
        self.assertEqual(calls, ['a'])

    def test_pause_after_arrival_does_not_repeat_successful_navigation(self):
        plan = self.plan(laps=1)
        control, transport = Control(plan), Transport()
        calls = []
        def execute(mission, transport, emit):
            calls.append(mission['steps'][0]['waypoint'])
            if len(calls) == 1:
                control.request('pause')
                return dict(state='CANCELED', gate_closed=True, goals_terminal=True,
                    steps=[dict(state='CANCELED', navigation_state='SUCCEEDED')], errors=[])
            return execute_mission(mission, transport, emit)
        with redirect_stdout(StringIO()):
            result = run_loop(control, plan, transport, execute, lambda: None,
                              lambda _: control.request('resume'))
        self.assertEqual(result, 'SUCCEEDED')
        self.assertEqual(calls, ['a', 'b'])


if __name__ == '__main__':
    unittest.main()
