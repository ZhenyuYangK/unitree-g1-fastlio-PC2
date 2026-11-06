"""Behavioral checks for bounded detours and navigation-only obstacle waits."""
import copy
import math
from types import SimpleNamespace as S
import unittest

from g1_task_manager.nav_guard_policy import (ObstacleWait, RouteBudget, cell,
    dynamic_block_on_path, evaluation_state, path_length, waiting_for_this_goal, failure_for_this_goal)


def path(points):
    return S(header=S(frame_id='map'), poses=[S(pose=S(position=S(x=x, y=y))) for x, y in points])


def grid():
    return S(header=S(frame_id='map'), info=S(width=10, height=10, resolution=1.,
        origin=S(position=S(x=0., y=0.), orientation=S(x=0., y=0., z=0., w=1.))), data=[0]*100)


class RouteBudgetTests(unittest.TestCase):
    def test_length_uses_route_not_straight_line(self):
        self.assertEqual(path_length(path([(0, 0), (0, 4), (5, 4), (5, 0)])), 13.)

    def test_exactly_ten_extra_allowed_but_not_more(self):
        budget = RouteBudget(5.)
        self.assertTrue(budget.allows(15.))
        self.assertFalse(budget.allows(15.001))

    def test_repeated_replanning_cannot_reset_allowance(self):
        budget = RouteBudget(5.)
        budget.advance(8.)
        self.assertTrue(budget.allows(7.))
        self.assertFalse(budget.allows(7.1))
        budget.advance(3.)
        self.assertFalse(budget.allows(5.))

    def test_bad_paths_never_permitted(self):
        for points in ([], [(math.nan, 0.)], [(0., math.inf)]):
            with self.assertRaises(ValueError):
                path_length(path(points))
        for value in (math.nan, math.inf, -1.):
            self.assertFalse(RouteBudget(5.).allows(value))


class WaitTests(unittest.TestCase):
    def test_stops_immediately_but_speaks_after_one_second(self):
        policy = ObstacleWait()
        self.assertFalse(policy.observe(0., False, True))
        self.assertFalse(policy.speech_due(.99))
        self.assertTrue(policy.speech_due(1.))

    def test_one_attempt_even_during_indefinite_wait(self):
        policy = ObstacleWait()
        policy.observe(0., False, True)
        policy.spoken = True
        for now in (1., 20., 60., 60000.):
            policy.observe(now, False, True)
            self.assertFalse(policy.speech_due(now))

    def test_two_continuous_seconds_and_new_episode_after_recovery(self):
        policy = ObstacleWait()
        policy.observe(0., False, True)
        policy.spoken = True
        self.assertFalse(policy.observe(1., True))
        self.assertFalse(policy.observe(2.9, True))
        self.assertTrue(policy.observe(3., True))
        policy.observe(4., False, True)
        self.assertEqual(policy.episode, 2)
        self.assertTrue(policy.speech_due(5.))

    def test_brief_clear_does_not_repeat_speech_or_release(self):
        policy = ObstacleWait()
        policy.observe(0., False, True)
        policy.spoken = True
        policy.observe(1., True)
        policy.observe(2., False, True)
        self.assertFalse(policy.observe(3., True))
        self.assertFalse(policy.observe(4.9, True))
        self.assertTrue(policy.observe(5., True))
        self.assertEqual(policy.episode, 1)

    def test_sensor_fault_or_clear_probe_never_requests_speech(self):
        policy = ObstacleWait()
        policy.observe(0., False, False)
        self.assertFalse(policy.speech_due(20.))
        policy.observe(21., False, True)
        policy.observe(21.5, True)
        self.assertFalse(policy.speech_due(22.))


class EvidenceTests(unittest.TestCase):
    def test_only_live_obstacle_on_remaining_path_counts(self):
        live, reference = grid(), grid()
        route = path([(1., 1.), (2., 1.), (3., 1.)])
        live.data[12] = 100
        self.assertTrue(dynamic_block_on_path(route, live, reference))
        self.assertFalse(dynamic_block_on_path(route, live, reference, 2))
        reference.data[12] = 100
        self.assertFalse(dynamic_block_on_path(route, live, reference))
        reference.data[12] = -1
        self.assertFalse(dynamic_block_on_path(route, live, reference))

    def test_rotated_grid_origin_is_respected(self):
        value = grid()
        value.info.origin.orientation.z = math.sin(math.pi / 4)
        value.info.origin.orientation.w = math.cos(math.pi / 4)
        value.data[12] = 100
        self.assertEqual(cell(value, -1.5, 2.5), 100)

    def test_dwb_failure_reasons_are_not_all_obstacles(self):
        def message(total, critic):
            return S(twists=[S(total=total, scores=[S(name=critic, raw_score=-1.)])])
        self.assertEqual(evaluation_state(message(0., 'BaseObstacle')), 'CLEAR')
        self.assertEqual(evaluation_state(message(-1., 'BaseObstacle')), 'OBSTACLE')
        self.assertEqual(evaluation_state(message(-1., 'Oscillation')), 'FAULT')
        self.assertEqual(evaluation_state(S(twists=[])), 'FAULT')

    def test_selected_zero_motion_and_obstacle_failures_are_stationary_obstruction(self):
        zero = S(total=0., scores=[], traj=S(velocity=S(x=0., y=0., theta=0.)))
        blocked = S(total=-1., scores=[S(name='BaseObstacle', raw_score=-1.)])
        message = S(twists=[zero, blocked], best_index=0)
        self.assertEqual(evaluation_state(message), 'STATIONARY_OBSTACLE')
        zero.traj.velocity.theta = .2
        self.assertEqual(evaluation_state(message), 'CLEAR')
        zero.traj.velocity.theta = 0.
        blocked.scores[0].name = 'Oscillation'
        self.assertEqual(evaluation_state(message), 'CLEAR')


class TimeoutTests(unittest.TestCase):
    def test_failure_explanation_is_fresh_and_keeps_goal_identity(self):
        pose = S(header=S(frame_id='map'), pose=S(position=S(x=1., y=2.),
                 orientation=S(x=0., y=0., z=0., w=1.)))
        status = dict(schema='G1_NAVIGATION_WAIT_V1', stamp_ns=1000, state='IDLE',
                      last_fault='ACTION_ACCEPTANCE_TIMEOUT:/follow_path',
                      last_goal=dict(frame='map', x=1., y=2., qx=0., qy=0., qz=0., qw=1.))
        self.assertEqual(failure_for_this_goal(status, .1, 900, pose), status['last_fault'])
        self.assertEqual(failure_for_this_goal(status, 1., 900, pose), '')
        self.assertEqual(failure_for_this_goal(status, .1, 1100, pose), '')
        pose.pose.position.x = 3.
        self.assertEqual(failure_for_this_goal(status, .1, 900, pose), '')
        status.update(state='STOP_UNCONFIRMED', pending_owned_goals=[dict(uuid='known')])
        self.assertIn('停止尚未确认', failure_for_this_goal(status, .1, 900, pose))

    def test_only_fresh_matching_navigation_wait_pauses_timeout(self):
        pose = S(header=S(frame_id='map'), pose=S(position=S(x=1., y=2.),
                 orientation=S(x=0., y=0., z=0., w=1.)))
        status = dict(schema='G1_NAVIGATION_WAIT_V1', active=True, stamp_ns=1000, state='BLOCKED', obstacle_wait=True,
                      goal=dict(frame='map', x=1., y=2., qx=0., qy=0., qz=0., qw=1.))
        self.assertTrue(waiting_for_this_goal(status, .1, 900, pose))
        self.assertFalse(waiting_for_this_goal(status, 1., 900, pose))
        self.assertFalse(waiting_for_this_goal(status, .1, 1100, pose))
        for state in ('IDLE', 'SPEAKING_ZH', 'DWELLING', 'CANCELING', 'MOVING'):
            changed = dict(status, state=state)
            self.assertFalse(waiting_for_this_goal(changed, .1, 900, pose))
        changed = copy.deepcopy(status)
        changed['goal']['x'] = 3.
        self.assertFalse(waiting_for_this_goal(changed, .1, 900, pose))


if __name__ == '__main__':
    unittest.main()
