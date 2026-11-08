import copy
import unittest
from types import SimpleNamespace as S

from g1_task_manager.navigation_recovery import DataHold, matched_status


class RecoveryPolicyTests(unittest.TestCase):
    def test_release_requires_stop_gate_ack_and_continuous_fresh_data(self):
        hold = DataHold()
        hold.begin(0., 'ODOMETRY_STALE')
        self.assertFalse(hold.observe(.1, True))
        self.assertFalse(hold.ready(.8))
        hold.stopped = hold.acknowledged = True
        self.assertTrue(hold.ready(.8))
        hold.released = True
        hold.observe(.9, False)
        self.assertFalse(hold.released)
        self.assertFalse(hold.observe(1., True))
        self.assertFalse(hold.ready(1.49))
        self.assertTrue(hold.ready(1.5))
        self.assertFalse(hold.ready(3.))

    def test_recovery_count_never_resets_when_data_returns(self):
        hold = DataHold()
        for i in range(3):
            hold.begin(i*10., 'OBSTACLE_SENSOR_STALE')
            hold.begin(i*10.+.1, 'TASK_TF_STALE')
            self.assertEqual(hold.episode, i+1)
            hold.finish()
        with self.assertRaisesRegex(RuntimeError, 'DATA_RECOVERY_LIMIT'):
            hold.begin(40., 'ODOMETRY_STALE')

    def test_hard_faults_never_enter_automatic_recovery(self):
        for reason in ['ODOMETRY_JUMP', 'NAVIGATION_CLOCK_INVALID',
                       'OBSTACLE_SENSOR_CLOCK_INVALID', '定位未确认',
                       'PREVIOUS_GOALS_STOP_UNCONFIRMED']:
            with self.assertRaises(ValueError):
                DataHold().begin(0., reason)

    def test_handshake_rejects_old_future_cross_goal_or_malformed_status(self):
        pose = S(header=S(frame_id='map'), pose=S(position=S(x=1., y=2.),
                 orientation=S(x=0., y=0., z=0., w=1.)))
        status = dict(schema='G1_NAVIGATION_WAIT_V1', active=True, stamp_ns=1000,
                      session='a'*32, goal=dict(frame='map', x=1., y=2.,
                                              qx=0., qy=0., qz=0., qw=1.))
        self.assertIs(matched_status(status, .1, 900, pose, 1100), status)
        for changes in [dict(active=False), dict(stamp_ns=899), dict(stamp_ns=1101),
                        dict(session='bad'), dict(session='g'*32),
                        dict(goal=dict(status['goal'], x=3.)),
                        dict(goal=dict(status['goal'], qw=float('nan')))]:
            changed = copy.deepcopy(status)
            changed.update(changes)
            self.assertIsNone(matched_status(changed, .1, 900, pose, 1100))
        for age in [-.1, .501, float('nan')]:
            self.assertIsNone(matched_status(status, age, 900, pose, 1100))


if __name__ == '__main__':
    unittest.main()
