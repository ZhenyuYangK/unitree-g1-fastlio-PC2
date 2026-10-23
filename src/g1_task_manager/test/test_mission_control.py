"""One-use release tokens never carry permission into another phase or run."""
from concurrent.futures import ThreadPoolExecutor
import unittest

from g1_task_manager.mission_control import MissionControl


MISSION = {
    'id': 'tour', 'name': '实验室参观',
    'steps': [{'waypoint': 'point_01'}, {'waypoint': 'point_02'}],
}


class MissionControlTest(unittest.TestCase):
    def setUp(self):
        self.control = MissionControl(simulated=True)
        self.control.start(MISSION)

    def wait(self, index=1):
        return self.control.phase('WAITING_FOR_CONTINUE', index)

    def test_nonwaiting_phases_reject_release_without_saving_it_for_later(self):
        for phase in ('STARTING', 'CHECKING', 'NAVIGATING', 'ARRIVED', 'SPEAKING_ZH',
                      'SPEAKING_EN', 'DWELLING', 'CONTINUING', 'CLEANING_UP'):
            with self.subTest(phase=phase):
                self.control.phase(phase, 1)
                state = self.control.snapshot()
                accepted, _ = self.control.continue_wait(state['run_id'], 'guessed-wait-id')
                self.assertFalse(accepted)
                self.assertFalse(self.control.released())
                self.wait()
                self.assertFalse(self.control.released())

    def test_current_token_is_accepted_only_once(self):
        state = self.wait()
        token = (state['run_id'], state['wait_id'])
        self.assertTrue(self.control.continue_wait(*token)[0])
        self.assertTrue(self.control.released())
        self.assertEqual(self.control.snapshot()['state'], 'CONTINUING')
        self.assertEqual(self.control.snapshot()['wait_id'], '')
        self.assertFalse(self.control.continue_wait(*token)[0])

    def test_parallel_duplicate_requests_accept_only_one(self):
        state = self.wait()
        token = (state['run_id'], state['wait_id'])
        with ThreadPoolExecutor(max_workers=8) as executor:
            results = list(executor.map(lambda _: self.control.continue_wait(*token)[0], range(16)))
        self.assertEqual(results.count(True), 1)
        self.assertTrue(self.control.released())

    def test_empty_wrong_and_expired_tokens_leave_current_wait_unchanged(self):
        first = self.wait()
        self.control.phase('CHECKING', 2)
        current = self.wait(2)
        self.assertNotEqual(first['wait_id'], current['wait_id'])
        invalid_tokens = [
            ('', current['wait_id']), (current['run_id'], ''),
            ('another-run', current['wait_id']), (current['run_id'], 'another-wait'),
            (first['run_id'], first['wait_id']),
        ]
        for token in invalid_tokens:
            with self.subTest(token=token):
                self.assertFalse(self.control.continue_wait(*token)[0])
                self.assertFalse(self.control.released())
                self.assertEqual(self.control.snapshot(), current)
        self.assertTrue(self.control.continue_wait(current['run_id'], current['wait_id'])[0])

    def test_previous_run_cannot_release_same_mission_in_a_new_run(self):
        old = self.wait()
        self.control.finish('CANCELED')
        self.control.start(MISSION)
        current = self.wait()
        self.assertNotEqual(old['run_id'], current['run_id'])
        for token in ((old['run_id'], old['wait_id']), (old['run_id'], current['wait_id']),
                      (current['run_id'], old['wait_id'])):
            with self.subTest(token=token):
                self.assertFalse(self.control.continue_wait(*token)[0])
                self.assertFalse(self.control.released())
        self.assertTrue(self.control.continue_wait(current['run_id'], current['wait_id'])[0])

    def test_finish_and_idle_reject_all_release_requests(self):
        idle = MissionControl()
        self.assertFalse(idle.continue_wait('run', 'wait')[0])
        for outcome in ('SUCCEEDED', 'CANCELED', 'FAILED', 'STOP_UNCONFIRMED'):
            with self.subTest(outcome=outcome):
                self.control.start(MISSION)
                state = self.wait(2)
                self.control.finish(outcome)
                self.assertFalse(self.control.continue_wait(state['run_id'], state['wait_id'])[0])
                snapshot = self.control.snapshot()
                self.assertFalse(snapshot['active'])
                self.assertEqual(snapshot['state'], outcome)
                self.assertEqual(snapshot['wait_id'], '')
                self.assertFalse(self.control.released())

    def test_status_reports_station_and_next_station_without_mutating_control(self):
        first = self.wait()
        self.assertTrue(first['simulated'])
        self.assertTrue(first['active'])
        self.assertEqual(first['mission_id'], 'tour')
        self.assertEqual(first['total_steps'], 2)
        self.assertEqual(first['current_step'], 1)
        self.assertEqual(first['current_waypoint'], 'point_01')
        self.assertEqual(first['next_waypoint'], 'point_02')
        first['wait_id'] = 'client modified snapshot'
        self.assertNotEqual(self.control.snapshot()['wait_id'], first['wait_id'])
        last = self.wait(2)
        self.assertEqual(last['current_waypoint'], 'point_02')
        self.assertEqual(last['next_waypoint'], '')


if __name__ == '__main__':
    unittest.main()
