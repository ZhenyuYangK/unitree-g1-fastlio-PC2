import unittest
from g1_task_manager.health import localization_status_reason, parse_localization_status


class LocalizationHealthTest(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.status = dict(state='TRACKING', valid=True, stamp_ns=self.now - 100_000_000,
                           last_success_stamp_ns=self.now - 500_000_000)

    def test_live_tracking(self):
        self.assertEqual(localization_status_reason(self.status, .1, self.now), '')

    def test_live_status_with_old_scan_is_not_healthy(self):
        self.status['last_success_stamp_ns'] = self.now - 4_000_000_000
        self.assertTrue(localization_status_reason(self.status, .1, self.now))

    def test_lost_and_ambiguous_are_not_healthy(self):
        for state in ('LOST', 'VERIFYING', 'VERIFYING_CANDIDATES', 'RETRYING', 'SEARCHING', 'AMBIGUOUS'):
            self.status['state'] = state
            self.assertTrue(localization_status_reason(self.status, .1, self.now))

    def test_invalid_and_unreachable_are_not_healthy(self):
        for field, value in (('valid', 1), ('stamp_ns', True),
                             ('state', []), ('last_success_stamp_ns', self.now + 1),
                             ('stamp_ns', 10 ** 1000),
                             ('last_success_stamp_ns', 1 << 63)):
            status = dict(self.status)
            status[field] = value
            self.assertTrue(localization_status_reason(status, .1, self.now))
        self.assertTrue(localization_status_reason(self.status, 1.01, self.now))
        self.assertTrue(localization_status_reason(None, None, self.now))
        self.assertTrue(localization_status_reason(self.status, .1, 10 ** 1000))

    def test_bad_json_never_reuses_prior_valid_status(self):
        for value in ('garbage', '[]', 'null', '{'):
            self.assertIsNone(parse_localization_status(value))


if __name__ == '__main__':
    unittest.main()
