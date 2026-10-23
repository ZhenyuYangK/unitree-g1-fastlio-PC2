"""No ROS transport: readiness must never accept temporary candidate states."""
import importlib.util
from pathlib import Path
import unittest

script = Path(__file__).resolve().parents[3] / 'script/g1_wait_localization.py'
spec = importlib.util.spec_from_file_location('wait_localization', script)
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class WaitLocalizationTest(unittest.TestCase):
    def test_verification_and_retry_are_pending_even_with_valid_flag(self):
        now = 10_000_000_000
        for state in ['WAITING_DATA', 'COLLECTING', 'SEARCHING', 'VERIFYING_CANDIDATES', 'RETRYING']:
            status = dict(state=state, valid=True, stamp_ns=now, last_success_stamp_ns=now)
            self.assertFalse(helper.assess_status(status, now)[0])
            self.assertFalse(helper.terminal_failure(status, now))

    def test_only_fresh_final_failure_ends_wait(self):
        now = 10_000_000_000
        for state in ['FAILED', 'AMBIGUOUS']:
            self.assertTrue(helper.terminal_failure(dict(state=state, stamp_ns=now), now))
            self.assertFalse(helper.terminal_failure(dict(state=state, stamp_ns=now-2_000_000_000), now))
        self.assertFalse(helper.terminal_failure(dict(state=[], stamp_ns=now), now))

    def test_confirmed_pose_still_requires_a_fresh_successful_scan(self):
        now = 10_000_000_000
        status = dict(state='LOCALIZED', valid=True, stamp_ns=now, last_success_stamp_ns=now)
        self.assertTrue(helper.assess_status(status, now)[0])
        status['last_success_stamp_ns'] = now - 4_000_000_000
        self.assertFalse(helper.assess_status(status, now)[0])


if __name__ == '__main__':
    unittest.main()
