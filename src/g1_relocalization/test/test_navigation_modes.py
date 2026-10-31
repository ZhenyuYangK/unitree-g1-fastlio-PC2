"""Exercise the actual Bash start flow with stack operations replaced by stubs."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[3] / 'script/g1_navigation.sh'


class NavigationModesTest(unittest.TestCase):
    def run_flow(self, body, timeout='180'):
        with tempfile.TemporaryDirectory(prefix='g1 modes ') as directory:
            config = Path(directory) / 'local.env'
            config.write_text('')
            env = dict(os.environ, G1_WS=directory, G1_LOCAL_ENV=str(config),
                       G1_ENABLE_RVIZ='0', G1_LOCALIZATION_TIMEOUT_SEC=timeout)
            script = '''set -Eeuo pipefail
source {script}
g1_lock_stack() {{ :; }}
check_prerequisites() {{ :; }}
assert_not_running() {{ :; }}
stop_from_pid_file() {{ echo STOP; }}
g1_prepare_livox_config() {{ :; }}
generate_runtime_files() {{ :; }}
wait_for_topic() {{ :; }}
wait_for_node() {{ :; }}
activate_map_server() {{ :; }}
g1_start_rviz() {{ :; }}
sleep() {{ :; }}
timeout() {{ shift; "$@"; }}
ros2() {{ :; }}
start_process() {{ printf 'START'; printf ' <%s>' "$@"; printf '\\n'; }}
publish_fixed_pose() {{ echo FIXED_POSE; }}
start_nav2_stack() {{ echo NAV2 "$@"; }}
wait_for_localization() {{ echo WAIT "$@"; return 0; }}
{body}
'''.format(script=shlex.quote(str(SCRIPT)), body=body)
            return subprocess.run(['bash', '-c', script], env=env, text=True,
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)

    def test_auto_is_global_and_never_publishes_fixed_pose(self):
        result = self.run_flow('main start-auto-dry')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('<g1_relocalization:initialization_mode:=global>', result.stdout)
        self.assertIn('<g1_relocalization:global_initialization_timeout_sec:=180.0>', result.stdout)
        self.assertIn('<g1_relocalization:global_search_timeout_sec:=180.0>', result.stdout)
        self.assertIn('WAIT 200.0', result.stdout)
        self.assertIn('NAV2 yes', result.stdout)
        self.assertNotIn('FIXED_POSE', result.stdout)

    def test_fixed_and_manual_keep_manual_node_mode(self):
        for command in ('start', 'start-dry', 'start-fixed', 'start-fixed-dry'):
            result = self.run_flow('main ' + command)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn('<g1_relocalization:initialization_mode:=manual>', result.stdout)
            self.assertEqual('FIXED_POSE' in result.stdout, 'fixed' in command)

    def test_failed_wait_returns_nonzero_without_starting_nav2_or_stopping_nodes(self):
        result = self.run_flow('''wait_for_localization() { echo WAIT_FAILED; return 1; }
main start-auto''')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertNotIn('NAV2 ', result.stdout)
        self.assertIn('2D Pose Estimate', result.stdout)
        self.assertNotIn('STOP', result.stdout.split('WAIT_FAILED')[-1])

    def test_invalid_budget_fails_before_any_process_starts(self):
        for value in ('0', '-1', 'nan', 'inf', 'nope'):
            result = self.run_flow('main start-auto', value)
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertNotIn('START <', result.stdout)


if __name__ == '__main__':
    unittest.main()
