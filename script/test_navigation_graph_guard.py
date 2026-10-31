"""Exercise restart discovery grace without weakening duplicate-node protection."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().with_name('g1_navigation.sh')


class NavigationGraphGuardTests(unittest.TestCase):
    def run_guard(self, mode):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'local.env'
            config.write_text('')
            env = {k: v for k, v in os.environ.items() if not k.startswith('G1_')}
            env.update(G1_WS=directory, G1_LOCAL_ENV=str(config))
            code = '''set -Eeuo pipefail
source "$1"
sleep() { :; }
timeout() { shift; "$@"; }
ros2() {
  local count=0
  [[ ! -f "$G1_WS/queries" ]] || read -r count < "$G1_WS/queries"
  count=$((count + 1)); echo "$count" > "$G1_WS/queries"
  case "$GUARD_TEST_MODE" in
    empty) : ;;
    transient) if (( count < 3 )); then echo /controller_server; fi ;;
    persistent) echo /controller_server ;;
    failed) return 1 ;;
  esac
}
wait_for_previous_nodes
echo GUARD_PASSED
'''
            env['GUARD_TEST_MODE'] = mode
            result = subprocess.run(['bash', '-c', code, '_', str(SCRIPT)],
                                    env=env, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=10)
            count = int((Path(directory) / 'queries').read_text())
            return result, count

    def test_empty_graph_passes_immediately(self):
        result, count = self.run_guard('empty')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(count, 1)

    def test_stale_names_must_disappear_before_start(self):
        result, count = self.run_guard('transient')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(count, 3)
        self.assertIn('等待已停止节点', result.stdout)

    def test_persistent_nodes_still_block(self):
        result, count = self.run_guard('persistent')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(count, 5)
        self.assertNotIn('GUARD_PASSED', result.stdout)
        self.assertIn('请由原启动方停止', result.stdout)

    def test_discovery_failure_blocks_without_retry(self):
        result, count = self.run_guard('failed')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(count, 1)
        self.assertNotIn('GUARD_PASSED', result.stdout)


if __name__ == '__main__':
    unittest.main()
