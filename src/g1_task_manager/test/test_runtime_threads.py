"""Check process entry points without starting ROS or importing the numeric stack."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class RuntimeThreadTests(unittest.TestCase):
    def test_cli_limits_inherited_pool_before_ros_import(self):
        code = '''
import os, sys
from g1_task_manager.task_cli import main
try:
    main(['--help'])
except SystemExit as exc:
    assert exc.code == 0
assert os.environ['OPENBLAS_NUM_THREADS'] == '1'
assert 'rclpy' not in sys.modules
assert 'numpy' not in sys.modules
'''
        for inherited in (None, '8'):
            env = dict(os.environ, PYTHONPATH=str(ROOT / 'src/g1_task_manager'))
            env.pop('OPENBLAS_NUM_THREADS', None)
            if inherited is not None:
                env['OPENBLAS_NUM_THREADS'] = inherited
            result = subprocess.run([sys.executable, '-c', code], env=env,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_shell_exports_limit_to_children(self):
        with tempfile.NamedTemporaryFile() as config:
            env = dict(os.environ, G1_WS=str(ROOT), G1_LOCAL_ENV=config.name,
                       OPENBLAS_NUM_THREADS='8', G1_ROS_DISTRO='foxy')
            result = subprocess.run(
                ['bash', '--noprofile', '--norc', '-c',
                 'source "$1" && python3 -c "import os; assert os.environ[\'OPENBLAS_NUM_THREADS\'] == \'1\'"',
                 '_', str(ROOT / 'script/g1_env.sh')],
                env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
