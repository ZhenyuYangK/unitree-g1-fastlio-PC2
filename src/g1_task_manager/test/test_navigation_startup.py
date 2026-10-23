"""Exercise the actual launcher with fake ROS commands and managed processes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import textwrap
import unittest


class NavigationStartupTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='g1_nav_startup_')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.script = self.root / 'g1_navigation_real.sh'
        source = Path(__file__).resolve().parents[3] / 'script/g1_navigation_real.sh'
        shutil.copyfile(source, self.script)
        for name in ('g1_process.py', 'g1_livox_config.py'):
            shutil.copyfile(source.parent / name, self.root / name)
        (self.root / 'g1_env.sh').write_text(
            'export G1_WS="$G1_FAKE_ROOT"\ng1_source_environment() { :; }\n'
            'g1_lock_stack() { :; }\ng1_error() { echo "$*" >&2; }\n', encoding='utf-8')
        (self.root / 'g1_wait_localization.py').write_text('pass\n', encoding='utf-8')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.environment = dict(os.environ, G1_FAKE_ROOT=str(self.root),
                                PATH=str(self.bin) + ':' + os.environ['PATH'])
        self._executable(self.bin / 'python3', '''
            #!/usr/bin/python3
            import os, pathlib, sys, time
            root = pathlib.Path(os.environ['G1_FAKE_ROOT'])
            if len(sys.argv) > 1 and sys.argv[1].endswith('g1_process.py'):
                if 'status' in sys.argv and '--file' in sys.argv:
                    record = sys.argv[sys.argv.index('--file') + 1]
                    if record.endswith('mapping.pids'):
                        raise SystemExit(1)
                    if record.endswith('navigation.pids'):
                        raise SystemExit(1 if (root / 'nav_inactive').exists() else 0)
            if len(sys.argv) > 1 and sys.argv[1].endswith('g1_navigation_readiness.py'):
                mode = sys.argv[sys.argv.index('--mode') + 1]
                if (root / 'nav_inactive').exists():
                    raise SystemExit(1)
                if mode == 'real':
                    deadline = time.monotonic() + 1.0
                    while not (root / 'real_running').exists() and time.monotonic() < deadline:
                        time.sleep(0.02)
                    raise SystemExit(0 if (root / 'real_running').exists() else 1)
                raise SystemExit(0)
            if sys.argv[1:3] != ['-m', 'g1_task_manager.audio_readiness']:
                os.execv('/usr/bin/python3', ['python3'] + sys.argv[1:])
            if (root / 'bad_audio').exists():
                print('[ERROR] dry_run=true')
                raise SystemExit(1)
            if '--probe' not in sys.argv:
                deadline = time.monotonic() + 1.0
                while not (root / 'audio_running').exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
            if (root / 'audio_running').exists():
                if (root / 'fail_audio_ready').exists():
                    raise SystemExit(1)
                print('[OK] audio ready')
                raise SystemExit(0)
            raise SystemExit(3 if '--probe' in sys.argv else 1)
        ''')
        self._executable(self.bin / 'ros2', '''
            #!/usr/bin/python3
            import os, pathlib, sys
            root = pathlib.Path(os.environ['G1_FAKE_ROOT'])
            args = sys.argv[1:]
            if args[:2] == ['lifecycle', 'get']:
                print('inactive [2]' if (root / 'nav_inactive').exists() else 'active [3]')
            elif args[:2] == ['topic', 'list']:
                print('/cmd_vel_nav')
            elif args[:2] == ['node', 'list']:
                if (root / 'real_running').exists():
                    print('/g1_cmd_vel_dry_run')
            elif args[:2] == ['param', 'get']:
                print({'dry_run': 'Boolean value is: False',
                       'cmd_vel_topic': 'String value is: /cmd_vel_nav',
                       'enable_service_name': 'String value is: /g1_loco_bridge/enable_real_nav'}[args[-1]])
            elif args[:2] == ['run', 'g1_audio_bridge']:
                os.execv(str(root / 'fake_bridge'), ['fake_bridge'] + args[2:])
            elif args[:2] == ['pkg', 'prefix']:
                print(root / 'install')
            elif args[:2] == ['service', 'call']:
                print('success: true')
            elif args[:2] == ['run', 'g1_loco_bridge']:
                if (root / 'fail_real_start').exists():
                    raise SystemExit(1)
                os.execv(str(root / 'fake_bridge'), ['fake_bridge', 'real'])
            else:
                raise SystemExit('unexpected fake ros2 call: ' + repr(args))
        ''')
        self._executable(self.bin / 'sleep', '#!/bin/sh\nexec /bin/sleep 0.15\n')
        bridge = self.root / 'fake_bridge'
        self._executable(bridge, '''
            #!/usr/bin/python3
            import os, pathlib, signal, sys, time
            root = pathlib.Path(os.environ['G1_FAKE_ROOT'])
            kind = 'real' if sys.argv[1:2] == ['real'] else 'audio'
            state = root / (kind + '_running')
            def stop(*_):
                state.unlink(missing_ok=True)
                raise SystemExit(0)
            signal.signal(signal.SIGINT, stop)
            signal.signal(signal.SIGTERM, stop)
            state.touch()
            if kind == 'audio':
                (root / 'audio_arguments').write_text(' '.join(sys.argv[1:]))
            with (root / (kind + '_starts')).open('a') as stream:
                stream.write(str(os.getpid()) + '\\n')
            while True:
                time.sleep(0.05)
        ''')
        executable = self.root / 'install/lib/g1_audio_bridge/g1_audio_bridge'
        executable.parent.mkdir(parents=True)
        executable.symlink_to(bridge)
        # Run stop before removing fixtures, including when an assertion fails.
        self.addCleanup(lambda: self.invoke('stop'))

    def _executable(self, path, source):
        path.write_text(textwrap.dedent(source).lstrip(), encoding='utf-8')
        path.chmod(0o755)

    def invoke(self, command):
        return subprocess.run(['bash', str(self.script), command], env=self.environment,
                              capture_output=True, text=True, timeout=20)

    def assert_ok(self, result):
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_start_repeated_start_status_and_stop_manage_one_audio_instance(self):
        self.assert_ok(self.invoke('start'))
        self.assert_ok(self.invoke('start'))
        self.assertEqual(len((self.root / 'audio_starts').read_text().splitlines()), 1)
        self.assert_ok(self.invoke('status'))
        self.assert_ok(self.invoke('stop'))
        self.assertFalse((self.root / 'audio_running').exists())
        self.assertFalse((self.root / 'runtime/navigation/audio_bridge.pid').exists())

    def test_existing_motion_bridge_still_gets_missing_audio_bridge(self):
        self.assert_ok(self.invoke('start'))
        env = dict(self.environment, G1_WS=str(self.root))
        subprocess.run(['/usr/bin/python3', str(self.root / 'g1_process.py'), 'stop',
                        '--file', str(self.root / 'runtime/navigation/audio_bridge.pid')],
                       env=env, check=True, capture_output=True)
        self.assert_ok(self.invoke('start'))
        self.assertTrue((self.root / 'audio_running').exists())
        self.assertEqual(len((self.root / 'real_starts').read_text().splitlines()), 1)

    def test_external_audio_is_reused_and_not_stopped(self):
        (self.root / 'audio_running').touch()
        self.assert_ok(self.invoke('start'))
        self.assertFalse((self.root / 'audio_starts').exists())
        self.assert_ok(self.invoke('stop'))
        self.assertTrue((self.root / 'audio_running').exists())

    def test_incompatible_audio_aborts_without_starting_motion(self):
        (self.root / 'bad_audio').touch()
        result = self.invoke('start')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('dry_run=true', result.stdout)
        self.assertFalse((self.root / 'real_starts').exists())
        self.assertFalse((self.root / 'audio_starts').exists())

    def test_audio_readiness_failure_rolls_back_only_new_audio(self):
        (self.root / 'fail_audio_ready').touch()
        self.assertNotEqual(self.invoke('start').returncode, 0)
        self.assertFalse((self.root / 'audio_running').exists())
        self.assertFalse((self.root / 'real_starts').exists())

    def test_motion_start_failure_also_stops_owned_audio(self):
        (self.root / 'fail_real_start').touch()
        self.assertNotEqual(self.invoke('start').returncode, 0)
        self.assertFalse((self.root / 'audio_running').exists())

    def test_inactive_navigation_does_not_start_audio(self):
        (self.root / 'nav_inactive').touch()
        self.assertNotEqual(self.invoke('start').returncode, 0)
        self.assertFalse((self.root / 'audio_starts').exists())

    def test_default_volume_is_preserved_and_invalid_override_is_rejected(self):
        self.assert_ok(self.invoke('start'))
        self.assertIn('volume:=-1', (self.root / 'audio_arguments').read_text())
        self.assert_ok(self.invoke('stop'))
        self.environment['G1_AUDIO_VOLUME'] = '101'
        self.assertNotEqual(self.invoke('start').returncode, 0)
        self.assertFalse((self.root / 'audio_running').exists())

    def test_explicit_volume_is_passed_to_the_managed_audio_bridge(self):
        self.environment['G1_AUDIO_VOLUME'] = '42'
        self.assert_ok(self.invoke('start'))
        self.assertIn('volume:=42', (self.root / 'audio_arguments').read_text())


if __name__ == '__main__':
    unittest.main()
