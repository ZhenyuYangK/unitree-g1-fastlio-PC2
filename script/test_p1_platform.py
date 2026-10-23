#!/usr/bin/env python3
"""Offline P1 regressions. Only temporary files and harmless child processes."""
import ast
import copy
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parent
ROOT = SCRIPT.parent
sys.path.insert(0, str(SCRIPT))
sys.path.insert(0, str(ROOT / 'src/g1_task_manager'))

import g1_build_guard
import g1_livox_config
import g1_mapping_save
import g1_mapping_state
import g1_platform
import g1_process
from g1_task_manager import paths


def clean_env(**values):
    env = {k: v for k, v in os.environ.items() if not k.startswith('G1_') and k not in (
        'ROS_DISTRO', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
        'PYTHONPATH', 'LD_LIBRARY_PATH', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'ROS_DOMAIN_ID', 'WS')}
    env.update(values)
    return env


class EnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='g1 p1 ')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def bash(self, code, **env):
        return subprocess.run(['bash', '--noprofile', '--norc', '-c', code, '_', str(SCRIPT / 'g1_env.sh')],
                              env=clean_env(G1_WS=str(self.root), **env),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_source_has_no_ros_or_runtime_side_effects(self):
        result = self.bash('set -eu; source "$1"; printf "%s" "$G1_ROS_DISTRO"')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'foxy')
        self.assertFalse((self.root / 'runtime').exists())

    def test_exported_override_and_relative_site_paths(self):
        config = self.root / 'local.env'
        config.write_text('G1_WS=/does/not/exist\nG1_HOST_IP=10.1.1.10\nG1_WAYPOINTS_FILE="site data/points.yaml"\n')
        result = self.bash('set -eu; source "$1"; printf "%s|%s|%s" "$G1_WS" "$G1_HOST_IP" "$G1_WAYPOINTS_FILE"',
                           G1_LOCAL_ENV=str(config), G1_HOST_IP='10.1.1.20')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, '{}|10.1.1.20|{}/site data/points.yaml'.format(self.root, self.root))

    def test_local_network_values_exported(self):
        config = self.root / 'local.env'
        config.write_text('G1_HOST_IP=10.1.1.10\nG1_NETWORK_INTERFACE=eth9\nG1_LIDAR_IP=10.1.1.11\n')
        result = self.bash("source \"$1\"; python3 -c 'import os; print(os.environ[\"G1_HOST_IP\"])'", G1_LOCAL_ENV=str(config))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '10.1.1.10')

    def test_humble_and_hidden_prefix_rejected(self):
        for env in ({'ROS_DISTRO': 'humble'}, {'PYTHONPATH': '/opt/ros/humble/lib/python3.10/site-packages'}):
            result = self.bash('source "$1"; g1_load_ros', **env)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('ROS', result.stderr)

    def test_explicit_missing_underlay_fails(self):
        setup = self.root / 'foxy.bash'
        setup.write_text('export ROS_DISTRO=foxy\n')
        result = self.bash('source "$1"; g1_load_ros', G1_ROS_SETUP=str(setup), G1_PCL_SETUP=str(self.root / 'missing'))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('missing', result.stderr)

    def test_release_pair_required(self):
        setup = self.root / 'foxy.bash'
        setup.write_text('export ROS_DISTRO=foxy\n')
        result = self.bash('source "$1"; g1_load_ros', G1_ROS_SETUP=str(setup), G1_RMW_RELEASE_TEST='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('成对', result.stderr)

    def test_domain_validation(self):
        for value in ('-1', '233', 'x'):
            self.assertNotEqual(self.bash('source "$1"', ROS_DOMAIN_ID=value).returncode, 0)
        self.assertEqual(self.bash('source "$1"', ROS_DOMAIN_ID='0').returncode, 0)

    def test_stack_lock_is_reentrant(self):
        result = self.bash('source "$1"; g1_lock_stack; g1_lock_stack')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_other_manager_cannot_take_lock(self):
        result = self.bash('source "$1"; g1_lock_stack; bash -c \'exec 9>&-; source "$1"; g1_lock_stack\' _ "$1"; result=$?; exit "$result"')
        self.assertNotEqual(result.returncode, 0)


class DataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='g1 p1 data ')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_network_matches_interface_subnet_and_domain(self):
        env = dict(G1_NETWORK_INTERFACE='eth9', G1_HOST_IP='10.2.3.4', G1_LIDAR_IP='10.2.3.5', ROS_DOMAIN_ID='17')
        addresses = [dict(ifname='eth9', addr_info=[dict(local='10.2.3.4', prefixlen=24)])]
        self.assertEqual(g1_livox_config.network_values(env, addresses), ('10.2.3.4', '10.2.3.5', 17))
        for key, value in (('G1_HOST_IP', '10.2.3.9'), ('G1_LIDAR_IP', '10.9.1.1'),
                           ('G1_LIDAR_IP', '10.2.3.4'), ('G1_LIDAR_IP', '127.0.0.1'),
                           ('G1_NETWORK_INTERFACE', 'eth9<xml>'), ('ROS_DOMAIN_ID', '233')):
            invalid = dict(env, **{key: value})
            with self.assertRaises(ValueError):
                g1_livox_config.network_values(invalid, addresses)

    def test_livox_generation_does_not_change_source(self):
        source = ROOT / 'src/livox_ros_driver2/config/MID360_config.json'
        original = source.read_bytes()
        result = g1_livox_config.generate(source, '10.2.3.4', '10.2.3.5')
        self.assertEqual(result['MID360']['host_net_info']['imu_data_ip'], '10.2.3.4')
        self.assertEqual(result['lidar_configs'][0]['ip'], '10.2.3.5')
        self.assertEqual(source.read_bytes(), original)

    def test_mapping_session_handles_spaces_and_refuses_existing_map(self):
        source = ROOT / 'src/FAST_LIO_ROS2/config/mid360.yaml'
        state = g1_mapping_state.create_session(self.root, source, self.root / 'config.yaml',
                                                self.root / 'session.json', 'maps/my map.pcd')
        self.assertEqual(state['output'], str(self.root / 'maps/my map.pcd'))
        Path(state['output']).write_text('existing map')
        with self.assertRaises(ValueError):
            g1_mapping_state.create_session(self.root, source, self.root / 'config.yaml',
                                             self.root / 'session.json', 'maps/my map.pcd')
        self.assertEqual(Path(state['output']).read_text(), 'existing map')

    def test_save_requires_response_new_file_and_nonempty_content(self):
        output = self.root / 'map.pcd'
        with self.assertRaises(RuntimeError):
            g1_mapping_save.verify_saved(output, None, True)
        output.touch()
        with self.assertRaises(RuntimeError):
            g1_mapping_save.verify_saved(output, None, True)
        output.write_text('new map')
        previous = g1_mapping_save.fingerprint(output)
        with self.assertRaises(RuntimeError):
            g1_mapping_save.verify_saved(output, None, False)
        with self.assertRaises(RuntimeError):
            g1_mapping_save.verify_saved(output, previous, True)
        output.write_text('another new map')
        self.assertEqual(g1_mapping_save.verify_saved(output, previous, True), 15)

    def test_paths_source_and_explicit_override(self):
        with patch.dict(os.environ, clean_env(), clear=True):
            self.assertEqual(paths.workspace(), ROOT)
        with patch.dict(os.environ, clean_env(G1_WS=str(self.root), G1_WAYPOINTS_FILE='custom points.yaml'), clear=True):
            self.assertEqual(paths.default_waypoints_file(), self.root / 'custom points.yaml')

    def test_vendor_checksums(self):
        self.assertGreater(g1_platform.check_vendors(ROOT)['checked_files'], 60)

    def test_build_guard_refuses_old_and_changed_platform(self):
        build, install, log = [self.root / name for name in ('build_pc2', 'install_pc2', 'log_pc2')]
        for path in (build, install, log): path.mkdir()
        env = dict(G1_WS=str(self.root), G1_BUILD_DIR=str(build), G1_INSTALL_DIR=str(install), G1_LOG_DIR=str(log))
        with patch.dict(os.environ, env), patch.object(g1_build_guard, 'check_rmw', return_value={'sha256': 'a'}):
            g1_build_guard.main()
            g1_build_guard.main()
            with patch.object(g1_build_guard.platform, 'machine', return_value='changed-architecture'):
                with self.assertRaises(RuntimeError): g1_build_guard.main()
            with patch.dict(os.environ, G1_BUILD_DIR=str(self.root / 'build')):
                with self.assertRaises(RuntimeError): g1_build_guard.main()

    def test_build_guard_ignores_ldd_aslr_addresses(self):
        build, install, log = [self.root / name for name in ('build_pc2', 'install_pc2', 'log_pc2')]
        for path in (build, install, log): path.mkdir()
        env = dict(G1_WS=str(self.root), G1_BUILD_DIR=str(build), G1_INSTALL_DIR=str(install), G1_LOG_DIR=str(log))
        with patch.dict(os.environ, env):
            with patch.object(g1_build_guard, 'check_rmw', return_value={'sha256': 'a', 'ldd': '(0x1234)'}):
                g1_build_guard.main()
            with patch.object(g1_build_guard, 'check_rmw', return_value={'sha256': 'a', 'ldd': '(0x9999)'}):
                g1_build_guard.main()

    def test_rmw_checks_actual_dds_pair_and_architecture(self):
        name = 'rmw_cyclonedds_cpp'
        prefix = self.root / 'overlay'
        (prefix / 'lib').mkdir(parents=True)
        (prefix / 'lib' / ('lib' + name + '.so')).write_bytes(b'rmw')
        marker = prefix / 'share/ament_index/resource_index/packages' / name
        marker.parent.mkdir(parents=True)
        marker.touch()
        (prefix / 'share' / name).mkdir()
        (prefix / 'share' / name / 'package.xml').write_text('<package><version>1.0.0</version></package>')
        dds = self.root / 'dds/libddsc.so.0'
        dds.parent.mkdir()
        dds.write_bytes(b'dds')
        def run(argv):
            return (0, 'libddsc.so.0 => {} (0x1234)'.format(dds)) if argv[0] == 'ldd' else (0, 'Machine: AArch64')
        env = clean_env(AMENT_PREFIX_PATH=str(prefix), RMW_IMPLEMENTATION=name,
                        G1_RMW_SETUP=str(prefix / 'local_setup.bash'), G1_DDS_LIB_DIR=str(dds.parent))
        with patch.dict(os.environ, env, clear=True), patch.object(g1_platform, 'command', side_effect=run), \
                patch.object(g1_platform.platform, 'machine', return_value='aarch64'):
            result = g1_platform.check_rmw()
            self.assertEqual(result['version'], '1.0.0')
            self.assertEqual(result['dds_library'], str(dds))
            with patch.dict(os.environ, G1_DDS_LIB_DIR=str(self.root / 'wrong-dds')):
                with self.assertRaises(RuntimeError): g1_platform.check_rmw()
            with patch.object(g1_platform, 'command', return_value=(0, 'libddsc.so.0 => not found')):
                with self.assertRaises(RuntimeError): g1_platform.check_rmw()
            with patch.object(g1_platform.platform, 'machine', return_value='x86_64'):
                with self.assertRaises(RuntimeError): g1_platform.check_rmw()


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='g1 p1 process ')
        self.root = Path(self.temp.name)
        self.record = self.root / 'processes.pids'
        self.env = patch.dict(os.environ, G1_WS=str(self.root))
        self.env.start()
        self.groups = []

    def tearDown(self):
        for pgid in self.groups:
            try: os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError: pass
        for process in g1_process.started_processes:
            process.wait(timeout=2)
        g1_process.started_processes.clear()
        self.env.stop()
        self.temp.cleanup()

    def launch(self):
        g1_process.start(self.record, 'worker', self.root / 'worker.log',
                         [sys.executable, '-c', 'import time; time.sleep(60)'])
        data = g1_process.read_records(self.record)
        self.groups.append(data[-1]['pgid'])
        return data[-1]

    def test_managed_process_can_be_stopped(self):
        item = self.launch()
        self.assertTrue(g1_process.members(item))
        g1_process.stop(self.record)
        self.assertFalse(g1_process.members(item))
        self.assertFalse(self.record.exists())

    def test_new_session_archives_old_log(self):
        log = self.root / 'worker.log'
        log.write_text('NDT success from previous session')
        self.launch()
        self.assertNotIn('NDT success', log.read_text())
        archives = list(self.root.glob('worker.*.log'))
        self.assertEqual(len(archives), 1)
        self.assertIn('NDT success', archives[0].read_text())

    def test_reused_pid_or_changed_command_is_rejected(self):
        item = self.launch()
        for key in ('start', 'command_sha256', 'token'):
            changed = dict(item, **{key: 'wrong'})
            with self.assertRaises(RuntimeError): g1_process.members(changed)
        self.assertTrue(g1_process.identity(item['pid']))

    def test_old_boot_record_does_not_signal_current_process(self):
        item = self.launch()
        item['boot_id'] = 'previous-boot'
        self.assertEqual(g1_process.members(item), [])
        self.assertTrue(g1_process.identity(item['pid']))

    def test_legacy_live_pid_is_never_killed(self):
        self.record.write_text(str(os.getpid()) + '\n')
        with self.assertRaises(RuntimeError): g1_process.stop(self.record)
        self.assertTrue(self.record.exists())

    def test_foreign_workspace_record_is_rejected(self):
        self.launch()
        with patch.dict(os.environ, G1_WS=str(self.root / 'another-workspace')):
            with self.assertRaises(RuntimeError): g1_process.read_records(self.record)

    def test_orphan_child_remains_owned_after_leader_exit(self):
        command = [sys.executable, '-c',
                   'import subprocess, sys, time; subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]); subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True); time.sleep(.4)']
        g1_process.start(self.record, 'orphan', self.root / 'orphan.log', command)
        item = g1_process.read_records(self.record)[0]
        self.groups.append(item['pgid'])
        time.sleep(.6)
        self.assertTrue(g1_process.members(item))
        child_groups = {info['pgid'] for info in g1_process.members(item)}
        self.groups.extend(child_groups)
        self.assertEqual(len(child_groups), 2)
        g1_process.stop(self.record)
        self.assertFalse(g1_process.members(item))

    def test_cli_argument_parsing(self):
        result = subprocess.run([sys.executable, str(SCRIPT / 'g1_process.py'), 'status', '--file', str(self.record)],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.assertEqual(result.returncode, 1, result.stderr)


class CompatibilityTests(unittest.TestCase):
    def test_python38_syntax_and_bash(self):
        for path in list(SCRIPT.glob('*.py')) + list((ROOT / 'src/g1_task_manager/g1_task_manager').glob('*.py')):
            ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))
        for path in SCRIPT.glob('*.sh'):
            subprocess.run(['bash', '-n', str(path)], check=True)

    def test_offline_entrypoints_work_after_relocation(self):
        with tempfile.TemporaryDirectory(prefix='g1 relocated ') as folder:
            root = Path(folder)
            shutil.copytree(SCRIPT, root / 'script', ignore=shutil.ignore_patterns('__pycache__'))
            shutil.copytree(ROOT / 'src/g1_task_manager', root / 'src/g1_task_manager', ignore=shutil.ignore_patterns('__pycache__'))
            for name, command in (('g1_waypoints.sh', 'path'), ('g1_cruise.sh', '--help')):
                result = subprocess.run(['bash', str(root / 'script' / name), command], env=clean_env(),
                                        cwd='/tmp', stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                if name == 'g1_waypoints.sh': self.assertIn(str(root), result.stdout)

    def test_relocated_cmake_and_explicit_vendor_override(self):
        # 只验证配置和生成步骤；stub 不代表已链接 ROS 或通过 PC2 编译。
        with tempfile.TemporaryDirectory(prefix='g1 cmake relocated ') as folder:
            root = Path(folder)
            stubs = root / 'stubs'
            for package in ('ament_cmake', 'rclcpp', 'geometry_msgs', 'std_srvs', 'unitree_api'):
                config = stubs / 'share' / package / 'cmake' / (package + 'Config.cmake')
                config.parent.mkdir(parents=True)
                config.write_text('function(ament_target_dependencies)\nendfunction()\nfunction(ament_package)\nendfunction()\n')
            shutil.copytree(ROOT / 'third_party/unitree_ros2_example_include', root / 'third_party/unitree_ros2_example_include')
            for package in ('g1_loco_bridge', 'g1_unitree_ros2_probe'):
                shutil.copytree(ROOT / 'src' / package, root / 'src' / package)
                build = root / 'build' / package
                cmd = ['cmake', '-S', str(root / 'src' / package), '-B', str(build),
                       '-DCMAKE_PREFIX_PATH=' + str(stubs), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON']
                result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                self.assertEqual(result.returncode, 0, result.stdout)
                commands = (build / 'compile_commands.json').read_text()
                self.assertIn('-std=c++17', commands)
                self.assertNotIn('/home/g1test', commands)
                self.assertNotIn('libunitree_sdk2.a', commands)
                result = subprocess.run(cmd + ['-DUNITREE_ROS2_EXAMPLE_INCLUDE=' + str(root / 'missing')],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
