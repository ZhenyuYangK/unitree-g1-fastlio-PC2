"""Exercise configuration generation and the actual shell safety decisions."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/g1_nav2_bringup'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


generator = module('generator', PACKAGE / 'tools/generate_nav2_params.py')
readiness = module('readiness', ROOT / 'script/g1_navigation_readiness.py')


class NavigationConfigTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='g1 p4 config ')
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)

    def generate(self, profile='safe'):
        return generator.generate(PACKAGE / 'config/nav2_params.yaml', self.path / 'nav.yaml', profile,
            {mode: (ROOT / ('src/g1_loco_bridge/config/g1_loco_bridge_nav2_' + suffix + '.yaml'),
                    self.path / (mode + '.yaml')) for mode, suffix in (('dry', 'dry_run'), ('real', 'real'))},
            self.path / 'manifest.json')

    def test_profiles_generate_matching_nav2_and_both_bridges(self):
        for profile, cap in (('safe', 0.2), ('target', 1.0), ('safe', 0.2)):
            record = self.generate(profile)
            nav = generator.load_yaml(record['files']['nav']['path'])
            follow = nav['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual(follow['max_vel_x'], cap)
            self.assertEqual(follow['max_speed_xy'], cap)
            for mode in ('dry', 'real'):
                bridge = readiness.load_expected(self.path / 'manifest.json', mode, profile)['g1_cmd_vel_dry_run']
                self.assertEqual(bridge['max_linear_x'], cap)
                self.assertEqual(bridge['max_angular_z'], 0.4)
                self.assertEqual(bridge['max_linear_accel'], 0.2)
                self.assertEqual(bridge['max_linear_decel'], 0.3)
                self.assertEqual(bridge['max_angular_accel'], 4.0)
                self.assertEqual(bridge['max_angular_decel'], 4.0)
                self.assertFalse(bridge['allow_reverse'])
                self.assertFalse(bridge['allow_lateral'])
                self.assertTrue(bridge['require_localization'])
                self.assertEqual(bridge['real_backend'], mode == 'real')

    def test_changed_runtime_yaml_and_requested_profile_are_rejected(self):
        self.generate()
        with self.assertRaisesRegex(ValueError, '速度档'):
            readiness.load_expected(self.path / 'manifest.json', 'real', 'target')
        (self.path / 'nav.yaml').write_text('changed')
        with self.assertRaisesRegex(ValueError, '运行时配置'):
            readiness.load_expected(self.path / 'manifest.json', 'real', 'safe')

    def test_unsafe_or_nonfinite_source_limits_are_rejected(self):
        source = generator.load_yaml(PACKAGE / 'config/nav2_params.yaml')
        for key, value in (('min_vel_x', -0.1), ('max_vel_y', 0.1), ('acc_lim_y', 0.0),
                           ('max_vel_x', float('nan')), ('acc_lim_theta', 8.0)):
            changed = copy.deepcopy(source)
            changed['controller_server']['ros__parameters']['FollowPath'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                generator.navigation_parameters(changed, 'target')

    def test_foxy_tree_and_plugins_use_supported_contract_without_backup(self):
        self.generate()
        nav = generator.load_yaml(self.path / 'nav.yaml')
        self.assertNotIn('behavior_server', nav)
        self.assertIn('goal_checker_plugin', nav['controller_server']['ros__parameters'])
        self.assertEqual(nav['recoveries_server']['ros__parameters']['recovery_plugins'], ['spin', 'wait'])
        tree = ET.parse(PACKAGE / 'behavior_trees/navigate_g1_timeout.xml')
        self.assertIsNotNone(tree.find('.//SequenceStar'))
        self.assertIsNone(tree.find('.//BackUp'))
        for tag in ('ComputePathToPose', 'FollowPath', 'Spin', 'Wait'):
            self.assertEqual(tree.find('.//' + tag).get('server_timeout'), '1000')
        for name in ('controller_server', 'bt_navigator'):
            self.assertEqual(nav[name]['ros__parameters']['odom_topic'], '/Odometry_nav')

    def shell(self, script, body):
        config = self.path / 'local.env'
        config.write_text('')
        env = dict(os.environ, G1_WS=str(self.path), G1_LOCAL_ENV=str(config))
        code = '''set -Eeuo pipefail
source {script}
g1_lock_stack() {{ :; }}
load_env() {{ :; }}
check_nav() {{ echo NAV_CHECK; }}
processes() {{ echo PROCESS "$@"; return 0; }}
{body}
'''.format(script=shlex.quote(str(ROOT / 'script' / script)), body=body)
        return subprocess.run(['bash', '-c', code], env=env, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)

    def test_real_reuse_cannot_skip_actual_backend_verification(self):
        result = self.shell('g1_navigation_real.sh', '''check_bridge_mode() { echo CHECK_MODE "$@"; return 1; }
main start''')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('CHECK_MODE real', result.stdout)
        self.assertNotIn('复用已核验', result.stdout)
        self.assertNotIn('PROCESS start', result.stdout)

    def test_real_enable_requires_all_checks_before_enable(self):
        result = self.shell('g1_navigation_real.sh', '''check_bridge_mode() { echo CHECK_MODE "$@"; return 1; }
ros2() { echo UNEXPECTED_ROS_CALL; }
main enable''')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('NAV_CHECK', result.stdout)
        self.assertIn('CHECK_MODE real --enable', result.stdout)
        self.assertNotIn('UNEXPECTED_ROS_CALL', result.stdout)

    def test_bridge_mode_matching_requires_typed_booleans(self):
        self.assertFalse(readiness.parameter_matches(False, True))
        self.assertFalse(readiness.parameter_matches(1, True))
        self.assertTrue(readiness.parameter_matches([1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]))

    def test_actual_navigation_script_generates_all_runtime_files(self):
        (self.path / 'src').symlink_to(ROOT / 'src', target_is_directory=True)
        maps = self.path / 'maps'
        maps.mkdir()
        (maps / 'map_multilayer.pgm').write_text('P2\n1 1\n255\n0\n')
        (maps / 'map_multilayer.yaml').write_text('image: old.pgm\nresolution: 0.05\n')
        config = self.path / 'local.env'
        config.write_text('')
        for profile in ('safe', 'target'):
            env = dict(os.environ, G1_WS=str(self.path), G1_LOCAL_ENV=str(config), G1_NAV_PROFILE=profile)
            code = 'source {}; generate_runtime_files'.format(shlex.quote(str(ROOT / 'script/g1_navigation.sh')))
            result = subprocess.run(['bash', '-c', code], env=env, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout)
            runtime = self.path / 'runtime/navigation'
            expected = readiness.load_expected(runtime / 'navigation_config.json', 'real', profile)
            self.assertEqual(expected['g1_cmd_vel_dry_run']['max_linear_x'], 0.2 if profile == 'safe' else 1.0)
            fastlio = generator.load_yaml(runtime / 'mid360_navigation.yaml')
            self.assertFalse(fastlio['/**']['ros__parameters']['pcd_save']['pcd_save_en'])
            self.assertEqual(generator.load_yaml(runtime / 'map_runtime.yaml')['image'], str(maps / 'map_multilayer.pgm'))


if __name__ == '__main__':
    unittest.main()
