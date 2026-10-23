#!/usr/bin/env python3
"""Generate matching Foxy Nav2/bridge configurations without loading ROS."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import yaml

VELOCITY_KEYS = (
    'min_vel_x', 'min_vel_y', 'max_vel_x', 'max_vel_y', 'max_vel_theta',
    'min_speed_xy', 'max_speed_xy', 'min_speed_theta', 'acc_lim_x', 'acc_lim_y',
    'acc_lim_theta', 'decel_lim_x', 'decel_lim_y', 'decel_lim_theta',
)


def load_yaml(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('Expected YAML mapping: ' + str(path))
    return data


def node_params(data, node):
    return data[node]['ros__parameters']


def navigation_parameters(source, profile):
    if profile not in ('safe', 'target'):
        raise ValueError('Unknown navigation profile: ' + profile)
    data = copy.deepcopy(source)
    controller = node_params(data, 'controller_server')
    follow = controller['FollowPath']
    for key in VELOCITY_KEYS:
        value = follow[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Nonfinite or nonnumeric velocity parameter: ' + key)
    if any(follow[key] != 0 for key in ('min_vel_x', 'min_vel_y', 'max_vel_y')):
        raise ValueError('Navigation requires forward-only motion without lateral velocity')
    target = {'max_vel_x': 1.0, 'max_speed_xy': 1.0, 'max_vel_theta': 0.4,
              'acc_lim_x': 0.2, 'decel_lim_x': -0.3, 'acc_lim_theta': 4.0, 'decel_lim_theta': -4.0}
    if any(follow[key] != value for key, value in target.items()):
        raise ValueError('Source must describe the reviewed P4 target limits')
    # DWB needs a nonzero increment even for its single zero lateral sample.
    if follow['acc_lim_y'] <= 0 or follow['decel_lim_y'] >= 0 or follow['vy_samples'] != 1:
        raise ValueError('DWB requires positive lateral acceleration and one zero lateral sample')
    if profile == 'safe':
        follow['max_vel_x'] = follow['max_speed_xy'] = 0.2
    controller.update(odom_topic='/Odometry_nav', use_sim_time=False)
    if 'goal_checker_plugins' in controller or 'goal_checker_plugin' not in controller:
        raise ValueError('Expected Foxy goal_checker_plugin parameter')
    bt = node_params(data, 'bt_navigator')
    bt.update(odom_topic='/Odometry_nav', use_sim_time=False, default_bt_xml_filename='')
    if 'default_nav_to_pose_bt_xml' in bt or 'behavior_server' in data:
        raise ValueError('Humble parameters cannot be used by the Foxy launch')
    recovery = node_params(data, 'recoveries_server')
    recovery.update(global_frame='camera_init', transform_tolerance=0.5,
                    max_rotational_vel=follow['max_vel_theta'], rotational_acc_lim=follow['acc_lim_theta'])
    if recovery['recovery_plugins'] != ['spin', 'wait']:
        raise ValueError('Only spin/wait recoveries are supported; reverse motion is disabled')
    return data


def bridge_parameters(source, navigation, mode):
    data = copy.deepcopy(source)
    params = node_params(data, 'g1_cmd_vel_dry_run')
    follow = node_params(navigation, 'controller_server')['FollowPath']
    params.update(
        dry_run=mode == 'dry', require_localization=True,
        cmd_vel_topic='/cmd_vel_nav', allow_reverse=False, allow_lateral=False,
        enable_service_name='/g1_loco_bridge/enable' + ('_real_nav' if mode == 'real' else ''),
        max_linear_x=follow['max_vel_x'], max_angular_z=follow['max_vel_theta'],
        max_linear_accel=follow['acc_lim_x'], max_linear_decel=-follow['decel_lim_x'],
        max_angular_accel=follow['acc_lim_theta'], max_angular_decel=-follow['decel_lim_theta'])
    if params['min_effective_linear_x'] > params['max_linear_x']:
        raise ValueError('Bridge minimum speed exceeds profile cap')
    return data


def flatten_parameters(parameters, prefix=''):
    flat = {}
    for key, value in parameters.items():
        name = prefix + key
        if isinstance(value, dict):
            flat.update(flatten_parameters(value, name + '.'))
        else:
            flat[name] = value
    return flat


def generate(source, output, profile='safe', bridge_sources=None, manifest=None):
    navigation = navigation_parameters(load_yaml(source), profile)
    documents = {'nav': (Path(output), navigation)}
    for mode, (bridge_source, bridge_output) in (bridge_sources or {}).items():
        documents[mode] = (Path(bridge_output), bridge_parameters(load_yaml(bridge_source), navigation, mode))
    expected = {}
    for node in ('controller_server', 'bt_navigator', 'recoveries_server'):
        params = node_params(navigation, node)
        if node == 'controller_server':
            selected = {'odom_topic': params['odom_topic'], 'use_sim_time': False,
                        'FollowPath': {k: params['FollowPath'][k] for k in VELOCITY_KEYS}}
        elif node == 'bt_navigator':
            selected = {'odom_topic': params['odom_topic'], 'use_sim_time': False}
        else:
            selected = {k: params[k] for k in ('global_frame', 'robot_base_frame',
                        'transform_tolerance', 'max_rotational_vel', 'rotational_acc_lim', 'recovery_plugins')}
        expected[node] = flatten_parameters(selected)
    expected['navigation_odometry'] = {
        'input_topic': '/Odometry', 'output_topic': '/Odometry_nav',
        'body_frame': 'body', 'navigation_frame': 'nav_base',
        'body_to_nav_rotation_xyzw': [1.0, 0.0, 0.0, 0.0], 'use_sim_time': False}
    destinations = [p.resolve() for p, _ in documents.values()]
    if len(set(destinations)) != len(destinations):
        raise ValueError('Output files must differ')
    record = {'schema_version': 1, 'profile': profile, 'expected': expected, 'files': {}}
    for mode, (path, data) in documents.items():
        path = path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
        temporary.replace(path)
        record['files'][mode] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    if manifest:
        path = Path(manifest).resolve()
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(path)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('output')
    parser.add_argument('--profile', choices=('safe', 'target'), default='safe')
    parser.add_argument('--bridge-real', nargs=2, metavar=('SOURCE', 'OUTPUT'))
    parser.add_argument('--bridge-dry', nargs=2, metavar=('SOURCE', 'OUTPUT'))
    parser.add_argument('--manifest')
    args = parser.parse_args()
    bridges = {mode: value for mode, value in (('real', args.bridge_real), ('dry', args.bridge_dry)) if value}
    try:
        result = generate(args.source, args.output, args.profile, bridges, args.manifest)
    except (ValueError, KeyError, OSError, yaml.YAMLError) as exc:
        parser.exit(1, '导航参数生成失败: ' + str(exc) + '\n')
    print('Nav2 profile: {}; forward cap: {} m/s; odometry: /Odometry_nav'.format(
        result['profile'], 0.2 if args.profile == 'safe' else 1.0))
    for item in result['files'].values():
        print(item['path'])


if __name__ == '__main__':
    main()
