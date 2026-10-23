#!/usr/bin/env python3
"""Mapping session and runtime YAML generation (Python 3.8, no ROS imports)."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import uuid

from g1_livox_config import atomic_json


def create_session(root, source, config, session, output=None):
    import yaml
    root = Path(root).resolve()
    output = Path(output or 'maps/map_{}.pcd'.format(datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    if not output.is_absolute():
        output = root / output
    output = output.resolve()
    if output.exists():
        raise ValueError('地图输出已存在，拒绝覆盖：{}'.format(output))
    output.parent.mkdir(parents=True, exist_ok=True)
    data = yaml.safe_load(Path(source).read_text())
    params = data['/**']['ros__parameters']
    params['map_file_path'] = str(output)
    params['pcd_save']['pcd_save_en'] = True
    Path(config).parent.mkdir(parents=True, exist_ok=True)
    Path(config).write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    state = dict(version=1, session_id=uuid.uuid4().hex, output=str(output),
                 config=str(Path(config).resolve()), started_at=datetime.now(timezone.utc).isoformat())
    atomic_json(session, state)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    create = sub.add_parser('create')
    create.add_argument('--source', required=True)
    create.add_argument('--config', required=True)
    create.add_argument('--session', required=True)
    create.add_argument('--output', default=os.environ.get('G1_MAPPING_OUTPUT'))
    show = sub.add_parser('path')
    show.add_argument('--session', required=True)
    args = parser.parse_args()
    try:
        if args.action == 'create':
            state = create_session(os.environ['G1_WS'], args.source, args.config, args.session, args.output)
        else:
            state = json.loads(Path(args.session).read_text())
        print(state['output'])
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, '[ERROR] {}\n'.format(exc))


if __name__ == '__main__':
    main()
