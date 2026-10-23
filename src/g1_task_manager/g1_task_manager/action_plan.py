"""Resolve single-point requests without changing the site's mission files."""
from contextlib import contextmanager
from pathlib import Path
import tempfile

import yaml

from .mission_store import load_missions
from .waypoint_store import WaypointStore


def validate_request(identifier, waypoints_file, missions_file):
    store = WaypointStore(waypoints_file).load()
    if identifier.startswith('waypoint:'):
        waypoint = identifier[len('waypoint:'):]
        if waypoint not in dict(store.list_waypoints()):
            raise ValueError('Unknown waypoint: ' + waypoint)
    elif identifier not in load_missions(missions_file, store):
        raise ValueError('Unknown mission: ' + identifier)
    return store


@contextmanager
def action_plan(identifier, waypoints_file, missions_file, runtime):
    store = validate_request(identifier, waypoints_file, missions_file)
    if not identifier.startswith('waypoint:'):
        yield identifier, str(missions_file)
        return
    waypoint = identifier[len('waypoint:'):]
    runtime = Path(runtime)
    runtime.mkdir(parents=True, exist_ok=True)
    # ':' cannot occur in a configured mission ID. Only this explicit namespace
    # constructs an ephemeral one-step mission; no public YAML is overwritten.
    with tempfile.TemporaryDirectory(prefix='single_point_', dir=str(runtime)) as temporary:
        path = Path(temporary) / 'mission.yaml'
        path.write_text(yaml.safe_dump(dict(schema_version=1, map_id=store.map_id, missions={
            'web_single_point': dict(name='前往' + dict(store.list_waypoints())[waypoint]['name'],
                                     steps=[dict(waypoint=waypoint, timeout_sec=180.0, dwell_sec=0.0, advance_mode='auto')])}),
            allow_unicode=True, sort_keys=False), encoding='utf-8')
        load_missions(path, store)
        yield 'web_single_point', str(path)
