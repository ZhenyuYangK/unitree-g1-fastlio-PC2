"""Shared workspace and site paths for source and isolated colcon installs."""
import os
from pathlib import Path


def workspace():
    explicit = os.environ.get('G1_WS') or os.environ.get('WS')
    if explicit:
        return Path(explicit).expanduser().resolve()
    # 支持源码、symlink-install，以及 install_pc2/<pkg>/lib/python*/site-packages。
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src/g1_task_manager/package.xml').is_file() and (parent / 'script').is_dir():
            return parent
    raise RuntimeError('无法确定工作空间，请设置 G1_WS')


def site_path(variable, default):
    value = Path(os.environ.get(variable) or default).expanduser()
    return value.resolve() if value.is_absolute() else (workspace() / value).resolve()


def default_waypoints_file():
    return site_path('G1_WAYPOINTS_FILE', 'task_data/maps/laboratory_map_v1/waypoints.yaml')


def default_missions_file():
    return str(site_path('G1_MISSIONS_FILE', 'task_data/missions/missions.yaml'))
