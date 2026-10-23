"""Load the PC2 source helpers independently of the caller's working directory."""
from pathlib import Path
import sys

SOURCE_WORKSPACE = Path(__file__).resolve().parents[3]
TASK_SOURCE = SOURCE_WORKSPACE / 'src/g1_task_manager'
SCRIPT_SOURCE = SOURCE_WORKSPACE / 'script'
for path in (TASK_SOURCE, SCRIPT_SOURCE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from g1_task_manager.paths import workspace


def site_file(root, value, default):
    path = Path(value or default).expanduser()
    return (path if path.is_absolute() else root / path).resolve()
