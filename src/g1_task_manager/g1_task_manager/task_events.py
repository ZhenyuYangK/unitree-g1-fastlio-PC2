"""Machine-readable worker events, separate from human-readable CLI output."""
import json
import os

PREFIX = 'G1_TASK_EVENT '


def safe_print(*args, **kwargs):
    """A closed supervision terminal must never interrupt motion cleanup."""
    try:
        print(*args, **kwargs)
    except BrokenPipeError:
        pass


def emit_event(kind, **fields):
    if os.environ.get('G1_TASK_EVENTS') == '1':
        safe_print(PREFIX + json.dumps(dict(kind=kind, **fields), ensure_ascii=False,
                                      allow_nan=False), flush=True)
