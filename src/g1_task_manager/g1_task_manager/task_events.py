"""Machine-readable worker events, separate from human-readable CLI output."""
import json
import os

PREFIX = 'G1_TASK_EVENT '


def emit_event(kind, **fields):
    if os.environ.get('G1_TASK_EVENTS') == '1':
        print(PREFIX + json.dumps(dict(kind=kind, **fields), ensure_ascii=False,
                                 allow_nan=False), flush=True)
