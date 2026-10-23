"""Supervise the existing CLI without a shell; cancel only after handlers are ready."""
import json
import os
import selectors
import signal
import subprocess

from .task_events import PREFIX


def run_worker(command, cancel, on_event, on_log):
    env = dict(os.environ, G1_TASK_EVENTS='1', PYTHONUNBUFFERED='1')
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, start_new_session=True,
                               env=env)
    ready = False
    interrupted = False
    report = None
    pending = b''
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            while selector.get_map():
                # Before READY there can be no motion. Waiting avoids killing the
                # worker between startup and installing its safe signal handler.
                if cancel.is_set() and ready and report is None and not interrupted:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    interrupted = True
                for key, _ in selector.select(timeout=0.05):
                    chunk = os.read(key.fileobj.fileno(), 65536)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        chunk = b'\n' if pending else b''
                    pending += chunk
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        line = line.decode('utf-8', errors='replace')
                        if line.startswith(PREFIX):
                            data = json.loads(line[len(PREFIX):])
                            if data['kind'] == 'ready':
                                ready = True
                            elif data['kind'] == 'result':
                                report = data
                            on_event(data)
                        elif line:
                            on_log(line)
            rc = process.wait()
        finally:
            # Never abandon a live motion worker if parsing/publishing raises.
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                process.wait()
            process.stdout.close()
    if report is None:
        return {'state': 'STOP_UNCONFIRMED', 'gate_closed': False,
                'goals_terminal': False, 'report_path': '',
                'errors': ['Worker exited without a final report (rc={})'.format(rc)]}
    if report['state'] in ('SUCCEEDED', 'CHECKED') and rc != 0:
        report = dict(report, state='STOP_UNCONFIRMED',
                      errors=['Worker exit code conflicts with final report: {}'.format(rc)])
    return report
