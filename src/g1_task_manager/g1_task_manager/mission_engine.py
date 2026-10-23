"""顺序执行策略，与ROS传输分离，便于验证失败和停止路径。"""
import time

from .task_events import emit_event


class MissionError(RuntimeError):
    pass


class MissionCanceled(Exception):
    pass


def execute_mission(mission, transport, emit):
    report = {'mission_id': mission['id'], 'state': 'RUNNING', 'steps': [],
              'gate_closed': False, 'goals_terminal': False, 'errors': []}
    started = time.monotonic()
    try:
        transport.close_gate()
        emit_event("ready")  # Cancellation handlers and cleanup scope are active.
        for index, step in enumerate(mission['steps'], 1):
            emit_event('progress', state='CHECKING', current_step=index,
                       total_steps=len(mission['steps']), current_waypoint=step['waypoint'])
            emit('STEP {}/{} {}'.format(index, len(mission['steps']), step['waypoint']))
            record = {'index': index, 'waypoint': step['waypoint'], 'state': 'CHECKING'}
            report['steps'].append(record)
            # 每一段都重新检查TF和定位；检查期间安全门关闭。
            transport.ready()
            record['state'] = 'NAVIGATING'
            leg_start = time.monotonic()
            transport.prepare_goal(step)
            emit_event('progress', state='NAVIGATING', current_step=index,
                       total_steps=len(mission['steps']), current_waypoint=step['waypoint'])
            transport.open_gate()
            status = transport.wait_result(step['timeout_sec'])
            record['elapsed_sec'] = round(time.monotonic() - leg_start, 3)
            record['state'] = status
            # 到达立即关闭安全门，停留和下一段检查都在关门状态下进行。
            transport.close_gate()
            if status != 'SUCCEEDED':
                raise MissionError('{}: {}'.format(step['waypoint'], status))
            emit_event('progress', state='ARRIVED', current_step=index,
                       total_steps=len(mission['steps']), current_waypoint=step['waypoint'])
            emit('ARRIVED {}'.format(step['waypoint']))
            transport.dwell(step['dwell_sec'])
        report['state'] = 'SUCCEEDED'
    except (MissionCanceled, KeyboardInterrupt) as exc:
        report['state'] = 'CANCELED'
        report['errors'].append(str(exc) or '用户取消')
    except Exception as exc:
        report['state'] = 'FAILED'
        report['errors'].append(str(exc))
    finally:
        if report['steps'] and report['steps'][-1]['state'] in ('CHECKING', 'NAVIGATING'):
            report['steps'][-1]['state'] = report['state']
        # 不等Nav2取消回包就先关门；取消仅针对本次任务发送的目标。
        transport.begin_cleanup()
        try:
            transport.close_gate()
            report['gate_closed'] = True
        except Exception as exc:
            report['errors'].append('关门未确认：{}'.format(exc))
        try:
            report['goals_terminal'] = bool(transport.cancel_pending())
            if not report['goals_terminal']:
                report['errors'].append('目标终止未确认，禁止继续下一任务')
        except Exception as exc:
            report['errors'].append('取消未确认：{}'.format(exc))
        # 开门请求若曾超时，再次发送关闭请求，避免只依赖第一次收尾。
        if not report['gate_closed']:
            try:
                transport.close_gate()
                report['gate_closed'] = True
            except Exception as exc:
                report['errors'].append('再次关门未确认：{}'.format(exc))
        if not report['gate_closed'] or not report['goals_terminal']:
            report['state'] = 'STOP_UNCONFIRMED'
        report['elapsed_sec'] = round(time.monotonic() - started, 3)
    return report
