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
        transport.start_mission(mission)
        transport.close_gate()
        emit_event("ready")  # Cancellation handlers and cleanup scope are active.
        transport.preflight(mission)
        for index, step in enumerate(mission['steps'], 1):
            transport.phase('CHECKING', index)
            emit('STEP {}/{} {}'.format(index, len(mission['steps']), step['waypoint']))
            record = {'index': index, 'waypoint': step['waypoint'], 'state': 'CHECKING'}
            report['steps'].append(record)
            # 每一段都重新检查TF和定位；检查期间安全门关闭。
            transport.ready()
            record['state'] = 'NAVIGATING'
            leg_start = time.monotonic()
            transport.prepare_goal(step)
            transport.phase('NAVIGATING', index)
            transport.open_gate()
            status = transport.wait_result(step['timeout_sec'])
            record['navigation_state'] = status
            record['elapsed_sec'] = round(time.monotonic() - leg_start, 3)
            record['state'] = status
            # 到达立即关闭安全门，停留和下一段检查都在关门状态下进行。
            transport.close_gate()
            if status != 'SUCCEEDED':
                raise MissionError('{}: {}'.format(step['waypoint'], status))
            transport.phase('ARRIVED', index)
            emit('ARRIVED {}'.format(step['waypoint']))
            speech = step.get('speech')
            if speech:
                languages = ('zh', 'en') if speech['language'] == 'both' else (speech['language'],)
                for language in languages:
                    record['state'] = 'SPEAKING_' + language.upper()
                    transport.phase(record['state'], index,
                                    '播报并等待配置时长；无硬件播放完成回调')
                    emit('{} {}'.format(record['state'], step['waypoint']))
                    transport.speak(speech['content'][language])
            record['state'] = 'DWELLING'
            transport.phase('DWELLING', index)
            transport.dwell(step['dwell_sec'])
            if step.get('advance_mode', 'auto') == 'manual':
                record['state'] = 'WAITING_FOR_CONTINUE'
                transport.wait_for_continue(index)
            record['state'] = 'SUCCEEDED'
        report['state'] = 'SUCCEEDED'
    except (MissionCanceled, KeyboardInterrupt) as exc:
        report['state'] = 'CANCELED'
        report['errors'].append(str(exc) or '用户取消')
    except Exception as exc:
        report['state'] = 'FAILED'
        report['errors'].append(str(exc))
    finally:
        # Disable signal interruption before any feedback/logging in cleanup.
        transport.begin_cleanup()
        if report['steps'] and report['steps'][-1]['state'] != 'SUCCEEDED':
            report['steps'][-1]['state'] = report['state']
        try:
            transport.phase('CLEANING_UP', len(report['steps']))
        except Exception as exc:
            report['errors'].append('收尾状态反馈失败：{}'.format(exc))
        # 不等Nav2取消回包就先关门；取消仅针对本次任务发送的目标。
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
        # Audio acceptance/cancel acknowledgements are drained only AFTER all
        # motion cleanup, so a slow voice bridge cannot delay stopping motion.
        try:
            report['errors'].extend(transport.finish_cleanup())
        except Exception as exc:
            report['errors'].append('语音停止请求未确认：{}'.format(exc))
        transport.finish_mission(report['state'])
        report['elapsed_sec'] = round(time.monotonic() - started, 3)
    return report
