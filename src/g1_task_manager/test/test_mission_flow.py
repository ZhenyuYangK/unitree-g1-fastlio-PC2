"""Offline contract tests for station narration, release, and motion cleanup."""
import contextlib
import copy
import io
import os
import unittest
from unittest.mock import patch

from g1_task_manager.mission_control import MissionControl, MissionProgress
from g1_task_manager.mission_engine import MissionCanceled, MissionError, execute_mission


def tour(stations=2):
    steps = []
    for index in range(1, stations + 1):
        steps.append({
            'waypoint': 'point_{:02d}'.format(index),
            'timeout_sec': 3.0,
            'dwell_sec': 0.2,
            'advance_mode': 'manual',
            'speech': {
                'mode': 'tts',
                'language': 'both',
                'content': {
                    'zh': {'speaker_id': 0, 'text': '中文{}'.format(index), 'wait_sec': 0.4},
                    'en': {'speaker_id': 1, 'text': 'English {}'.format(index), 'wait_sec': 0.6},
                },
                'failure_policy': 'abort',
            },
        })
    return {'id': 'tour', 'name': '实验室参观', 'steps': steps}


class RecordingTransport(MissionProgress):
    """Advance virtual time and handle control callbacks without ROS or sleeping."""
    def __init__(self, on_spin=None, fail_speech=None):
        self.control = MissionControl(simulated=True)
        self.events = []
        self.gate_open = False
        self.virtual_time = 0.0
        self.on_spin = on_spin
        self.fail_speech = fail_speech
        self.spin_count = 0

    def phase(self, state, index, message=''):
        super().phase(state, index, message)
        self.events.append(('phase', state, index))

    def close_gate(self):
        self.gate_open = False
        self.events.append(('close_gate',))

    def ready(self):
        if self.gate_open:
            raise AssertionError('readiness checks require a closed motion gate')
        self.events.append(('ready', self.control.snapshot()['current_step']))

    def prepare_goal(self, step):
        if self.gate_open:
            raise AssertionError('goal preparation requires a closed motion gate')
        self.events.append(('goal', step['waypoint']))

    def open_gate(self):
        self.gate_open = True
        self.events.append(('open_gate',))

    def wait_result(self, timeout):
        self.events.append(('navigation', timeout))
        return 'SUCCEEDED'

    def speak(self, content):
        if self.gate_open:
            raise AssertionError('narration requires a closed motion gate')
        self.events.append(('speech', content['text']))
        if content['speaker_id'] == self.fail_speech:
            raise MissionError('simulated audio request failure')
        self._spin(content['wait_sec'])
        self.events.append(('speech_done', content['text']))

    def dwell(self, seconds):
        if self.gate_open:
            raise AssertionError('dwell requires a closed motion gate')
        self.events.append(('dwell', seconds))
        self._spin(seconds)

    def _spin(self, duration):
        self.spin_count += 1
        if self.spin_count > 10000:
            raise AssertionError('test failed to release or cancel the manual wait')
        self.virtual_time += duration
        if self.on_spin is not None:
            self.on_spin(self)

    def begin_cleanup(self):
        self.events.append(('cleanup',))

    def cancel_pending(self):
        self.events.append(('cancel_navigation',))
        return True

    def finish_mission(self, state):
        super().finish_mission(state)
        self.events.append(('finished', state))


def run(mission, transport):
    with contextlib.redirect_stdout(io.StringIO()):
        return execute_mission(mission, transport, lambda message: None)


def release_current_wait(transport):
    state = transport.control.snapshot()
    if state['state'] == 'WAITING_FOR_CONTINUE':
        accepted, message = transport.control.continue_wait(state['run_id'], state['wait_id'])
        if not accepted:
            raise AssertionError(message)
        transport.events.append(('release', state['current_step']))


class MissionFlowTest(unittest.TestCase):
    def assert_safe_cleanup(self, transport, report, state):
        self.assertEqual(report['state'], state)
        self.assertTrue(report['gate_closed'])
        self.assertTrue(report['goals_terminal'])
        self.assertFalse(transport.gate_open)
        cleanup = transport.events.index(('cleanup',))
        self.assertEqual(transport.events[cleanup:], [
            ('cleanup',), ('phase', 'CLEANING_UP', len(report['steps'])),
            ('close_gate',), ('cancel_navigation',), ('finished', state)])
        self.assertFalse(transport.control.snapshot()['active'])

    def test_failed_audio_preflight_stops_before_any_navigation_goal_or_gate_open(self):
        transport = RecordingTransport()
        with patch.object(transport, 'preflight', side_effect=MissionError('语音预检失败')):
            report = run(tour(), transport)
        self.assert_safe_cleanup(transport, report, 'FAILED')
        self.assertEqual(report['steps'], [])
        self.assertIn('语音预检失败', report['errors'])
        self.assertFalse(any(event[0] in ('goal', 'open_gate') for event in transport.events))

    def test_bilingual_wait_dwell_and_release_precede_next_station_checks(self):
        waits = []

        def continue_when_waiting(transport):
            state = transport.control.snapshot()
            if state['state'] == 'WAITING_FOR_CONTINUE':
                self.assertFalse(transport.gate_open)
                self.assertEqual(sum(event[0] == 'goal' for event in transport.events),
                                 state['current_step'])
                self.assertFalse(any(event[0] == 'finished' for event in transport.events))
                waits.append(state)
                release_current_wait(transport)

        transport = RecordingTransport(on_spin=continue_when_waiting)
        mission = tour()
        report = run(mission, transport)
        self.assert_safe_cleanup(transport, report, 'SUCCEEDED')
        self.assertEqual([step['state'] for step in report['steps']], ['SUCCEEDED'] * 2)
        self.assertEqual(len(waits), 2)
        self.assertEqual(waits[0]['next_waypoint'], 'point_02')
        self.assertEqual(waits[1]['next_waypoint'], '')
        self.assertIn('最后一站', waits[1]['message'])
        self.assertEqual(waits[0]['run_id'], waits[1]['run_id'])
        self.assertNotEqual(waits[0]['wait_id'], waits[1]['wait_id'])

        expected = [('close_gate',)]
        for index, step in enumerate(mission['steps'], 1):
            expected.extend([
                ('phase', 'CHECKING', index), ('ready', index),
                ('goal', step['waypoint']), ('phase', 'NAVIGATING', index),
                ('open_gate',), ('navigation', 3.0), ('close_gate',),
                ('phase', 'ARRIVED', index), ('phase', 'SPEAKING_ZH', index),
                ('speech', '中文{}'.format(index)), ('speech_done', '中文{}'.format(index)),
                ('phase', 'SPEAKING_EN', index), ('speech', 'English {}'.format(index)),
                ('speech_done', 'English {}'.format(index)),
                ('phase', 'DWELLING', index), ('dwell', 0.2),
                ('phase', 'WAITING_FOR_CONTINUE', index), ('release', index),
                ('phase', 'CONTINUING', index),
            ])
        expected.extend([
            ('cleanup',), ('phase', 'CLEANING_UP', 2), ('close_gate',),
            ('cancel_navigation',), ('finished', 'SUCCEEDED')])
        self.assertEqual(transport.events, expected)
        self.assertAlmostEqual(transport.virtual_time, 2 * (0.4 + 0.6 + 0.2 + 0.05))

    def test_manual_wait_outlives_navigation_timeout_without_automatic_departure(self):
        waiting_ticks = 0

        def wait_then_release(transport):
            nonlocal waiting_ticks
            state = transport.control.snapshot()
            if state['state'] != 'WAITING_FOR_CONTINUE':
                return
            waiting_ticks += 1
            self.assertFalse(transport.gate_open)
            self.assertEqual(sum(event[0] == 'goal' for event in transport.events), 1)
            if waiting_ticks < 401:
                self.assertFalse(transport.control.released())
            else:
                self.assertGreater(transport.virtual_time, 20.0)
                raise MissionCanceled('cancel after extended manual wait')

        transport = RecordingTransport(on_spin=wait_then_release)
        report = run(tour(), transport)
        self.assertEqual(waiting_ticks, 401)
        self.assertEqual(len(report['steps']), 1)
        self.assert_safe_cleanup(transport, report, 'CANCELED')

    def test_cancel_during_each_speech_dwell_or_manual_wait_stops_the_mission(self):
        for phase in ('SPEAKING_ZH', 'SPEAKING_EN', 'DWELLING', 'WAITING_FOR_CONTINUE'):
            with self.subTest(phase=phase):
                def cancel(transport):
                    if transport.control.snapshot()['state'] == phase:
                        raise MissionCanceled('operator canceled during ' + phase)

                transport = RecordingTransport(on_spin=cancel)
                report = run(tour(), transport)
                self.assert_safe_cleanup(transport, report, 'CANCELED')
                self.assertEqual(len(report['steps']), 1)
                self.assertEqual(report['steps'][0]['state'], 'CANCELED')
                self.assertFalse(any(event == ('goal', 'point_02') for event in transport.events))

    def test_either_language_failure_aborts_before_manual_wait_or_next_navigation(self):
        for speaker in (0, 1):
            with self.subTest(speaker=speaker):
                transport = RecordingTransport(on_spin=release_current_wait, fail_speech=speaker)
                report = run(tour(), transport)
                self.assert_safe_cleanup(transport, report, 'FAILED')
                self.assertEqual(len(report['steps']), 1)
                self.assertIn('audio request failure', report['errors'][0])
                self.assertFalse(any(event[0] in ('dwell', 'release') for event in transport.events))
                self.assertFalse(any(event == ('phase', 'WAITING_FOR_CONTINUE', 1)
                                     for event in transport.events))
                self.assertFalse(any(event == ('goal', 'point_02') for event in transport.events))
                speech_events = [event for event in transport.events if event[0] == 'speech']
                self.assertEqual(len(speech_events), speaker + 1)

    def test_single_language_only_speaks_its_selected_content(self):
        for language, expected in (('zh', '中文1'), ('en', 'English 1')):
            with self.subTest(language=language):
                mission = tour(1)
                speech = mission['steps'][0]['speech']
                speech['language'] = language
                speech['content'] = {language: speech['content'][language]}
                transport = RecordingTransport(on_spin=release_current_wait)
                report = run(mission, transport)
                self.assertEqual(report['state'], 'SUCCEEDED')
                self.assertEqual([event for event in transport.events if event[0] == 'speech'],
                                 [('speech', expected)])

    def test_stale_previous_station_release_does_not_release_next_station(self):
        previous = None

        def release_with_delayed_duplicate(transport):
            nonlocal previous
            state = transport.control.snapshot()
            if state['state'] != 'WAITING_FOR_CONTINUE':
                return
            if previous is not None:
                accepted, _ = transport.control.continue_wait(*previous)
                self.assertFalse(accepted)
                self.assertFalse(transport.control.released())
                self.assertEqual(transport.control.snapshot()['state'], 'WAITING_FOR_CONTINUE')
            previous = (state['run_id'], state['wait_id'])
            release_current_wait(transport)
            accepted, _ = transport.control.continue_wait(*previous)
            self.assertFalse(accepted)

        transport = RecordingTransport(on_spin=release_with_delayed_duplicate)
        report = run(tour(), transport)
        self.assert_safe_cleanup(transport, report, 'SUCCEEDED')
        self.assertEqual([event for event in transport.events if event[0] == 'release'],
                         [('release', 1), ('release', 2)])

    def test_legacy_steps_without_speech_or_advance_mode_keep_timed_auto_flow(self):
        mission = copy.deepcopy(tour())
        for step in mission['steps']:
            del step['speech']
            del step['advance_mode']
        transport = RecordingTransport()
        report = run(mission, transport)
        self.assert_safe_cleanup(transport, report, 'SUCCEEDED')
        self.assertEqual(len(report['steps']), 2)
        self.assertEqual([event for event in transport.events if event[0] == 'dwell'],
                         [('dwell', 0.2), ('dwell', 0.2)])
        self.assertFalse(any(event[0] in ('speech', 'release') for event in transport.events))
        self.assertFalse(any(event[:2] == ('phase', 'WAITING_FOR_CONTINUE')
                             for event in transport.events))

    def test_disconnected_output_does_not_interrupt_waiting_or_motion_cleanup(self):
        transport = RecordingTransport(on_spin=release_current_wait)
        with patch.dict(os.environ, {'G1_TASK_EVENTS': '1'}), patch(
                'builtins.print', side_effect=BrokenPipeError('terminal disconnected')):
            report = run(tour(), transport)
        self.assert_safe_cleanup(transport, report, 'SUCCEEDED')
        self.assertEqual([event for event in transport.events if event[0] == 'release'],
                         [('release', 1), ('release', 2)])

    def test_cleanup_feedback_failure_still_closes_gate_and_cancels_navigation(self):
        transport = RecordingTransport(on_spin=release_current_wait)
        original_phase = transport.phase

        def fail_cleanup_phase(state, index, message=''):
            if state == 'CLEANING_UP':
                transport.events.append(('phase', state, index))
                raise BrokenPipeError('feedback pipe closed')
            return original_phase(state, index, message)

        with patch.object(transport, 'phase', side_effect=fail_cleanup_phase):
            report = run(tour(), transport)
        self.assert_safe_cleanup(transport, report, 'SUCCEEDED')
        self.assertTrue(any('收尾状态反馈失败' in error for error in report['errors']))

    def test_audio_cancellation_acknowledgements_wait_until_motion_cleanup_is_complete(self):
        transport = RecordingTransport(fail_speech=0)

        def finish_audio_cleanup():
            self.assertFalse(transport.gate_open)
            self.assertEqual(transport.events[-1], ('cancel_navigation',))
            return ['simulated audio cancellation warning']

        with patch.object(transport, 'finish_cleanup', side_effect=finish_audio_cleanup) as cleanup:
            report = run(tour(), transport)
        cleanup.assert_called_once_with()
        self.assert_safe_cleanup(transport, report, 'FAILED')
        self.assertIn('simulated audio cancellation warning', report['errors'])


if __name__ == '__main__':
    unittest.main()
