import copy
from pathlib import Path
import tempfile
import unittest

import yaml

from g1_task_manager.mission_store import load_missions
from g1_task_manager.waypoint_store import WaypointConfigError, WaypointStore


class MissionStoreTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='g1_mission_store_test_')
        self.addCleanup(self.directory.cleanup)
        directory = Path(self.directory.name)
        waypoint_path = directory / 'waypoints.yaml'
        waypoint_path.write_text(yaml.safe_dump({
            'schema_version': 1,
            'map_id': 'laboratory_map_v1',
            'frame_id': 'map',
            'base_frame': 'nav_base',
            'waypoints': {
                'point_{:02d}'.format(index): {
                    'pose': {'x': index, 'y': 0.0, 'yaw': 0.0},
                } for index in range(1, 6)
            },
        }), encoding='utf-8')
        self.waypoints = WaypointStore(waypoint_path).load()
        self.mission_path = directory / 'missions.yaml'
        self.data = {
            'schema_version': 1,
            'map_id': 'laboratory_map_v1',
            'missions': {'tour': {'steps': [{'waypoint': 'point_01'}]}},
        }
        self.speech = {
            'mode': 'tts',
            'language': 'both',
            'content': {
                'zh': {'speaker_id': 0, 'text': ' 欢迎参观。 ', 'wait_sec': 12},
                'en': {'speaker_id': 1, 'text': ' Welcome. ', 'wait_sec': 15},
            },
            'failure_policy': 'abort',
        }

    def load(self, data=None):
        self.mission_path.write_text(
            yaml.safe_dump(self.data if data is None else data, allow_unicode=True),
            encoding='utf-8')
        return load_missions(self.mission_path, self.waypoints)

    def speech_data(self):
        data = copy.deepcopy(self.data)
        step = data['missions']['tour']['steps'][0]
        step['speech'] = copy.deepcopy(self.speech)
        step['advance_mode'] = 'manual'
        return data

    def test_legacy_task_defaults_to_timed_auto_without_speech(self):
        mission = self.load()['tour']
        step = mission['steps'][0]
        self.assertEqual(mission['name'], 'tour')
        self.assertEqual(step['timeout_sec'], 60.0)
        self.assertEqual(step['dwell_sec'], 2.0)
        self.assertEqual(step['advance_mode'], 'auto')
        self.assertIsNone(step['speech'])
        self.assertEqual(step['goal'], self.waypoints.get_waypoint('point_01'))

    def test_bilingual_speech_preserves_document_structure(self):
        step = self.load(self.speech_data())['tour']['steps'][0]
        self.assertEqual(step['advance_mode'], 'manual')
        speech = step['speech']
        self.assertEqual(set(speech), {'mode', 'language', 'content', 'failure_policy'})
        self.assertEqual(speech['language'], 'both')
        self.assertEqual(speech['content']['zh'], {
            'speaker_id': 0, 'text': '欢迎参观。', 'wait_sec': 12.0})
        self.assertEqual(speech['content']['en'], {
            'speaker_id': 1, 'text': 'Welcome.', 'wait_sec': 15.0})

    def test_single_language_requires_only_selected_content(self):
        for language in ('zh', 'en'):
            with self.subTest(language=language):
                data = self.speech_data()
                speech = data['missions']['tour']['steps'][0]['speech']
                speech['language'] = language
                speech['content'] = {language: speech['content'][language]}
                result = self.load(data)['tour']['steps'][0]['speech']
                self.assertEqual(set(result['content']), {language})

    def test_unselected_language_may_be_incomplete(self):
        data = self.speech_data()
        speech = data['missions']['tour']['steps'][0]['speech']
        speech['language'] = 'zh'
        speech['content']['en'] = {'text': 'For a later English tour.'}
        result = self.load(data)['tour']['steps'][0]['speech']
        self.assertEqual(result['content']['en'], speech['content']['en'])

    def test_abort_is_safe_default_and_manual_without_speech_is_valid(self):
        data = self.speech_data()
        del data['missions']['tour']['steps'][0]['speech']['failure_policy']
        self.assertEqual(self.load(data)['tour']['steps'][0]['speech']['failure_policy'], 'abort')
        del data['missions']['tour']['steps'][0]['speech']
        step = self.load(data)['tour']['steps'][0]
        self.assertEqual(step['advance_mode'], 'manual')
        self.assertIsNone(step['speech'])

    def test_invalid_modes_and_failure_policies_are_rejected(self):
        for key, values in (
                ('mode', ('wav', None, True, {}, [])),
                ('language', ('ZH', 'fr', None, {}, [])),
                ('failure_policy', ('skip', 'continue', None, False, {}, []))):
            for value in values:
                with self.subTest(key=key, value=value):
                    data = self.speech_data()
                    data['missions']['tour']['steps'][0]['speech'][key] = value
                    with self.assertRaises(WaypointConfigError):
                        self.load(data)
        for value in ('automatic', None, True, {}, []):
            with self.subTest(advance_mode=value):
                data = copy.deepcopy(self.data)
                data['missions']['tour']['steps'][0]['advance_mode'] = value
                with self.assertRaises(WaypointConfigError):
                    self.load(data)

    def test_speaker_id_is_nonnegative_int32(self):
        for value in (-1, 2147483648, 10 ** 1000, True, False, 0.0, '0', None):
            with self.subTest(value=value):
                data = self.speech_data()
                data['missions']['tour']['steps'][0]['speech']['content']['zh']['speaker_id'] = value
                with self.assertRaises(WaypointConfigError):
                    self.load(data)
        data = self.speech_data()
        data['missions']['tour']['steps'][0]['speech']['content']['zh']['speaker_id'] = 2147483647
        self.assertEqual(self.load(data)['tour']['steps'][0]['speech']['content']['zh']['speaker_id'],
                         2147483647)

    def test_speech_wait_is_finite_positive_and_text_nonempty(self):
        for key, values in (
                ('wait_sec', (0, -1, float('inf'), float('-inf'), float('nan'),
                              True, None, '12', 10 ** 1000)),
                ('text', ('', ' \n\t ', None, 12, False, [], {}))):
            for value in values:
                with self.subTest(key=key, value=value):
                    data = self.speech_data()
                    data['missions']['tour']['steps'][0]['speech']['content']['zh'][key] = value
                    with self.assertRaises(WaypointConfigError):
                        self.load(data)

    def test_selected_content_requires_every_field(self):
        for language in ('zh', 'en'):
            for key in ('speaker_id', 'text', 'wait_sec'):
                with self.subTest(language=language, key=key):
                    data = self.speech_data()
                    del data['missions']['tour']['steps'][0]['speech']['content'][language][key]
                    with self.assertRaises(WaypointConfigError):
                        self.load(data)
            data = self.speech_data()
            del data['missions']['tour']['steps'][0]['speech']['content'][language]
            with self.assertRaises(WaypointConfigError):
                self.load(data)

    def test_speech_content_must_be_mappings(self):
        for path in (('speech',), ('speech', 'content'), ('speech', 'content', 'zh')):
            for value in (None, [], 'invalid'):
                with self.subTest(path=path, value=value):
                    data = self.speech_data()
                    target = data['missions']['tour']['steps'][0]
                    for key in path[:-1]:
                        target = target[key]
                    target[path[-1]] = value
                    with self.assertRaises(WaypointConfigError):
                        self.load(data)

    def test_unknown_fields_are_rejected_at_every_level(self):
        for path in ((), ('missions', 'tour'), ('missions', 'tour', 'steps', 0),
                     ('missions', 'tour', 'steps', 0, 'speech'),
                     ('missions', 'tour', 'steps', 0, 'speech', 'content'),
                     ('missions', 'tour', 'steps', 0, 'speech', 'content', 'zh'),
                     ('missions', 'tour', 'steps', 0, 'speech', 'content', 'en')):
            with self.subTest(path=path):
                data = self.speech_data()
                target = data
                for key in path:
                    target = target[key]
                target['typo'] = 'not supported'
                with self.assertRaises(WaypointConfigError):
                    self.load(data)

    def test_duplicate_speech_keys_are_rejected(self):
        self.mission_path.write_text('''schema_version: 1
map_id: laboratory_map_v1
missions:
  tour:
    steps:
      - waypoint: point_01
        speech:
          mode: tts
          language: zh
          language: en
''', encoding='utf-8')
        with self.assertRaisesRegex(WaypointConfigError, '重复键'):
            load_missions(self.mission_path, self.waypoints)

    def test_later_invalid_step_rejects_entire_mission(self):
        data = self.speech_data()
        data['missions']['tour']['steps'].append({
            'waypoint': 'point_02', 'advance_mode': 'invalid'})
        with self.assertRaises(WaypointConfigError):
            self.load(data)

    def test_tracked_tour_example_loads_all_five_manual_stations(self):
        example = Path(__file__).resolve().parents[1] / 'config' / 'tour_missions.example.yaml'
        steps = load_missions(example, self.waypoints)['five_point_tour']['steps']
        self.assertEqual(len(steps), 5)
        self.assertTrue(all(step['advance_mode'] == 'manual' for step in steps))
        self.assertTrue(all(step['speech']['language'] == 'both' for step in steps))


if __name__ == '__main__':
    unittest.main()
