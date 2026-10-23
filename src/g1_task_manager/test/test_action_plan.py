"""Single-point Action plans use the existing validated mission pipeline."""
from pathlib import Path
import tempfile
import unittest
import yaml

from g1_task_manager.action_plan import action_plan, validate_request
from g1_task_manager.mission_store import load_missions


class ActionPlanTests(unittest.TestCase):
    def test_single_point_is_ephemeral_and_does_not_edit_site_missions(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            points=root/'points.yaml'; missions=root/'missions.yaml'
            points.write_text(yaml.safe_dump(dict(schema_version=1,map_id='test',frame_id='map',base_frame='nav_base',
                waypoints={'point_01':dict(pose=dict(x=1.,y=2.,yaw=.5))})))
            missions.write_text(yaml.safe_dump(dict(schema_version=1,map_id='test',missions={'tour':dict(steps=[dict(waypoint='point_01')])})))
            original=missions.read_bytes()
            store=validate_request('waypoint:point_01',points,missions)
            with action_plan('waypoint:point_01',points,missions,root/'runtime') as (identifier,path):
                step=load_missions(path,store)[identifier]['steps'][0]
                self.assertEqual(step['advance_mode'],'auto')
                self.assertIsNone(step['speech'])
                self.assertEqual(step['dwell_sec'],0)
                self.assertEqual(step['goal']['pose']['yaw'],.5)
                self.assertEqual(missions.read_bytes(),original)
            self.assertFalse(Path(path).exists())
            with self.assertRaises(ValueError):
                validate_request('waypoint:missing',points,missions)
            with action_plan('tour',points,missions,root/'runtime') as plan:
                self.assertEqual(plan,('tour',str(missions)))
