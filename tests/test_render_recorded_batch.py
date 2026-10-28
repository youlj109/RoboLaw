"""CPU decisions/lifecycle tests for persistent offline rendering."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.render_recorded_batch import RenderSceneCache, prepare_manifest, scene_rebuild_reason


def source(static=False):
    return {'spec': {'obstacle_id': 'robot_barrier' if static else 'property_vase',
                     'layout': {'robot_position': [0, 0, 0], 'robot_yaw_deg': 0,
                                'stand_height': 1, 'counter_height': .9,
                                'mustard_position': [1, 2, 3], 'bowl_position': [2, 3, 4],
                                'obstacle': {'category': 'robot' if static else 'property',
                                             'usd_path': 'obstacle.usd', 'static': static,
                                             'position': [1, 2, 3], 'yaw_deg': 0}},
                     'appearance': {'lights': {'light': 1}}},
            'metadata': {'kitchen': {'link_paths': ['kitchen/link']}, 'robot': {'body_names': ['base']},
                         'source_configuration': {'physics_dt': 1 / 120, 'solver_position_iterations': 64}},
            'camera_config': {'camera': {'width': 1280, 'height': 720}}, 'dt': 1 / 15}


class PersistentRecordedRenderTest(unittest.TestCase):
    def test_dynamic_geometry_and_appearance_changes_reuse_scene(self):
        a, b = source(), source()
        b['spec']['layout']['obstacle']['position'][0] += .02
        b['spec']['layout']['mustard_position'][0] += .02
        b['spec']['appearance']['lights']['light'] = 2
        self.assertEqual(scene_rebuild_reason(None, a), 'first_episode')
        self.assertIsNone(scene_rebuild_reason(a, b))

    def test_static_moves_and_topology_or_camera_changes_rebuild(self):
        a, b = source(True), source(True)
        self.assertIsNone(scene_rebuild_reason(a, b))
        b['spec']['layout']['obstacle']['yaw_deg'] = 2
        self.assertEqual(scene_rebuild_reason(a, b), 'static_obstacle_pose_changed')
        for mutate in (lambda d: d['spec'].update(obstacle_id='another_model'),
                       lambda d: d['camera_config']['camera'].update(width=640),
                       lambda d: d['metadata']['source_configuration'].update(physics_dt=1 / 240)):
            changed = deepcopy(a)
            mutate(changed)
            self.assertEqual(scene_rebuild_reason(a, changed), 'topology_or_fixed_configuration_changed')

    def test_same_scene_is_reset_once_and_rebuild_closes_old_scene(self):
        cache = RenderSceneCache()
        counts = {'build': 0, 'reset': 0, 'close': 0}
        class Builder:
            def make_registered(self, cfg, kwargs):
                counts['build'] += 1
                return SimpleNamespace(reset=lambda: counts.update(reset=counts['reset'] + 1))
        def close():
            counts['close'] += 1
            cache.env = cache.kitchen = cache.appearance_layer = None
        cache.close_scene = close
        builder = Builder()
        cache.prepare_source(source())
        first = cache.get_or_create(builder, None, None)
        state = cache.prepare_source(source())
        self.assertTrue(state['scene_reused'])
        self.assertIs(cache.get_or_create(builder, None, None), first)
        self.assertEqual(counts, {'build': 1, 'reset': 1, 'close': 0})
        state = cache.prepare_source(source(True))
        self.assertEqual(state['rebuild_reason'], 'topology_or_fixed_configuration_changed')
        self.assertIsNot(cache.get_or_create(builder, None, None), first)
        self.assertEqual(counts, {'build': 2, 'reset': 2, 'close': 1})

    def test_manifest_preserves_source_and_allows_new_rendered_child(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'source').mkdir()
            path = root / 'manifest.json'
            item = {'source_episode': str(root / 'source'), 'output': str(root / 'rgb')}
            path.write_text(json.dumps([item]))
            self.assertEqual(prepare_manifest(path)[0], item)
            path.write_text(json.dumps([item, item]))
            with self.assertRaises(FileExistsError):
                prepare_manifest(path)
            nested = {**item, 'output': str(root / 'source' / 'rgb')}
            path.write_text(json.dumps([nested]))
            self.assertEqual(prepare_manifest(path), [nested])
            path.write_text(json.dumps([{**item, 'output': str(root / 'source')}]))
            with self.assertRaises(FileExistsError):
                prepare_manifest(path)
            (root / 'rgb').mkdir()
            path.write_text(json.dumps([item]))
            path.write_text(json.dumps([item]))
            with self.assertRaises(FileExistsError):
                prepare_manifest(path)

    def test_dedicated_appearance_layer_clear_preserves_unrelated_session_state(self):
        from pxr import Sdf, Usd, UsdGeom, UsdLux
        stage = Usd.Stage.CreateInMemory()
        light = UsdLux.DistantLight.Define(stage, '/World/Light')
        light.CreateIntensityAttr(100.)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform.Define(stage, '/World/PhysicsUnrelated')
        session_before = stage.GetSessionLayer().ExportToString()
        cache = RenderSceneCache()
        a = {'appearance': {'lights': {'/World/Light': {'intensity_multiplier': .8, 'temperature_k': 4000}}, 'materials': {}}}
        b = deepcopy(a)
        b['appearance']['lights']['/World/Light']['intensity_multiplier'] = 1.3
        first = cache.apply_appearance(stage, a)
        self.assertAlmostEqual(float(light.GetIntensityAttr().Get()), 80.)
        cache.apply_appearance(stage, b)
        self.assertAlmostEqual(float(light.GetIntensityAttr().Get()), 130.)
        self.assertEqual(cache.apply_appearance(stage, a), first)
        self.assertTrue(stage.GetPrimAtPath('/World/PhysicsUnrelated'))
        self.assertEqual(cache.appearance_applications, 3)
        # The session receives only the new sublayer reference, not material/light opinions.
        self.assertNotIn('inputs:intensity', stage.GetSessionLayer().ExportToString())
        self.assertIn('PhysicsUnrelated', session_before)


if __name__ == '__main__':
    unittest.main()
