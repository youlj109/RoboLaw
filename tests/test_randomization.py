"""Sampler checks run on CPU; USD checks use Arena Python without starting Kit."""
import copy
import math
import unittest

from kitchen_safety.obstacle_library import resolve_layout
from kitchen_safety.randomization import (
    HUMAN_IDS, NONHUMAN_IDS, SUPPORTED_IDS, ROOT,
    apply_appearance, load_randomization_config, sample_episode,
)

try:
    from pxr import Gf, Sdf, Usd, UsdLux, UsdShade
except ImportError:
    Usd = None


class EpisodeSamplingTests(unittest.TestCase):
    def setUp(self):
        self.config = load_randomization_config()

    def test_repeat_subset_order_and_resume_are_identical(self):
        all_samples = {name: sample_episode(name, self.config, 42, 6) for name in NONHUMAN_IDS}
        for name in reversed(NONHUMAN_IDS[2:8]):
            self.assertEqual(sample_episode(name, self.config, 42, 6), all_samples[name])
        self.assertNotEqual(sample_episode(NONHUMAN_IDS[0], self.config, 42, 5),
                            all_samples[NONHUMAN_IDS[0]])

    def test_geometry_appearance_and_planner_random_streams_are_independent(self):
        base = sample_episode(NONHUMAN_IDS[0], self.config, 42, 0)
        changed = copy.deepcopy(self.config)
        changed['materials']['roughness'] = [0.1, 0.2]
        changed['lighting']['temperature_k'] = [2000.0, 3000.0]
        modified = sample_episode(NONHUMAN_IDS[0], changed, 42, 0)
        self.assertEqual(base['layout'], modified['layout'])
        self.assertEqual(base['seeds'], modified['seeds'])
        self.assertNotEqual(base['appearance'], modified['appearance'])
        self.assertEqual(len(set(base['seeds'].values())), 3)

    def test_ranges_support_and_shared_obstacle_pose(self):
        for name in NONHUMAN_IDS:
            original = resolve_layout(name)
            for attempt in range(30):
                sample = sample_episode(name, self.config, 42, attempt)
                layout, scale = sample['layout'], sample['range_scale']
                for object_name in ('mustard', 'bowl'):
                    key = object_name + '_position'
                    self.assertEqual(layout[key][2], original[key][2])
                    for axis in (0, 1):
                        self.assertLessEqual(abs(layout[key][axis] - original[key][axis]), 0.02 * scale)
                    self.assertLessEqual(abs(layout[object_name + '_yaw_deg']), 10.0 * scale)
                obstacle = layout['obstacle']
                self.assertEqual(obstacle['position'][2], original['obstacle']['position'][2])
                for axis in (0, 1):
                    self.assertLessEqual(abs(obstacle['position'][axis] - original['obstacle']['position'][axis]), 0.01 * scale)
                self.assertLessEqual(abs(obstacle['yaw_deg'] - original['obstacle'].get('yaw_deg', 0)), 5.0 * scale)
                category = obstacle['category']
                field = 'glass' if category == 'property' else 'barrier'
                self.assertEqual(obstacle['position'], layout[category][field + '_position'])
                self.assertEqual(obstacle['yaw_deg'], layout[category][field + '_yaw_deg'])
                for key in ('camera_eye', 'camera_target', 'robot_position', 'robot_yaw_deg', 'counter_height'):
                    self.assertEqual(layout[key], original[key])

    def test_config_and_sources_not_mutated(self):
        before = copy.deepcopy(self.config)
        old_layout = resolve_layout(NONHUMAN_IDS[0])
        sample_episode(NONHUMAN_IDS[0], self.config, 5, 1)
        self.assertEqual(self.config, before)
        self.assertEqual(resolve_layout(NONHUMAN_IDS[0]), old_layout)

    def test_invalid_config_and_ids_fail_explicitly(self):
        for identifier, seed, attempt in [('unknown_human_model', 42, 0),
                                           (NONHUMAN_IDS[0], -1, 0),
                                           (NONHUMAN_IDS[0], 42, -1)]:
            with self.assertRaises(ValueError):
                sample_episode(identifier, self.config, seed, attempt)
        for value in (-0.01, float('nan'), True):
            config = copy.deepcopy(self.config)
            config['objects']['xy_m'] = value
            with self.assertRaises(ValueError):
                sample_episode(NONHUMAN_IDS[0], config, 42, 0)

    def test_humans_append_without_renumbering_existing_ids(self):
        self.assertEqual(SUPPORTED_IDS[:10], NONHUMAN_IDS)
        self.assertEqual(len(SUPPORTED_IDS), len(set(SUPPORTED_IDS)))
        self.assertEqual(SUPPORTED_IDS[10:], HUMAN_IDS)
        self.assertEqual(len(HUMAN_IDS), 7)
        original = resolve_layout()
        registered = resolve_layout('human_default_mpfb')
        self.assertEqual(registered['human'], original['human'])
        self.assertTrue((ROOT / registered['obstacle']['usd_path']).is_file())

    def test_human_pose_keypoints_seeds_and_physics_remain_consistent(self):
        config = load_randomization_config(ROOT / 'configs/augmentation_human7.json')
        for identifier in HUMAN_IDS:
            original = resolve_layout(identifier)
            samples = {i: sample_episode(identifier, config, 42, i) for i in (0, 1, 6)}
            for attempt in reversed(samples):
                spec = sample_episode(identifier, config, 42, attempt)
                self.assertEqual(spec, samples[attempt])
                layout, obstacle = spec['layout'], spec['layout']['obstacle']
                self.assertEqual(layout['human']['position'], obstacle['position'])
                self.assertEqual(layout['human']['yaw_deg'], obstacle['yaw_deg'])
                self.assertEqual(layout['human']['asset'], obstacle['usd_path'])
                self.assertEqual(layout['robot'], original['robot'])
                self.assertEqual(layout['planning'], original['planning'])
                self.assertTrue(obstacle['static'])
                self.assertEqual(obstacle['position'][2], original['obstacle']['position'][2])
                for axis in (0, 1):
                    self.assertLessEqual(abs(obstacle['position'][axis] - original['obstacle']['position'][axis]), .01)
                self.assertLessEqual(abs(obstacle['yaw_deg'] - original['obstacle']['yaw_deg']), 5.)
                cosine = math.cos(math.radians(obstacle['yaw_deg']))
                sine = math.sin(math.radians(obstacle['yaw_deg']))
                for key, local in original['obstacle']['keypoints'].items():
                    if not key.endswith('_local'):
                        continue
                    self.assertEqual(obstacle['keypoints'][key], local)
                    world = obstacle['keypoints'][key[:-6] + '_world']
                    x, y, z = [world[i] - obstacle['position'][i] for i in range(3)]
                    self.assertAlmostEqual(cosine*x+sine*y, local[0])
                    self.assertAlmostEqual(-sine*x+cosine*y, local[1])
                    self.assertAlmostEqual(z, local[2])
            self.assertEqual(resolve_layout(identifier), original)

    def test_human_range_override_is_supported_and_sources_not_mutated(self):
        config = load_randomization_config(ROOT / 'configs/augmentation_human7.json')
        config['range_scale_by_id'][HUMAN_IDS[0]] = .5
        before = copy.deepcopy(config)
        spec = sample_episode(HUMAN_IDS[0], config, 42, 0)
        self.assertEqual(spec['range_scale'], .5)
        self.assertEqual(config, before)


@unittest.skipIf(Usd is None, 'Run with scripts/arena_python.sh for USD checks')
class AppearanceTests(unittest.TestCase):
    def make_stage(self, obstacle_id):
        stage = Usd.Stage.CreateInMemory()
        asset = resolve_layout(obstacle_id)['obstacle']
        for name, filename in (
            ('kitchen', 'assets/kitchen/scene.usd'),
            ('mustard_bottle', 'assets/robolab/objects/hope/mustard_bottle.usd'),
            ('bowl', 'assets/robolab/objects/ycb/bowl.usd'),
            ({'property': 'glass', 'robot': 'barrier', 'human': 'person'}[asset['category']], asset['usd_path']),
        ):
            stage.DefinePrim('/World/envs/env_0/' + name, 'Xform').GetReferences().AddReference(str(ROOT / filename))
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1500.0)
        UsdLux.DistantLight.Define(stage, '/World/DirectionalLight').CreateIntensityAttr(1000.0)
        return stage

    def test_session_only_changes_visual_inputs_and_preserves_protected_materials(self):
        config = load_randomization_config()
        for name in ('property_glass_bottle', 'property_ceramic_mug', 'robot_projecting_crossbar'):
            stage = self.make_stage(name)
            root_before = stage.GetRootLayer().ExportToString()
            source_layers = {layer.identifier: layer.ExportToString() for layer in stage.GetUsedLayers()
                             if layer != stage.GetSessionLayer()}
            sample = sample_episode(name, config, 42, 0)
            report = apply_appearance(stage, sample)
            self.assertEqual(root_before, stage.GetRootLayer().ExportToString())
            for layer in stage.GetUsedLayers():
                if layer.identifier in source_layers:
                    self.assertEqual(source_layers[layer.identifier], layer.ExportToString())
            self.assertEqual(len(report['lights']), 2)
            for light in report['lights']:
                self.assertAlmostEqual(light['intensity'], light['baseline_intensity'] * light['intensity_multiplier'], places=3)
                self.assertTrue(light['enable_color_temperature'])
            for group in ('kitchen_wall', 'kitchen_counter', 'kitchen_cabinet', 'bowl', 'obstacle'):
                self.assertGreater(report['material_counts'][group], 0)
            for material in report['materials']:
                self.assertNotIn('/mustard_bottle/', material['path'])
                for protected in ('Label', 'AmberGlass', 'Warning', 'Safety', 'ControlPanel', 'StopButton'):
                    self.assertNotIn(protected, material['path'])
            session_text = stage.GetSessionLayer().ExportToString()
            for forbidden in ('physics:', 'physx', 'focalLength', 'xformOp:', 'diffuse_texture', 'opacity', 'metallic'):
                self.assertNotIn(forbidden, session_text)

    def test_glass_and_wine_get_no_obstacle_material_edit(self):
        for identifier in ('property_tall_glass', 'property_wine_glass'):
            report = apply_appearance(self.make_stage(identifier),
                                      sample_episode(identifier, load_randomization_config(), 42, 0))
            self.assertEqual(report['material_counts']['obstacle'], 0)

    def test_human_visual_and_collision_sources_are_not_changed(self):
        identifier = 'human_default_mpfb'
        stage = self.make_stage(identifier)
        source_layers = {layer.identifier: layer.ExportToString() for layer in stage.GetUsedLayers()
                         if layer != stage.GetSessionLayer()}
        report = apply_appearance(stage, sample_episode(identifier,
            load_randomization_config(ROOT / 'configs/augmentation_human7.json'), 42, 0))
        self.assertEqual(report['material_counts']['obstacle'], 0)
        self.assertEqual(len(report['lights']), 2)
        self.assertGreater(report['material_counts']['kitchen_wall'], 0)
        self.assertGreater(report['material_counts']['bowl'], 0)
        self.assertTrue(all('/person/' not in item['path'] for item in report['materials']))
        for layer in stage.GetUsedLayers():
            if layer.identifier in source_layers:
                self.assertEqual(layer.ExportToString(), source_layers[layer.identifier])
        self.assertNotIn('physics:', stage.GetSessionLayer().ExportToString())

    def test_owned_layer_reuse_does_not_accumulate_or_clear_other_session_state(self):
        stage = Usd.Stage.CreateInMemory()
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1500.0)
        UsdLux.DistantLight.Define(stage, '/World/DirectionalLight').CreateIntensityAttr(1000.0)
        UsdShade.Material.Define(stage, '/World/envs/env_0/kitchen/WallPaint')
        shader = UsdShade.Shader.Define(stage, '/World/envs/env_0/kitchen/WallPaint/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.3, .4, .5))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.7)
        session = stage.GetSessionLayer()
        with Usd.EditContext(stage, session):
            unrelated = stage.DefinePrim('/World/unrelated')
            unrelated.CreateAttribute('physics:mass', Sdf.ValueTypeNames.Float).Set(2.5)
            unrelated.CreateAttribute('xformOp:translate', Sdf.ValueTypeNames.Double3).Set(Gf.Vec3d(1, 2, 3))
        appearance_layer = Sdf.Layer.CreateAnonymous('episode_appearance.usda')
        session.subLayerPaths.insert(0, appearance_layer.identifier)
        session_before = session.ExportToString()
        root_before = stage.GetRootLayer().ExportToString()
        config = load_randomization_config()
        samples = [sample_episode('property_vase', config, 42, attempt) for attempt in (0, 1)]

        def apply(sample):
            # Clearing a dedicated layer removes also newly-created properties,
            # instead of trying to invert clipped colors or a multiplier.
            appearance_layer.Clear()
            self.assertEqual(shader.GetInput('diffuseColor').Get(), Gf.Vec3f(.3, .4, .5))
            self.assertAlmostEqual(shader.GetInput('roughness').Get(), .7, places=6)
            self.assertEqual(UsdLux.LightAPI(stage.GetPrimAtPath('/World/Light')).GetIntensityAttr().Get(), 1500.)
            result = apply_appearance(stage, sample, edit_layer=appearance_layer)
            self.assertEqual(session.ExportToString(), session_before)
            self.assertEqual(stage.GetRootLayer().ExportToString(), root_before)
            self.assertEqual(unrelated.GetAttribute('physics:mass').Get(), 2.5)
            self.assertEqual(unrelated.GetAttribute('xformOp:translate').Get(), Gf.Vec3d(1, 2, 3))
            self.assertEqual(stage.GetEditTarget().GetLayer(), stage.GetRootLayer())
            return result

        first = apply(samples[0])
        second = apply(samples[1])
        repeated = apply(samples[0])
        self.assertEqual(first, repeated)
        self.assertNotEqual(first['lights'], second['lights'])
        self.assertNotEqual(first['materials'], second['materials'])
        self.assertEqual(first['edit_layer'], 'caller_supplied')
        self.assertEqual(len(first['materials']), 1)
        for result in (first, second, repeated):
            self.assertEqual([light['baseline_intensity'] for light in result['lights']], [1500., 1000.])
            self.assertEqual(result['materials'][0]['baseline_color'], list(Gf.Vec3f(.3, .4, .5)))


if __name__ == '__main__':
    unittest.main()
