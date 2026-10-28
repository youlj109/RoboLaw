"""CPU geometry/default-state and real USD scoping checks, without Kit."""
import copy
import math
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import torch
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

from kitchen_safety.randomization import load_randomization_config, sample_episode
from kitchen_safety.vectorized_scene import (
    apply_vectorized_appearance, author_static_poses, configure_vectorized_geometry,
    geometry_parameters, reset_vectorized_objects,
)


class VectorizedSceneTests(unittest.TestCase):
    def specs(self, identifier='property_vase', count=2):
        config = load_randomization_config()
        return [sample_episode(identifier, config, 42, index) for index in range(count)]

    def test_static_variation_changes_replication_but_dynamic_and_identical_static_do_not(self):
        cases = [(self.specs(), True), (self.specs('robot_narrow_opening'), False)]
        same = self.specs('robot_narrow_opening', 1)[0]
        cases.append(([same, copy.deepcopy(same)], True))
        for specs, expected in cases:
            cfg = SimpleNamespace(scene=SimpleNamespace(num_envs=2, replicate_physics=True),
                                  events=SimpleNamespace(mustard_bottle='template', bowl='template'))
            fake = SimpleNamespace(EventTermCfg=lambda **kwargs: SimpleNamespace(**kwargs))
            with patch.dict(sys.modules, {'isaaclab.managers': fake}):
                manifest = configure_vectorized_geometry(cfg, specs)
            self.assertEqual(cfg.scene.replicate_physics, expected)
            self.assertEqual(manifest['replicate_physics_after'], expected)
            self.assertIsNone(cfg.events.mustard_bottle)
            self.assertEqual(cfg.events.vectorized_object_poses.mode, 'reset')
            if not expected:
                self.assertEqual(cfg.events.vectorized_static_geometry.mode, 'prestartup')

    def test_partial_reset_uses_absolute_env_ids_and_local_default_buffers(self):
        class Scene(dict):
            num_envs = 3
            env_origins = torch.tensor([[0., 0., 0.], [30., 0., 0.], [60., 5., 0.]])
        class Asset:
            def __init__(self):
                self.data = SimpleNamespace(default_root_pose=torch.zeros(3, 7), default_root_vel=torch.ones(3, 6))
            def write_root_pose_to_sim_index(self, root_pose, env_ids):
                self.pose, self.ids = root_pose.clone(), env_ids.clone()
            def write_root_velocity_to_sim_index(self, root_velocity, env_ids):
                self.velocity = root_velocity.clone()
            def reset(self, env_ids):
                self.reset_ids = env_ids.clone()
        tables = geometry_parameters(self.specs(count=3))['poses_local_xyz_xyzw']
        scene = Scene({name: Asset() for name in tables})
        env = SimpleNamespace(scene=scene, device='cpu')
        ids = torch.tensor([2, 0])
        reset_vectorized_objects(env, ids, tables)
        for name, values in tables.items():
            local = torch.tensor(values)
            expected = local[ids].clone()
            expected[:, :3] += scene.env_origins[ids]
            torch.testing.assert_close(scene[name].pose, expected)
            torch.testing.assert_close(scene[name].data.default_root_pose[ids], local[ids])
            self.assertTrue(torch.equal(scene[name].data.default_root_pose[1], torch.zeros(7)))
            self.assertTrue(torch.equal(scene[name].velocity, torch.zeros(2, 6)))

    def test_static_world_translation_yaw_scale_and_no_source_asset_edit(self):
        stage = Usd.Stage.CreateInMemory()
        paths, origins = ['/World/envs/env_0', '/World/envs/env_1'], [[-30., 0., 0.], [0., 30., 0.]]
        specs = self.specs('robot_narrow_opening')
        table = geometry_parameters(specs)['poses_local_xyz_xyzw']['barrier']
        for path, origin in zip(paths, origins):
            UsdGeom.Xform.Define(stage, path).AddTranslateOp().Set(Gf.Vec3d(*origin))
            UsdGeom.Xform.Define(stage, path + '/barrier').AddScaleOp().Set(Gf.Vec3f(2., 1., 1.))
        root_before = stage.GetRootLayer().ExportToString()
        edits = author_static_poses(stage, paths, origins, {'barrier': table})
        self.assertEqual(len(edits), 2)
        self.assertEqual(stage.GetRootLayer().ExportToString(), root_before)
        cache = UsdGeom.XformCache()
        for index, path in enumerate(paths):
            world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path + '/barrier'))
            np.testing.assert_allclose(world.ExtractTranslation(), np.array(table[index][:3]) + origins[index])
            heading = np.asarray(world.TransformDir(Gf.Vec3d(1., 0., 0.)))
            yaw = math.radians(specs[index]['layout']['obstacle']['yaw_deg'])
            np.testing.assert_allclose(heading, [2 * math.cos(yaw), 2 * math.sin(yaw), 0.], atol=1e-8)

    def appearance_stage(self):
        stage = Usd.Stage.CreateInMemory()
        root = '/World/envs/env_0'
        UsdGeom.Xform.Define(stage, root)
        UsdShade.Material.Define(stage, root + '/bowl/Looks/opaque')
        shader = UsdShade.Shader.Define(stage, root + '/bowl/Looks/opaque/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.5, .5, .5))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.4)
        second = UsdGeom.Xform.Define(stage, '/World/envs/env_1')
        second.GetPrim().GetInherits().AddInherit(root)
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
        UsdLux.DistantLight.Define(stage, '/World/DirectionalLight').CreateIntensityAttr(500.)
        return stage

    def test_inherited_material_baselines_are_read_before_any_env_edits(self):
        stage = self.appearance_stage()
        specs = self.specs()
        for spec, multiplier in zip(specs, [.5, 1.5]):
            spec['appearance']['materials']['bowl']['color_multiplier'] = [multiplier] * 3
        root_before = stage.GetRootLayer().ExportToString()
        report = apply_vectorized_appearance(stage, specs, lighting='unchanged')
        for index, expected in enumerate([.25, .75]):
            shader = UsdShade.Shader(stage.GetPrimAtPath(f'/World/envs/env_{index}/bowl/Looks/opaque/Shader'))
            np.testing.assert_allclose(shader.GetInput('diffuseColor').Get(), [expected] * 3)
            self.assertEqual(report['environments'][index]['seeds'], specs[index]['seeds'])
        self.assertEqual(stage.GetRootLayer().ExportToString(), root_before)
        self.assertEqual(UsdLux.DomeLight(stage.GetPrimAtPath('/World/Light')).GetIntensityAttr().Get(), 1000.)
        self.assertFalse(report['independent_lighting_render_verified'])

    def test_different_shared_lights_reject_before_edits_and_linked_schema_is_scoped(self):
        stage = self.appearance_stage()
        specs = self.specs()
        session_before = stage.GetSessionLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, 'Independent light seeds'):
            apply_vectorized_appearance(stage, specs)
        self.assertEqual(stage.GetSessionLayer().ExportToString(), session_before)
        report = apply_vectorized_appearance(stage, specs, lighting='linked_experimental')
        self.assertFalse(report['independent_lighting_render_verified'])
        self.assertEqual(len(report['lights']), 4)
        for index in range(2):
            prim = stage.GetPrimAtPath(f'/World/envs/env_{index}/EpisodeLights/Light')
            for name in ('lightLink', 'shadowLink'):
                collection = Usd.CollectionAPI(prim, name)
                self.assertFalse(collection.GetIncludeRootAttr().Get())
                self.assertEqual(collection.GetIncludesRel().GetTargets(), [Sdf.Path(f'/World/envs/env_{index}')])
            expected = 1000 * specs[index]['appearance']['lights']['/World/Light']['intensity_multiplier']
            self.assertAlmostEqual(UsdLux.LightAPI(prim).GetIntensityAttr().Get(), expected, places=3)
        self.assertEqual(UsdLux.LightAPI(stage.GetPrimAtPath('/World/Light')).GetIntensityAttr().Get(), 0.)


if __name__ == '__main__':
    unittest.main()
