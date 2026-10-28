import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.banana_task_solver_probe import (
    BODY_NAMES, FIELDS, KETTLE, PAIRS, analyze, create_overlay, sha, validate_overlay, clamp_matches)


class BananaTaskSolverProbeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def source(self):
        from pxr import Usd, UsdPhysics
        path = self.root/'source.usda'
        stage = Usd.Stage.CreateNew(str(path)); stage.DefinePrim('/World')
        stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        for name in BODY_NAMES:
            prim = stage.DefinePrim(KETTLE+name)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.4)
        UsdPhysics.RevoluteJoint.Define(stage, '/World/KettleA01/hinge').CreateBody1Rel().SetTargets([KETTLE+BODY_NAMES[2]])
        stage.GetRootLayer().Save()
        return path, stage

    def test_both_pairs_only_author_eight_solver_values(self):
        from pxr import Usd, UsdPhysics
        source, _ = self.source(); digest = sha(source)
        for position, velocity in PAIRS:
            output = self.root/f'{position}_{velocity}.usda'
            report = create_overlay(source, output, position, velocity)
            self.assertTrue(report['authoring_validation']['passed'])
            self.assertEqual(len(report['authoring_validation']['authored_solver_attributes']), 8)
            composed = Usd.Stage.Open(str(output))
            for name in BODY_NAMES:
                prim = composed.GetPrimAtPath(KETTLE+name)
                self.assertAlmostEqual(UsdPhysics.MassAPI(prim).GetMassAttr().Get(), .4)
                self.assertEqual([prim.GetAttribute(field).Get() for field in FIELDS], [position, velocity])
            self.assertTrue(composed.GetPrimAtPath('/World/KettleA01/hinge').IsA(UsdPhysics.RevoluteJoint))
        self.assertEqual(sha(source), digest)

    def test_changed_mass_joint_or_extra_body_authoring_rejected(self):
        from pxr import Sdf, Usd
        source, _ = self.source(); output = self.root/'probe.usda'
        create_overlay(source, output, 128, 8)
        stage = Usd.Stage.Open(str(output))
        stage.GetPrimAtPath(KETTLE+BODY_NAMES[0]).CreateAttribute('physics:mass', Sdf.ValueTypeNames.Float).Set(100.)
        with self.assertRaisesRegex(ValueError, 'Disallowed authored property'):
            validate_overlay(stage.GetRootLayer(), source, 128, 8)

    def test_wrong_pair_missing_source_body_and_overwrite_rejected(self):
        source, stage = self.source(); output = self.root/'probe.usda'
        with self.assertRaisesRegex(ValueError, 'predeclared'):
            create_overlay(source, output, 128, 16)
        create_overlay(source, output, 128, 8)
        with self.assertRaises(FileExistsError):
            create_overlay(source, output, 128, 8)
        stage.RemovePrim(KETTLE+BODY_NAMES[-1]); stage.GetRootLayer().Save()
        with self.assertRaisesRegex(ValueError, 'inventory'):
            create_overlay(source, self.root/'other.usda', 255, 16)

    def fixture(self):
        n = 101
        paths = ['/World/envs/env_0/kitchen'+(KETTLE+name)[len('/World'):] for name in BODY_NAMES] + ['free']
        def poses(shape):
            value = np.zeros(shape); value[..., 6] = 1.; return value
        data = dict(physics_step=np.arange(n), q=np.zeros((n, 13)), dq=np.zeros((n, 13)),
            background=poses((n, 5, 7)), background_velocity=np.zeros((n, 5, 6)),
            banana=poses((n, 13)), bowl=poses((n, 13)))
        meta = dict(background_paths=paths, physics_dt_s=.1, joint_names=['j'+str(i) for i in range(13)],
            reset_physics_step=0, final_physics_step=100, actions=50, decimation=2, requested_duration_seconds=10., callback_errors=[],
            background_topology=dict(joint_connected_native_paths=paths[:4], unobserved_joint_connected_bodies=[]))
        return data, meta

    def test_world_gate_stays_failed_even_when_parent_relative_is_stationary(self):
        data, meta = self.fixture()
        rotation = Rotation.from_euler('z', np.r_[0., np.full(100, .01)][:, None])
        for i in range(4):
            data['background'][:, i, 3:7] = rotation.as_quat()
            data['background'][:, i, :3] = rotation.apply(np.tile([0. if i == 0 else .3, 0., 0.], (101, 1)))
        report = analyze(data, meta)
        self.assertFalse(report['checks']['original_world_background_stability'])
        self.assertTrue(all(row['max_translation_m'] < 1e-12 for row in report['kettle_parent_relative_diagnostic']))

    def test_full_prefix_required_and_free_body_settling_is_explicit(self):
        data, meta = self.fixture()
        self.assertTrue(analyze(data, meta)['passed'])
        data['background'][1, 4, 0] = .1
        self.assertTrue(analyze(data, meta)['passed'])
        data['background'][60, 4, 0] = .003
        self.assertFalse(analyze(data, meta)['passed'])
        data, meta = self.fixture(); data['physics_step'][1] = 2
        self.assertFalse(analyze(data, meta)['checks']['full_post_reset_physics_coverage'])
        data, meta = self.fixture(); meta['final_physics_step'] = 101
        self.assertFalse(analyze(data, meta)['passed'])

    def test_scene_clamp_or_illegal_request_rejected(self):
        clamp = dict(min_position_iteration_count=1, max_position_iteration_count=255,
                     min_velocity_iteration_count=0, max_velocity_iteration_count=255)
        self.assertTrue(all(clamp_matches(clamp,*pair) for pair in PAIRS))
        self.assertFalse(clamp_matches(clamp,256,16))
        self.assertFalse(clamp_matches(clamp,128,-1))
        clamp['max_position_iteration_count']=128
        self.assertFalse(clamp_matches(clamp,255,16))

    def test_missing_native_body_or_nonfinite_trace_rejected(self):
        data, meta = self.fixture(); meta['background_paths'][0] = 'missing'
        with self.assertRaisesRegex(ValueError, 'four native'):
            analyze(data, meta)
        data, meta = self.fixture(); data['background_velocity'][0, 0, 0] = np.nan
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            analyze(data, meta)


if __name__ == '__main__':
    unittest.main()
