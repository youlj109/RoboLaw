"""Real USD/FCL full-world archive replay, including spawn workers; no Kit."""
from concurrent.futures import ProcessPoolExecutor
import copy
import json
import multiprocessing
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.deferred_safety import DeferredSafetyBuffer
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.vectorized_validation import (load_physics_archive, replay_action_clearances,
                                                validate_physics_archive)


JOINT_NAMES = tuple(f'panda_joint{i}' for i in range(1, 8)) + CollisionWorld.GRIPPER_JOINT_NAMES
ARM_IDS = np.arange(7)
Q = np.r_[np.deg2rad([0, -36, 0, -144, 0, 108, 0]), np.zeros(6)]
ORIGINS = [[0., 0., 0.], [30., -30., 0.]]
POSES = {'mustard': [1.98, -.38, 1., 0., 0., 0., 1.],
         'bowl': [2.5, -.3, .962, 0., 0., 0., 1.],
         'obstacle': [3.15, .2, 1.5, 0., 0., 0., 1.]}
LAYOUT = {'robot_position': [2.2, -.93, 0.], 'robot_yaw_deg': 90., 'stand_height': .8,
          'obstacle': {'category': 'robot', 'static': True}, 'planning': {}}


def write_stage(path):
    stage = Usd.Stage.CreateInMemory()
    for index, origin in enumerate(ORIGINS):
        root = f'/World/envs/env_{index}'
        UsdGeom.Xform.Define(stage, root).AddTranslateOp().Set(Gf.Vec3d(*origin))
        for name, key in [('mustard_bottle', 'mustard'), ('bowl', 'bowl'), ('barrier', 'obstacle')]:
            UsdGeom.Xform.Define(stage, root + '/' + name).AddTranslateOp().Set(Gf.Vec3d(*POSES[key][:3]))
            shape = UsdGeom.Sphere.Define(stage, root + '/' + name + '/body')
            shape.CreateRadiusAttr(.02)
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        # A genuine non-target kitchen collider intersects link7 only at q0=1.
        trap = UsdGeom.Cube.Define(stage, root + '/kitchen/trap')
        trap.CreateSizeAttr(.04)
        trap.AddTranslateOp().Set(Gf.Vec3d(1.897, -.736, 1.397))
        UsdPhysics.CollisionAPI.Apply(trap.GetPrim())
    # A collider owned by another environment occupies env_0's robot location.
    # It must not enter env_0's CollisionWorld, despite being in the full stage.
    poison = UsdGeom.Cube.Define(stage, '/World/envs/env_1/neighbour_poison')
    poison.CreateSizeAttr(.1)
    poison.AddTranslateOp().Set(Gf.Vec3d(2.2 - 30., -.93 + 30., 1.133))
    UsdPhysics.CollisionAPI.Apply(poison.GetPrim())
    stage.Export(str(path))


def write_archive(path, *, env_index=0, middle_arm_collision=False, middle_bottle_collision=False, count=3):
    buffer = DeferredSafetyBuffer(hazard_key='barrier')
    origin = np.asarray(ORIGINS[env_index])
    for index in range(count):
        q = Q.copy()
        if index == 1 and middle_arm_collision:
            q[0] = 1.
        pose = np.array([3., .2, 1.5, 0., 0., 0., 1.])
        if index == 1 and middle_bottle_collision:
            pose[0] = 3.15
        pose[:3] += origin
        buffer.append(q=q, mustard=pose, phase='release', action_index=7)
    buffer.write_npz(path, arm_ids=ARM_IDS, joint_names=JOINT_NAMES)


def job(root, index=0):
    settled = copy.deepcopy(POSES)
    for pose in settled.values():
        pose[:3] = (np.asarray(pose[:3]) + ORIGINS[index]).tolist()
    return {'stage_path': str(root / 'stage.usdc'), 'physics_states_path': str(root / f'physics{index}.npz'),
            'env_root': f'/World/envs/env_{index}', 'env_origin': ORIGINS[index],
            'layout': LAYOUT, 'settled_poses': settled, 'state_frame': 'world'}


class VectorizedValidationTests(unittest.TestCase):
    def test_action_clearance_replay_matches_online_fcl_and_restores_dynamic_world(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_stage(root / 'stage.usdc')
            stage = Usd.Stage.Open(str(root / 'stage.usdc'))
            kin = DroidKinematics([2.2, -.93, .8], 90.)
            world = CollisionWorld(stage, kin, env_root='/World/envs/env_1', env_origin=ORIGINS[1])
            archive = DeferredSafetyBuffer(hazard_key='barrier')
            expected = []
            for index, phase in enumerate(('approach', 'above_bowl', 'release')):
                q = Q.copy()
                q[0] += index * .02
                q[7:] = index * .01
                bottle = np.array([3.15-index*.05, .2, 1.5, 0., 0., 0., 1.])
                hazard = np.array(POSES['obstacle'])
                hazard[0] += index * .01
                attachment = np.eye(4) if phase == 'above_bowl' else None
                world.attachment = attachment
                world.set_obstacle_pose('barrier', hazard)
                ignored = ('mustard_bottle',) + (('bowl',) if phase == 'release' else ())
                distance = world.clearance(q[:7], ignored, mode='execution', object_pose=bottle,
                    gripper_q=dict(zip(JOINT_NAMES[7:], q[7:])), obstacle_filter=('barrier',))
                expected.append(dict(frame=10+index, phase=phase, **distance))
                archive.append(q=q, mustard=bottle, phase=phase, action_index=10+index,
                               attachment=attachment, hazard_pose=hazard)
            # The replay must preserve the current, unrelated planning state.
            world.attachment = np.eye(4) * 2.
            world.last_collision = ('sentinel', 'collision')
            world.last_clearance = {'sentinel': True}
            world.set_obstacle_pose('barrier', np.array(POSES['obstacle']) + [0., .07, 0., 0., 0., 0., 0.])
            world.set_obstacle_pose('bowl', POSES['bowl'])  # cached root != changed hazard
            cache = world._last_obstacle_pose_call
            attachment = world.attachment
            geometry = [(obj.getRotation().copy(), obj.getTranslation().copy()) for _, obj, _, _ in world.objects]
            report = replay_action_clearances(archive, world, ARM_IDS, JOINT_NAMES)
            self.assertEqual(report['records'], expected)
            self.assertEqual(report['samples'], 3)
            self.assertEqual(report['minimum_hazard_clearance_m'], min(row['distance_m'] for row in expected))
            self.assertEqual(report['records'][0]['robot_part'], 'bottle')
            self.assertFalse(report['records'][0]['valid'])  # diagnostic never substitutes safety rejection
            self.assertIs(world.attachment, attachment)
            self.assertEqual(world.last_collision, ('sentinel', 'collision'))
            self.assertEqual(world.last_clearance, {'sentinel': True})
            self.assertIs(world._last_obstacle_pose_call, cache)
            for (_, obj, _, _), (rotation, translation) in zip(world.objects, geometry):
                np.testing.assert_array_equal(obj.getRotation(), rotation)
                np.testing.assert_array_equal(obj.getTranslation(), translation)
            def failed_query(*args, **kwargs):
                raise RuntimeError('FCL query failed')
            world.clearance = failed_query
            with self.assertRaisesRegex(RuntimeError, 'FCL query failed'):
                replay_action_clearances(archive, world, ARM_IDS, JOINT_NAMES)
            self.assertIs(world.attachment, attachment)
            self.assertEqual(world.last_clearance, {'sentinel': True})
            for (_, obj, _, _), (_, translation) in zip(world.objects, geometry):
                np.testing.assert_array_equal(obj.getTranslation(), translation)

    def test_worker_keeps_all_action_diagnostics_even_after_physics_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_stage(root / 'stage.usdc')
            write_archive(root / 'physics1.npz', env_index=1, middle_arm_collision=True)
            diagnostics = DeferredSafetyBuffer(hazard_key='barrier')
            for index in range(3):
                pose = np.array(POSES['mustard'])
                pose[:3] += ORIGINS[1]
                diagnostics.append(q=Q, mustard=pose, phase='release', action_index=5+index)
            diagnostics.write_npz(root / 'diagnostics.npz', arm_ids=ARM_IDS, joint_names=JOINT_NAMES)
            request = {**job(root, 1), 'action_clearance_states_path': str(root / 'diagnostics.npz')}
            report = validate_physics_archive(request)
            self.assertFalse(report['passed'], report)
            self.assertEqual(report['checks'], 2)
            replay = report['action_clearance_diagnostics']
            self.assertTrue(replay['completed'])
            self.assertEqual([row['frame'] for row in replay['records']], [5, 6, 7])
            self.assertEqual(replay['archive']['state_frame'], 'world')
            json.dumps(report, allow_nan=False)
            # No action diagnostics before an early failure is a valid empty
            # report, but an empty physics archive remains insufficient.
            DeferredSafetyBuffer(hazard_key='barrier').write_npz(
                root / 'diagnostics.npz', arm_ids=ARM_IDS, joint_names=JOINT_NAMES)
            report = validate_physics_archive(request)
            self.assertTrue(report['action_clearance_diagnostics']['completed'])
            self.assertEqual(report['action_clearance_diagnostics']['records'], [])
            self.assertIsNone(report['action_clearance_diagnostics']['minimum_hazard_clearance_m'])

    def test_spawn_two_env_archive_finds_intermediate_nonhazard_collision(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_stage(root / 'stage.usdc')
            write_archive(root / 'physics0.npz', middle_arm_collision=True)
            write_archive(root / 'physics1.npz', env_index=1)
            with ProcessPoolExecutor(max_workers=2, mp_context=multiprocessing.get_context('spawn')) as pool:
                bad, good = list(pool.map(validate_physics_archive, [job(root, 0), job(root, 1)]))
            self.assertFalse(bad['passed'], bad)
            self.assertEqual(bad['checks'], 2)
            self.assertEqual(bad['failure']['physics_check'], 2)
            self.assertEqual(bad['failure']['action_index'], 7)
            self.assertEqual(bad['failure']['check'], 'executed_constraint')
            self.assertIn('/kitchen/trap', bad['failure']['obstacle'])
            self.assertTrue(good['passed'], good)
            self.assertEqual(good['checks'], 3)
            self.assertEqual(good['valid_calls'], 6)
            for index, report in enumerate([bad, good]):
                self.assertTrue(all(path.startswith(f'/World/envs/env_{index}/') for path in report['world']['collider_paths']))
                np.testing.assert_allclose(report['settled_poses_env_local']['bowl'], POSES['bowl'])
                json.dumps(report, allow_nan=False)

    def test_free_bottle_intermediate_hazard_collision_and_archive_origin_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_stage(root / 'stage.usdc')
            write_archive(root / 'physics1.npz', env_index=1, middle_bottle_collision=True)
            report = validate_physics_archive(job(root, 1))
            self.assertFalse(report['passed'], report)
            self.assertEqual(report['checks'], 2)
            self.assertEqual(report['failure']['check'], 'free_object_hazard')
            self.assertEqual(report['failure']['robot_part'], 'bottle')
            buffer, arm_ids, names, metadata = load_physics_archive(
                root / 'physics1.npz', env_origin=ORIGINS[1], state_frame='world')
            np.testing.assert_allclose(buffer._states[1].mustard[:3], [3.15, .2, 1.5])
            self.assertEqual(metadata['samples'], 3)
            np.testing.assert_array_equal(arm_ids, ARM_IDS)
            self.assertEqual(names, JOINT_NAMES)

    def test_empty_missing_dynamic_pose_and_hazard_only_stage_do_not_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_stage(root / 'stage.usdc')
            write_archive(root / 'physics0.npz', count=0)
            report = validate_physics_archive(job(root))
            self.assertFalse(report['passed'])
            self.assertEqual(report['checks'], 0)
            self.assertIn('Empty physics archive', report['error'])
            write_archive(root / 'physics0.npz')
            dynamic = job(root)
            dynamic['layout'] = copy.deepcopy(LAYOUT)
            dynamic['layout']['obstacle']['static'] = False
            report = validate_physics_archive(dynamic)
            self.assertFalse(report['passed'])
            self.assertIn('measured pose at every', report['error'])
            stage = Usd.Stage.Open(str(root / 'stage.usdc'))
            stage.RemovePrim('/World/envs/env_0/kitchen')
            stage.GetRootLayer().Save()
            report = validate_physics_archive(job(root))
            self.assertFalse(report['passed'])
            self.assertIn('hazard-only archive is insufficient', report['error'])


if __name__ == '__main__':
    unittest.main()
