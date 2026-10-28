"""CPU-only replay tests using real FCL distances and CollisionWorld limits."""
import itertools
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import fcl
import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.deferred_safety import DeferredSafetyBuffer, HAZARDS


JOINT_NAMES = tuple(f'joint_{i}' for i in range(13))
ARM_IDS = np.arange(7)


def pose(x=0., tilt=0.):
    return np.r_[x, 0., 0., Rotation.from_euler('y', tilt, degrees=True).as_quat()]


def joints(x=1.):
    q = np.zeros(13)
    q[0] = x
    return q


def collision_world():
    """A 10 cm hazard box with mesh bounds and measured robot/bottle spheres."""
    world = CollisionWorld.__new__(CollisionWorld)
    world.fcl = fcl
    world.attachment = None
    world.last_collision = None
    world.last_clearance = None
    world._last_obstacle_pose_call = None
    world._pair_selection_key = None
    world._pair_selection_cache = {}
    world._last_robot_shapes = None
    world.default_clearance = .002
    world.clearances = {'glass': .06}
    world.execution_clearances = {'glass': .05}
    name = '/World/glass/body'
    vertices = np.array(list(itertools.product([-.05, .05], repeat=3)))
    mesh = (vertices, np.empty((0, 3), dtype=np.int32))
    world.meshes = {name: mesh}
    low, high = vertices.min(0), vertices.max(0)
    obj = fcl.CollisionObject(fcl.Box(.1, .1, .1))
    world.objects = [(name, obj, low, high)]
    world.low, world.high = low[None].copy(), high[None].copy()
    world._object_reference = [(np.eye(3), np.zeros(3), low.copy(), high.copy(), mesh)]
    world._root_reference_transforms = {'/World/glass': np.eye(4)}

    def shapes(q, gripper_q=None, object_pose=None, *, _cache_robot=False):
        candidates = [('arm', np.r_[q[0], 0., 0.])]
        if object_pose is not None:
            candidates.append(('bottle', np.asarray(object_pose)[:3]))
        for label, center in candidates:
            yield label, fcl.CollisionObject(fcl.Sphere(.01), fcl.Transform(center)), center - .01, center + .01

    world.shapes = shapes
    return world


class DeferredSafetyTests(unittest.TestCase):
    def test_detects_intermediate_physics_collision_between_safe_action_endpoints(self):
        world = collision_world()
        buffer = DeferredSafetyBuffer()
        for x in [.2, 0., .2]:
            buffer.append(q=joints(x), mustard=pose(2.), phase='approach', action_index=7)
        # Action endpoints alone pass, but the second of its physics states fails.
        self.assertTrue(world.valid(joints(.2)[:7], mode='execution'))
        report = buffer.validate(world, ARM_IDS, JOINT_NAMES)
        self.assertFalse(report['passed'])
        self.assertEqual(report['samples'], 3)
        self.assertEqual(report['checks'], 2)
        self.assertEqual(report['valid_calls'], 3)
        self.assertEqual(report['failure']['physics_check'], 2)
        self.assertEqual(report['failure']['action_index'], 7)
        self.assertEqual(report['failure']['check'], 'executed_constraint')
        self.assertEqual(report['failure']['required_m'], .05)
        self.assertTrue(report['failure']['intersecting'])

    def test_released_bottle_is_checked_separately_and_support_allowances_match(self):
        world = collision_world()
        buffer = DeferredSafetyBuffer()
        buffer.append(q=joints(), mustard=pose(0.), phase='release', action_index=4)
        with patch.object(world, 'valid', wraps=world.valid) as valid:
            report = buffer.validate(world, ARM_IDS, JOINT_NAMES)
        self.assertFalse(report['passed'])
        self.assertEqual(report['failure']['check'], 'free_object_hazard')
        self.assertEqual(report['failure']['robot_part'], 'bottle')
        first, second = valid.call_args_list
        self.assertEqual(first.args[1], ('mustard_bottle', 'bowl'))
        self.assertIsNone(first.kwargs['object_pose'])
        self.assertEqual(second.kwargs['obstacle_filter'], HAZARDS)
        np.testing.assert_array_equal(second.kwargs['object_pose'], pose(0.))

    def test_uses_existing_carried_tilt_limit_and_restores_original_attachment(self):
        world = collision_world()
        original = np.eye(4)
        original[0, 3] = .3
        world.attachment = original
        world.last_collision = ('original', 'collision')
        world.last_clearance = {'original': True}
        for tilt, expected in [(27.9, True), (28.1, False)]:
            buffer = DeferredSafetyBuffer()
            buffer.append(q=joints(), mustard=pose(2., tilt), phase='lift',
                          action_index=8, attachment=np.eye(4))
            report = buffer.validate(world, ARM_IDS, JOINT_NAMES)
            self.assertEqual(report['passed'], expected)
            self.assertIs(world.attachment, original)
            self.assertEqual(world.last_collision, ('original', 'collision'))
            self.assertEqual(world.last_clearance, {'original': True})
            self.assertEqual(report['valid_calls'], 1)
            if not expected:
                self.assertTrue(report['failure']['geometry_valid'])
                self.assertFalse(report['failure']['valid'])
                self.assertEqual(report['failure']['reason'], ('carried_object', 'orientation constraint'))

    def test_dynamic_geometry_mesh_and_cache_restore_after_failure_or_exception(self):
        for raises in [False, True]:
            with self.subTest(raises=raises):
                world = collision_world()
                world.set_obstacle_pose('glass', pose(.5))
                original_cache = world._last_obstacle_pose_call
                original_item = world.objects[0]
                original_mesh = world.meshes['/World/glass/body']
                low, high = world.low.copy(), world.high.copy()
                original_transform = world.objects[0][1].getTranslation().copy()
                buffer = DeferredSafetyBuffer()
                buffer.append(q=joints(.2), mustard=pose(2.), phase='pregrasp', action_index=2,
                              hazard_pose=pose(.2))
                if raises:
                    with patch.object(world, 'valid', side_effect=RuntimeError('query failed')):
                        with self.assertRaisesRegex(RuntimeError, 'query failed'):
                            buffer.validate(world, ARM_IDS, JOINT_NAMES)
                else:
                    report = buffer.validate(world, ARM_IDS, JOINT_NAMES)
                    self.assertFalse(report['passed'])
                self.assertIs(world.objects[0], original_item)
                self.assertIs(world.meshes['/World/glass/body'], original_mesh)
                self.assertIs(world._last_obstacle_pose_call, original_cache)
                np.testing.assert_array_equal(world.low, low)
                np.testing.assert_array_equal(world.high, high)
                np.testing.assert_array_equal(world.objects[0][1].getTranslation(), original_transform)
                # A restored pose-cache hit must refer to restored FCL geometry.
                world.set_obstacle_pose('glass', pose(.5))
                self.assertTrue(world.valid(joints(.2)[:7], mode='execution'))

    def test_joint_mapping_attachment_transitions_and_npz_preserve_copied_states(self):
        world = collision_world()
        arm_ids = np.array([6, 5, 4, 3, 2, 1, 0])
        q = np.arange(13, dtype=float)
        mustard = pose(2.)
        attachment = np.eye(4)
        hazard = pose(0.)
        buffer = DeferredSafetyBuffer()
        buffer.append(q=q, mustard=mustard, phase='lift', action_index=2,
                      attachment=attachment, hazard_pose=hazard)
        buffer.append(q=q, mustard=mustard, phase='release', action_index=3)
        q[:], mustard[:], attachment[:], hazard[:] = 99, 99, 99, 99
        with patch.object(world, 'valid', wraps=world.valid) as valid:
            report = buffer.validate(world, arm_ids, JOINT_NAMES)
        self.assertTrue(report['passed'])
        self.assertEqual(report['checks'], 2)
        self.assertEqual(report['valid_calls'], 3)
        np.testing.assert_array_equal(valid.call_args_list[0].args[0], np.arange(6, -1, -1))
        self.assertEqual(valid.call_args_list[0].kwargs['gripper_q'],
                         {f'joint_{i}': float(i) for i in range(7, 13)})
        self.assertEqual(valid.call_args_list[0].args[1], ('mustard_bottle',))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'physics_states.npz'
            saved = buffer.write_npz(path, arm_ids=arm_ids, joint_names=JOINT_NAMES)
            self.assertEqual(saved['samples'], 2)
            with np.load(path, allow_pickle=False) as data:
                for value in data.values():
                    self.assertFalse(value.dtype.hasobject)
                np.testing.assert_array_equal(data['q'], np.tile(np.arange(13), (2, 1)))
                np.testing.assert_array_equal(data['mustard'], np.tile(pose(2.), (2, 1)))
                np.testing.assert_array_equal(data['attachment'][0], np.eye(4))
                np.testing.assert_array_equal(data['attached'], [True, False])
                np.testing.assert_array_equal(data['has_hazard_pose'], [True, False])
                np.testing.assert_array_equal(data['hazard_pose'][0], pose(0.))
                np.testing.assert_array_equal(data['phase'], ['lift', 'release'])
                np.testing.assert_array_equal(data['action_index'], [2, 3])
                np.testing.assert_array_equal(data['physics_check'], [1, 2])
                np.testing.assert_array_equal(data['arm_ids'], arm_ids)
                np.testing.assert_array_equal(data['joint_names'], JOINT_NAMES)

    def test_malformed_state_rejected_before_any_world_mutation(self):
        buffer = DeferredSafetyBuffer()
        for override in [{'q': np.zeros(7)}, {'mustard': np.zeros(7)},
                         {'hazard_pose': np.r_[0., 0., 0., 0., 0., 0., np.nan]},
                         {'attachment': np.eye(3)}, {'action_index': -1}]:
            args = dict(q=joints(), mustard=pose(2.), phase='lift', action_index=0)
            args.update(override)
            with self.assertRaises(ValueError):
                buffer.append(**args)
        self.assertEqual(len(buffer), 0)
        world = collision_world()
        with self.assertRaises(ValueError):
            buffer.validate(world, np.zeros(7, dtype=int), JOINT_NAMES)
        self.assertIsNone(world.attachment)


if __name__ == '__main__':
    unittest.main()
