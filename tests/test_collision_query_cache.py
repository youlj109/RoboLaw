"""Cache invalidation checks against real wine-glass USD and robot FCL geometry."""
import unittest
from unittest.mock import patch

import numpy as np

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.deferred_safety import DeferredSafetyBuffer
from kitchen_safety.obstacle_library import resolve_layout
from kitchen_safety.planning import DroidKinematics
from scripts.audit_collection_clearance import hazard_stage


class CollisionQueryCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        layout = resolve_layout('property_wine_glass')
        stage, _ = hazard_stage(layout, 'property', layout['obstacle'])
        base = np.array(layout['robot_position'], dtype=float)
        base[2] += layout['stand_height']
        cls.world = CollisionWorld(stage, DroidKinematics(base, layout['robot_yaw_deg']))
        cls.base = cls.world.kin.base.copy()
        cls.pose = np.r_[layout['obstacle']['position'], [0., 0., 0., 1.]]

    def setUp(self):
        self.world.kin.base[:] = self.base
        self.world.attachment = None
        self.world.clearances = {'person': .03, 'glass': .06, 'barrier': .02}
        self.world.execution_clearances = {'person': .02, 'glass': .05, 'barrier': .01}
        self.world.default_clearance = .002
        self.world._last_robot_shapes = None
        self.world.set_obstacle_pose('Glass', self.pose)
        self.q = np.array([0., -.6, 0., -2.4, 0., 1.8, 0.])
        self.gripper = dict(zip(self.world.GRIPPER_JOINT_NAMES, [.3, .3, -.3, .3, -.3, -.3]))

    def query(self, **kwargs):
        return self.world.clearance(self.q, mode='execution', gripper_q=self.gripper, **kwargs)

    def test_static_selection_reused_but_live_rules_change_actual_acceptance(self):
        self.world.set_obstacle_pose('Glass', self.pose + [2., 0, 0, 0, 0, 0, 0])
        result = self.query()
        self.assertGreater(result['distance_m'], 0)
        with patch.object(self.world, '_matches', side_effect=AssertionError('unexpected topology rescan')):
            self.assertEqual(self.query(), result)
        self.world.execution_clearances['glass'] = result['distance_m'] + .01
        self.assertFalse(self.world.valid(self.q, mode='execution', gripper_q=self.gripper))
        self.assertEqual(self.query()['required_m'], result['distance_m'] + .01)
        self.world.execution_clearances.clear()
        self.world.default_clearance = result['distance_m'] + .02
        self.assertFalse(self.world.valid(self.q, mode='execution', gripper_q=self.gripper))
        self.world.default_clearance = .002
        self.assertTrue(self.world.valid(self.q, mode='execution', gripper_q=self.gripper))
        self.world.clearances['glass'] = result['distance_m'] + .03
        self.assertFalse(self.world.valid(self.q, mode='planning', gripper_q=self.gripper))

    def test_exact_robot_state_reuse_and_tiny_mutation_rebuild_geometry(self):
        self.query()
        with patch.object(self.world.kin, 'links', wraps=self.world.kin.links) as links:
            self.query()
            self.assertEqual(links.call_count, 0)
            self.q[0] += 1e-8
            self.query()
            self.assertEqual(links.call_count, 1)
            self.gripper['finger_joint'] += 1e-8
            self.query()
            self.assertEqual(links.call_count, 2)
            self.world.kin.base[0, 3] += 1e-8
            self.query()
            self.assertEqual(links.call_count, 3)
        # Cached geometry must equal a fresh public shape construction exactly.
        cached = self.world.shapes(self.q, self.gripper, _cache_robot=True)
        fresh = self.world.shapes(self.q, self.gripper)
        self.assertEqual(len(cached), len(fresh))
        for (label, obj, lo, hi), (other, fresh_obj, fresh_lo, fresh_hi) in zip(cached, fresh):
            self.assertEqual(label, other)
            np.testing.assert_array_equal(lo, fresh_lo)
            np.testing.assert_array_equal(hi, fresh_hi)
            np.testing.assert_array_equal(obj.getRotation(), fresh_obj.getRotation())
            np.testing.assert_array_equal(obj.getTranslation(), fresh_obj.getTranslation())

    def test_dynamic_obstacle_pose_changes_clearance_despite_selection_cache(self):
        original = self.query()
        original_selection = self.world._pair_selection_cache
        self.world.set_obstacle_pose('Glass', self.pose + [.15, 0, 0, 0, 0, 0, 0])
        moved = self.query()
        self.assertIs(self.world._pair_selection_cache, original_selection)
        self.assertNotAlmostEqual(original['distance_m'], moved['distance_m'], places=6)
        self.world.set_obstacle_pose('Glass', self.pose)
        self.assertEqual(self.query(), original)

    def test_bottle_pose_attachment_and_tilt_are_never_hidden_by_robot_cache(self):
        bottle = self.pose.copy()
        bottle[:3] += [1., 0., 0.]
        far = self.query(object_pose=bottle)
        bottle[:3] = self.pose[:3] + [0., 0., .1]
        near = self.query(object_pose=bottle)
        self.assertFalse(near['valid'])
        self.assertLessEqual(near['distance_m'], far['distance_m'])
        self.assertEqual(near['robot_part'], 'bottle')
        self.world.attachment = np.eye(4)
        upright = np.r_[self.pose[:3] + [1., 0., 0.], [0., 0., 0., 1.]]
        self.world.valid(self.q, mode='execution', gripper_q=self.gripper, object_pose=upright)
        upright[3:] = [1., 0., 0., 0.]  # Same joints, upside-down measured bottle.
        self.assertFalse(self.world.valid(self.q, mode='execution', gripper_q=self.gripper, object_pose=upright))
        self.assertEqual(self.world.last_collision, ('carried_object', 'orientation constraint'))
        # A planning attachment mutation must also move the virtual bottle.
        a = self.world.shapes(self.q, self.gripper, _cache_robot=True)[-1][1].getTranslation().copy()
        self.world.attachment[0, 3] += .1
        b = self.world.shapes(self.q, self.gripper, _cache_robot=True)[-1][1].getTranslation()
        self.assertGreater(np.linalg.norm(a - b), .09)

    def test_real_geometry_detects_intermediate_collision_with_lists_or_iterators(self):
        # Actual robot FK/hulls plus wine USD collision pieces: the middle
        # physics state has a released bottle inside the glass safety margin.
        far = self.pose.copy()
        far[:3] += [1., 0., 0.]
        near = self.pose.copy()
        near[:3] += [0., 0., .1]
        kwargs = dict(mode='execution', gripper_q=self.gripper, obstacle_filter=('glass',))
        self.assertTrue(self.world.valid(self.q, object_pose=far, **kwargs))
        self.assertFalse(self.world.valid(self.q, object_pose=near, **kwargs))
        expected = self.world.clearance(self.q, object_pose=near, **kwargs)
        self.assertEqual(expected['robot_part'], 'bottle')
        original_shapes = self.world.shapes
        for as_iterator in (False, True):
            with self.subTest(as_iterator=as_iterator):
                def shapes(*args, **kw):
                    values = original_shapes(*args, **kw)
                    return iter(values) if as_iterator else values
                with patch.object(self.world, 'shapes', side_effect=shapes):
                    self.assertFalse(self.world.valid(self.q, object_pose=near, **kwargs))
                    self.assertEqual(self.world.clearance(self.q, object_pose=near, **kwargs), expected)
                    buffer = DeferredSafetyBuffer()
                    full_q = np.r_[self.q, [self.gripper[name] for name in self.world.GRIPPER_JOINT_NAMES]]
                    for bottle in (far, near, far):
                        buffer.append(q=full_q, mustard=bottle, phase='release', action_index=8)
                    joint_names = tuple(f'arm_{i}' for i in range(7)) + tuple(self.world.GRIPPER_JOINT_NAMES)
                    report = buffer.validate(self.world, np.arange(7), joint_names)
                    self.assertFalse(report['passed'])
                    self.assertEqual(report['failure']['physics_check'], 2)
                    self.assertEqual(report['failure']['check'], 'free_object_hazard')
                    self.assertEqual(report['failure']['robot_part'], 'bottle')

    def test_filter_ignore_and_in_place_topology_changes_invalidate_indices(self):
        result = self.query(obstacle_filter=('glass',))
        self.assertIsNotNone(result['obstacle'])
        self.assertIsNone(self.query(obstacle_filter=('barrier',))['obstacle'])
        self.assertIsNone(self.world.clearance(self.q, ('glass',), mode='execution')['obstacle'])
        original_objects = list(self.world.objects)
        try:
            self.world.objects[:] = [(name.replace('/Glass/', '/Barrier/'), obj, lo, hi)
                                     for name, obj, lo, hi in original_objects]
            self.assertIsNone(self.query(obstacle_filter=('glass',))['obstacle'])
            renamed = self.query(obstacle_filter=('barrier',))
            self.assertAlmostEqual(renamed['distance_m'], result['distance_m'])
            self.assertEqual(renamed['required_m'], .01)
        finally:
            self.world.objects[:] = original_objects


if __name__ == '__main__':
    unittest.main()
