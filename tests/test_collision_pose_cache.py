"""Actual USD/FCL geometry checks; Arena Python on CPU, without starting Kit."""
import unittest
from unittest.mock import patch

import numpy as np

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.obstacle_library import resolve_layout
from kitchen_safety.planning import DroidKinematics
from scripts.audit_collection_clearance import hazard_stage


class ObstaclePoseCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = resolve_layout('property_wine_glass')
        stage, _ = hazard_stage(cls.layout, 'property', cls.layout['obstacle'])
        base = np.asarray(cls.layout['robot_position'], dtype=float)
        base[2] += cls.layout['stand_height']
        cls.world = CollisionWorld(stage, DroidKinematics(base, cls.layout['robot_yaw_deg']))
        cls.pose = np.r_[cls.layout['obstacle']['position'], [0., 0., 0., 1.]]
        cls.query = cls.world.fcl.CollisionObject(cls.world.fcl.Sphere(.01),
            cls.world.fcl.Transform(cls.pose[:3] + [.2, 0., .15]))

    def setUp(self):
        self.world.set_obstacle_pose('Glass', self.pose)
        self.original_low = self.world.low.copy()
        self.original_high = self.world.high.copy()
        self.original_distance = self.distance()

    def distance(self):
        fcl = self.world.fcl
        return min(fcl.distance(self.query, obj, fcl.DistanceRequest(), fcl.DistanceResult())
                   for _, obj, _, _ in self.world.objects)

    def assert_restored(self):
        np.testing.assert_allclose(self.world.low, self.original_low, rtol=0, atol=1e-12)
        np.testing.assert_allclose(self.world.high, self.original_high, rtol=0, atol=1e-12)
        self.assertAlmostEqual(self.distance(), self.original_distance, places=12)

    def test_identical_pose_skips_transform_and_preserves_real_geometry(self):
        self.assertEqual(len(self.world.objects), 103)
        with patch('kitchen_safety.collision_planner.np.linalg.inv', side_effect=AssertionError('unexpected transform')):
            self.assertEqual(self.world.set_obstacle_pose('Glass', self.pose.copy()), 103)
        self.assert_restored()

    def test_tiny_pose_change_updates_geometry_and_distance(self):
        moved = self.pose.copy()
        moved[0] += 1e-7  # Small enough that np.allclose would incorrectly hide it.
        self.world.set_obstacle_pose('Glass', moved)
        np.testing.assert_allclose(self.world.low, self.original_low + [1e-7, 0, 0], rtol=0, atol=1e-12)
        self.assertLess(self.distance(), self.original_distance - 1e-9)

    def test_a_b_a_alias_or_parent_key_cannot_reuse_stale_geometry(self):
        for key, pose in [('glass', self.pose + [.02, 0, 0, 0, 0, 0, 0]),
                          ('/World/Glass', self.pose + [.02, 0, 0, 0, 0, 0, 0]),
                          ('/World', np.array([.02, 0, 0, 0, 0, 0, 1.]))]:
            with self.subTest(key=key):
                self.world.set_obstacle_pose(key, pose)
                self.assertLess(self.distance(), self.original_distance - .001)
                self.world.set_obstacle_pose('Glass', self.pose)
                self.assert_restored()

    def test_caller_array_mutation_does_not_mutate_cached_pose(self):
        caller = self.pose.copy()
        self.world.set_obstacle_pose('Glass', caller)
        caller[0] += .02
        self.world.set_obstacle_pose('Glass', caller)
        np.testing.assert_allclose(self.world.low, self.original_low + [.02, 0, 0], rtol=0, atol=1e-12)
        self.assertLess(self.distance(), self.original_distance - .001)
        caller[0] += .01
        np.testing.assert_array_equal(self.world._last_obstacle_pose_call[1], self.pose + [.02, 0, 0, 0, 0, 0, 0])
        self.world.set_obstacle_pose('Glass', caller)
        np.testing.assert_allclose(self.world.low, self.original_low + [.03, 0, 0], rtol=0, atol=1e-12)

    def test_invalid_pose_still_raises_and_keeps_geometry(self):
        for invalid in (np.r_[self.pose[:6], np.nan], np.zeros(7), self.pose[:6]):
            with self.assertRaises(ValueError):
                self.world.set_obstacle_pose('Glass', invalid)
        self.assert_restored()


if __name__ == '__main__':
    unittest.main()
