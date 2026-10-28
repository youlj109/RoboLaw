"""Same production FCL/tilt predicates, with only dense-point order changed."""
from collections import Counter
from types import SimpleNamespace
import unittest

import fcl
import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.planning_checks import check_planned_path, dense_path_samples


class GeometryWorld(CollisionWorld):
    def __init__(self, obstacle_x=10., carrying=False):
        self.fcl = fcl
        self.attachment = np.eye(4) if carrying else None
        self.gripper_joint_position = .376
        self.kin = SimpleNamespace(fk=self.pose)
        self.obstacle = fcl.CollisionObject(fcl.Box(.02, .02, .02),
                                          fcl.Transform([obstacle_x, 0., 0.]))
        self.seen, self.context = [], []

    @staticmethod
    def pose(q):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler('x', q[1]).as_matrix()
        pose[0, 3] = q[0]
        return pose

    def _pairs(self, q, ignore, *args, **kwargs):
        if 'obstacle' in ignore:
            return []
        sphere = fcl.CollisionObject(fcl.Sphere(.01), fcl.Transform([q[0], 0., 0.]))
        return [(-.002, 'arm', '/World/obstacle', sphere, self.obstacle, .002)]

    def valid(self, q, ignore=(), **kwargs):
        self.seen.append(np.array(q, copy=True))
        self.context.append((ignore, self.gripper_joint_position, self.attachment))
        return super().valid(q, ignore, **kwargs)


def sequential(world, path, ignore=(), resolution=.02):
    return all(world.edge(a, b, ignore, resolution=resolution)
               for a, b in zip(path[:-1], path[1:]))


class PlanningChecksTest(unittest.TestCase):
    def test_legal_sample_multiset_is_bitwise_equal_including_repeated_endpoints(self):
        for dtype in (np.float32, np.float64):
            path = np.zeros((5, 7), dtype=dtype)
            path[:, 0] = [.071, .211, .211, .319, .401]
            path[:, 1] = [.015, -.017, -.017, .031, .011]
            original = path.copy()
            old, new = GeometryWorld(), GeometryWorld()
            self.assertTrue(sequential(old, path))
            self.assertTrue(check_planned_path(new, path))
            packed = lambda samples: Counter((q.dtype.str, q.tobytes()) for q in samples)
            self.assertEqual(packed(new.seen), packed(old.seen))
            self.assertEqual(packed(dense_path_samples(path)), packed(old.seen))
            self.assertEqual(len(new.seen), len(old.seen))
            self.assertGreater(max(packed(new.seen).values()), 1)
            np.testing.assert_array_equal(path, original)

    def test_late_real_collision_rejects_early_and_preserves_original_margin(self):
        path = np.zeros((129, 7))
        path[:, 0] = np.linspace(0., 1., len(path))
        old, new = GeometryWorld(obstacle_x=.75), GeometryWorld(obstacle_x=.75)
        self.assertFalse(sequential(old, path))
        self.assertFalse(check_planned_path(new, path))
        self.assertLessEqual(len(new.seen), 4)
        self.assertLess(len(new.seen), len(old.seen)//4)
        self.assertTrue(new.last_clearance['intersecting'])
        self.assertEqual(new.last_clearance['required_m'], .002)

    def test_late_carried_tilt_keeps_original_28_degree_predicate(self):
        path = np.zeros((129, 7))
        path[:, 0] = np.linspace(0., 1., len(path))
        path[:, 1] = .6*np.maximum(0., 1.-np.abs(path[:, 0]-.75)/.3)
        old, new = GeometryWorld(carrying=True), GeometryWorld(carrying=True)
        self.assertFalse(sequential(old, path))
        self.assertFalse(check_planned_path(new, path))
        self.assertLessEqual(len(new.seen), 4)
        self.assertEqual(new.last_collision, ('carried_object', 'orientation constraint'))

    def test_empty_and_single_point_paths_make_no_checks_as_original_all(self):
        for path in ([], np.empty((0, 7)), np.zeros((1, 7))):
            world = GeometryWorld()
            self.assertTrue(check_planned_path(world, path))
            self.assertTrue(sequential(world, path))
            self.assertEqual(world.seen, [])

    def test_short_path_still_checks_dense_middle_and_live_ignore_grip_attachment(self):
        path = np.zeros((2, 7))
        path[-1, 0] = 1.
        world = GeometryWorld(obstacle_x=.75, carrying=True)
        self.assertTrue(world.valid(path[0]))
        self.assertTrue(world.valid(path[-1]))
        self.assertFalse(check_planned_path(world, path))
        world.seen.clear()
        world.context.clear()
        ignore, attachment = ('obstacle', 'bowl'), world.attachment
        self.assertTrue(check_planned_path(world, path, ignore))
        self.assertEqual(len(world.seen), 51)
        for actual_ignore, grip, actual_attachment in world.context:
            self.assertIs(actual_ignore, ignore)
            self.assertEqual(grip, .376)
            self.assertIs(actual_attachment, attachment)


if __name__ == '__main__':
    unittest.main()
