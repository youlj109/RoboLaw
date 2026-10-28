"""CPU-only path validation tests with real FCL geometry and unchanged tilt rules."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import fcl
import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.curobo_backend import CuroboPlanner, _hierarchical_sample_indices


class CpuValidity:
    def __init__(self, values):
        self.values = np.asarray(values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class GeometryWorld(CollisionWorld):
    """Moving sphere and optional carried tilt, using real production predicates."""
    def __init__(self, *, obstacle_x=10., carrying=False):
        self.fcl = fcl
        self.attachment = np.eye(4) if carrying else None
        self.kin = SimpleNamespace(fk=self.pose)
        self.obstacle = fcl.CollisionObject(fcl.Box(.02, .02, .02),
                                          fcl.Transform([obstacle_x, 0., 0.]))
        self.seen = []

    @staticmethod
    def pose(q):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler('x', q[1]).as_matrix()
        pose[0, 3] = q[0]
        return pose

    def _pairs(self, q, *args, **kwargs):
        sphere = fcl.CollisionObject(fcl.Sphere(.01), fcl.Transform([q[0], 0., 0.]))
        return [(-.002, 'arm', '/World/obstacle', sphere, self.obstacle, .002)]

    def valid(self, q, ignore=(), **kwargs):
        self.seen.append(np.array(q, copy=True))
        return super().valid(q, ignore, **kwargs)


def planner(world, *, rejected_index=None):
    result = CuroboPlanner.__new__(CuroboPlanner)
    result.world = world
    result._last_grip = .37
    result._prepare_path_query = Mock(return_value=0.)
    result.args = SimpleNamespace(to_device=lambda values: values)

    def valid(samples):
        values = np.ones(len(samples), dtype=bool)
        if rejected_index is not None:
            values[rejected_index] = False
        return CpuValidity(values)

    result.motion_gen = SimpleNamespace(ik_solver=SimpleNamespace(check_valid=Mock(side_effect=valid)))
    return result


class PathEarlyRejectionTests(unittest.TestCase):
    def test_hierarchical_order_is_a_complete_permutation_for_small_and_large_paths(self):
        for count in (1, 2, 3, 8, 129, 68209):
            with self.subTest(count=count):
                indices = list(_hierarchical_sample_indices(count))
                self.assertEqual(sorted(indices), list(range(count)))
                self.assertEqual(indices[0], 0)
                if count > 1:
                    self.assertEqual(indices[1], count - 1)

    def test_known_curobo_failure_skips_redundant_fcl_and_reports_unchecked_samples(self):
        path = np.zeros((129, 7))
        path[:, 0] = np.linspace(0., 1., len(path))
        original = path.copy()
        world = GeometryWorld()
        p = planner(world, rejected_index=120)
        result = p.validate_path(path, ('bowl',), 1, phase='above_bowl')
        self.assertFalse(result['valid'])
        self.assertFalse(result['validation_complete'])
        self.assertEqual(result['early_rejection'], 'curobo_constraint')
        self.assertEqual(result['curobo_failures'][0]['sample'], 120)
        self.assertEqual(result['curobo_samples_checked'], 129)
        self.assertEqual(result['fcl_samples_unchecked'], 129)
        self.assertEqual(world.seen, [])
        p._prepare_path_query.assert_called_once_with(('bowl',), 1, 'above_bowl')
        np.testing.assert_array_equal(path, original)

    def test_late_real_collision_is_rejected_without_scanning_a_long_safe_prefix(self):
        path = np.zeros((129, 7))
        path[:, 0] = np.linspace(0., 1., len(path))
        world = GeometryWorld(obstacle_x=.75)
        result = planner(world).validate_path(path)
        self.assertFalse(result['valid'])
        self.assertLessEqual(result['fcl_samples_checked'], 4)
        self.assertGreater(result['fcl_samples_unchecked'], 120)
        failure = result['fcl_failures'][0]
        self.assertEqual(failure['sample'], 96)
        self.assertTrue(failure['measurement']['intersecting'])
        self.assertEqual(failure['measurement']['required_m'], .002)

    def test_late_carried_tilt_uses_original_28_degree_predicate(self):
        path = np.zeros((129, 7))
        path[:, 0] = np.linspace(0., 1., len(path))
        path[:, 1] = .6 * np.maximum(0., 1. - np.abs(path[:, 0] - .75) / .3)
        world = GeometryWorld(carrying=True)
        result = planner(world).validate_path(path)
        self.assertFalse(result['valid'])
        self.assertLessEqual(result['fcl_samples_checked'], 4)
        self.assertEqual(result['fcl_failures'][0]['collision'],
                         ('carried_object', 'orientation constraint'))
        self.assertEqual(result['early_rejection'], 'independent_geometry')

    def test_legal_path_checks_every_original_dense_point_and_never_changes_the_path(self):
        path = np.zeros((2, 7))
        path[1, 0] = 1.
        original = path.copy()
        world = GeometryWorld()
        p = planner(world)
        result = p.validate_path(path, resolution=.02)
        self.assertTrue(result['valid'])
        self.assertTrue(result['validation_complete'])
        self.assertIsNone(result['early_rejection'])
        self.assertEqual(result['curobo_samples_checked'], 51)
        self.assertEqual(result['fcl_samples_checked'], 51)
        self.assertEqual(result['fcl_samples_unchecked'], 0)
        np.testing.assert_allclose(sorted(q[0] for q in world.seen), np.linspace(0., 1., 51))
        np.testing.assert_array_equal(path, original)
        self.assertEqual(result['curobo_failures'], [])
        self.assertEqual(result['fcl_failures'], [])

    def test_dense_intermediate_collision_is_not_missed_when_input_endpoints_are_safe(self):
        path = np.zeros((2, 7))
        path[1, 0] = 1.
        world = GeometryWorld(obstacle_x=.75)
        self.assertTrue(world.valid(path[0]))
        self.assertTrue(world.valid(path[-1]))
        result = planner(world).validate_path(path, resolution=.02)
        self.assertFalse(result['valid'])
        self.assertGreater(result['fcl_failures'][0]['fraction'], .7)
        self.assertLess(result['fcl_failures'][0]['fraction'], .8)


if __name__ == '__main__':
    unittest.main()
