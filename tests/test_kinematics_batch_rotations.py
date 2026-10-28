"""Exact FK/IK comparisons against the preceding scalar SciPy implementation."""
from copy import deepcopy
from types import MethodType
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics, transform


def scalar_links(self, q):
    t = self.base.copy()
    result = {'/panda/panda_link0': t.copy()}
    for i, (a, b, name) in enumerate(self.chain):
        r = np.eye(4)
        if i < 7:
            r[:3, :3] = Rotation.from_euler('x', q[i]).as_matrix()
        t = t @ a @ r @ b
        result[name] = t.copy()
    return result


def scalar_fk(self, q):
    t = list(self.links(q).values())[-1].copy()
    t[:3, 3] += t[:3, :3] @ np.array([.131, 0, 0])
    return t


class BatchKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fast = DroidKinematics([1.9, -.4, .9], 15.)
        cls.old = deepcopy(cls.fast)
        cls.old.links = MethodType(scalar_links, cls.old)
        cls.old.fk = MethodType(scalar_fk, cls.old)
        cls.initial_base = cls.fast.base.copy()

    def setUp(self):
        self.fast.base[:] = self.initial_base
        self.old.base[:] = self.initial_base

    def test_all_link_matrices_and_tcp_are_bitwise_equal_in_float32_and_float64(self):
        rng = np.random.default_rng(17)
        q = rng.uniform(self.fast.limits[:, 0], self.fast.limits[:, 1], (100, 7))
        q = np.vstack([q, self.fast.limits[:, 0], self.fast.limits[:, 1], np.zeros(7)])
        for dtype in (np.float32, np.float64):
            for row in q.astype(dtype):
                expected, actual = self.old.links(row), self.fast.links(row)
                self.assertEqual(list(expected), list(actual))
                for key in expected:
                    np.testing.assert_array_equal(actual[key], expected[key])
                np.testing.assert_array_equal(self.fast.fk(row), self.old.fk(row))

    def test_live_base_updates_and_list_inputs_remain_exact(self):
        q = [0., -.6, 0., -2.4, 0., 1.8, 0.]
        for yaw in (0., 45., -120.):
            base = transform([2., -.7, .95], Rotation.from_euler('z', yaw, degrees=True).as_quat())
            self.fast.base[:] = base
            self.old.base[:] = base
            np.testing.assert_array_equal(self.fast.fk(q), self.old.fk(q))
            returned = self.fast.links(q)
            returned['/panda/panda_link0'][:] = 0
            np.testing.assert_array_equal(self.fast.base, base)

    def test_warm_started_ik_retains_exact_solution(self):
        seed = np.array([0., -.6, 0., -2.4, 0., 1.8, 0.])
        for index, delta in ((0, .02), (2, -.03), (4, .01)):
            goal = seed.copy()
            goal[index] += delta
            target = self.old.fk(goal)
            expected = self.old.ik(target, seed, attempts=1)
            actual = self.fast.ik(target, seed, attempts=1)
            np.testing.assert_array_equal(actual, expected)


if __name__ == '__main__':
    unittest.main()
