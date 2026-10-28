import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.search_banana_grasp_stability_cpu import finite_candidates


class FiniteCandidateTests(unittest.TestCase):
    def test_six_candidates_keep_exact_height_and_orthogonal_rotation(self):
        target, source = np.eye(4), np.eye(4)
        target[:3, 3] = [-2.28, 2.15, .94]
        source[:3, :3] = Rotation.from_euler("z", .001).as_matrix()
        values = finite_candidates(target, source, [-.033, 0, 0], [-.021, .011, -.001], [-.337, -.941, 0])
        self.assertEqual(len(values), 6)
        self.assertEqual(len({row["id"] for row in values}), 6)
        for row in values:
            actual = row["target_tcp"]
            self.assertEqual(actual[2, 3], target[2, 3])
            np.testing.assert_allclose(actual[:3, :3].T @ actual[:3, :3], np.eye(3), atol=1e-12)
            np.testing.assert_array_equal(actual[:3, 0], [0, 0, -1])
            self.assertAlmostEqual(np.linalg.det(actual[:3, :3]), 1.)
        np.testing.assert_array_equal(values[0]["target_tcp"][:3, 3], target[:3, 3])

    def test_pca_axis_sign_does_not_change_candidates(self):
        args = (np.eye(4), np.eye(4), [0, 0, 0], [.01, .01, 0])
        first = finite_candidates(*args, [-.337, -.941, .001])
        second = finite_candidates(*args, [.337, .941, -.001])
        for a, b in zip(first, second):
            np.testing.assert_array_equal(a["target_tcp"], b["target_tcp"])

    def test_nonhorizontal_axis_rejected(self):
        with self.assertRaises(ValueError):
            finite_candidates(np.eye(4), np.eye(4), [0, 0, 0], [0, 0, 0], [0, 0, 1])


if __name__ == "__main__":
    unittest.main()
