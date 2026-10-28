import unittest
import numpy as np
from scripts.extensions.replicator.analyze_banana_grasp_contact_lines import triangle_intersection, line_metrics


class SourceIntersectionTests(unittest.TestCase):
    def test_crossing_triangles_return_the_actual_line(self):
        first = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        second = [[.25, -.5, -1], [.25, -.5, 1], [.25, 1, 0]]
        points = triangle_intersection(first, second)
        np.testing.assert_allclose(points[:, 0], .25)
        np.testing.assert_allclose(points[:, 2], 0)
        np.testing.assert_allclose(sorted(points[:, 1]), [0, .75])

    def test_disjoint_and_coplanar(self):
        first = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        self.assertEqual(len(triangle_intersection(first, np.array(first) + [0, 0, 1])), 0)
        inside = [[.1, .1, 0], [.3, .1, 0], [.1, .3, 0]]
        points = triangle_intersection(first, inside)
        self.assertTrue(np.all(points[:, :2] >= .1 - 1e-12))
        self.assertTrue(np.all(points[:, :2] <= .3 + 1e-12))

    def test_pinch_line_torque_direction_and_offset(self):
        row = line_metrics([-1, 0, 0], [1, 0, 0], [0, .01, 0], [0, 0, -1])
        self.assertAlmostEqual(row["COM_proxy_distance_to_infinite_contact_line_m"], .01)
        self.assertAlmostEqual(row["gravity_torque_along_first_to_second_axis_Nm"], -.01)
        mirrored = line_metrics([1, 0, 0], [-1, 0, 0], [0, .01, 0], [0, 0, -1])
        self.assertAlmostEqual(mirrored["gravity_torque_along_first_to_second_axis_Nm"], .01)


if __name__ == "__main__":
    unittest.main()
