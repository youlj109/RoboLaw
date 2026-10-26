import unittest
import numpy as np
from kitchen_safety.extensions.doors.release_direction_lwx import release_direction


class ReleaseDirectionTest(unittest.TestCase):
    def test_default_preserves_previous_direction(self):
        for normal in ([1., 0., 0.], [.2, .98, 0.], [.3, -.8, .05]):
            for tilt in (-60., 0., 45.):
                old = -np.array(normal)*np.cos(np.deg2rad(tilt)) + np.array([0., 0., np.sin(np.deg2rad(tilt))])
                old /= np.linalg.norm(old)
                np.testing.assert_array_equal(release_direction(normal, tilt), old)

    def test_positive_yaw_withdraws_towards_handle_side(self):
        result = release_direction([0., 1., 0.], 0., 30.)
        np.testing.assert_allclose(result, [.5, -np.sqrt(3.)/2., 0.], atol=1e-15)
        self.assertAlmostEqual(np.linalg.norm(result*.04), .04)
        self.assertGreater(np.dot(result, [0., -1., 0.]), 0.)

    def test_invalid_inputs_rejected(self):
        for normal, tilt, yaw in [([0., 0., 0.], 0., 0.), ([1., 2.], 0., 0.), ([1., 0., 0.], 0., 61.), ([1., 0., 0.], float('nan'), 0.), ([1., 0., 0.], 0., float('inf'))]:
            with self.assertRaises(ValueError):release_direction(normal, tilt, yaw)


if __name__ == '__main__':unittest.main()
