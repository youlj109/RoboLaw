import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.release_pose_lwx import release_pose, release_progress


class ReleasePoseTests(unittest.TestCase):
    def test_ease_out_is_bounded_and_monotone_with_a_slower_finish(self):
        fractions = np.linspace(0., 1., 21)
        linear = np.array([release_progress(f) for f in fractions])
        curved = np.array([release_progress(f, 'ease_out') for f in fractions])
        np.testing.assert_array_equal(linear, fractions)
        self.assertEqual(curved[0], 0.)
        self.assertEqual(curved[-1], 1.)
        self.assertTrue(np.all(np.diff(curved) > 0.))
        self.assertTrue(np.all(curved >= linear))
        self.assertTrue(np.all(curved <= 1.))
        self.assertLess(np.diff(curved)[-1], np.diff(linear)[-1])
        for bad in [-.1, 1.1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                release_progress(bad, 'ease_out')
        with self.assertRaises(ValueError):
            release_progress(.5, 'unknown')

    def test_disabled_option_preserves_original_fixed_target(self):
        start = np.eye(4)
        start[:3, 3] = [1., 2., 1.5]
        normal = np.array([.8, .6, 0.])
        for fraction in np.linspace(.05, 1., 20):
            expected = start.copy()
            direction = -normal * np.cos(np.deg2rad(-55.)) + [0., 0., np.sin(np.deg2rad(-55.))]
            direction /= np.linalg.norm(direction)
            expected[:3, 3] += direction * .03 * fraction
            actual = release_pose(start, -.6, -.7, [0., 0., 0.], [0., 0., 1.], normal, .03, -55., fraction)
            np.testing.assert_array_equal(actual, expected)

    def test_following_preserves_initial_relative_pose_then_retracts(self):
        start = np.eye(4)
        start[:3, :3] = Rotation.from_euler('xyz', [.2, -.1, .4]).as_matrix()
        start[:3, 3] = [2., 1., 1.6]
        hinge, axis, normal = np.array([1., 1., .8]), np.array([0., 0., 2.]), np.array([1., 0., 0.])
        delta = Rotation.from_euler('z', -.08).as_matrix()
        actual = release_pose(start, -.6, -.68, hinge, axis, normal, .03, 0., 1., True)
        np.testing.assert_allclose(actual[:3, :3], delta @ start[:3, :3], atol=1e-14)
        np.testing.assert_allclose(actual[:3, 3], hinge + delta @ (start[:3, 3] - hinge - .03 * normal), atol=1e-14)
        stationary = release_pose(start, -.6, -.6, hinge, axis, normal, .03, -55., .5, True)
        original = release_pose(start, -.6, -.6, hinge, axis, normal, .03, -55., .5)
        np.testing.assert_allclose(stationary, original, atol=1e-14)

    def test_original_bounds_and_axis_are_enforced(self):
        for retreat, tilt, fraction in [(.041, 0., .5), (.03, -61., .5), (.03, 0., 1.1)]:
            with self.assertRaises(ValueError):
                release_pose(np.eye(4), 0., .1, [0., 0., 0.], [0., 0., 1.], [1., 0., 0.], retreat, tilt, fraction)
        with self.assertRaises(ValueError):
            release_pose(np.eye(4), 0., .1, [0., 0., 0.], [0., 0., 0.], [1., 0., 0.], .03, 0., .5, True)

    def test_position_following_keeps_measured_tool_orientation(self):
        start = np.eye(4)
        start[:3, 3] = [1., 0., .5]
        actual = release_pose(start, 0., .1, [0., 0., 0.], [0., 0., 1.], [1., 0., 0.], 0., 0., 0., True, False)
        np.testing.assert_array_equal(actual[:3, :3], start[:3, :3])
        np.testing.assert_allclose(actual[:3, 3], [np.cos(.1), np.sin(.1), .5])


if __name__ == '__main__':
    unittest.main()
