import unittest
import numpy as np
from kitchen_safety.extensions.replicator.video_projection import camera_points, project_points, project_polygon


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.k = np.asarray([[100., 0, 50.], [0, 100., 50.], [0, 0, 1.]])
        self.pose = np.asarray([0., 0, 0, 0, 0, 0, 1.])

    def test_ros_forward_right_down_and_camera_translation(self):
        points = np.asarray([[0, 0, 1], [.1, .2, 1], [0, 0, -1]])
        pixels, valid = project_points(points, self.pose, self.k, (100, 100))
        np.testing.assert_allclose(pixels[:2], [[50, 50], [60, 70]])
        self.assertEqual(valid.tolist(), [True, True, False])
        pose = self.pose.copy(); pose[0] = 1
        np.testing.assert_allclose(camera_points([[1, 0, 1]], pose), [[0, 0, 1]])

    def test_behind_camera_and_crossing_near_plane_are_clipped(self):
        self.assertEqual(len(project_polygon([[-1,-1,-1], [1,-1,-1], [0,1,-1]], self.k, (100,100))), 0)
        polygon = project_polygon([[-.1,-.1,-1], [.1,-.1,1], [0,.1,1]], self.k, (100,100))
        self.assertGreaterEqual(len(polygon), 3)
        self.assertTrue(np.isfinite(polygon).all())
        self.assertTrue(np.all(polygon >= 0) and np.all(polygon <= 99))

    def test_offscreen_and_bad_quaternion_are_rejected(self):
        _, valid = project_points([[100,0,1]], self.pose, self.k, (100,100))
        self.assertFalse(valid[0])
        with self.assertRaises(ValueError):
            camera_points([[0,0,1]], np.zeros(7))


if __name__ == '__main__':
    unittest.main()
