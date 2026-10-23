import unittest
import numpy as np
import fcl
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.source_sphere_screen_lwx import source_spheres_safe


class SourceSphereScreenTest(unittest.TestCase):
    def test_cylinder_aabb_corner_is_not_a_collision(self):
        source = fcl.CollisionObject(fcl.Cylinder(.1, .4), fcl.Transform())
        hz = [('cylinder', source, np.array([-.1, -.1, -.2]), np.array([.1, .1, .2]))]
        # Inside the source AABB, yet >.04m from the actual cylindrical surface.
        self.assertTrue(source_spheres_safe(hz, [[.105, .105, 0]], [.005], .03))
        self.assertFalse(source_spheres_safe(hz, [[.105, 0, 0]], [.005], .03))

    def test_matches_unscreened_exact_fcl_for_rotated_sources(self):
        rng = np.random.default_rng(20261024)
        for _ in range(64):
            size = rng.uniform(.03, .4, 3)
            rot = Rotation.random(random_state=rng).as_matrix()
            pos = rng.uniform(-.25, .25, 3)
            source = fcl.CollisionObject(fcl.Box(*size), fcl.Transform(rot, pos))
            extent = np.abs(rot) @ (size / 2)
            centers = rng.uniform(-.5, .5, (8, 3))
            radii = rng.uniform(.01, .08, 8)
            expected = True
            for center, radius in zip(centers, radii):
                sphere = fcl.CollisionObject(fcl.Sphere(float(radius)), fcl.Transform(center))
                collision = fcl.collide(sphere, source, fcl.CollisionRequest(), fcl.CollisionResult())
                distance = fcl.distance(sphere, source, fcl.DistanceRequest(), fcl.DistanceResult())
                if collision or distance < .05:
                    expected = False
                    break
            self.assertEqual(expected, source_spheres_safe(
                [('box', source, pos - extent, pos + extent)], centers, radii, .05))


if __name__ == '__main__':
    unittest.main()
