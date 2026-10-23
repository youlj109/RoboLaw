"""Check that discovery filtering never excludes actual source box collisions."""
import unittest
import numpy as np
import fcl
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.obstruction_broadphase_lwx import potential_intersection_rows


class ObstructionBroadphaseTests(unittest.TestCase):
    def test_random_rotated_source_collisions_are_retained(self):
        rng = np.random.default_rng(842)
        cube = np.asarray([[x, y, z] for x in [-.5, .5] for y in [-.5, .5] for z in [-.5, .5]])
        robot, bounds = [], []
        for _ in range(96):
            rotation = Rotation.random(random_state=rng).as_matrix()
            dimensions = rng.uniform(.03, .4, 3)
            center = rng.uniform(-.2, .2, 3)
            robot.append(fcl.CollisionObject(fcl.Box(*dimensions), fcl.Transform(rotation, center)))
            corners = (cube * dimensions) @ rotation.T + center
            bounds.append([[corners.min(0), corners.max(0)]])
        hit_count = 0
        for center in [[0., 0., 0.], [.2, -.1, .1], [2., 2., 2.]]:
            center = np.asarray(center)
            hazard = fcl.CollisionObject(fcl.Box(.12, .09, .15), fcl.Transform(center))
            selected = potential_intersection_rows(bounds, [[center - [.06, .045, .075], center + [.06, .045, .075]]])
            for index, body in enumerate(robot):
                if fcl.collide(body, hazard, fcl.CollisionRequest(), fcl.CollisionResult()):
                    hit_count += 1
                    self.assertTrue(selected[index])
        self.assertGreater(hit_count, 10)

    def test_touching_source_bounds_are_retained(self):
        robot = [[[[0., 0., 0.], [1., 1., 1.]]], [[[3., 3., 3.], [4., 4., 4.]]]]
        result = potential_intersection_rows(robot, [[[1., 0., 0.], [2., 1., 1.]]])
        self.assertEqual(result.tolist(), [True, False])


if __name__ == '__main__':
    unittest.main()
