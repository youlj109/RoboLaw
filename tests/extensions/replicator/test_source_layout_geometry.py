"""CPU regressions for actual surface support and strict candidate thresholds."""
import unittest

import fcl
import numpy as np
from scripts.extensions.replicator.audit_source_layout_geometry import RAY_BOUNDS, ray_z, minimum_distance, REQUIRED_M
from scripts.extensions.replicator.search_source_layout_candidates import nearest_grid, TranslatedShapes


class SourceLayoutGeometryTests(unittest.TestCase):
    def test_ray_rejects_mesh_aabb_hole(self):
        # Four triangles leave a real square hole although the mesh AABB covers it.
        vertices = np.array([[-2,-2,0], [2,-2,0], [2,-1,0], [-2,-1,0],
                             [-2,1,0], [2,1,0], [2,2,0], [-2,2,0]], dtype=float)
        faces = np.array([[0,1,2], [0,2,3], [4,5,6], [4,6,7]])
        meshes = [('counter', vertices, faces)]
        RAY_BOUNDS.clear()
        self.assertIsNone(ray_z(meshes, np.array([0., 0., .003])))
        self.assertAlmostEqual(ray_z(meshes, np.array([0., 1.5, .003]))['vertical_gap_m'], .003)

    def test_strict_two_mm_has_no_epsilon_relaxation(self):
        def box(name, x):
            return (name, fcl.CollisionObject(fcl.Box(1.,1.,1.), fcl.Transform(np.array([x,0.,0.]))),
                    np.array([x-.5,-.5,-.5]), np.array([x+.5,.5,.5]))
        left = box('a',0.)
        self.assertFalse(minimum_distance([left], [box('b',1.001999975)], REQUIRED_M)['valid'])
        self.assertTrue(minimum_distance([left], [box('b',1.002001)], REQUIRED_M)['valid'])

    def test_translations_are_absolute_not_accumulated(self):
        row = ('box', fcl.CollisionObject(fcl.Box(1.,1.,1.), fcl.Transform(np.array([2.,0.,0.]))),
               np.array([1.5,-.5,-.5]), np.array([2.5,.5,.5]))
        shapes = TranslatedShapes([row], np.array([[1.5,-.5,-.5], [2.5,.5,.5]]))
        shapes.at([.4,0,0]); shifted = shapes.at([.1,0,0])[0]
        np.testing.assert_allclose(shifted[1].getTranslation(), [2.1,0,0])
        np.testing.assert_allclose(shifted[2], [1.6,-.5,-.5])

    def test_predeclared_grid_is_unique_bounded_and_nearest_first(self):
        grid = nearest_grid(.05, .025, [0., .001])
        self.assertEqual(grid[:2], [(0.,0.,0.), (0.,0.,.001)])
        self.assertEqual(len(grid), len(set(grid)))
        self.assertTrue(all(x*x+y*y <= .05**2 + 1e-12 for x,y,z in grid))
        distances = [round(sum(v*v for v in row), 12) for row in grid]
        self.assertEqual(distances, sorted(distances))


if __name__ == '__main__':
    unittest.main()
