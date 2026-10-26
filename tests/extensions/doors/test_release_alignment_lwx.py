import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.release_alignment_lwx import release_roll_pose
class ReleaseAlignmentTest(unittest.TestCase):
    def test_default_identity_and_no_position_waypoint(self):
        start=np.eye(4);start[:3,3]=[1.,2.,3.]
        np.testing.assert_array_equal(release_roll_pose(start,0.,1.),start)
        np.testing.assert_array_equal(release_roll_pose(start,-45.,1.)[:3,3],start[:3,3])
    def test_local_axis(self):
        start=np.eye(4);start[:3,:3]=Rotation.from_euler('z',70.,degrees=True).as_matrix()
        wanted=start[:3,:3]@Rotation.from_euler('x',-22.5,degrees=True).as_matrix()
        np.testing.assert_allclose(release_roll_pose(start,-45.,.5)[:3,:3],wanted)
    def test_invalid_inputs(self):
        for delta,fraction in [(float('nan'),.5),(-61.,.5),(0.,2.)]:
            with self.assertRaises(ValueError):release_roll_pose(np.eye(4),delta,fraction)
if __name__=='__main__':unittest.main()
