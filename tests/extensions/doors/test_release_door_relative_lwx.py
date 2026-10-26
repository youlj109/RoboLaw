"""Check rigid measured-door feedback independently of simulator control."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.release_door_relative_lwx import follow_measured_door_pose
class DoorRelativeReleaseTest(unittest.TestCase):
 def test_measured_door_rotation_preserves_relative_tcp(self):
  T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.2,-.4,.7]).as_matrix();T[:3,3]=[.5,.2,1.6];h=np.array([.3,-.1,1.]);a=np.array([0.,0.,1.]);theta=-.21;R=Rotation.from_rotvec(a*theta).as_matrix()
  actual=follow_measured_door_pose(T,h,a,theta)
  np.testing.assert_allclose(actual[:3,3]-h,R@(T[:3,3]-h));np.testing.assert_allclose(actual[:3,:3],R@T[:3,:3]);np.testing.assert_allclose(follow_measured_door_pose(actual,h,a,-theta),T,atol=1e-12)
 def test_zero_feedback_keeps_full_pose(self):
  T=np.eye(4);T[:3,3]=[1.,2.,3.];np.testing.assert_array_equal(follow_measured_door_pose(T,[0,0,0],[0,0,1],0.),T)
 def test_rejects_invalid_axis_or_nonfinite_feedback(self):
  for axis,angle in [([0,0,0],.1),([0,0,2],.1),([0,0,1],float('nan'))]:
   with self.assertRaises(ValueError):follow_measured_door_pose(np.eye(4),[0,0,0],axis,angle)
if __name__=='__main__':unittest.main()
