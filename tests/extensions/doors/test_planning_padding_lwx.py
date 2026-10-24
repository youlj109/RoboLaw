"""Conservative planning-only padding invariants; no CUDA or simulation."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_planner import TaskPlanner
from kitchen_safety.curobo_backend import CuroboPlanner

class PlanningPaddingTests(unittest.TestCase):
 def planner(self,extra):
  planner=TaskPlanner.__new__(TaskPlanner);planner.hazard_extra_padding_m=extra
  planner.T_B_W=np.eye(4);planner.T_B_W[:3,:3]=Rotation.from_euler('z',67,degrees=True).as_matrix();planner.T_B_W[:3,3]=[1.1,-.8,.2]
  return planner
 def test_extra_reserves_space_without_moving_source_or_box_center(self):
  planner=self.planner(.01);lo=np.array([4.8,-.75,1.37]);hi=lo+[.02,.18,.51]
  pose,dims=planner._planning_box(lo,hi,.02)
  R=Rotation.from_quat(np.array(pose[3:])[[1,2,3,0]]).as_matrix();center=np.array(pose[:3]);halves=np.array(dims)/2
  # Every original world corner plus 29mm in any world direction must remain
  # strictly within this extra-padded box, including after the robot-base turn.
  import itertools
  for corner in itertools.product(*zip(lo,hi)):
   for offset in [np.eye(3)[j]*sign*.029 for j in range(3) for sign in [-1,1]]:
    world_point=np.array(corner)+offset;base_point=planner.T_B_W[:3,:3]@world_point+planner.T_B_W[:3,3]
    self.assertTrue(np.all(np.abs(R.T@(base_point-center))<halves))
  np.testing.assert_array_equal(lo,[4.8,-.75,1.37])
 def test_background_and_zero_default_preserve_shared_geometry(self):
  lo=np.array([-.2,.1,.7]);hi=np.array([.3,.4,1.2])
  for extra,padding in [(.01,0.),(0.,.02),(0.,.06)]:
   planner=self.planner(extra);actual=planner._planning_box(lo,hi,padding);expected=CuroboPlanner._planning_box(planner,lo,hi,padding)
   np.testing.assert_array_equal(actual[0],expected[0]);np.testing.assert_array_equal(actual[1],expected[1])
 def test_invalid_padding_rejected_before_cuda_initialization(self):
  for value in [-.001,.031,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):TaskPlanner(None,None,hazard_extra_padding_m=value)

if __name__=='__main__':unittest.main()
