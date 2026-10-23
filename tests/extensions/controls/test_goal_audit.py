"""Regression: don't reject equivalent TCP goals by mixing metres and matrix entries."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.controls.audit_avoidance import prepress_goal_checks, counterfactual_hazard_pose

class GoalAuditTests(unittest.TestCase):
    def setUp(self):
        self.target=np.eye(4)
        self.target[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
        self.target[:3,3]=[2.08087763,-.44,1.02713965]

    def test_opposite_small_tracking_errors_are_valid(self):
        baseline=self.target.copy();actual=self.target.copy()
        baseline[:3,:3]@=Rotation.from_rotvec([0,0,.0006]).as_matrix()
        actual[:3,:3]@=Rotation.from_rotvec([0,0,-.0007]).as_matrix()
        actual[0,3]+=.00045
        self.assertGreater(np.abs(baseline-actual).max(),.001)
        checks=prepress_goal_checks(baseline,actual)
        self.assertTrue(checks['baseline']['passed'])
        self.assertTrue(checks['actual']['passed'])

    def test_shifted_task_goal_is_rejected(self):
        changed=self.target.copy();changed[0,3]+=.01
        self.assertFalse(prepress_goal_checks(self.target,changed)['actual']['passed'])

    def test_rotated_task_goal_is_rejected(self):
        changed=self.target.copy();changed[:3,:3]@=Rotation.from_rotvec([.03,0,0]).as_matrix()
        self.assertFalse(prepress_goal_checks(self.target,changed)['actual']['passed'])

class CounterfactualPoseTests(unittest.TestCase):
    def test_recorded_settling_uses_measured_final_pose(self):
        result={'variant':'property','initial_state':{'hazard_pose':[0,0,.921,0,0,0,1]}}
        settled=[0,0,.920,0,0,0,1]
        rows=[{'phase':'setup','after':{'hazard_pose':settled}}]
        self.assertEqual(counterfactual_hazard_pose(result,rows,None),settled)

    def test_legacy_data_keeps_its_measured_initial_pose(self):
        pose=[0,0,.920,0,0,0,1]
        result={'variant':'property','initial_state':{'hazard_pose':pose}}
        self.assertEqual(counterfactual_hazard_pose(result,[],None),pose)

if __name__=='__main__':unittest.main()
