"""Bounds and orientation protection for the optional release controller."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.release_position_lwx import bounded_position_command

class Kin:
    def __init__(self):self.limits=np.tile([-2.,2.],(7,1))
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=q[:3];return pose

class ReleasePositionTests(unittest.TestCase):
    def test_increment_limit_preserved(self):
        q=bounded_position_command(Kin(),np.zeros(7),np.ones(3),np.eye(3))
        self.assertLessEqual(np.abs(q).max(),.025+1e-10)
        self.assertGreater(q[0],.02)
    def test_original_joint_limits_preserved(self):
        k=Kin();k.limits[0]=[-.01,.01]
        q=bounded_position_command(k,np.zeros(7),[1.,0,0],np.eye(3))
        self.assertLess(q[0],.01)
    def test_no_feasible_interval_rejected(self):
        seed=np.zeros(7);seed[0]=3.
        with self.assertRaises(ValueError):bounded_position_command(Kin(),seed,[0,0,0],np.eye(3))
    def test_rotation_limit_rejected(self):
        class Rotated(Kin):
            def fk(self,q):
                p=super().fk(q);p[:3,:3]=Rotation.from_euler('z',20,degrees=True).as_matrix();return p
        with self.assertRaises(RuntimeError):bounded_position_command(Rotated(),np.zeros(7),[0,0,0],np.eye(3))
    def test_solver_nonconvergence_rejected(self):
        with patch('kitchen_safety.extensions.doors.release_position_lwx.least_squares',return_value=SimpleNamespace(success=False)):
            with self.assertRaises(RuntimeError):bounded_position_command(Kin(),np.zeros(7),[0,0,0],np.eye(3))
    def test_nonfinite_seed_rejected(self):
        seed=np.zeros(7);seed[0]=np.nan
        with self.assertRaises(ValueError):bounded_position_command(Kin(),seed,[0,0,0],np.eye(3))

if __name__=='__main__':unittest.main()
