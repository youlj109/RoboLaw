"""Safety-relevant shortcut contract tests with a transparent FK model."""
import unittest
import numpy as np
from kitchen_safety.extensions.doors.pregrasp_shortcuts_lwx import shortcut

class LinearKin:
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=q[:3];return pose

class ShortcutTests(unittest.TestCase):
    def setUp(self):
        self.path=np.zeros((9,7));self.path[:,0]=np.linspace(0,1,9);self.path[:,1]=[0,.1,.2,.3,.4,.3,.2,.1,0]
    def check(self,valid=True,complete=True,unchecked=0):
        return lambda path:dict(valid=valid,validation_complete=complete,fcl_samples_unchecked=unchecked)
    def test_safe_chord_preserves_endpoints_and_speed(self):
        out,trials=shortcut(self.path,LinearKin(),self.check())
        self.assertTrue(any(x['accepted'] for x in trials))
        np.testing.assert_array_equal(out[[0,-1]],self.path[[0,-1]])
        self.assertLessEqual(np.abs(np.diff(out,axis=0)).max(),np.abs(np.diff(self.path,axis=0)).max()+1e-12)
        self.assertEqual(out.shape,self.path.shape)
    def test_collision_rejected_without_mutating_input(self):
        original=self.path.copy();out,trials=shortcut(self.path,LinearKin(),self.check(valid=False))
        np.testing.assert_array_equal(out,original);np.testing.assert_array_equal(self.path,original)
        self.assertFalse(any(x['accepted'] for x in trials))
    def test_incomplete_validation_rejected(self):
        out,trials=shortcut(self.path,LinearKin(),self.check(complete=False))
        np.testing.assert_array_equal(out,self.path)
    def test_unchecked_fcl_sample_rejected(self):
        out,trials=shortcut(self.path,LinearKin(),self.check(unchecked=1))
        np.testing.assert_array_equal(out,self.path)
    def test_nonfinite_input_rejected(self):
        self.path[1,0]=np.nan
        with self.assertRaises(ValueError):shortcut(self.path,LinearKin(),self.check())

if __name__=='__main__':unittest.main()
