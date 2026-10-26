import unittest
import numpy as np
from kitchen_safety.extensions.doors.pull_backoff_lwx import select_pull_command
class Kinematics:
    limits=np.array([[-1.,1.]]*7)
    def ik(self,target,q,attempts=1):
        result=q.copy();result[0]+=float(target[0]);return result
class PullBackoffTests(unittest.TestCase):
    def choose(self,gpu,fcl):
        return select_pull_command(Kinematics(),np.zeros(7),lambda lead:np.array([lead]),.03,gpu,fcl)
    def test_rejected_long_lead_uses_dense_checked_positive_progress(self):
        q,d=self.choose(lambda points:points[:,0]<.02,lambda q:True)
        self.assertAlmostEqual(q[0],.018);self.assertAlmostEqual(d['selected_lead_rad'],.018)
        self.assertEqual(len(d['attempts']),2);self.assertEqual(d['attempts'][-1]['fcl_checked_samples'],9)
    def test_source_failure_is_independent_of_gpu(self):
        q,d=self.choose(lambda points:np.ones(len(points),bool),lambda q:q[0]<.01)
        self.assertAlmostEqual(q[0],.006);self.assertEqual(len(d['attempts']),4)
        self.assertIn('fcl_failed_sample',d['attempts'][0])
    def test_unsafe_measured_start_cannot_be_escaped_by_skipping_it(self):
        q,d=self.choose(lambda points:np.ones(len(points),bool),lambda q:q[0]>.0001)
        self.assertIsNone(q);self.assertTrue(all(v['fcl_failed_sample']==0 for v in d['attempts']))
    def test_malformed_gpu_result_never_passes(self):
        with self.assertRaises(RuntimeError):self.choose(lambda points:[True],lambda q:True)
    def test_preserves_joint_step_cap(self):
        q,d=self.choose(lambda points:np.ones(len(points),bool),lambda q:True)
        self.assertLessEqual(abs(q).max(),.025+1e-12);self.assertAlmostEqual(d['selected_lead_rad'],.03)
if __name__=='__main__':unittest.main()
