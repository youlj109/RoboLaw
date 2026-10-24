import unittest
import numpy as np
from kitchen_safety.extensions.doors.release_hold_lwx import bounded_hold_command

class ReleaseHoldTests(unittest.TestCase):
    def test_retains_command_when_observation_lags(self):
        observed=np.zeros(7); target=np.full(7,.018)
        np.testing.assert_allclose(bounded_hold_command(observed,target),target)
        np.testing.assert_allclose(bounded_hold_command(observed+.01,target),target)

    def test_large_error_respects_original_increment_and_direction(self):
        observed=np.arange(7,dtype=float)/10; target=observed+np.array([.1,-.2,0,.3,-.4,.5,-.6])
        result=bounded_hold_command(observed,target)
        self.assertLessEqual(np.abs(result-observed).max(),.025+1e-12)
        np.testing.assert_allclose(result-observed,(target-observed)*(.025/.6))

    def test_at_target_stays_at_target_and_inputs_unchanged(self):
        observed=np.arange(7,dtype=float); target=observed.copy()
        result=bounded_hold_command(observed,target)
        np.testing.assert_array_equal(result,target)
        np.testing.assert_array_equal(observed,target)

    def test_rejects_nonfinite_and_wrong_shape(self):
        for invalid in [np.zeros(6),np.full(7,np.nan),np.full(7,np.inf)]:
            with self.assertRaises(ValueError):bounded_hold_command(invalid,np.zeros(7))
            with self.assertRaises(ValueError):bounded_hold_command(np.zeros(7),invalid)

if __name__=='__main__':unittest.main()
