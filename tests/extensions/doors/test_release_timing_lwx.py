import unittest
import numpy as np
from kitchen_safety.extensions.doors.release_pose_lwx import release_progress
from kitchen_safety.extensions.doors.release_timing_lwx import timed_release_progress


class ReleaseTimingTest(unittest.TestCase):
    def test_default_exactly_preserves_old_progress(self):
        for profile in ['linear','ease_out']:
            for fraction in np.linspace(0.,1.,21):
                self.assertEqual(timed_release_progress(float(fraction),profile),release_progress(float(fraction),profile))

    def test_early_completion_holds_same_endpoint(self):
        values=[timed_release_progress(float(f),'ease_out',.25) for f in np.linspace(0.,1.,21)]
        self.assertEqual(values[0],0.)
        self.assertTrue(all(x==1. for x in values[5:]))
        self.assertTrue(np.all(np.diff(values)>=0.))
        self.assertLessEqual(max(values),1.)

    def test_invalid_time_parameters(self):
        for fraction,completion in [(0.,0.),(0.,1.1),(-.1,.25),(float('nan'),.25),(.1,float('inf'))]:
            with self.assertRaises(ValueError):timed_release_progress(fraction,'linear',completion)


if __name__=='__main__':unittest.main()
