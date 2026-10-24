import unittest
from kitchen_safety.extensions.doors.task_model import opening_reserve_reached

class MeasuredOpeningReserve(unittest.TestCase):
 def test_positive_opening_needs_extra_motion(self):
  self.assertFalse(opening_reserve_reached(.30,.30,0.,.04))
  self.assertFalse(opening_reserve_reached(.33,.30,0.,.04))
  self.assertTrue(opening_reserve_reached(.35,.30,0.,.04))
 def test_negative_opening_keeps_source_direction(self):
  self.assertFalse(opening_reserve_reached(-.65,-.628,-1.57,.04))
  self.assertTrue(opening_reserve_reached(-.67,-.628,-1.57,.04))
  self.assertFalse(opening_reserve_reached(-.58,-.628,-1.57,.04))
 def test_zero_reserve_does_not_imply_native_success(self):
  self.assertTrue(opening_reserve_reached(.30,.30,0.,0.))
  self.assertFalse(opening_reserve_reached(.29,.30,0.,0.))
 def test_invalid_physical_parameters_rejected(self):
  for args in [(float('nan'),.3,0.,.04),(.3,.3,0.,-.01),(.3,.3,0.,.081),(.3,float('inf'),0.,.02)]:
   with self.assertRaises(ValueError):opening_reserve_reached(*args)
