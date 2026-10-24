import copy
import unittest
from kitchen_safety.extensions.doors.source_necessity_lwx import has_original_source_clearance_violation

class SourceNecessityTests(unittest.TestCase):
    def setUp(self):
        self.risk = dict(available=True, source_geometry_safe=False,
            violations=[dict(distance_m=.02, required_m=.05, slack_m=-.03,
                             valid=False, robot_part='robot_hull', hazard_part='glass/Foot',
                             intersecting=False)])
    def test_original_margin_violation_need_not_intersect(self):
        self.assertTrue(has_original_source_clearance_violation(self.risk))
    def test_safe_source_with_proxy_only_violation_is_rejected(self):
        self.risk['source_geometry_safe']=True
        self.risk['conservative_violations']=self.risk['violations']
        self.assertFalse(has_original_source_clearance_violation(self.risk))
    def test_missing_source_measurement_is_rejected(self):
        self.risk['violations']=[]
        self.assertFalse(has_original_source_clearance_violation(self.risk))
    def test_inconsistent_or_nonfinite_distance_is_rejected(self):
        for key,value in [('distance_m',float('inf')),('slack_m',-.01),('required_m',0),('distance_m',True)]:
            risk=copy.deepcopy(self.risk);risk['violations'][0][key]=value
            self.assertFalse(has_original_source_clearance_violation(risk))
    def test_zero_slack_is_not_violation(self):
        self.risk['violations'][0].update(distance_m=.05,slack_m=0)
        self.assertFalse(has_original_source_clearance_violation(self.risk))
    def test_missing_original_source_identity_is_rejected(self):
        self.risk['violations'][0].pop('hazard_part')
        self.assertFalse(has_original_source_clearance_violation(self.risk))
    def test_unavailable_source_is_rejected(self):
        self.risk['available']=False
        self.assertFalse(has_original_source_clearance_violation(self.risk))

if __name__=='__main__':unittest.main()
