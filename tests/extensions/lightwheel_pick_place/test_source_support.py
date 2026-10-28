"""Source-only support invariants, including the measured false-rejection case."""
import unittest
from kitchen_safety.extensions.lightwheel_pick_place.source_support import support_allowed,update_detached

class NativeSupportTests(unittest.TestCase):
    def allowed(self,**overrides):
        args=dict(phase='lift',detached=False,hull_min_z=.9174073819,root=[3.83668,-.34270,.92563],
                  initial_hull_min_z=.916084642,initial_root=[3.83651,-.34284,.9234714])
        args.update(overrides);return support_allowed(**args)
    def test_measured_native_rest_rolling_is_allowed(self):
        self.assertTrue(self.allowed())
    def test_going_below_original_native_support_is_forbidden(self):
        self.assertFalse(self.allowed(hull_min_z=.9155))
    def test_measured_asset_origin_can_roll_down_while_hull_rises(self):
        self.assertTrue(self.allowed(root=[3.8361609,-.3385991,.9229596853],hull_min_z=.9173795925))
    def test_source_xy_and_phase_are_bounded(self):
        self.assertFalse(self.allowed(root=[3.90,-.34284,.92563]))
        self.assertFalse(self.allowed(phase='transfer'))
    def test_detachment_requires_separation_and_loss_of_contact(self):
        self.assertFalse(update_detached(detached=False,hull_min_z=.923,surface_top=.92,source_contact_count=1))
        self.assertFalse(update_detached(detached=False,hull_min_z=.921,surface_top=.92,source_contact_count=0))
        self.assertTrue(update_detached(detached=False,hull_min_z=.923,surface_top=.92,source_contact_count=0))
    def test_detachment_cannot_be_reversed_by_a_fall(self):
        self.assertTrue(update_detached(detached=True,hull_min_z=.916,surface_top=.92,source_contact_count=3))
        self.assertFalse(self.allowed(detached=True))
if __name__=='__main__':unittest.main()
