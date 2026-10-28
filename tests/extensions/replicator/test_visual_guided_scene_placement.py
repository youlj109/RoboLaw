"""Semantic feature localization must not be confused with full-asset proximity."""
import unittest
import numpy as np
from kitchen_safety.extensions.replicator.visual_guided_scene_placement import closest_xy,feature_measure,features

class VisualGuidedPlacementTests(unittest.TestCase):
 def test_nearest_xy_is_clamped_to_original_task_segment(self):
  p,t=closest_xy([3.,2.,99.],np.array([0.,0.,1.]),np.array([2.,0.,1.]))
  np.testing.assert_allclose(p,[2.,0.]);self.assertEqual(t,1.)
 def test_pointing_away_from_task_is_rejected_even_when_root_is_close(self):
  feature={'name':'hand','point':[1.,0.,1.],'origin':[0.,0.,1.]}
  measure=feature_measure(feature,[0,0,0],0,np.array([-1.,0.,1.]),np.array([-.5,0.,1.]),[.9,1.3])
  self.assertEqual(measure['pointing_alignment_cosine'],-1.)
  self.assertGreater(measure['distance_to_nominal_workspace_m'],1.)
 def test_rotation_applies_to_anatomical_feature_and_origin(self):
  feature={'name':'hand','point':[1.,0.,1.],'origin':[0.,0.,1.]}
  measure=feature_measure(feature,[2.,3.,0.],90,np.array([2.,4.,1.]),np.array([2.,5.,1.]),[.9,1.3])
  np.testing.assert_allclose(measure['world_point'],[2.,4.,1.],atol=1e-12)
  self.assertAlmostEqual(measure['distance_to_nominal_workspace_m'],0.)
  self.assertAlmostEqual(measure['pointing_alignment_cosine'],1.)
 def test_actual_clamp_reference_rotation_preserves_true_jaw_end(self):
  from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,variants
  hazard=Hazard(next(a for a in variants() if a['id']=='robot_steel_clamp'))
  rows=features(hazard);lower=next(r for r in rows if r['name']=='PinchLowerBeam_outer_end')
  # Source barrier +X becomes assembled +Y through barrier_rotated.usda.
  np.testing.assert_allclose(lower['point'],[0.,.33,.99],atol=1e-7)
  self.assertIn('PinchLowerBeam',lower['source_prim'])
  self.assertGreater(lower['point'][1],lower['origin'][1])

if __name__=='__main__':unittest.main()
