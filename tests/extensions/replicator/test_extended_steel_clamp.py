"""Derived fixture must extend only the declared source beam/jaw directions."""
import hashlib
import json
from pathlib import Path
import unittest
import numpy as np
from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,variants
ROOT=Path(__file__).resolve().parents[3]

class ExtendedSteelClampTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  source=next(a for a in variants() if a['id']=='robot_steel_clamp')
  cls.asset=json.loads((ROOT/'configs/extensions/replicator/visual_scene_variants_reach650_assets.json').read_text())['robot_steel_clamp']
  cls.original=Hazard(source);cls.extended=Hazard(cls.asset)
  cls.old={r[0].rsplit('/',1)[-1]:r for r in cls.original.rows};cls.new={r[0].rsplit('/',1)[-1]:r for r in cls.extended.rows}
 def test_unmodified_support_and_height_geometry(self):
  self.assertEqual(set(self.old),set(self.new));self.assertEqual(len(self.old),7)
  modified={'PinchUpperBeam','PinchLowerBeam','UpperJawFace','LowerJawFace'}
  for name,old in self.old.items():
   new=self.new[name]
   np.testing.assert_allclose(old[4][[0,2]],new[4][[0,2]],rtol=0,atol=1e-12)
   np.testing.assert_allclose(old[5][[0,2]],new[5][[0,2]],rtol=0,atol=1e-12)
   if name not in modified:
    np.testing.assert_array_equal(old[4],new[4]);np.testing.assert_array_equal(old[5],new[5])
   else:
    self.assertAlmostEqual(float(new[5][1]),.65,places=7)
    self.assertAlmostEqual(float(new[4][1]),float(old[4][1]),places=7)
 def test_metadata_describes_derived_asset_without_acceptance_claim(self):
  self.assertEqual(self.asset['asset_variant'],'reach650')
  self.assertFalse(self.asset['asset_validation']['native_physics_verified'])
  self.assertFalse(self.asset['asset_validation']['task_accepted'])
  self.assertGreater(self.asset['dimensions']['overall_xyz_m'][1],.78)
  self.assertIn('source_original_metadata',self.asset)
  for path,expected in json.loads((ROOT/self.asset['derivation_path']).read_text())['source_layers_sha256'].items():
   self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),expected)

if __name__=='__main__':unittest.main()
