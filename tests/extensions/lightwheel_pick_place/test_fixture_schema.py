"""Canonical channel repair preserves source drive numbers and joint freedom."""
import unittest
from pxr import Usd,UsdGeom,UsdPhysics
from kitchen_safety.extensions.lightwheel_pick_place.fixture_schema import canonicalize_drive_channels
class FixtureSchemaTests(unittest.TestCase):
 def scene(self,kind):
  s=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(s,'/kitchen');j=(UsdPhysics.RevoluteJoint if kind=='revolute' else UsdPhysics.PrismaticJoint).Define(s,'/kitchen/joint');return s,j
 def test_revolute_numbers_and_limits_are_preserved(self):
  s,j=self.scene('revolute');j.CreateAxisAttr('X');j.CreateLowerLimitAttr(-45.);j.CreateUpperLimitAttr(45.);drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'X');drive.CreateDampingAttr(.05);drive.CreateStiffnessAttr(0.);drive.CreateMaxForceAttr(10000.);r=canonicalize_drive_channels(s,'/kitchen');new=UsdPhysics.DriveAPI(j.GetPrim(),'angular');self.assertEqual(len(r),1);self.assertAlmostEqual(new.GetDampingAttr().Get(),.05);self.assertEqual(new.GetMaxForceAttr().Get(),10000.);self.assertEqual(j.GetLowerLimitAttr().Get(),-45.);self.assertEqual(j.GetUpperLimitAttr().Get(),45.)
 def test_prismatic_linear_channel_is_materialized(self):
  s,j=self.scene('prismatic');j.CreateAxisAttr('Y');UsdPhysics.DriveAPI.Apply(j.GetPrim(),'Y').CreateDampingAttr(2.);canonicalize_drive_channels(s,'/kitchen');self.assertEqual(UsdPhysics.DriveAPI(j.GetPrim(),'linear').GetDampingAttr().Get(),2.)
 def test_existing_canonical_drive_takes_precedence(self):
  s,j=self.scene('revolute');j.CreateAxisAttr('X');UsdPhysics.DriveAPI.Apply(j.GetPrim(),'X').CreateDampingAttr(2.);UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular').CreateDampingAttr(7.);self.assertEqual(canonicalize_drive_channels(s,'/kitchen'),[]);self.assertEqual(UsdPhysics.DriveAPI(j.GetPrim(),'angular').GetDampingAttr().Get(),7.)
if __name__=='__main__':unittest.main()
