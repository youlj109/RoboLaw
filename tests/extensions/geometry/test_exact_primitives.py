"""Independent regression cases for obstacle geometry used in causal audits."""
import unittest
import fcl
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from kitchen_safety.geometry_audit import static_collision_shapes,minimum_distance

class ExactPrimitiveTests(unittest.TestCase):
    def cylinder(self,axis='Z'):
        s=Usd.Stage.CreateInMemory();p=UsdGeom.Cylinder.Define(s,'/hazard')
        p.CreateRadiusAttr(1.);p.CreateHeightAttr(2.);p.CreateAxisAttr(axis)
        UsdPhysics.CollisionAPI.Apply(p.GetPrim()).CreateCollisionEnabledAttr(True)
        return s,p
    def sphere(self,xyz):
        p=np.array(xyz);r=.01
        return [('probe',fcl.CollisionObject(fcl.Sphere(r),fcl.Transform(p)),p-r,p+r)]
    def test_box_corner_is_not_a_cylinder_collision(self):
        s,_=self.cylinder();result=minimum_distance(self.sphere([.9,.9,0]),static_collision_shapes(s,'/hazard'),.1)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['distance_m'],np.sqrt(1.62)-1-.01,places=6)
    def test_authored_axis_and_rotation_are_respected(self):
        s,p=self.cylinder('Y');p.AddRotateZOp().Set(90)
        result=minimum_distance(self.sphere([1.2,0,0]),static_collision_shapes(s,'/hazard'),.1)
        self.assertTrue(result['valid']);self.assertAlmostEqual(result['distance_m'],.19,places=6)
    def test_unsupported_elliptical_cylinder_is_rejected(self):
        s,p=self.cylinder();p.AddScaleOp().Set(Gf.Vec3f(2,1,1))
        with self.assertRaisesRegex(ValueError,'Elliptical'):
            static_collision_shapes(s,'/hazard')

if __name__=='__main__':unittest.main()
