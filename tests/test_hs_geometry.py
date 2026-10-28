"""Analytical regressions: overlapping bounds, poses, and missing coverage."""
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom, UsdPhysics
sys.path.insert(0, '/workspace')
from kitchen_safety.hs_geometry import HumanDistanceGeometry, bounds_distance

ROOT='/World/envs/env_0'


def box(stage,path,size=(2.,.1,.1)):
    x,y,z=np.asarray(size)/2
    vertices=[[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],
              [-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]]
    # Include quads to cover authored USD polygon tessellation.
    faces=[0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]
    mesh=UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr(vertices);mesh.CreateFaceVertexCountsAttr([4]*6)
    mesh.CreateFaceVertexIndicesAttr(faces)
    return mesh.GetPrim()


def fixture(origin=(0,0,0)):
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage,ROOT)
    for path in ('Robot/base_link','mustard_bottle'):
        parent=UsdGeom.Xform.Define(stage,ROOT+'/'+path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(parent)
        collision=UsdGeom.Xform.Define(stage,str(parent.GetPath())+'/collision').GetPrim()
        UsdPhysics.CollisionAPI.Apply(collision)
        box(stage,str(collision.GetPath())+'/mesh')
    person=UsdGeom.Xform.Define(stage,ROOT+'/person')
    person.AddTranslateOp().Set(Gf.Vec3d(*origin))
    person.AddRotateZOp().Set(45.)
    UsdPhysics.CollisionAPI.Apply(box(stage,ROOT+'/person/mesh'))
    return stage


def pose(offset,origin=(0,0,0),angle=45):
    return [*(np.asarray(origin)+np.asarray(offset)),
            *Rotation.from_euler('z',angle,degrees=True).as_quat()]


class HSGeometryTests(unittest.TestCase):
    def test_overlapping_aabb_has_positive_mesh_gap(self):
        stage=fixture()
        g=HumanDistanceGeometry(stage,ROOT,['base_link'])
        self.assertFalse(g.errors)
        normal=np.array([-1.,1.,0])/np.sqrt(2)
        r=g.sample({'base_link':pose(normal*.3)},{'mustard_bottle':pose(normal*.5)})
        self.assertTrue(r['complete'])
        self.assertAlmostEqual(r['features']['d_robot_h_min_gt_m'],.2,places=6)
        self.assertAlmostEqual(r['features']['d_ee_h_min_gt_m'],.2,places=6)
        self.assertAlmostEqual(r['features']['d_obj_h_min_gt_m'],.4,places=6)
        human=next(e for e in g.entries if e['group']=='human')
        robot=next(e for e in g.entries if e['group']=='robot')
        self.assertEqual(bounds_distance(robot,human),0.)
        nearest=r['nearest']['robot']
        self.assertIsNotNone(nearest['nearest_points_world_m'])
        self.assertAlmostEqual(np.linalg.norm(np.diff(nearest['nearest_points_world_m'],axis=0)),.2,places=6)

    def test_pose_update_and_environment_translation(self):
        origin=(17.,-8.,3.)
        g=HumanDistanceGeometry(fixture(origin),ROOT,['base_link'])
        normal=np.array([-1.,1.,0])/np.sqrt(2)
        for offset in (.3,.7,.4):
            r=g.sample({'base_link':pose(normal*offset,origin)},
                       {'mustard_bottle':pose(normal*.5,origin)})
            self.assertAlmostEqual(r['features']['d_robot_h_min_gt_m'],offset-.1,places=6)
            self.assertAlmostEqual(r['features']['d_obj_h_min_gt_m'],.4,places=6)

    def test_mesh_intersection_is_zero_with_separate_flag(self):
        g=HumanDistanceGeometry(fixture(),ROOT,['base_link'])
        r=g.sample({'base_link':pose([0,0,0])},{'mustard_bottle':pose([0,0,0])})
        self.assertTrue(r['complete'])
        self.assertEqual(r['features']['d_robot_h_min_gt_m'],0.)
        self.assertTrue(r['nearest']['robot']['mesh_intersection'])
        self.assertNotIn('human_contact_flag_gt',r['features'])

    def test_missing_or_invalid_pose_is_unknown(self):
        g=HumanDistanceGeometry(fixture(),ROOT,['base_link'])
        for robot in ({},{'base_link':[0,0,0,0,0,0,0]}):
            r=g.sample(robot,{'mustard_bottle':pose([0,1,0])})
            self.assertFalse(r['complete'])
            self.assertTrue(all(v is None for v in r['features'].values()))

    def test_unsupported_human_collider_does_not_silently_disappear(self):
        stage=fixture()
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage,ROOT+'/person/unsupported').GetPrim())
        g=HumanDistanceGeometry(stage,ROOT,['base_link'])
        r=g.sample({'base_link':pose([0,1,0])},{'mustard_bottle':pose([0,2,0])})
        self.assertFalse(r['complete'])
        self.assertTrue(all(v is None for v in r['features'].values()))

    def test_authored_actor_scale_survives_rigid_pose_update(self):
        stage=fixture()
        actor=UsdGeom.Xformable(stage.GetPrimAtPath(ROOT+'/Robot/base_link'))
        actor.AddScaleOp().Set(Gf.Vec3f(1,2,1))
        g=HumanDistanceGeometry(stage,ROOT,['base_link'])
        normal=np.array([-1.,1.,0])/np.sqrt(2)
        r=g.sample({'base_link':pose(normal*.4)},{'mustard_bottle':pose(normal*.5)})
        self.assertTrue(r['complete'])
        self.assertAlmostEqual(r['features']['d_robot_h_min_gt_m'],.25,places=6)


if __name__=='__main__':
    unittest.main(verbosity=2)
