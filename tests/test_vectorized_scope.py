import unittest
from unittest.mock import patch
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.grasp_contacts import BottleGraspContacts

class VectorizedScopeTests(unittest.TestCase):
    def test_translated_collision_world_and_live_pose_are_local_and_isolated(self):
        stage=Usd.Stage.CreateInMemory()
        for i,offset in enumerate(([0.,0.,0.],[30.,-30.,0.])):
            root=UsdGeom.Xform.Define(stage,f'/World/envs/env_{i}')
            root.AddTranslateOp().Set(Gf.Vec3d(*offset))
            box=UsdGeom.Cube.Define(stage,f'/World/envs/env_{i}/glass')
            box.CreateSizeAttr(.1)
            box.AddTranslateOp().Set(Gf.Vec3d(2.,-.4,1.))
            UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        # Robot-shape construction is independent of environment scoping.
        with patch.object(CollisionWorld,'_robot_hulls',return_value=[]), patch.object(CollisionWorld,'_gripper_geometry',return_value=([],[])):
            worlds=[CollisionWorld(stage,None,env_root=f'/World/envs/env_{i}',env_origin=offset)
                    for i,offset in enumerate(([0.,0.,0.],[30.,-30.,0.]))]
        for i,w in enumerate(worlds):
            self.assertEqual(len(w.objects),1)
            self.assertIn(f'/env_{i}/',w.objects[0][0])
        np.testing.assert_allclose(worlds[0].low,worlds[1].low,atol=1e-12)
        np.testing.assert_allclose(worlds[0].high,worlds[1].high,atol=1e-12)
        initial=worlds[0].low.copy()
        worlds[1].set_obstacle_pose('glass',[2.02,-.4,1.,0.,0.,0.,1.])
        np.testing.assert_allclose(worlds[1].low,initial+[.02,0,0],atol=1e-12)
        np.testing.assert_array_equal(worlds[0].low,initial)

    def test_contacts_match_bottle_coordinates_in_translated_environment(self):
        class Contact:
            def get_contact_data(self,dt):
                return (np.array([[2.]]),np.array([[32.,-30.4,1.1]]),np.array([[0.,0.,1.]]),np.array([[0.]]),np.array([[1]]),np.array([[0]]))
        class Physics:
            def __init__(self): self.calls=[]
            def create_rigid_contact_view(self,paths,**kwargs):
                self.calls.append((paths,kwargs['filter_patterns']));return Contact()
        physics=Physics()
        contacts=BottleGraspContacts(physics,1/120,env_root='/World/envs/env_1',env_origin=[30.,-30.,0.])
        row=contacts.sample('lift',0,[2.,-.4,1.,0.,0.,0.,1.])
        for paths,filters in physics.calls:
            self.assertIn('/env_1/',paths[0]);self.assertIn('/env_1/',filters[0][0])
        for finger in row['fingers'].values():
            np.testing.assert_allclose(finger['points_bottle_local_m'],[[0.,0.,.1]],atol=1e-12)
            np.testing.assert_allclose(finger['points_world_m'],[[32.,-30.4,1.1]])

if __name__=='__main__':unittest.main()
