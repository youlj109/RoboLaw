"""A recorded rigid-body pose must retain authored USD mesh scale."""
import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
class Object:
    def setTransform(self, transform): self.rotation,self.translation=transform
class ScaledBackgroundTest(unittest.TestCase):
    def test_nonuniform_scale_is_retained_when_pose_changes(self):
        root='/World/envs/env_0/kitchen/item'
        name=root+'/mesh';local=np.array([[0,0,0],[1,0,0],[0,2,0],[0,0,3]],float)
        source=np.eye(4);source[:3,:3]=Rotation.from_euler('z',.3).as_matrix()@np.diag([.01,.02,.03]);source[:3,3]=[.5,-.4,.9]
        vertices=local@source[:3,:3].T+source[:3,3];faces=np.array([[0,1,2],[0,1,3],[0,2,3],[1,2,3]])
        lo,hi=vertices.min(0),vertices.max(0);obj=Object();world=CollisionWorld.__new__(CollisionWorld)
        world.fcl=SimpleNamespace(Transform=lambda r,t:(r,t));world._last_obstacle_pose_call=None
        world.objects=[(name,obj,lo,hi)];world.meshes={name:(vertices,faces)}
        world._root_reference_transforms={root:source};world._object_reference=[(np.eye(3),np.zeros(3),lo,hi,(vertices,faces))]
        world.low=np.array([lo]);world.high=np.array([hi])
        target_rotation=Rotation.from_euler('xyz',[.1,.2,.6]);pose=np.r_[[-.2,.7,1.2],target_rotation.as_quat()]
        self.assertEqual(world.set_obstacle_pose(root,pose),1)
        expected=local@(target_rotation.as_matrix()@np.diag([.01,.02,.03])).T+pose[:3]
        np.testing.assert_allclose(world.meshes[name][0],expected,atol=1e-12)
        np.testing.assert_allclose(world.low[0],expected.min(0),atol=1e-12)
        np.testing.assert_allclose(obj.rotation.T@obj.rotation,np.eye(3),atol=1e-12)
if __name__=='__main__':unittest.main()
