"""CPU geometry/cache invariants for independent mustard candidate search."""
import unittest
import fcl
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.search_mustard_grasp_cpu import RigidRows,declarations


class GraspSearchTests(unittest.TestCase):
    def test_rigid_shape_transform_is_absolute_and_bounds_are_conservative(self):
        obj=fcl.CollisionObject(fcl.Box(.2,.4,.6),fcl.Transform(np.eye(3),[.1,.2,.3]))
        tcp=np.eye(4);tcp[:3,3]=[.7,-.1,.5]
        low=np.array([0.,0.,0.]);high=np.array([.2,.4,.6])
        cache=RigidRows([('palm',obj,low,high)],tcp)
        target=np.eye(4);target[:3,:3]=Rotation.from_euler('xyz',[23,41,-18],degrees=True).as_matrix();target[:3,3]=[-.3,.1,1.]
        row=cache.at(target)[0];first=np.array(row[1].getTranslation());lo,hi=row[2:]
        delta=target@np.linalg.inv(tcp)
        corners=np.array([[x,y,z] for x in (0.,.2) for y in (0.,.4) for z in (0.,.6)])
        transformed=corners@delta[:3,:3].T+delta[:3,3]
        self.assertTrue(np.all(transformed>=lo-1e-12));self.assertTrue(np.all(transformed<=hi+1e-12))
        cache.at(np.eye(4));again=cache.at(target)[0]
        np.testing.assert_allclose(again[1].getTranslation(),first,atol=1e-12)
        np.testing.assert_allclose(cache.at(tcp)[0][1].getTranslation(),[.1,.2,.3],atol=1e-12)

    def test_declared_grid_retains_real_reference_and_covariant_body_offset(self):
        prepared={'profile':{'source_bottle_geometry':{'default_tcp_point_bottle_local_m':[.001,.002,0.]}}}
        reference={'ordered_transfer_candidates':[{'name':'real','bottle_to_tcp_local':np.eye(4).tolist()}]}
        rows=declarations(prepared,reference)
        self.assertEqual(rows[0],reference['ordered_transfer_candidates'][0]);self.assertEqual(len(rows),217)
        a=next(r for r in rows if r['name']=='grid_tilt35_yaw0_offset0.02_z0.01')
        b=next(r for r in rows if r['name']=='grid_tilt35_yaw90_offset0.02_z0.01')
        r=Rotation.from_euler('z',90,degrees=True).as_matrix();center=np.array([.001,.002,0.])
        np.testing.assert_allclose(np.array(b['bottle_to_tcp_local'])[:3,3]-center,r@(np.array(a['bottle_to_tcp_local'])[:3,3]-center),atol=1e-12)
        self.assertEqual(reference['ordered_transfer_candidates'][0]['name'],'real')


if __name__=='__main__':unittest.main()
