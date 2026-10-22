"""CUDA geometry checks: analytical cases, complete edges, slivers and cache motion."""
import unittest
import numpy as np
from kitchen_safety.fcl_distance import primitive_mesh

class WarpMeshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from kitchen_safety import warp_mesh_backend
            cls.w=warp_mesh_backend
        except (ImportError,RuntimeError) as exc:
            raise unittest.SkipTest(str(exc))

    def mesh(self,v,f,t=(0,0,0)):
        m=self.w.BVHModel();m.beginModel();m.addSubModel(np.array(v,float),np.array(f,np.int32));m.endModel()
        return self.w.CollisionObject(m,self.w.Transform(np.eye(3),t))

    def distance(self,a,b):
        r=self.w.DistanceResult()
        return self.w.distance(a,b,self.w.DistanceRequest(),r)

    def test_gap_and_pose_cache_invalidation(self):
        v,f,_=primitive_mesh('Cube',size=1.)
        a=self.mesh(v,f);b=self.mesh(v,f,(1.25,0,0))
        self.assertAlmostEqual(self.distance(a,b),.25,places=6)
        hits=self.w.STATS['cache_hits'];self.distance(a,b)
        self.assertGreater(self.w.STATS['cache_hits'],hits)
        b.setTransform(self.w.Transform(np.eye(3),(1.5,0,0)))
        self.assertAlmostEqual(self.distance(a,b),.5,places=6)

    def test_edge_minimum_not_vertex_sampling(self):
        a=self.mesh([[-1,0,0],[1,0,0],[0,2,0]],[[0,1,2]])
        b=self.mesh([[0,-1,.01],[0,1,.01],[2,0,.01]],[[0,1,2]])
        self.assertAlmostEqual(self.distance(a,b),.01,places=6)
        self.assertAlmostEqual(self.distance(b,a),.01,places=6)

    def test_intersection_and_coplanar_contact(self):
        a=self.mesh([[-1,-1,0],[1,-1,0],[0,1,0]],[[0,1,2]])
        for vertices in ([[0,0,-1],[0,0,1],[1,0,1]],[[-.5,0,0],[.5,0,0],[0,.5,0]]):
            b=self.mesh(vertices,[[0,1,2]])
            self.assertAlmostEqual(self.distance(a,b),0,places=7)
            r=self.w.CollisionResult();self.w.collide(a,b,self.w.CollisionRequest(),r)
            self.assertTrue(r.is_collision)

    def test_degenerate_authored_segment_is_retained(self):
        a=self.mesh([[0,0,0],[1,0,0],[.5,0,0]],[[0,1,2]])
        b=self.mesh([[0,1,0],[1,1,0],[.5,1,0]],[[0,1,2]])
        self.assertAlmostEqual(self.distance(a,b),1,places=6)

    def test_random_triangle_distance_and_collision_reference(self):
        try:import fcl
        except ImportError:raise unittest.SkipTest('Optional FCL reference unavailable')
        from scipy.spatial.transform import Rotation
        rng=np.random.default_rng(428)
        for _ in range(50):
            av=rng.normal(size=(3,3))*.1;bv=rng.normal(size=(3,3))*.1
            rotation=Rotation.random(random_state=rng).as_matrix();translation=rng.uniform(-.2,.2,size=3)
            a=self.mesh(av,[[0,1,2]]);b=self.mesh(bv,[[0,1,2]])
            b.setTransform(self.w.Transform(rotation,translation))
            old=[]
            for vertices in (av,bv):
                m=fcl.BVHModel();m.beginModel();m.addSubModel(vertices,np.array([[0,1,2]]));m.endModel();old.append(fcl.CollisionObject(m))
            old[1].setTransform(fcl.Transform(rotation,translation))
            ref=max(0.,fcl.distance(*old,fcl.DistanceRequest(),fcl.DistanceResult()))
            self.assertAlmostEqual(self.distance(a,b),ref,delta=1.e-6)
            r=self.w.CollisionResult();self.w.collide(a,b,self.w.CollisionRequest(),r)
            rr=fcl.CollisionResult();fcl.collide(*old,fcl.CollisionRequest(),rr)
            self.assertEqual(r.is_collision,rr.is_collision)

if __name__=='__main__':unittest.main()
