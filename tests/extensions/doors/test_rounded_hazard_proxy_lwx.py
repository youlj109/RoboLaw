"""Prove full Euclidean clearance containment using every convex facet."""
import unittest
from collections import Counter
import numpy as np
from scipy.spatial import ConvexHull
from kitchen_safety.extensions.doors.rounded_hazard_proxy_lwx import rounded_outer_aabb
class RoundedHazardProxyTest(unittest.TestCase):
 def test_source_convex_hull_plus_full_clearance_is_contained(self):
  angle=np.linspace(0,2*np.pi,24,endpoint=False);source=np.array([[.025*np.cos(a),.025*np.sin(a),z] for a in angle for z in [0.,.15]])
  v,f=rounded_outer_aabb(source.min(0),source.max(0),.06,source);h=ConvexHull(v);n=h.equations[:,:3];d=h.equations[:,3]
  exact_support=(source@n.T).max(0)+.06*np.linalg.norm(n,axis=1)
  self.assertGreaterEqual(float((-d-exact_support).min()),-1e-9)
  # Every edge belongs to exactly two triangles, all triangles point outward.
  edges=Counter(tuple(sorted((int(a),int(b)))) for face in f for a,b in zip(face,np.roll(face,-1)))
  self.assertTrue(all(count==2 for count in edges.values()))
  center=v.mean(0);tri=v[f];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);self.assertTrue(np.all(np.einsum('ij,ij->i',normal,tri.mean(1)-center)>0))
 def test_no_clearance_is_silently_removed(self):
  with self.assertRaises(ValueError):rounded_outer_aabb([0,0,0],[1,1,1],0.)
if __name__=='__main__':unittest.main()
