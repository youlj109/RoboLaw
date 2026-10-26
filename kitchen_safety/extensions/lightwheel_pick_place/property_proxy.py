"""Conservative Euclidean stand-off mesh for the original compound vase."""
import itertools
import numpy as np
from scipy.spatial import ConvexHull

def padded_hull(vertices, radius):
    vertices=np.asarray(vertices,dtype=float)
    directions=np.array([v for v in itertools.product((-1.,0.,1.),repeat=3) if any(v)])
    directions/=np.linalg.norm(directions,axis=1)[:,None]
    ball=ConvexHull(directions)
    # Circumscription guarantees coverage of the entire Euclidean margin ball.
    inradius=float((-ball.equations[:,3]).min())
    offsets=directions/inradius*radius
    source=vertices[ConvexHull(vertices).vertices]
    points=(source[:,None,:]+offsets[None,:,:]).reshape(-1,3)
    hull=ConvexHull(points);faces=hull.simplices.copy()
    for i,face in enumerate(faces):
        a,b,c=points[face]
        if np.dot(np.cross(b-a,c-a),hull.equations[i,:3])<0:faces[i]=face[::-1]
    return points,faces,{'planning_margin_m':float(radius),'unit_ball_polytope_inradius':inradius,'source_vertices':len(vertices),'proxy_vertices':len(points),'coverage':'convex hull of original collision vertices plus a circumscribed Euclidean margin ball'}

class PlanningView:
    def __init__(self,world):
        self.original=world;self.objects=list(world.objects);self.meshes=dict(world.meshes)
        names=[n for n,_,_,_ in self.objects if world._matches(n,'glass')]
        self.report=None
        if not names:return
        geometry=[]
        for n,obj,lo,hi in self.objects:
            if n not in names:continue
            geometry.append(self.meshes[n][0] if n in self.meshes else np.array(list(itertools.product(*zip(lo,hi)))))
        vertices=np.concatenate(geometry)
        v,f,self.report=padded_hull(vertices,max(world._margin(n,'planning') for n in names))
        self.objects=[o for o in self.objects if o[0] not in names]
        self.meshes={n:m for n,m in self.meshes.items() if n not in names}
        name='/planning_property_margin_mesh'
        self.objects.append((name,None,v.min(0),v.max(0)));self.meshes[name]=(v,f)
    def __getattr__(self,name):return getattr(self.original,name)
