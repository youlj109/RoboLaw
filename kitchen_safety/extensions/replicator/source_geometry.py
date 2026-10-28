"""Independent source-collider checks; never changes planning/physics geometry."""
from pathlib import Path
import hashlib
import numpy as np
import fcl
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
ROOT=Path(__file__).resolve().parents[3]

class SourceBottleGeometry:
    def __init__(self, hull=False):
        path=ROOT/'assets/robolab/objects/hope/mustard_bottle.usd'
        self.stage=Usd.Stage.CreateInMemory(); root=UsdGeom.Xform.Define(self.stage,'/Bottle')
        root.GetPrim().GetReferences().AddReference(str(path));root.ClearXformOpOrder()
        cache=UsdGeom.XformCache(); self.parts=[]; descriptions=[]
        for prim in Usd.PrimRange(root.GetPrim(),Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
            if not prim.IsA(UsdGeom.Mesh):raise ValueError('Expected source bottle mesh collider')
            mesh=UsdGeom.Mesh(prim); transform=np.asarray(cache.GetLocalToWorldTransform(prim)).T
            vertices=np.asarray(mesh.GetPointsAttr().Get())@transform[:3,:3].T+transform[:3,3]
            if hull: faces=ConvexHull(vertices).simplices.astype(np.int32)
            else:
                indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
                for count in mesh.GetFaceVertexCountsAttr().Get():
                    p=indices[offset:offset+count];offset+=count
                    faces.extend([[p[0],p[i],p[i+1]] for i in range(1,count-1)])
                faces=np.asarray(faces,dtype=np.int32)
            model=fcl.BVHModel();model.beginModel(len(vertices),len(faces));model.addSubModel(vertices,faces);model.endModel()
            self.parts.append((str(prim.GetPath()),fcl.CollisionObject(model),vertices))
            descriptions.append(dict(path=str(prim.GetPath()),vertices=len(vertices),triangles=len(faces),source_approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(),local_to_root=transform.tolist()))
        self.provenance=dict(source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),mode='source_vertex_convex_hull_triangle_surface' if hull else 'authored_source_collision_triangle_surface',parts=descriptions,limitations='Neither source triangle mesh nor its convex hull reproduces PhysX cooked convex-decomposition/contact offsets. Zero/positive FCL surface distances are not a physical contact guarantee.')
    def shapes(self,pose):
        pose=np.asarray(pose);rotation=Rotation.from_quat(pose[3:7]).as_matrix();translation=pose[:3];rows=[]
        for name,obj,vertices in self.parts:
            obj.setTransform(fcl.Transform(rotation,translation));v=vertices@rotation.T+translation
            rows.append(('source_bottle'+name,obj,v.min(0),v.max(0)))
        return rows

def source_clearance(world,bottle,q,gripper,pose,hazard,required=.02):
    shapes=[s for s in world.shapes(q,gripper) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
    return minimum_distance(shapes+bottle.shapes(pose),hazard,required)
