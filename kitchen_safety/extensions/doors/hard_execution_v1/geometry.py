"""Read-only source door cuboids retained as affine triangles for live audits."""
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
class MeasuredDoorGeometry:
    def __init__(self,stage,path):
        import fcl
        self.rows=[];self.reference=[];cache=UsdGeom.XformCache()
        faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]],dtype=np.int32)
        for p in Usd.PrimRange(stage.GetPrimAtPath(path),Usd.TraverseInstanceProxies()):
            if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
            if not p.IsA(UsdGeom.Cube):raise RuntimeError('Unexpected non-Cube source door collider '+str(p.GetPath()))
            size=float(UsdGeom.Cube(p).GetSizeAttr().Get())/2.;v=np.array([[x,y,z] for x in (-size,size) for y in (-size,size) for z in (-size,size)])
            m=np.array(cache.GetLocalToWorldTransform(p)).T;v=v@m[:3,:3].T+m[:3,3]
            model=fcl.BVHModel();model.beginModel(8,12);model.addSubModel(v,faces);model.endModel()
            self.rows.append((str(p.GetPath()),fcl.CollisionObject(model),v.min(0),v.max(0)));self.reference.append(v)
    def measured(self,delta):
        rows=[]
        for (name,obj,_,_),vertices in zip(self.rows,self.reference):
            obj.setRotation(delta[:3,:3]);obj.setTranslation(delta[:3,3]);v=vertices@delta[:3,:3].T+delta[:3,3];rows.append((name,obj,v.min(0),v.max(0)))
        return rows
