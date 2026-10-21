import sys,json
sys.path.insert(0,'/workspace')
import numpy as np,h5py,fcl
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.collision_planner import CollisionWorld
s=Usd.Stage.CreateInMemory();person=UsdGeom.Xform.Define(s,'/World/person');person.GetPrim().GetReferences().AddReference('/workspace/assets/people/mpfb/collidable.usda');person.AddTranslateOp().Set(Gf.Vec3d(3.08,-.91,0))
w=CollisionWorld(s,DroidKinematics([2.2,-.93,.8],90))
with h5py.File('/workspace/datasets/kitchen_auto/property/episode_001/trajectory.h5') as f:q=f['obs/q'][::4,:7]
shapes=[w.shapes(v) for v in q]
for dx in [-.25,-.15,0,.1]:
 for dy in [-.25,-.15,0,.15,.25]:
  off=np.array([dx,dy,0.]);minimum=.20;frame=None
  for name,obj,lo,hi in w.objects:obj.setTranslation(off)
  for k,ss in enumerate(shapes):
   for label,shape,a,b in ss:
    for name,obj,lo,hi in w.objects:
     lower=np.linalg.norm(np.maximum(np.maximum(lo+off-b,a-hi-off),0))
     if lower>max(0,minimum):continue
     d=fcl.distance(shape,obj,fcl.DistanceRequest(),fcl.DistanceResult())
     if d<minimum:minimum=d;frame=k*4
     if d<0:break
    if minimum<0:break
   if minimum<0:break
  print('LAYOUT',[round(3.08+dx,2),round(-.91+dy,2),0],'min',round(minimum,4),'frame',frame,flush=True)
