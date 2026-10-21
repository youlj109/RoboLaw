"""CPU check of MPFB candidate placement against the exact static DROID stand."""
import json
from pathlib import Path
import numpy as np,fcl
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
ROOT=Path(__file__).resolve().parents[3];ep=ROOT/'datasets/skill_trials/lightwheel_pick_place/base_attempt12';r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);s=Usd.Stage.Open(str(ep/'scene.usda'));s.SetEditTarget(s.GetSessionLayer());spec=json.loads((ROOT/'configs/extensions/lightwheel_pick_place/variants.json').read_text())['human'];x=UsdGeom.Xform.Define(s,'/World/envs/env_0/person');x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(0,0,0));w=CollisionWorld(s,kin);cache=UsdGeom.XformCache();stands=[]
for p in Usd.PrimRange(s.GetPrimAtPath('/World/envs/env_0/Robot'),Usd.TraverseInstanceProxies()):
 if 'stand' not in str(p.GetPath()).lower() or not p.HasAPI(UsdPhysics.CollisionAPI) or not p.IsA(UsdGeom.Mesh):continue
 m=UsdGeom.Mesh(p);v=np.array(m.GetPointsAttr().Get());mat=np.array(cache.GetLocalToWorldTransform(p)).T;v=v@mat[:3,:3].T+mat[:3,3];indices=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
 for count in m.GetFaceVertexCountsAttr().Get():
  polygon=indices[offset:offset+count];offset+=count
  for j in range(1,count-1):faces.append([polygon[0],polygon[j],polygon[j+1]])
 model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,np.array(faces,dtype=np.int32));model.endModel();stands.append((str(p.GetPath()),fcl.CollisionObject(model)))
 print('STAND',str(p.GetPath()),v.min(0),v.max(0),flush=True)
rows=[]
for pos in [[x,-1.05,0] for x in [3.1,3.2,3.25,3.3,3.35,3.4,3.55,3.7]]:
 w.set_obstacle_pose('person',[*pos,0,0,0,1]);measurements=[]
 for hn,ho,*_ in w.objects:
  if not w._matches(hn,'person'):continue
  for sn,so in stands:
   raw=float(fcl.distance(ho,so,fcl.DistanceRequest(),fcl.DistanceResult()));hit=bool(fcl.collide(ho,so,fcl.CollisionRequest(),fcl.CollisionResult()));measurements.append({'hazard_part':hn,'stand':sn,'distance_m':raw,'intersecting':hit})
 row={'position':pos,'intersecting':any(x['intersecting'] for x in measurements),'minimum':min(measurements,key=lambda x:x['distance_m'])};rows.append(row);print(json.dumps(row),flush=True)
(ROOT/'reports/skill_trials/lightwheel_pick_place/human_stand_candidates.json').write_text(json.dumps({'episode':str(ep),'candidates':rows},indent=2))
