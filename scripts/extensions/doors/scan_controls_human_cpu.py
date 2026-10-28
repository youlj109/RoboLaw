"""Suggest controls human positions without editing config or running physics."""
import json
from pathlib import Path
import h5py,numpy as np,fcl
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
ROOT=Path(__file__).resolve().parents[3];ep=ROOT/'datasets/skill_trials/controls/base_attempt03';r=json.loads((ep/'result.json').read_text());b=r['initial_robot_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);records=json.loads((ep/'records.json').read_text());meta=json.loads((ep/'cameras.json').read_text());cams={}
with h5py.File(ep/'images.h5') as f:
 for name,m in meta.items():cams[name]=(f[name]['position_world'][0],Rotation.from_quat(f[name]['quaternion_world_ros_xyzw'][0]).as_matrix(),np.array(m['intrinsics']),m['width'],m['height'])
s=Usd.Stage.Open(str(ROOT/'reports/skill_trials/controls/inspection/stage.usda'));s.SetEditTarget(s.GetSessionLayer());spec=json.loads((ROOT/'configs/extensions/controls/left_button_trial.json').read_text())['variants']['human'];x=UsdGeom.Xform.Define(s,'/World/envs/env_0/person');x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(0,0,0));w=ControlsCollisionWorld(s,kin,env_root='/World/envs/env_0');xc=UsdGeom.XformCache();fixed=[]
for p in Usd.PrimRange(s.GetPrimAtPath('/World/envs/env_0/Robot/panda_link0'),Usd.TraverseInstanceProxies()):
 if not p.HasAPI(UsdPhysics.CollisionAPI) or not p.IsA(UsdGeom.Mesh):continue
 m=UsdGeom.Mesh(p);v=np.array(m.GetPointsAttr().Get());mat=np.array(xc.GetLocalToWorldTransform(p)).T;v=v@mat[:3,:3].T+mat[:3,3];idx=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
 for n in m.GetFaceVertexCountsAttr().Get():
  poly=idx[offset:offset+n];offset+=n
  for j in range(1,n-1):faces.append([poly[0],poly[j],poly[j+1]])
 model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,np.array(faces,dtype=np.int32));model.endModel();fixed.append((str(p.GetPath()),fcl.CollisionObject(model),v.min(0),v.max(0)))
rows=[];names=list(json.loads((ROOT/'reports/skill_trials/controls/inspection/state.json').read_text())['robot_joints']);ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
for pos in [[x,y,0] for x in [2.65,2.75,2.85,2.95,3.05] for y in [-.9,-1.1,-1.3,-1.5]]:
 quat=Rotation.from_euler('z',-30,degrees=True).as_quat();w.set_obstacle_pose('person',[*pos,*quat]);hazards=[o for o in w.objects if w._matches(o[0],'person')];overlap=[];standmin=float('inf')
 for hn,ho,hl,hh in hazards:
  for on,oo,ol,oh in [o for o in w.objects if not w._matches(o[0],'person')]+fixed:
   isfixed='/Robot/' in on
   broad=float(np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0)))
   if broad>.15 and not isfixed:continue
   raw=float(fcl.distance(ho,oo,fcl.DistanceRequest(),fcl.DistanceResult()))
   if isfixed:standmin=min(standmin,raw)
   support=('/floor' in on and hl[2]>=oh[2]-.006) or ('/counter_' in on and hl[2]>=oh[2]-.006)
   if not support and (raw<0 or (raw==0 and fcl.collide(ho,oo,fcl.CollisionRequest(),fcl.CollisionResult()))):overlap.append(on)
 row={'position':pos,'yaw_deg':-30,'initial_intersections':sorted(set(overlap)),'stand_fixed_base_minimum_m':standmin}
 if not overlap:
  measures=[]
  for i,record in enumerate(records):
   q=np.array(record['after']['q']);joints=dict(zip(names,q));c=w.clearance(q[ids],mode='execution',gripper_q={k:joints[k] for k in w.GRIPPER_JOINT_NAMES},obstacle_filter=['person']);measures.append({'action_index':i,'phase':record['phase'],**c})
  row['baseline_violations']=sum(not c['valid'] for c in measures);row['baseline_minimum']=min(measures,key=lambda c:c['slack_m'])
  points=np.concatenate([w.meshes[hn][0][::max(1,len(w.meshes[hn][0])//500)] for hn,ho,*_ in hazards if hn in w.meshes]);vis={}
  for name,(origin,rot,K,width,height) in cams.items():
   local=(points-origin)@rot;uv=local@K.T;uv=uv[:,:2]/np.maximum(uv[:,2:],1e-9);inside=(local[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height);vis[name]=float(inside.mean())
  row['frustum_fraction']=vis
 rows.append(row);print(json.dumps(row),flush=True)
(ROOT/'reports/skill_trials/doors/controls_human_candidates.json').write_text(json.dumps({'source_episode':str(ep),'scope':'Exact fixed base/stand and source kitchen initial intersections plus 282-state baseline geometric counterfactual. Frustum has no occlusion test.','candidates':rows},indent=2))
