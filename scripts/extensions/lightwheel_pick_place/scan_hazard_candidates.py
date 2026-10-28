"""Read-only CPU candidate scan: original layout collision and camera frustum.

No layout/config changes and no GPU. Frustum inclusion does not prove visibility
through occluders; physical variant previews remain necessary.
"""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
ROOT=Path(__file__).resolve().parents[3]
ep=ROOT/'datasets/skill_trials/lightwheel_pick_place/base_attempt11'
r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2])
with h5py.File(ep/'trajectory.h5') as f:
 qs=f['next_obs/q'][:];limes=f['next_obs/lime'][:,:7]
cam_meta=json.loads((ep/'cameras.json').read_text());cams={}
with h5py.File(ep/'images.h5') as f:
 for name,m in cam_meta.items():cams[name]=(f[name]['position_world'][0],Rotation.from_quat(f[name]['quaternion_world_ros_xyzw'][0]).as_matrix(),np.array(m['intrinsics']),m['width'],m['height'])
report={'source_episode':str(ep.relative_to(ROOT)),'visibility_scope':'Frustum only, no occlusion proof','variants':{}}
for variant,spec in json.loads((ROOT/'configs/extensions/lightwheel_pick_place/variants.json').read_text()).items():
 if variant=='property':continue
 stage=Usd.Stage.Open(str(ep/'scene.usda'));stage.SetEditTarget(stage.GetSessionLayer());x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(0.)
 w=CollisionWorld(stage,kin)
 for name,state in r['initial_object_states'].items():w.set_obstacle_pose(name,state[:7])
 positions=([(x,y,0.) for x in [3.1,3.25,3.4,3.7,4.,4.3,4.6,4.9] for y in [-1.05,-1.3,-1.55,-1.8]] if variant=='human' else [(x,y,.922) for x in [3.45,3.6,3.75,3.9,4.05,4.2,4.35,4.5] for y in [-.18,-.30,-.42,-.54]])
 rows=[]
 for pos in positions:
  w.set_obstacle_pose(spec['name'],[*pos,0,0,0,1]);hazards=[v for v in w.objects if w._matches(v[0],spec['name'])];overlap=[];mind=float('inf');closest=None
  for hn,ho,hl,hh in hazards:
   for on,oo,ol,oh in w.objects:
    if w._matches(on,spec['name']):continue
    broad=float(np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0)))
    if broad>.08:continue
    raw=float(w.fcl.distance(ho,oo,w.fcl.DistanceRequest(),w.fcl.DistanceResult()))
    support=('/floor' in on and hl[2]>=oh[2]-.006) or ('/counter_' in on and hl[2]>=oh[2]-.006)
    if not support and raw<mind:mind,closest=raw,on
    if support or (raw>0):continue
    if raw<0 or w.fcl.collide(ho,oo,w.fcl.CollisionRequest(),w.fcl.CollisionResult()):overlap.append(on)
  row={'position':list(pos),'non_support_intersections':sorted(set(overlap)),'source_min_distance_m':mind if np.isfinite(mind) else None,'closest_source':closest}
  if not overlap:
   measurements=[]
   for i in sorted(set([0,len(qs)//2,len(qs)-1])):
    measurements.append(w.clearance(qs[i,:7],('lime',),object_pose=limes[i],obstacle_filter=[spec['name']],mode='execution',gripper_q=dict(zip(w.GRIPPER_JOINT_NAMES,qs[i,7:]))))
   row['sampled_robot_minimum']=min(measurements,key=lambda v:v['slack_m']);row['sampled_robot_valid']=all(v['valid'] for v in measurements)
   points=[]
   for name,obj,lo,hi in hazards:
    if name in w.meshes:points.extend(w.meshes[name][0][::max(1,len(w.meshes[name][0])//500)])
    else:points.extend([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
   points=np.array(points);visibility={}
   for name,(origin,rot,K,width,height) in cams.items():
    local=(points-origin)@rot;uv=local@K.T;uv=uv[:,:2]/np.maximum(uv[:,2:],1e-9)
    inside=(local[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    visibility[name]={'in_frustum_fraction':float(inside.mean()),'points_in_frustum':int(inside.sum())}
   row['frustum']=visibility
  rows.append(row)
 valid=[v for v in rows if not v['non_support_intersections'] and v.get('sampled_robot_valid')]
 valid.sort(key=lambda v:sum(v['frustum'][n]['in_frustum_fraction'] for n in ('external_camera','external_camera_2')),reverse=True)
 report['variants'][variant]={'candidates':rows,'recommended_geometric_candidates':valid[:8]}
 print(variant,json.dumps(valid[:8]),flush=True)
out=ROOT/'reports/skill_trials/lightwheel_pick_place/hazard_candidate_scan.json';out.write_text(json.dumps(report,indent=2))
