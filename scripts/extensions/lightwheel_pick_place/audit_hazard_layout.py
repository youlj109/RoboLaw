"""CPU audit of proposed hazards against every original collider and saved rollout.

Counterfactual replay is geometry evidence only; it cannot replace a new physical
variant rollout with the original predicate, contacts and cameras.
"""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
p=argparse.ArgumentParser();p.add_argument('--episode',required=True);p.add_argument('--output',required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[3];ep=Path(a.episode);r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2])
report={'episode':str(ep),'source_yaml_sha256':r['source_yaml_sha256'],'variants':{}}
for variant,spec in json.loads((root/'configs/extensions/lightwheel_pick_place/variants.json').read_text()).items():
 s=Usd.Stage.Open(str(ep/'scene.usda'));s.SetEditTarget(s.GetSessionLayer());x=UsdGeom.Xform.Define(s,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(root/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(spec.get('yaw_deg',0.))
 w=CollisionWorld(s,kin)
 for name,state in r['initial_object_states'].items():w.set_obstacle_pose(name,state[:7])
 overlaps=[];supports=[]
 for hn,ho,hl,hh in w.objects:
  if not w._matches(hn,spec['name']):continue
  for on,oo,ol,oh in w.objects:
   if w._matches(on,spec['name']):continue
   if np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0))>.001:continue
   raw=float(w.fcl.distance(ho,oo,w.fcl.DistanceRequest(),w.fcl.DistanceResult()))
   if raw>=0 and not (raw==0 and w.fcl.collide(ho,oo,w.fcl.CollisionRequest(),w.fcl.CollisionResult())):continue
   row={'hazard_part':hn,'source_collider':on,'raw_fcl_distance_m':raw,'hazard_bottom_m':float(hl[2]),'source_top_m':float(oh[2])}
   support=('/floor' in on and hl[2]>=oh[2]-.006) or ('/counter_' in on and hl[2]>=oh[2]-.006)
   (supports if support else overlaps).append(row)
 with h5py.File(ep/'trajectory.h5') as f:
  rows=[]
  for i,q in enumerate(f['next_obs/q']):
   c=w.clearance(q[:7],('lime',),object_pose=f['next_obs/lime'][i,:7],obstacle_filter=[spec['name']],mode='execution',gripper_q=dict(zip(w.GRIPPER_JOINT_NAMES,q[7:])))
   rows.append({'action':i,**c})
 report['variants'][variant]={'spec':spec,'non_support_intersections':overlaps,'support_contacts':supports,'baseline_action_states':len(rows),'baseline_violations':sum(not x['valid'] for x in rows),'baseline_minimum':min(rows,key=lambda x:x['slack_m']) if rows else None,'measurements':rows}
Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:{n:v for n,v in x.items() if n!='measurements'} for k,x in report['variants'].items()},indent=2))
