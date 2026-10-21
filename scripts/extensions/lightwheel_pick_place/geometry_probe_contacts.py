"""Read-only geometric support for the +15mm lime grasp and proposed vase placement."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
root=Path(__file__).resolve().parents[3];r=json.loads((root/'datasets/skill_trials/lightwheel_pick_place/base_attempt06/result.json').read_text());base=r['robot_base_pose']
kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2]);s=Usd.Stage.Open(str(root/'reports/skill_trials/lightwheel_pick_place/inspection/stage.usda'));s.SetEditTarget(s.GetSessionLayer())
for name,pose in r['initial_object_states'].items():
 x=UsdGeom.Xformable(s.GetPrimAtPath('/World/envs/env_0/'+name));x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(transform(pose[:3],pose[3:7]).T.tolist()))
spec=json.loads((root/'configs/extensions/lightwheel_pick_place/variants.json').read_text())['property'];x=UsdGeom.Xform.Define(s,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(root/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
world=CollisionWorld(s,kin);candidate=next(x for x in json.loads((root/'reports/skill_trials/lightwheel_pick_place/cpu_grasp_probe.json').read_text()) if x['tilt']==35 and x['yaw']==0 and x['offset']==.015);q=np.array(candidate['q'])
limes=[x for x in world.objects if world._matches(x[0],'lime')];rows=[]
for angle in np.linspace(0,.65,27):
 angles=dict(zip(world.GRIPPER_JOINT_NAMES,np.array([1,1,-1,1,-1,-1])*angle));shapes=world.shapes(q,gripper_q=angles);pairs={}
 for name,obj,lo,hi in shapes:
  if name not in ('gripper/left_inner_finger','gripper/right_inner_finger'):continue
  ds=[float(world.fcl.distance(obj,other,world.fcl.DistanceRequest(),world.fcl.DistanceResult())) for _,other,*_ in limes]
  pairs[name]={'minimum_raw_fcl_distance_m':min(ds),'overlap':min(ds)<0,'lower_z':float(lo[2]),'upper_z':float(hi[2])}
 rows.append({'finger_joint':float(angle),'inner_fingers':pairs,'counter_clearance':world.clearance(q,('lime','glass'),gripper_q=angles,obstacle_filter=['top_geometry_right'])})
# Compare vase pieces to all source colliders, excluding the vase itself.
intersections=[];support=[]
for hn,ho,hl,hh in world.objects:
 if not world._matches(hn,'glass'):continue
 for on,oo,ol,oh in world.objects:
  if world._matches(on,'glass'):continue
  bound=np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0))
  if bound>.001:continue
  raw=float(world.fcl.distance(ho,oo,world.fcl.DistanceRequest(),world.fcl.DistanceResult()))
  collides=raw<0 or (raw==0 and bool(world.fcl.collide(ho,oo,world.fcl.CollisionRequest(),world.fcl.CollisionResult())))
  if not collides:continue
  record={'vase_collider':hn,'other_collider':on,'raw_fcl_distance_m':raw,'vase_bottom':float(hl[2]),'other_top':float(oh[2])}
  if '/counter_' in on and hl[2]>=oh[2]-.006:support.append(record)
  else:intersections.append(record)
report={'candidate':{k:candidate[k] for k in ['tilt','yaw','offset']},'lime_collider_count':len(limes),'closing_sweep':rows,'vase_spec':spec,'vase_non_support_intersections':intersections,'vase_support_contacts':support,'scope':'Geometry-only overlapping fingers show grasp-contact possibility, not a physical grasp or task success.'}
(root/'reports/skill_trials/lightwheel_pick_place/geometry_probe_contacts.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
