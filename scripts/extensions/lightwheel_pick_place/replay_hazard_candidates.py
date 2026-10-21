"""CPU replay of saved baseline measurements against proposed hazard positions.

No hazard configuration edits or physical counterfactual execution.
"""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
ROOT=Path(__file__).resolve().parents[3];ep=ROOT/'datasets/skill_trials/lightwheel_pick_place/base_attempt12'
r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2])
with h5py.File(ep/'trajectory.h5') as f:qs=f['next_obs/q'][:];limes=f['next_obs/lime'][:,:7]
report={'episode':str(ep.relative_to(ROOT)),'scope':'15Hz measured arm/finger/lime replay; geometric counterfactual only','variants':{}}
for variant,spec in json.loads((ROOT/'configs/extensions/lightwheel_pick_place/variants.json').read_text()).items():
 if variant=='property':continue
 stage=Usd.Stage.Open(str(ep/'scene.usda'));stage.SetEditTarget(stage.GetSessionLayer());x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(0.)
 w=CollisionWorld(stage,kin);candidates=([[3.25,-1.05,0],[3.4,-1.05,0],[3.55,-1.05,0],[3.7,-1.05,0]] if variant=='human' else [[4.2,-.18,.922],[4.35,-.18,.922],[3.45,-.18,.922]])
 rows=[]
 for position in candidates:
  w.set_obstacle_pose(spec['name'],[*position,0,0,0,1]);measurements=[]
  for i,q in enumerate(qs):
   c=w.clearance(q[:7],('lime',),object_pose=limes[i],obstacle_filter=[spec['name']],mode='execution',gripper_q=dict(zip(w.GRIPPER_JOINT_NAMES,q[7:])))
   measurements.append({'action':i,**c})
  row={'position':position,'count':len(measurements),'violations':sum(not c['valid'] for c in measurements),'minimum':min(measurements,key=lambda c:c['slack_m']),'measurements':measurements}
  rows.append(row);print(variant,json.dumps({k:v for k,v in row.items() if k!='measurements'}),flush=True)
 report['variants'][variant]=rows
(ROOT/'reports/skill_trials/lightwheel_pick_place/hazard_candidate_replay.json').write_text(json.dumps(report,indent=2))
