"""CPU exact collision-mesh and primitive baseline comparison of current variants."""
import json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from scripts.extensions.doors.search_avoidance_layouts import clearance
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
root=Path(__file__).resolve().parents[3];ep=root/'datasets/avoidance_trials/doors/base_adaptive_attempt_01';m=json.loads((ep/'metadata.json').read_text());rows=json.loads((ep/'trajectory.json').read_text());results={}
for variant,spec in json.loads((root/'configs/extensions/doors/avoidance_variants.json').read_text()).items():
 if variant.startswith('_'):continue
 s=Usd.Stage.CreateInMemory();x=UsdGeom.Xform.Define(s,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(root/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(spec.get('yaw_deg',0));w=DoorCollisionWorld(s,DroidKinematics(m['robot_base'],m['robot_yaw_deg']),env_root='/World/envs/env_0');hazards=physical_hazards(s,w,spec['name']);out=[]
 for i,row in enumerate(rows):
  q=row['next_obs']['q'];angles=dict(zip(m['robot_joint_names'],q));shapes=physical_robot_shapes(w.shapes(np.array(q[:7]),{n:angles[n] for n in w.GRIPPER_JOINT_NAMES}));out.append({'index':i,'phase':row['phase'],**clearance(shapes,hazards,w.execution_clearances[spec['name']],True)})
 results[variant]={'minimum':min(out,key=lambda x:x['slack_m']),'violations':[x for x in out if not x['valid']]}
print(json.dumps({k:{'minimum':v['minimum'],'violations':len(v['violations'])} for k,v in results.items()},indent=2));(root/'reports/avoidance_trials/doors/counterfactual_physical.json').write_text(json.dumps(results,indent=2))
