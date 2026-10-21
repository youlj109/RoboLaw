"""CPU-only measured-layout grasp feasibility probes; no physical execution."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
r=json.loads(Path('datasets/skill_trials/lightwheel_pick_place/base_attempt06/result.json').read_text())
s=Usd.Stage.Open('reports/skill_trials/lightwheel_pick_place/inspection/stage.usda');s.SetEditTarget(s.GetSessionLayer())
for name,pose in r['initial_object_states'].items():
 x=UsdGeom.Xformable(s.GetPrimAtPath('/World/envs/env_0/'+name));x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(transform(pose[:3],pose[3:7]).T.tolist()))
base=r['robot_base_pose'];kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2]);world=CollisionWorld(s,kin)
center=(transform(r['initial_lime_pose'][:3],r['initial_lime_pose'][3:7])@np.r_[world.object_center,1])[:3]
seed=np.deg2rad([0,-60,0,-150,0,90,0]);rows=[]
for tilt,yaw,offset in [(35,0,0),(35,0,.01),(35,0,.015),(25,0,0),(25,0,.01),(20,0,.015),(35,-45,.01),(35,90,.01),(35,180,0),(35,180,.01),(45,-90,.01)]:
 row={'tilt':tilt,'yaw':yaw,'offset':offset};target=np.eye(4)
 target[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
 target[:3,3]=center+[0,0,.125+offset];fail=[]
 try:
  q=kin.ik(target,seed,attempts=8);row['pregrasp_q']=q.tolist();path=[q.copy()]
  for z in np.linspace(target[2,3],center[2]+offset,64):
   target[2,3]=z;q=kin.ik(target,q,attempts=1);path.append(q.copy())
   if not world.valid(q,('lime',)):fail.append({'z':float(z),'collision':world.last_clearance})
  row['q']=q.tolist();row['path']=np.array(path).tolist();row['fcl_failures']=fail
 except Exception as exc:row['error']=str(exc)
 rows.append(row);print({k:v if k!='fcl_failures' else (len(v),v[:1]) for k,v in row.items() if k!='path'},flush=True)
Path('reports/skill_trials/lightwheel_pick_place/cpu_grasp_probe.json').write_text(json.dumps(rows,indent=2))
