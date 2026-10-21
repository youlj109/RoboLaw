"""CPU candidate diagnostic: genuine handle line and full door motion geometry."""
import argparse,json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.planning import DroidKinematics,transform
ROOT=Path(__file__).resolve().parents[3];p=argparse.ArgumentParser();p.add_argument('--variant',required=True);a=p.parse_args();m=json.loads((ROOT/'datasets/avoidance_trials/doors/base_adaptive_attempt_01/metadata.json').read_text());b=json.loads((ROOT/'datasets/avoidance_trials/doors/base_adaptive_attempt_01/trajectory.json').read_text());spec=json.loads((ROOT/'configs/extensions/doors/avoidance_variants.json').read_text())[a.variant]
s=Usd.Stage.CreateInMemory();k=UsdGeom.Xform.Define(s,'/World/envs/env_0/kitchen');k.GetPrim().GetReferences().AddReference(str(ROOT/'assets/kitchen/scene.usd'))
for ob in [spec]+([spec['support']] if spec.get('support') else []):
 x=UsdGeom.Xform.Define(s,'/World/envs/env_0/'+ob['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/ob['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*ob['position']));x.AddRotateZOp().Set(ob.get('yaw_deg',0))
kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg']);w=DoorCollisionWorld(s,kin,env_root='/World/envs/env_0');w.allow_handle_contact=True;handle=np.array(m['handle_initial']);hinge=np.array(m['hinge']);R0=np.array([[0,1,0],[1,0,0],[0,0,-1.]]);axis=np.array(m['axis']);door='/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door';rng=np.random.default_rng(42);results=[]
for offset in [0.,.06,.10]:
 target=np.eye(4);target[:3,:3]=R0;target[:3,3]=handle+[0,-.13,offset];attempts=[]
 for seed in [np.array(b[96]['next_obs']['q'][:7])]+[rng.uniform(kin.limits[:,0],kin.limits[:,1]) for _ in range(12)]:
  w.move_body(door,np.eye(4));target[:3,:3]=R0;target[:3,3]=handle+[0,-.13,offset]
  try:q=kin.ik(target,seed,attempts=1)
  except RuntimeError:continue
  if not w.valid(q,mode='planning',gripper_q=dict.fromkeys(w.GRIPPER_JOINT_NAMES,0.)):continue
  initial=q.copy();failure=None;path=[]
  for alpha in np.linspace(0,1,50)[1:]:
   target[:3,:3]=R0;target[:3,3]=handle+[0,-.13*(1-alpha),offset]
   try:q=kin.ik(target,q,attempts=1)
   except RuntimeError as e:failure=str(e);break
   if not w.valid(q,mode='execution',gripper_q=dict.fromkeys(w.GRIPPER_JOINT_NAMES,0.)):failure={'phase':'approach','alpha':alpha,'collision':w.last_clearance};break
   path.append(q.tolist())
  if not failure:
   for angle in np.linspace(0,.34,35):
    rr=Rotation.from_rotvec(axis*angle).as_matrix();delta=np.eye(4);delta[:3,:3]=rr;delta[:3,3]=hinge-rr@hinge;w.move_body(door,delta);target[:3,:3]=rr@R0;target[:3,3]=hinge+rr@(handle+[0,0,offset]-hinge)
    try:q=kin.ik(target,q,attempts=1)
    except RuntimeError as e:failure=str(e);break
    grip={n:dict(zip(m['robot_joint_names'],b[165]['next_obs']['q']))[n] for n in w.GRIPPER_JOINT_NAMES}
    if not w.valid(q,mode='execution',gripper_q=grip):failure={'phase':'pull','angle':angle,'collision':w.last_clearance};break
    path.append(q.tolist())
  attempts.append({'initial_q':initial.tolist(),'failure':failure,'path':path})
  if failure is None:break
 results.append({'height_offset':offset,'attempts':attempts,'passed':any(x['failure'] is None for x in attempts)})
report={'variant':a.variant,'spec':spec,'results':results,'scope':'CPU full geometry hypothetical IK diagnostic only; actual contact/physics/self-collision still required.'};out=ROOT/f'reports/avoidance_trials/doors/{a.variant}_contact_corridor.json';out.write_text(json.dumps(report,indent=2));print(json.dumps([{k:v for k,v in r.items() if k!='attempts'}|{'attempts':[x['failure'] for x in r['attempts']]} for r in results],indent=2))
