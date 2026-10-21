"""CPU IK/full-geometry diagnosis from an actual failed transfer start."""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
p=argparse.ArgumentParser();p.add_argument('--episode',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--position',nargs=3,type=float);p.add_argument('--yaw',type=float,default=0.);a=p.parse_args()
r=json.loads((a.episode/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);stage=Usd.Stage.Open(str(a.episode/'scene.usda'))
if a.position:
 stage.SetEditTarget(stage.GetSessionLayer());root=UsdGeom.Xformable(stage.GetPrimAtPath('/World/envs/env_0/barrier'));root.ClearXformOpOrder();root.AddTranslateOp().Set(Gf.Vec3d(*a.position));root.AddRotateZOp().Set(a.yaw)
w=CollisionWorld(stage,kin)
with h5py.File(a.episode/'trajectory.h5') as f:
 q=f['next_obs/q'][-1];lime=f['next_obs/lime'][-1];tcp=f['next_obs/tcp'][-1];bowl=f['next_obs/wooden_bowl'][-1]
 for key in ('wooden_bowl','broccoli','sweet_potato','red_bell_pepper'):w.set_obstacle_pose(key,f['next_obs/'+key][-1,:7])
fingers={n:q[r['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};w.gripper_joint_position=fingers['finger_joint'];w.attachment=np.array(r['planning_attachment_tcp_to_lime']);w.allowed_phase='transfer'
target=kin.fk(q[:7]);target[:3,3]=bowl[:3]+[0,0,.18]+tcp-lime[:3]
rng=np.random.default_rng(42);seeds=[q[:7]]+[q[:7]+rng.normal(0,scale,7) for scale in [.1,.3,.6,.9] for _ in range(8)];rows=[]
for i,seed in enumerate(seeds):
 try:
  goal=kin.ik(target,seed,attempts=1);c=w.clearance(goal,('lime',),mode='planning',gripper_q=fingers)
  rows.append({'candidate':i,'q':goal.tolist(),'clearance':c});print('IK',i,c,flush=True)
 except RuntimeError as exc:rows.append({'candidate':i,'error':str(exc)})
report={'episode':str(a.episode),'override_position':a.position,'override_yaw':a.yaw,'actual_start_q':q[:7].tolist(),'target':target.tolist(),'actual_lime':lime.tolist(),'actual_tcp':tcp.tolist(),'actual_bowl':bowl.tolist(),'valid_goals':sum(v.get('clearance',{}).get('valid',False) for v in rows),'candidates':rows,'scope':'CPU kinematic IK and complete existing planning-margin FCL; not self-collision or MotionGen certificate.'}
a.output.write_text(json.dumps(report,indent=2));print('VALID_GOALS',report['valid_goals'],flush=True)
