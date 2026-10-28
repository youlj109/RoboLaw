"""Full-FCL CPU lift check from actual human01 closed grasp, no GPU/dynamics."""
from pathlib import Path
import json,h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
ROOT=Path(__file__).resolve().parents[3];ep=ROOT/'datasets/skill_trials/lightwheel_pick_place/human_attempt01';r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);w=CollisionWorld(Usd.Stage.Open(str(ep/'scene.usda')),kin)
with h5py.File(ep/'trajectory.h5') as f:
 q=f['next_obs/q'][-1];lime=f['next_obs/lime'][-1,:7];tcp=f['next_obs/tcp'][-1]
 for name in ('wooden_bowl','broccoli','sweet_potato','red_bell_pepper'):w.set_obstacle_pose(name,f['next_obs/'+name][-1,:7])
w.allowed_phase='lift';w.attachment=np.array(r['planning_attachment_tcp_to_lime']);w.source_resting_xy=np.array(r['source_support_contact_model']['source_resting_xy_m']);w.source_resting_min_z=r['source_support_contact_model']['source_resting_min_z_m'];held_min=float((w.lime_vertices@Rotation.from_quat(lime[3:]).as_matrix().T+lime[:3])[:,2].min());w.source_lift_floor_z=held_min-.0005
initial_bound=w.source_lift_floor_z;target=kin.fk(q[:7]);seed=q[:7];finger=dict(zip(w.GRIPPER_JOINT_NAMES,q[7:]));rows=[]
for i,z in enumerate(np.linspace(0,.16,82)):
 t=target.copy();t[2,3]+=z;seed=kin.ik(t,seed) if i else seed;pose=kin.fk(seed)@w.attachment;minimum=float((w.lime_vertices@pose[:3,:3].T+pose[:3,3])[:,2].min());c=w.clearance(seed,('lime',),mode='planning',gripper_q=finger,object_pose=pose);rows.append({'index':i,'dz_m':float(z),'q_arm':seed.tolist(),'minimum_lime_z_m':minimum,'source_lift_floor_z_m':w.source_lift_floor_z,**c});w.source_lift_floor_z=max(w.source_lift_floor_z,minimum-.0005)
report={'passed':all(x['valid'] for x in rows),'samples':len(rows),'original_rest_min_z_m':w.source_resting_min_z,'measured_closed_min_z_m':held_min,'initial_lower_bound_m':initial_bound,'minimum':min(rows,key=lambda x:x['slack_m']),'failed':[x for x in rows if not x['valid']],'measurements':rows,'scope':'CPU IK and complete FCL exact carried hull plus measured gripper; source-only lift support monotonic bound, no physics execution/self collision claim'}
(ROOT/'reports/skill_trials/lightwheel_pick_place/human01_lift_support_preflight.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='measurements'},indent=2))
raise SystemExit(0 if report['passed'] else 2)
