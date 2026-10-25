"""Offline actual-state contact-line diagnostic; never execute the candidate line."""
import json
from pathlib import Path
import h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task,BANANA_TASK
p=Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_review_20261025T122401Z');r=json.load(open(p/'result.json'));spec=resolve_task(BANANA_TASK);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:]).as_euler('xyz',degrees=True)[2]);ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
with h5py.File(p/'trajectory.h5') as h:state={k:v[-1] for k,v in h['next_obs'].items()}
q=state['q'][ids];world=CollisionWorld(Usd.Stage.Open(str(p/'scene.usda')),kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!=spec.pick_object));world.allowed_phase='approach';source=r['source_support_contact_model'];world.source_support_mode='native_rest_until_detached';world.source_detached=False;world.source_resting_min_z=source['source_resting_min_z_m'];world.source_resting_root=np.array(source['source_resting_root_m']);world.source_resting_xy=np.array(source['source_resting_xy_m'])
for n in spec.rigid_objects:
 if n!=spec.pick_object:world.set_obstacle_pose(n,state[n][:7])
initial=kin.fk(q);target=np.array(r['approach_contact_standoff_m']);delta=target-initial[:3,3];rows=[];path=[q.copy()]
for f in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(delta)/.002))+1)):
 T=initial.copy();T[:3,3]+=f*delta
 q=kin.ik(T,q,attempts=12);path.append(q.copy());c=world.clearance(q,(spec.pick_object,),mode='execution',object_pose=state[spec.pick_object][:7],gripper_q={n:state['q'][r['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES});rows.append({'fraction':float(f),**c})
np.savez('/root/autodl-tmp/B03-review/contact_line_probe.npz',positions=np.array(path));Path('/root/autodl-tmp/B03-review/contact_line_probe.json').write_text(json.dumps({'scope':'Offline only, never executed','all_fcl_clear':all(x['valid'] for x in rows),'minimum':min(rows,key=lambda x:x['slack_m']),'rows':rows},indent=2));print('FCL',all(x['valid'] for x in rows),min(rows,key=lambda x:x['slack_m']),flush=True)
