"""Read-only diagnosis of the saved failed descent; no PhysX rollout."""
import json,h5py,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from paired_planning import build_world
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
root=Path('datasets/skill_trials/lightwheel_pick_place');ep=root/'property_lgd_b04_round10_20261027T083745Z';r=json.load(open(ep/'result.json'));b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(ep,kin,None,{},task_spec=episode_spec(r));pick='green_ladle';world.bind_pick(pick)
with h5py.File(ep/'trajectory.h5') as h:
 ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][-1,ids];pose=h['next_obs/green_ladle'][-1,:7];gripper={n:float(h['next_obs/q'][-1,r['robot_joint_names'].index(n)]) for n in world.GRIPPER_JOINT_NAMES}
 for n in r['initial_object_states']:
  if n!=pick:world.set_obstacle_pose(n,h['next_obs/'+n][-1,:7])
world.attachment=np.linalg.inv(kin.fk(q))@transform(pose[:3],pose[3:7]);world.source_detached=False;world.allowed_phase='lift';world.placed_task_contents=set();world.gripper_joint_position=gripper['finger_joint'];world.planning_gripper_q=gripper
tcp=kin.fk(q);previous=q.copy();rows=[]
for index in range(81):
 target=tcp.copy();target[2,3]+=.002*index;candidate=kin.ik(target,previous,attempts=1);previous=candidate
 c=world.clearance(candidate,(pick,),mode='planning',obstacle_filter=('glass',));rows.append({'index':index,**c})
report={'all_safe':all(x['valid'] for x in rows),'minimum':min(rows,key=lambda x:x['slack_m']),'samples':len(rows),'measurements':rows}
Path('/root/autodl-tmp/B04-review/property_lift_true_clearance_round11.json').write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items() if k!='measurements'})
