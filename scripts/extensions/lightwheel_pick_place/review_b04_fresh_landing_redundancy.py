"""Read-only exact saved landing endpoint geometry review; zero physics steps."""
import json,hashlib
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from paired_planning import build_world,grip_joints
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
p=Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_fresh_matched_20261028T091300Z');r=json.loads((p/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(p,kin,None,{},task_spec=spec)
with h5py.File(p/'trajectory.h5') as h:
 i=len(h['phase'])-1;names=r['robot_joint_names'];q=np.array(h['next_obs/q'][i,:7]);allq=np.array(h['next_obs/q'][i]);pose=np.array(h['next_obs/'+spec.pick_object][i,:7]);finger={n:float(allq[names.index(n)]) for n in names if 'finger' in n or 'knuckle' in n}
 for n in spec.rigid_objects:
  if n!=spec.pick_object:world.set_obstacle_pose(n,h['next_obs/'+n][i,:7])
world.source_detached=True;world.allowed_phase='lower';world.gripper_joint_position=finger['finger_joint'];world.planning_gripper_q=finger;world.attachment=np.linalg.inv(kin.fk(q))@transform(pose[:3],pose[3:7])
proposal=json.loads(Path('/root/autodl-tmp/B04-review/measured_landing_plan31.json').read_text());body=np.array(proposal['body_goal']);target=body@np.linalg.inv(world.attachment);goal=kin.ik(target,np.array(proposal['joint_goal']),attempts=1)
checks={'start':world.clearance(q,(spec.pick_object,spec.destination_object),mode='execution',gripper_q=finger,object_pose=pose),'goal':world.clearance(goal,(spec.pick_object,spec.destination_object),mode='execution',gripper_q=finger,object_pose=np.r_[body[:3,3],Rotation.from_matrix(body[:3,:3]).as_quat()])}
report={'scope':'Exact saved final state, same landing goal and fresh measured attachment IK. Read-only FCL; no simulation and no acceptance upgrade.','episode':str(p),'result_sha256':hashlib.sha256((p/'result.json').read_bytes()).hexdigest(),'last_action':i,'q_start':q.tolist(),'q_goal':goal.tolist(),'actual_object_pose':pose.tolist(),'tcp_ik_error_m':float(np.linalg.norm(kin.fk(goal)[:3,3]-target[:3,3])),'checks':checks}
candidates=[]
rng=np.random.default_rng(r['seed'])
seeds=[q,np.array(proposal['joint_goal'])]+[q+rng.normal(0,.6,7) for _ in range(16)]
for index,initial in enumerate(seeds):
 try:candidate=kin.ik(target,initial,attempts=1)
 except RuntimeError as exc:
  candidates.append({'index':index,'valid':False,'error':str(exc)});continue
 pos=float(np.linalg.norm(kin.fk(candidate)[:3,3]-target[:3,3]));rot=float(Rotation.from_matrix(kin.fk(candidate)[:3,:3].T@target[:3,:3]).magnitude())
 c=world.clearance(candidate,(spec.pick_object,spec.destination_object),mode='execution',gripper_q=finger,object_pose=np.r_[body[:3,3],Rotation.from_matrix(body[:3,:3]).as_quat()])
 candidates.append({'index':index,'position_error_m':pos,'orientation_error_rad':rot,'q':candidate.tolist(),'clearance':c,'valid':bool(pos<=.001 and rot<=.02 and c['valid'])})
report['same_tcp_redundancy_candidates']=candidates
report['valid_candidate_indices']=[x['index'] for x in candidates if x['valid']]
Path('/root/autodl-tmp/B04-review/fresh_base_landing_query_redundancy_20261028.json').write_text(json.dumps(report,indent=2));print('VALID_REDUNDANCY_CANDIDATES',report['valid_candidate_indices'])
