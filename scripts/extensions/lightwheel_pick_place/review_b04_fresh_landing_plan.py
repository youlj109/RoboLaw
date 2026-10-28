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
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
review=json.loads(Path('/root/autodl-tmp/B04-review/fresh_base_landing_query_redundancy_20261028.json').read_text())
candidates=[x for x in review['same_tcp_redundancy_candidates'] if x['valid']]
planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=q.copy()
planner._prepare_path_query((spec.pick_object,spec.destination_object),1.,'lower')
queries=np.asarray([q]+[x['q'] for x in candidates]);report['curobo_start_goal_valid']=planner.motion_gen.ik_solver.check_valid(planner.args.to_device(queries)).detach().cpu().numpy().tolist()
report.update(passed=False,attempts=[])
for selected in candidates:
 try:
  path=planner.plan_joint_goal(q,np.array(selected['q']),(spec.pick_object,spec.destination_object),1.,phase='lower')
  checked=planner.validate_path(path['positions'],(spec.pick_object,spec.destination_object),1.,phase='lower')
  report['attempts'].append({'candidate':selected['index'],'passed':checked['valid']})
  if checked['valid']:
   report.update(passed=True,selected_candidate=selected['index'],validation=checked,positions=np.asarray(path['positions']).tolist(),planning_metadata=path.get('metadata'));break
 except RuntimeError as exc:report['attempts'].append({'candidate':selected['index'],'passed':False,'error':str(exc)})
Path('/root/autodl-tmp/B04-review/fresh_base_landing_plan_proof_20261028.json').write_text(json.dumps(report,indent=2));print('FRESH_SAME_TARGET_PLAN',report['passed'],report['curobo_start_goal_valid'],report['attempts'])
