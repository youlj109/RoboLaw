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
planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=q.copy();planner._prepare_path_query((spec.pick_object,spec.destination_object),1.,'lower')
rng=np.random.default_rng(r['seed']);ikseeds=[q,np.array(proposal['joint_goal'])]+[q+rng.normal(0,.6,7) for _ in range(16)]
report.update(passed=False,scope='Bounded offline same-source pre-landing hover adjustment. Only commanded hover Z changes before original native supported descent; no object setters, no physics, original cuRobo/FCL gates.',candidates=[])
for dz in [-.04,-.02,.02,.04]:
 hover=body.copy();hover[2,3]+=dz;target=hover@np.linalg.inv(world.attachment)
 for index,initial in enumerate(ikseeds):
  row={'hover_delta_z_m':dz,'ik_seed_index':index,'passed':False}
  try:
   goal=kin.ik(target,initial,attempts=1);fk=kin.fk(goal)
   if np.linalg.norm(fk[:3,3]-target[:3,3])>.001 or Rotation.from_matrix(fk[:3,:3].T@target[:3,:3]).magnitude()>.02:raise RuntimeError('IK target residual')
   check=world.clearance(goal,(spec.pick_object,spec.destination_object),mode='execution',gripper_q=finger,object_pose=np.r_[hover[:3,3],Rotation.from_matrix(hover[:3,:3]).as_quat()]);row['endpoint']=check
   if not check['valid']:raise RuntimeError('Original FCL endpoint not clear')
   valid=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(np.asarray([goal]))).all());row['curobo_endpoint_valid']=valid
   if not valid:raise RuntimeError('Original cuRobo endpoint not clear')
   path=planner.plan_joint_goal(q,goal,(spec.pick_object,spec.destination_object),1.,phase='lower');checked=planner.validate_path(path['positions'],(spec.pick_object,spec.destination_object),1.,phase='lower');row['path_validation']=checked
   if not checked['valid']:raise RuntimeError('Original path gates not passed')
   row['passed']=True;report.update(passed=True,selected_hover_body_goal=hover.tolist(),selected_q_goal=goal.tolist(),selected_ik_seed_index=index,selected_hover_delta_z_m=dz,positions=np.asarray(path['positions']).tolist(),validation=checked,planning_metadata=path.get('metadata'))
  except RuntimeError as exc:row['error']=str(exc)
  report['candidates'].append(row)
  if report['passed']:break
 if report['passed']:break
Path('/root/autodl-tmp/B04-review/fresh_base_landing_hover_proof_20261028.json').write_text(json.dumps(report,indent=2));print('FRESH_HOVER_PLAN',report['passed'],report.get('selected_hover_delta_z_m'),report.get('selected_ik_seed_index'))
