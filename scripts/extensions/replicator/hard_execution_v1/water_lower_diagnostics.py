"""Recorded bowl-lowering attribution: original query and actual six fingers.

Offline only. Preserve exact source-floor support refinement, every other
world pair, original self/joint constraints, attachment and full source FCL.
"""
import argparse,fcntl,json,hashlib
from copy import deepcopy
from pathlib import Path
import numpy as np
from scripts.extensions.replicator.hard_execution_v1.water_audit import build_world
from kitchen_safety.extensions.replicator.hard_execution_v1.water_mechanism import BODY
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
ROOT=Path(__file__).resolve().parents[4]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--cpu-only',action='store_true');p.add_argument('--refined',action='store_true');a=p.parse_args();a.episode=a.episode.resolve();a.output=a.output.resolve()
if a.output.exists():p.error('Refuse to overwrite evidence')
R=json.loads((a.episode/'result.json').read_text());M=json.loads((a.episode/'physics_metadata.json').read_text());W=json.loads((a.episode/'water_physics.json').read_text())['rows'];D=np.load(a.episode/'physics_states.npz');C=json.loads((a.episode/'grasp_contacts.json').read_text())
last=R['plans'][-1];step=last['start_physics_step'];index=int(np.where(D['physics_step']==step)[0][0]);path=a.episode/'rejected_plan_05_lower.npz';positions=np.load(path)['positions']
w,k,s=build_world();w.set_phase('lower');w.flowing=W[index]['flowing'];w.gripper_joint_positions=D['q'][index,7:].copy();w.gripper_joint_position=float(w.gripper_joint_positions[0]);w.payload_pose=pose_matrix(D['task_object_states'][index,0,:7]);w.set_obstacle_pose('bowl',D['task_object_states'][index,0,:7]);w.sync_background(M['background_paths'],D['background_rigid_poses'][index]);w.set_obstacle_pose(BODY,W[index]['lever_pose']);w.attachment=np.asarray(last['measured_planning_attachment'])
cpu=[]
for i,q in enumerate(positions):
 good=w.valid(q,gripper_q=w.gripper_joint_positions);cpu.append(dict(sample=i,valid=bool(good),violation=w.last_clearance,flow=w.last_flow_clearance))
report=dict(episode=str(a.episode),physics_step=int(step),positions=len(positions),actual_six_finger_q=w.gripper_joint_positions.tolist(),coupled_finger_q=(w.gripper_joint_position*np.array([1,1,-1,1,-1,-1])).tolist(),water_flowing=w.flowing,cpu_full_world=cpu,physics_launched=False,accepted=False,sources={str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),path,a.episode/'physics_states.npz',a.episode/'water_physics.json',ROOT/'kitchen_safety/extensions/replicator/hard_execution_v1/water_control.py']})
if a.refined:
 from kitchen_safety.extensions.replicator.hard_execution_v1.water_bowl_lower import WaterBowlPlanner,source_bowl_grip_clearance
 report['source_bowl_added_grip']=source_bowl_grip_clearance(w,k,positions)
 for name in ('water_bowl_lower','water_control','water_mechanism'):
  source=ROOT/('kitchen_safety/extensions/replicator/hard_execution_v1/'+name+'.py');report['sources'][str(source.relative_to(ROOT))]=hashlib.sha256(source.read_bytes()).hexdigest()
 # Negative control: put the analytical cuboid inside the source solid bowl
 # at the first candidate, in this isolated offline collision world only.
 # Restore the exact observed transform; no USD/native object state is edited.
 from kitchen_safety.extensions.replicator.hard_execution_v1.water_control import GRIP
 obj=next(row[1] for row in w.objects if row[0]==GRIP);saved=obj.getTransform()
 payload=k.fk(positions[0])@w.attachment
 shape=w.fcl.CollisionObject(w.payload_model,w.fcl.Transform(payload[:3,:3],payload[:3,3]))
 try:
  obj.setTranslation(payload[:3,3])
  report['negative_control_overlap_detected']=bool(w.fcl.collide(shape,obj,w.fcl.CollisionRequest(),w.fcl.CollisionResult()))
 finally:obj.setTransform(saved)
 if not report['negative_control_overlap_detected']:raise RuntimeError('Exact solid source pair failed collision negative control')
if not a.cpu_only:
 if ROOT!=Path('/workspace'):raise RuntimeError('GPU diagnostics require verified /workspace environment')
 with Path('/workspace/reports/skill_trials/gpu.lock').open('a+') as lock:
  fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
  from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
  from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components,sphere_pair_diagnostics
  planner=(WaterBowlPlanner if a.refined else BowlTransportCuroboPlanner)(w,k,dt=M['action_dt_s'],seed=42,max_joint_speed=.4)
  planner.bind_sink_lowering(R['controller_profile'],physics_step=int(step),arm_q=D['q'][index,:7],measured_tcp=pose_matrix(D['tcp_pose_w'][index]),measured_payload=w.payload_pose,raw_contact_rows=C)
  planner._prepare_path_query((),1.,'lower')
  report['original_query']=planner._pair_audit(positions,'lower')
  if a.refined:
   report['refined_dense_validation']=planner.validate_path(positions,grip_binary=1,phase='lower',resolution=.02)
  components,state=constraint_components(planner,positions)
  report['original_pairs'],spheres=sphere_pair_diagnostics(planner,state,components)
  # Read-only alternative representation: replace only inferred mimic angles
  # with the six native values, then regenerate the same payload spheres.
  locked=deepcopy(planner.robot_dict['kinematics']['lock_joints']);locked.update(zip(w.GRIPPER_JOINT_NAMES,w.gripper_joint_positions.tolist()))
  planner.motion_gen.update_locked_joints(locked,deepcopy(planner.robot_dict));planner._last_grip=w.gripper_joint_position
  planner._prepare_path_query((),1.,'lower')
  report['actual_six_query']=planner._pair_audit(positions,'lower')
  components,state=constraint_components(planner,positions)
  report['actual_six_pairs'],actual_spheres=sphere_pair_diagnostics(planner,state,components)
  np.savez_compressed(a.output.with_suffix('.npz'),positions=positions,original_robot_spheres=spheres,actual_six_robot_spheres=actual_spheres)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2,default=lambda v:v.tolist())+'\n')
summary={key:report[key] for key in ('physics_step','positions','water_flowing')};summary['cpu_valid']=all(row['valid'] for row in cpu)
if a.refined:summary.update(source_added_grip=report['source_bowl_added_grip']['passed'],minimum_source_added_grip_distance_m=report['source_bowl_added_grip']['minimum_distance_m'],negative_control_overlap_detected=report['negative_control_overlap_detected'])
if 'refined_dense_validation' in report:summary['refined_dense_validation']={key:report['refined_dense_validation'][key] for key in ('valid','samples','input_waypoints','curobo_failures','fcl_failures')}
for key in ('original_query','actual_six_query'):
 if key in report:summary[key]=dict(valid=report[key]['valid'],mandatory=report[key]['mandatory_checks'])
print(json.dumps(summary,indent=2),flush=True)
