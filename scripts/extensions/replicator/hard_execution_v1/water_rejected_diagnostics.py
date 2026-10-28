"""Offline recorded valve-path CUDA attribution; no Kit or physics advances."""
import argparse,fcntl,json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--cpu-only',action='store_true')
p.add_argument('--dense-arc',action='store_true',help='Also check 55 to .5 degrees every .5 degree using the recorded push reference; force flow on')
a=p.parse_args();a.episode=a.episode.resolve();a.output=a.output.resolve()
if a.output.exists():p.error('Refuse to overwrite evidence')
a.output.parent.mkdir(parents=True,exist_ok=True)
from scripts.extensions.replicator.hard_execution_v1.water_audit import build_world
from kitchen_safety.extensions.replicator.hard_execution_v1.water_control import ValvePlanner, BODY
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
D=np.load(a.episode/'physics_states.npz');W=json.loads((a.episode/'water_physics.json').read_text());M=json.loads((a.episode/'physics_metadata.json').read_text());P=json.loads((a.episode/'valve_plans.json').read_text())
last=P[-1];step=last['start_physics_step'];index=int(np.where(D['physics_step']==step)[0][0]);path=a.episode/f"valve_plan_{len(P):03d}_{last['phase']}.npz";candidate=np.load(path)['positions']
w,k,s=build_world()
def sync(i):
 w.set_phase('valve_turn');w.flowing=W['rows'][i]['flowing'];w.gripper_joint_positions=D['q'][i,7:].copy();w.gripper_joint_position=float(w.gripper_joint_positions[0]);w.payload_pose=pose_matrix(D['task_object_states'][i,0,:7]);w.set_obstacle_pose('bowl',D['task_object_states'][i,0,:7]);w.sync_background(M['background_paths'],D['background_rigid_poses'][i]);w.set_obstacle_pose(BODY,W['rows'][i]['lever_pose'])
sync(index);cpu=[]
for q in candidate:
 good=w.valid(q,gripper_q=D['q'][index,7:]);cpu.append(dict(valid=good,violation=w.last_clearance,flow=w.last_flow_clearance))
report=dict(episode=str(a.episode),candidate=str(path),physics_step=int(step),native_valve_angle_deg=W['rows'][index]['angle_deg'],actual_gripper_q=D['q'][index,7:].tolist(),cpu_full_world=cpu,no_physics_advance=True,accepted=False,sources={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [a.episode/'physics_states.npz',a.episode/'water_physics.json',path]})
report['implementation_sources']={str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),ROOT/'kitchen_safety/extensions/replicator/hard_execution_v1/water_control.py',ROOT/'kitchen_safety/extensions/replicator/hard_execution_v1/water_mechanism.py']}
report['valve_geometry_refinement']=w.valve_geometry_refinement
arc=[]
if a.dense_arc:
 from scipy.spatial.transform import Rotation
 turn=next(row for row in P if row['phase']=='valve_turn')
 reference_index=int(np.where(D['physics_step']==turn['start_physics_step'])[0][0])
 lever=pose_matrix(W['rows'][reference_index]['lever_pose'])
 reference=np.linalg.inv(lever)@pose_matrix(D['tcp_pose_w'][reference_index])
 reference_angle=W['rows'][reference_index]['angle_deg']
 q=D['q'][reference_index,:7].copy()
 for angle in np.arange(55.,0.,-.5):
  hypothetical=lever.copy()
  hypothetical[:3,:3]=Rotation.from_euler('z',angle-reference_angle,degrees=True).as_matrix()@lever[:3,:3]
  pose=np.r_[hypothetical[:3,3],Rotation.from_matrix(hypothetical[:3,:3]).as_quat()]
  w.set_obstacle_pose(BODY,pose);w.flowing=True
  row=dict(angle_deg=float(angle),lever_pose=pose.tolist(),flow_forced_on=True)
  try:
   q=k.ik(hypothetical@reference,q,attempts=2)
   good=w.valid(q,gripper_q=D['q'][index,7:])
   row.update(q=q.tolist(),cpu_valid=bool(good),violation=w.last_clearance,flow=w.last_flow_clearance)
  except Exception as error:row.update(cpu_valid=False,error=repr(error))
  arc.append(row)
 report['dense_arc']=dict(scope='Planning hypothesis using recorded push reference and six measured finger joints, each angle changes planning geometry only; no future physical state assumed',reference_physics_step=int(turn['start_physics_step']),push_reference_tcp_in_lever=reference.tolist(),samples=len(arc),cpu_valid=all(row['cpu_valid'] for row in arc),rows=arc)
if not a.cpu_only:
 if ROOT!=Path('/workspace'):raise RuntimeError('GPU diagnostics require verified /workspace environment')
 with Path('/workspace/reports/skill_trials/gpu.lock').open('a+') as lock:
  fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
  # Recreate the constructor reference before replaying the observed world.
  w,k,s=build_world();first=int(np.where(D['physics_step']==P[0]['start_physics_step'])[0][0]);sync(first)
  planner=ValvePlanner(w,k,dt=float(M['action_dt_s']),seed=42,max_joint_speed=.25)
  sync(index);planner.refresh_observed_world();planner._prepare_path_query((),1.,'valve_turn')
  from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components,sphere_pair_diagnostics
  components,state=constraint_components(planner,candidate);pairs,spheres=sphere_pair_diagnostics(planner,state,components)
  report.update(cuda_components=components,cuda_sphere_pairs=pairs,world_refresh=planner.observed_world_refreshes)
  np.savez_compressed(a.output.with_suffix('.npz'),robot_spheres=spheres,positions=candidate)
  if a.dense_arc:
   diagnosed=0
   for row in arc:
    if 'q' not in row:continue
    w.set_obstacle_pose(BODY,row['lever_pose']);w.flowing=True
    planner.refresh_observed_world();planner._prepare_path_query((),1.,'valve_turn')
    checks,state=constraint_components(planner,np.asarray([row['q']]))
    row['cuda_components']=checks
    if not checks['valid'] and diagnosed<5:
     row['cuda_pairs'],_=sphere_pair_diagnostics(planner,state,checks);diagnosed+=1
   report['dense_arc']['cuda_valid']=all(row.get('cuda_components',{}).get('valid',False) for row in arc)
   report['dense_arc']['cuda_rejected_angles_deg']=[row['angle_deg'] for row in arc if not row.get('cuda_components',{}).get('valid',False)]
   report['actual_cuda_locked_finger_joints']=planner.actual_finger_joint_values
a.output.write_text(json.dumps(report,indent=2,default=lambda v:v.tolist())+'\n')
print(json.dumps({k:report[k] for k in ['physics_step','native_valve_angle_deg','cpu_full_world']},indent=2),flush=True)
if 'cuda_components' in report:print(json.dumps({'cuda':report['cuda_components'],'pairs':report['cuda_sphere_pairs']},indent=2),flush=True)
if a.dense_arc:print(json.dumps({'dense_arc':{key:value for key,value in report['dense_arc'].items() if key not in ('rows','push_reference_tcp_in_lever')}}),flush=True)
