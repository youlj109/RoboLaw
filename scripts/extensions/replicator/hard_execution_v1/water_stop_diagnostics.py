"""Offline mechanical-stop TCP candidate using recorded physical valve state.

The valve geometry is never placed below its native zero-degree stop. Negative
angles specify robot TCP overtravel only. No Kit or physical execution.
"""
import argparse,fcntl,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.hard_execution_v1.water_audit import build_world
from kitchen_safety.extensions.replicator.hard_execution_v1.water_control import ValvePlanner,BODY
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
ROOT=Path(__file__).resolve().parents[4]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--episode',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--cpu-only',action='store_true')
a=p.parse_args();a.episode=a.episode.resolve();a.output=a.output.resolve()
if a.output.exists():p.error('Refuse to overwrite evidence')
D=np.load(a.episode/'physics_states.npz');W=json.loads((a.episode/'water_physics.json').read_text())['rows'];M=json.loads((a.episode/'physics_metadata.json').read_text());P=json.loads((a.episode/'valve_plans.json').read_text())
w,k,s=build_world();index=max(i for i,row in enumerate(W) if row['phase']=='valve_turn')
w.set_phase('valve_turn');w.flowing=True;w.gripper_joint_positions=D['q'][index,7:].copy();w.gripper_joint_position=float(w.gripper_joint_positions[0]);w.payload_pose=pose_matrix(D['task_object_states'][index,0,:7]);w.set_obstacle_pose('bowl',D['task_object_states'][index,0,:7]);w.sync_background(M['background_paths'],D['background_rigid_poses'][index])
observed_pose=np.asarray(W[index]['lever_pose']);observed_angle=W[index]['angle_deg']
turn=next(row for row in P if row['phase']=='valve_turn');start=int(np.where(D['physics_step']==turn['start_physics_step'])[0][0])
lever=pose_matrix(W[start]['lever_pose']);reference=np.linalg.inv(lever)@pose_matrix(D['tcp_pose_w'][start]);reference_angle=W[start]['angle_deg']
def lever_pose(angle):
 matrix=lever.copy();matrix[:3,:3]=Rotation.from_euler('z',angle-reference_angle,degrees=True).as_matrix()@lever[:3,:3]
 return matrix
def observed_geometry():w.set_obstacle_pose(BODY,observed_pose)
observed_geometry();q=D['q'][index,:7].copy();rows=[]
for angle in np.round(np.arange(.5,-.50001,-.05),8):
 target=lever_pose(angle)@reference
 q=k.ik(target,q,attempts=2)
 projected=lever_pose(max(0.,angle));physical_pose=np.r_[projected[:3,3],Rotation.from_matrix(projected[:3,:3]).as_quat()]
 row=dict(tcp_virtual_angle_deg=float(angle),physical_valve_angle_hypothesis_deg=float(max(0.,angle)),physical_valve_pose=physical_pose.tolist(),q=q.tolist(),target_tcp=target.tolist(),checks={})
 for label,pose in [('observed_static',observed_pose),('stop_limited_hypothesis',physical_pose)]:
  w.set_obstacle_pose(BODY,pose)
  valid=w.valid(q,gripper_q=D['q'][index,7:])
  row['checks'][label]=dict(cpu_valid=bool(valid),violation=w.last_clearance,flow=w.last_flow_clearance)
 rows.append(row)
report=dict(episode=str(a.episode),recorded_step=int(D['physics_step'][index]),observed_valve_angle_deg=float(observed_angle),actual_six_finger_q=D['q'][index,7:].tolist(),push_reference=reference.tolist(),rows=rows,cpu_valid=all(c['cpu_valid'] for row in rows for c in row['checks'].values()),scope='Planning-only robot press against original zero-degree mechanical stop; valve pose hypotheses never below0. All source/flow checks retained. Actual contact, forces, q/dq must still be measured during native execution.',sources={str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),ROOT/'kitchen_safety/extensions/replicator/hard_execution_v1/water_control.py',a.episode/'water_physics.json',a.episode/'physics_states.npz']},physics_launched=False)
if not a.cpu_only:
 if ROOT!=Path('/workspace'):raise RuntimeError('GPU diagnostics require verified /workspace environment')
 with Path('/workspace/reports/skill_trials/gpu.lock').open('a+') as lock:
  fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
  observed_geometry();planner=ValvePlanner(w,k,dt=M['action_dt_s'],seed=42,max_joint_speed=.25)
  from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components,sphere_pair_diagnostics
  for row in rows:
   for label,pose in [('observed_static',observed_pose),('stop_limited_hypothesis',row['physical_valve_pose'])]:
    w.set_obstacle_pose(BODY,pose);planner.refresh_observed_world();planner._prepare_path_query((),1.,'valve_turn')
    checks,state=constraint_components(planner,np.asarray([row['q']]))
    row['checks'][label]['cuda_components']=checks
    if not checks['valid']:row['checks'][label]['cuda_pairs'],_=sphere_pair_diagnostics(planner,state,checks)
  report['cuda_valid']=all(c['cuda_components']['valid'] for row in rows for c in row['checks'].values())
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:value for key,value in report.items() if key not in ('rows','push_reference')},indent=2),flush=True)
