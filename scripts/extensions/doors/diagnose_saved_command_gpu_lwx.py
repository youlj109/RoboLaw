"""Isolate a reconstructed rejected command in a new offline planner; never execute."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_planner import TaskPlanner
from kitchen_safety.planning import transform
p=argparse.ArgumentParser();p.add_argument('--diagnostic',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
d=json.loads(a.diagnostic.read_text());ep=Path(d['episode']);m=json.loads((ep/'metadata.json').read_text());rows=json.loads((ep/'trajectory.json').read_text());obs=rows[-1]['next_obs'];cp=m['controller_parameters']
if sha(ep/'metadata.json')!=d['metadata_sha256'] or sha(ep/'trajectory.json')!=d['trajectory_sha256']:raise ValueError('Recorded source changed')
config=json.loads(Path(m['variant_config']).read_text());stage,kin,w,g=reconstruct(m,config,m['variant_spec']);margin=cp.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin;w.allow_handle_contact=True
pose=obs['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]))
for body in g['moving_bodies']:w.move_body(body,delta)
w.gripper_joint_position=float(obs['q'][m['robot_joint_names'].index('finger_joint')]);planner=TaskPlanner(w,kin,dt=m['dt'],seed=cp['seed'],max_joint_speed=cp['max_joint_speed'],hazard_extra_padding_m=cp.get('planner_hazard_extra_padding_m',0.));planner._prepare_path_query(tuple(g['handle_colliders']),1.,'pull_door');q=np.asarray(d['command']);qt=planner.args.to_device(q[None]);enabled=[n for n in planner.obstacle_names if n not in g['handle_colliders']]
def test(names):
 names=set(names)
 for n in planner.obstacle_names:planner.motion_gen.world_coll_checker.enable_obstacle(n,enable=n in names)
 return bool(planner.motion_gen.ik_solver.check_valid(qt).all())
report={'scope':'New offline planner constraint isolation on a reconstructed rejected pull command. Collision objects are toggled only in this diagnostic planner; no physical replay, disabled-safety execution, future-state or acceptance claim.','diagnostic':str(a.diagnostic),'diagnostic_sha256':sha(a.diagnostic),'implementation_sha256':sha(__file__),'physics_replay':False,'unsafe_path_executed':False,'command':q.tolist(),'valid_with_world':test(enabled),'valid_without_world':test([]),'blocking_obstacles':[]}
def search(names):
 if test(names):return
 if len(names)==1:report['blocking_obstacles'].extend(names);return
 if len(report['blocking_obstacles'])>=12:return
 half=len(names)//2;search(names[:half]);search(names[half:])
if report['valid_without_world']:search(enabled)
report['interpretation']='World geometry can be isolated for this command.' if report['valid_without_world'] else 'Rejection remains without external world; self-collision or other internal constraints require diagnosis. CPU joint-limit validity alone is insufficient.'
report['complete']=True;a.output.write_text(json.dumps(report,indent=2)+'\n');print('SAVED_COMMAND_GPU',report['valid_with_world'],report['valid_without_world'],report['blocking_obstacles'])
