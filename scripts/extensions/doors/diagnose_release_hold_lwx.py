"""Compare release hold targets at recorded states; never replay physics."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.release_hold_lwx import bounded_hold_command
from kitchen_safety.planning import transform

p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
m=json.loads((a.episode/'metadata.json').read_text());rows=json.loads((a.episode/'trajectory.json').read_text());result=json.loads((a.episode/'result.json').read_text())
release=[r for r in rows if r['phase']=='release_handle'];last=release[-1];final_target=np.asarray(last['action'][:7]);obs=last['next_obs']
config=json.loads(Path(m['variant_config']).read_text());stage,kin,w,g=reconstruct(m,config,m['variant_spec']);w.allow_handle_contact=True
initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]);states=[('last_completed_release',obs)]
failure=result.get('failure') or ''
if 'EXECUTION_COLLISION ' in failure:
 failed=json.loads(failure.split('EXECUTION_COLLISION ',1)[1]);states.append(('rejected_hold_substep',failed))
report={'scope':'CPU conservative geometry at recorded door/gripper states only. Fixed-target samples are hypothetical commands, not future dynamics, GPU self collision, a safe replay or acceptance. A recorded rejected substep may already violate safety at interpolation start.','episode':str(a.episode),'trajectory_sha256':sha(a.episode/'trajectory.json'),'result_sha256':sha(a.episode/'result.json'),'implementation_sha256':sha(__file__),'physics_replay':False,'unsafe_path_executed':False,'states':[]}
for name,state in states:
 pose=state['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
 for body in g['moving_bodies']:w.move_body(body,delta)
 q=np.asarray(state['q'][:7]);fq={n:state['q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};item={'state':name,'door_q':state['door_q'],'remaining_target_joint_error_rad':float(np.abs(final_target-q).max()),'remaining_target_tcp_distance_m':float(np.linalg.norm(kin.fk(final_target)[:3,3]-kin.fk(q)[:3,3])),'modes':{}}
 for mode,command in [('feedback',q),('fixed_final_target',bounded_hold_command(q,final_target))]:
  count=max(1,int(np.ceil(np.abs(command-q).max()/.005)));checks=[w.clearance(q+(command-q)*f,mode='execution',gripper_q=fq) for f in np.linspace(0.,1.,count+1)]
  item['modes'][mode]={'command':command.tolist(),'joint_step_rad':float(np.abs(command-q).max()),'interpolation_samples':len(checks),'interpolation_all_valid':all(v['valid'] for v in checks),'minimum':min(checks,key=lambda v:v['slack_m']),'command_endpoint':checks[-1]}
 report['states'].append(item)
report['complete']=True;a.output.write_text(json.dumps(report,indent=2)+'\n')
for state in report['states']:
 print('HOLD_DIAG',state['state'],'remaining_joint',state['remaining_target_joint_error_rad'],'remaining_tcp_m',state['remaining_target_tcp_distance_m'],{k:(v['minimum']['slack_m'],v['command_endpoint']['slack_m']) for k,v in state['modes'].items()})
