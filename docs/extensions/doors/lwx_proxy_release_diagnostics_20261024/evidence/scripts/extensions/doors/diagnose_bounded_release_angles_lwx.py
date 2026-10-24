"""Offline bounded release angle proposals on recorded door motion; no simulation."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.release_position_lwx import bounded_position_command
from kitchen_safety.extensions.doors.release_pose_lwx import release_progress
from kitchen_safety.planning import transform
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--angles',default='-60,-55,-50,-45,-30,0');a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
angles=list(map(float,a.angles.split(',')))
if any(abs(v)>60 for v in angles):p.error('Outside existing direction bounds')
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
m=json.loads((a.episode/'metadata.json').read_text());rows=json.loads((a.episode/'trajectory.json').read_text());result=json.loads((a.episode/'result.json').read_text());cp=m['controller_parameters']
stage,kin,w,g=reconstruct(m,json.loads(Path(m['variant_config']).read_text()),m['variant_spec']);w.allow_handle_contact=True
margin=cp.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin
rr=[r for r in rows if r['phase'] in ['release_handle','released_hold']];before=rr[0]['obs'];start=kin.fk(np.asarray(before['q'][:7]));initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]);normal=Rotation.from_rotvec(np.asarray(g['axis'])*(before['door_q']-m['door_motion_reference_joint_rad'])).apply(Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).apply([1.,0.,0.]))
states=[r['next_obs'] for r in rr];failure=result.get('failure') or ''
if 'EXECUTION_COLLISION ' in failure:states.append(json.loads(failure.split('EXECUTION_COLLISION ',1)[1]))
report={'episode':str(a.episode),'trajectory_sha256':sha(a.episode/'trajectory.json'),'metadata_sha256':sha(a.episode/'metadata.json'),'result_sha256':sha(a.episode/'result.json'),'implementation_sha256':sha(__file__),'scope':'Offline CPU proposals at recorded door and finger poses. Ideal tracking sequences and proposals from measured arm seeds do not predict changed dynamics, GPU self collision, future safety or success. All source geometry and execution margins retained.','physics_replay':False,'unsafe_path_executed':False,'results':[]}
for angle in angles:
 direction=-normal*np.cos(np.deg2rad(angle))+np.array([0.,0.,np.sin(np.deg2rad(angle))]);direction/=np.linalg.norm(direction)
 for mode in ['measured_seed','ideal_tracking']:
  q=np.asarray(before['q'][:7]);samples=[]
  for i,state in enumerate(states):
   if mode=='measured_seed':q=np.asarray(rr[min(i,len(rr)-1)]['obs']['q'][:7])
   target=start[:3,3]+direction*cp['release_retreat_m']*release_progress(min(1.,(i+1)/20.),cp.get('release_retreat_profile','linear'))
   pose=state['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
   for body in g['moving_bodies']:w.move_body(body,delta)
   fingers={n:state['q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
   sample={'release_action_index':i,'recorded_door_q':state['door_q']}
   try:
    command=bounded_position_command(kin,q,target,start[:3,:3]);check=w.clearance(command,mode='execution',gripper_q=fingers);sample.update(command=command.tolist(),target_position=target.tolist(),position_error_m=float(np.linalg.norm(kin.fk(command)[:3,3]-target)),clearance=check,max_increment_rad=float(np.abs(command-q).max()));q=command
   except (RuntimeError,ValueError) as exc:sample['error']=str(exc)
   samples.append(sample)
  item={'angle_deg':angle,'mode':mode,'samples':samples,'solver_errors':sum('error' in s for s in samples),'invalid_commands':sum(not s.get('clearance',{}).get('valid',False) for s in samples),'minimum_slack_m':min((s['clearance']['slack_m'] for s in samples if 'clearance' in s),default=None)};report['results'].append(item);a.output.write_text(json.dumps(report,indent=2));print('BOUNDED_ANGLE',{k:v for k,v in item.items() if k!='samples'},flush=True)
report['complete']=True;a.output.write_text(json.dumps(report,indent=2)+'\n')
