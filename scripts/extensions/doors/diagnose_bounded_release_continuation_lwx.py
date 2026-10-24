"""Offline full release proposals, including the first unexecuted boundary."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.release_position_lwx import bounded_position_command
from kitchen_safety.extensions.doors.release_hold_lwx import bounded_hold_command
from kitchen_safety.extensions.doors.release_pose_lwx import release_progress
from kitchen_safety.planning import transform
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--angles',default='-45,-40,-35,-30,-25');a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
sha=lambda x:hashlib.sha256(Path(x).read_bytes()).hexdigest()
m=json.loads((a.episode/'metadata.json').read_text());rows=json.loads((a.episode/'trajectory.json').read_text());cp=m['controller_parameters'];rr=[r for r in rows if r['phase'] in ('release_handle','released_hold')]
if not rr:raise RuntimeError('No recorded release boundary')
if cp.get('release_command_policy')!='bounded_position_hold_v2' or not cp.get('release_retreat_m',0.):raise ValueError('Requires bounded_position_hold_v2 with nonzero retreat')
stage,kin,w,g=reconstruct(m,json.loads(Path(m['variant_config']).read_text()),m['variant_spec']);w.allow_handle_contact=True
margin=cp.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin
before=rr[0]['obs'];start=kin.fk(np.asarray(before['q'][:7]));initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]);normal=Rotation.from_rotvec(np.asarray(g['axis'])*(before['door_q']-m['door_motion_reference_joint_rad'])).apply(Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).apply([1.,0.,0.]))
report={'episode':str(a.episode),'implementation_sha256':sha(__file__),'input_sha256':{n:sha(a.episode/n) for n in ['trajectory.json','metadata.json','result.json']},'scope':'Offline CPU proposals only. Includes final measured seed for the first unexecuted command. Later unobserved door/finger states are explicitly frozen at the last observation. Does not predict changed dynamics, GPU constraints or physical success. Original 0.025rad and 10deg guards retained.','physics_replay':False,'results':[]}
angles=list(map(float,a.angles.split(',')))
if any(abs(v)>60 for v in angles):raise ValueError('Outside original direction bounds')
for angle in angles:
 direction=-normal*np.cos(np.deg2rad(angle))+np.array([0.,0.,np.sin(np.deg2rad(angle))]);direction/=np.linalg.norm(direction)
 for mode in ['measured_seed','ideal_tracking']:
  q=np.asarray(before['q'][:7]);samples=[];hold_target=None
  for i in range(25):
   observed=i<len(rr);seed_state=rr[i]['obs'] if observed else rr[-1]['next_obs'];world_state=rr[i]['next_obs'] if observed else rr[-1]['next_obs']
   if mode=='measured_seed':q=np.asarray(seed_state['q'][:7])
   pose=world_state['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
   for body in g['moving_bodies']:w.move_body(body,delta)
   fingers={n:world_state['q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
   sample={'release_action_index':i,'phase':'release_handle' if i<20 else 'released_hold','state_scope':'recorded_action' if observed else ('first_unexecuted_boundary' if i==len(rr) else 'unobserved_frozen_last_state'),'recorded_door_q':world_state['door_q']}
   try:
    if i<20:
     target=start[:3,3]+direction*cp['release_retreat_m']*release_progress((i+1)/20.,cp.get('release_retreat_profile','linear'));command=bounded_position_command(kin,q,target,start[:3,:3]);hold_target=command.copy()
    else:command=bounded_hold_command(q,hold_target)
    check=w.clearance(command,mode='execution',gripper_q=fingers);sample.update(command=command.tolist(),clearance=check,max_increment_rad=float(np.abs(command-q).max()),rotation_departure_deg=float(np.rad2deg(Rotation.from_matrix(kin.fk(command)[:3,:3]@start[:3,:3].T).magnitude())));q=command
   except (RuntimeError,ValueError) as exc:sample['error']=str(exc)
   samples.append(sample)
   if 'error' in sample:break
  item={'angle_deg':angle,'mode':mode,'samples':samples,'solver_errors':sum('error' in s for s in samples),'invalid_commands':sum('clearance' in s and not s['clearance']['valid'] for s in samples),'minimum_slack_m':min((s['clearance']['slack_m'] for s in samples if 'clearance' in s),default=None),'all25_proposed':len(samples)==25 and not any('error' in s for s in samples)};report['results'].append(item);print('CONTINUATION',{k:v for k,v in item.items() if k!='samples'},flush=True);a.output.write_text(json.dumps(report,indent=2))
report['complete']=True;a.output.write_text(json.dumps(report,indent=2)+'\n')
