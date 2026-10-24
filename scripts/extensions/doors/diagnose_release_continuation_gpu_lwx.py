"""Check offline release proposals against recorded door and gripper states."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from curobo.types.math import Pose
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_planner import TaskPlanner
from kitchen_safety.planning import transform
p=argparse.ArgumentParser();p.add_argument('--proposals',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
d=json.loads(a.proposals.read_text());ep=Path(d['episode']);m=json.loads((ep/'metadata.json').read_text());cp=m['controller_parameters'];rows=json.loads((ep/'trajectory.json').read_text());rr=[r for r in rows if r['phase'] in ('release_handle','released_hold')]
if not d.get('complete'):raise ValueError('Incomplete proposals')
for n,h in d['input_sha256'].items():
 if sha(ep/n)!=h:raise ValueError('Recorded input changed: '+n)
original=next(x for x in d['results'] if x['mode']=='measured_seed' and x['angle_deg']==cp['release_tilt_deg'])
diffs=[float(np.abs(np.asarray(x['command'])-np.asarray(rr[i]['action'][:7])).max()) for i,x in enumerate(original['samples'][:len(rr)])]
if max(diffs,default=0.)>1e-12:raise ValueError('Stored release command calibration failed')
stage,kin,w,g=reconstruct(m,json.loads(Path(m['variant_config']).read_text()),m['variant_spec']);margin=cp.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin;w.allow_handle_contact=True
planner=TaskPlanner(w,kin,dt=m['dt'],seed=cp['seed'],max_joint_speed=cp['max_joint_speed'],hazard_extra_padding_m=cp.get('planner_hazard_extra_padding_m',0.))
initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:])
report={'scope':'Offline GPU constraints at recorded pre-command door/gripper states, and explicitly frozen last state after the first unexecuted boundary. Commands are CPU proposals, not predicted dynamics or physical acceptance. Full world remains enabled except the same original handle colliders allowed by production.','episode':str(ep),'proposals':str(a.proposals),'proposals_sha256':sha(a.proposals),'implementation_sha256':sha(__file__),'physics_replay':False,'unsafe_path_executed':False,'stored_commands_calibrated':len(diffs),'maximum_stored_command_difference_rad':max(diffs,default=0.),'results':[]}
for item in d['results']:
 if not item.get('all25_proposed'):continue
 samples=[]
 for i,sample in enumerate(item['samples']):
  state=rr[i]['obs'] if i<len(rr) else rr[-1]['next_obs'];pose=state['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
  for body in g['moving_bodies']:w.move_body(body,delta)
  for j,(name,_,lo,hi) in enumerate(w.objects):
   if not any(name.startswith(body+'/') for body in g['moving_bodies']):continue
   rot,pos,rlo,rhi,mesh=w._object_reference[j]
   if mesh is not None:T=planner.T_B_W@delta@kin.base
   else:
    Tb=np.eye(4);Tb[:3,3]=(rlo+rhi)/2;T=planner.T_B_W@delta@Tb
   planner.motion_gen.world_coll_checker.update_obstacle_pose(name,Pose.from_list(planner._pose_list(T),planner.args))
  w.gripper_joint_position=float(state['q'][m['robot_joint_names'].index('finger_joint')]);planner._prepare_path_query(tuple(g['handle_colliders']),0.,'release_handle' if i<20 else 'released_hold')
  command=np.asarray(sample['command']);valid=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all());samples.append({'release_action_index':i,'full_world_valid':valid,'state_scope':'recorded_pre_command' if i<len(rr) else ('first_unexecuted_boundary' if i==len(rr) else 'unobserved_frozen_last_state'),'observed_door_q':state['door_q']})
 x={'angle_deg':item['angle_deg'],'mode':item['mode'],'samples':samples,'all25_gpu_valid':all(s['full_world_valid'] for s in samples),'rejected_indices':[s['release_action_index'] for s in samples if not s['full_world_valid']]};report['results'].append(x);a.output.write_text(json.dumps(report,indent=2));print('GPU_RELEASE_PROPOSAL',{k:v for k,v in x.items() if k!='samples'},flush=True)
report['complete']=True;a.output.write_text(json.dumps(report,indent=2)+'\n')
