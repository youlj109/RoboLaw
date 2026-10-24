"""Offline task grasp/approach/pull lookahead; no physical success claims."""
import argparse,json,itertools
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_model import door_target_pose,grasp_rotation,joint_position_from_openness,opening_command
from scipy.spatial.transform import Rotation
from curobo.types.math import Pose
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.pregrasp import check_pregrasp
from kitchen_safety.extensions.doors.task_planner import TaskPlanner as CuroboPlanner
p=argparse.ArgumentParser();p.add_argument('--metadata',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--offsets',default='0,.03,-.03,.06,-.06,.09,-.09');p.add_argument('--rolls',default='0,180,90,-90');p.add_argument('--pitches',default='0');p.add_argument('--seed',type=int,default=42);p.add_argument('--normal-offsets',default='0,-.005,-.01,-.015,-.02,-.025');p.add_argument('--limit-margin-rad',type=float,default=.005);p.add_argument('--pull-lead-rad',type=float,default=.05);p.add_argument('--check-release',action='store_true')
p.add_argument('--check-release-settled',action='store_true');p.add_argument('--release-extra-margin-m',type=float,default=0.)
p.add_argument('--opening-reserve-rad',type=float,default=0.)
p.add_argument('--pregrasp-shortcut-policy',choices=['none','bounded_chords_v1'],default='none')
a=p.parse_args()
if not 0.<=a.opening_reserve_rad<=.08:p.error('Opening reserve outside [0,.08]rad')
if a.output.exists():p.error('Evidence already exists')
m=json.loads(a.metadata.read_text());config=json.loads(a.config.read_text());stage,kin,world,g=reconstruct(m,config);kin.limits[:,0]+=a.limit_margin_rad;kin.limits[:,1]-=a.limit_margin_rad;planner=CuroboPlanner(world,kin,dt=m['dt'],seed=a.seed,max_joint_speed=.5,pregrasp_shortcut_policy=a.pregrasp_shortcut_policy)
initial=np.asarray(m['initial_q']);q0=initial[:7];fingers={n:initial[m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
reference=Path('datasets/doors/lwx_C02_C03_20261023/C02_base_04/trajectory.json');rows=json.loads(reference.read_text());qref=next(r['next_obs']['q'] for r in reversed(rows) if r['phase']=='grasp_handle');closed={n:qref[m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
approach=Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).apply([1.,0.,0.]);Rbase=np.column_stack([approach,np.cross([0,0,-1.],approach),[0,0,-1.]])
handles=tuple(g['handle_colliders']);hinge=np.asarray(g['hinge']);axis=np.asarray(g['axis']);lower,limit_upper=g['limits_rad'];upper=joint_position_from_openness(g['openness_threshold'],lower,limit_upper)+(-1. if lower<0. else 1.)*max(.015,a.opening_reserve_rad)
reference_joint=joint_position_from_openness(g['reset_openness'],*g['limits_rad'])
report={'opening_reserve_rad':a.opening_reserve_rad,'reference_joint_rad':reference_joint,'ik_command_margin_rad':a.limit_margin_rad,'pull_lead_rad':a.pull_lead_rad,'candidate_order':{'rolls':a.rolls,'pitches':a.pitches,'normal_offsets':a.normal_offsets,'height_offsets':a.offsets},'scope':'Offline projected geometry only; physical contact, stability and success unverified.','task':m['task'],'seed':a.seed,'candidates':[]}
if a.pregrasp_shortcut_policy!='none':report['pregrasp_shortcut_policy']=a.pregrasp_shortcut_policy
def save():a.output.write_text(json.dumps(report,indent=2))
def update_door(delta):
 for b in g['moving_bodies']:world.move_body(b,delta)
 for i,(n,_,lo,hi) in enumerate(world.objects):
  if not any(n.startswith(b+'/') for b in g['moving_bodies']):continue
  _,_,rlo,rhi,mesh=world._object_reference[i];T=planner.T_B_W@delta@kin.base if mesh is not None else planner.T_B_W@delta@np.vstack([np.c_[np.eye(3),(rlo+rhi)/2],[0,0,0,1]])
  planner.motion_gen.world_coll_checker.update_obstacle_pose(n,Pose.from_list(planner._pose_list(T),planner.args))
def valid(q,gripper,phase):
 planner._prepare_path_query(handles,gripper,phase)
 cv=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(q[None])).all());fv=world.valid(q,mode='execution',gripper_q=closed if gripper else fingers)
 return cv,fv,world.last_clearance
for pitch,roll,normal in itertools.product(map(float,a.pitches.split(',')),map(float,a.rolls.split(',')),map(float,a.normal_offsets.split(','))):
 for offset in map(float,a.offsets.split(',')):
  r={'pitch_deg':pitch,'normal_offset_m':normal,'offset_z':offset,'roll_deg':roll,'pregrasp':False,'approach':False,'pull':False};report['candidates'].append(r)
  try:
   world.allow_handle_contact=False;world.gripper_joint_position=0.;update_door(np.eye(4))
   R0=grasp_rotation(approach,roll,pitch);handle=np.asarray(g['handle_center'])+np.asarray([0,0,offset])+approach*normal;target=np.eye(4);target[:3,:3]=R0;target[:3,3]=handle-approach*.13
   bounds=np.asarray(g['handle_bounds'])
   if np.any(handle<bounds[0]-.003) or np.any(handle>bounds[1]+.003):r['error']='Requested handle point outside source collision bounds';save();continue
   plan=planner.plan_pose(q0,target,phase='door_pregrasp');v=check_pregrasp(planner,plan);r['pregrasp_validation']=v
   if not v['accepted']:save();continue
   r['pregrasp']=True;q=plan['positions'][-1];r['positions']=plan['positions'].tolist();r['target']=target.tolist();world.allow_handle_contact=True
   for alpha in np.linspace(0,1,50)[1:]:
    target[:3,3]=handle-approach*.13*(1-alpha);command=kin.ik(target,q,attempts=1);cv,fv,detail=valid(command,0.,'approach_handle')
    if not cv or not fv:r['rejected']={'phase':'approach','alpha':float(alpha),'q':command.tolist(),'curobo':cv,'fcl':fv,'detail':detail};break
    q=command
   else:r['approach']=True
   if not r['approach']:save();print('SCREEN',roll,normal,offset,'APPROACH_FAIL',r['rejected'],flush=True);continue
   world.gripper_joint_position=closed['finger_joint']
   for angle in np.linspace(reference_joint,upper,int(abs(upper-reference_joint)/.003)+2):
    rr=Rotation.from_rotvec(axis*(angle-reference_joint)).as_matrix();delta=np.eye(4);delta[:3,:3]=rr;delta[:3,3]=hinge-rr@hinge;update_door(delta)
    target=door_target_pose(hinge,axis,handle,R0,opening_command(angle,a.pull_lead_rad,lower+.01 if lower<0. else limit_upper-.01,lower),reference_joint);command=kin.ik(target,q,attempts=1);dq=command-q;command=q+dq/max(1.,float(np.max(np.abs(dq)))/.025)
    cv,fv,detail=valid(command,1.,'pull_door')
    if not cv or not fv:r['rejected']={'phase':'pull','angle':float(angle),'q':command.tolist(),'curobo':cv,'fcl':fv,'detail':detail};break
    q=command
   else:r['pull']=True
   if r['pull'] and a.check_release:
    cv,fv,detail=valid(q,0.,'release_handle');r['release']=bool(cv and fv)
    if r['release'] and a.check_release_settled:
     release_angle=opening_command(upper,a.pull_lead_rad,lower+.01 if lower<0. else limit_upper-.01,lower);rr=Rotation.from_rotvec(axis*(release_angle-reference_joint)).as_matrix();delta=np.eye(4);delta[:3,:3]=rr;delta[:3,3]=hinge-rr@hinge;update_door(delta);r['release_sweep']=[]
     for fraction in np.linspace(0.,1.,13):
      grip={n:closed[n]*(1-fraction)+fingers[n]*fraction for n in fingers};detail=world.clearance(q,mode='execution',gripper_q=grip,obstacle_filter=tuple(g['moving_bodies']));r['release_sweep'].append({'opening_fraction':float(fraction),'clearance':detail})
      if not detail['valid'] or detail['slack_m']<a.release_extra_margin_m:r['release']=False;break
    if not r['release']:r['rejected']={'phase':'release','curobo':cv,'fcl':fv,'detail':detail}
  except Exception as exc:r['error']=str(exc)
  save();print('SCREEN',roll,normal,offset,r['pregrasp'],r['approach'],r['pull'],r.get('rejected',r.get('error')),flush=True)
  if r['pull'] and (not a.check_release or r.get('release')):report['first_feasible']=r;report['search_complete']=True;save();raise SystemExit(0)
report['search_complete']=True;save();raise SystemExit(1)
