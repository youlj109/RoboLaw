"""Fast source-geometry preflight for one approved fixed target. No physics claims."""
import argparse,json,math,traceback
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from curobo.types.math import Pose
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_planner import TaskPlanner
from kitchen_safety.extensions.doors.pregrasp import check_pregrasp
from kitchen_safety.extensions.doors.approach_orientation_lwx import pregrasp_rotation,approach_rotation
from kitchen_safety.extensions.doors.task_model import grasp_rotation,door_target_pose,joint_position_from_openness,opening_command
p=argparse.ArgumentParser();p.add_argument('--metadata',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--variant',choices=['base','human','property','robot'],default='base');p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=42);p.add_argument('--reference-episode',type=Path,required=True)
p.add_argument('--dump-pregrasp',action='store_true')
a=p.parse_args();assert not a.output.exists();m=json.loads(a.metadata.read_text());c=json.loads(a.config.read_text());policy=m['controller_parameters'];stage,kin,world,g=reconstruct(m,c,None if a.variant=='base' else c[a.variant]);margin=policy.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin
planner=TaskPlanner(world,kin,dt=m['dt'],seed=a.seed,max_joint_speed=policy['max_joint_speed'],pregrasp_shortcut_policy=policy.get('pregrasp_shortcut_policy','none'),hazard_extra_padding_m=policy.get('planner_hazard_extra_padding_m',0.),pregrasp_goal_q=policy.get('pregrasp_goal_q'))
initial=np.asarray(m['initial_q']);q=initial[:7];names=m['robot_joint_names'];fingers={n:initial[names.index(n)] for n in world.GRIPPER_JOINT_NAMES};refmeta=json.loads((a.reference_episode/'metadata.json').read_text());rows=json.loads((a.reference_episode/'trajectory.json').read_text());closedrow=next(r['next_obs']['q'] for r in reversed(rows) if r['phase']=='grasp_handle');closed={n:closedrow[refmeta['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
approach=Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).apply([1,0,0]);R0=grasp_rotation(approach,policy['grasp_roll_deg'],policy.get('grasp_pitch_deg',0));handle=np.asarray(m['handle_initial']);handles=tuple(g['handle_colliders']);hinge=np.asarray(g['hinge']);axis=np.asarray(g['axis']);lower,upper=g['limits_rad'];reference=m.get('door_motion_reference_joint_rad',joint_position_from_openness(g['reset_openness'],lower,upper));threshold=joint_position_from_openness(g['openness_threshold'],lower,upper);goal=threshold+(-1 if lower<0 else 1)*max(.015,policy.get('opening_reserve_rad',0.));goal=max(lower+.01,min(upper-.01,goal))
r={'scope':'Offline sampled source geometry preflight only; contact, stability, actual avoidance and success unverified.','variant':a.variant,'pregrasp':False,'approach':False,'pull':False,'accepted_geometry':False,'metadata':str(a.metadata),'reference_episode':str(a.reference_episode),'seed':a.seed}
def save():a.output.write_text(json.dumps(r,indent=2))
def update_door(angle):
 rr=Rotation.from_rotvec(axis*(angle-reference)).as_matrix();delta=np.eye(4);delta[:3,:3]=rr;delta[:3,3]=hinge-rr@hinge
 for b in g['moving_bodies']:world.move_body(b,delta)
 for i,(n,_,lo,hi) in enumerate(world.objects):
  if not any(n.startswith(b+'/') for b in g['moving_bodies']):continue
  _,_,rlo,rhi,mesh=world._object_reference[i];T=planner.T_B_W@delta@kin.base if mesh is not None else planner.T_B_W@delta@np.vstack([np.c_[np.eye(3),(rlo+rhi)/2],[0,0,0,1]])
  planner.motion_gen.world_coll_checker.update_obstacle_pose(n,Pose.from_list(planner._pose_list(T),planner.args))
def validate(command,gripper,phase):
 planner._prepare_path_query(handles,gripper,phase);cv=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all());fv=bool(world.valid(command,mode='execution',gripper_q=closed if gripper else fingers));return cv and fv,{'curobo':cv,'source_fcl':fv,'detail':world.last_clearance}
try:
 target=np.eye(4);target[:3,:3]=pregrasp_rotation(approach,policy);target[:3,3]=handle-approach*policy.get('pregrasp_distance_m',.13);world.allow_handle_contact=False;world.gripper_joint_position=0.;plan=planner.plan_pose(q,target,phase='door_pregrasp');v=check_pregrasp(planner,plan);r['pregrasp_validation']=v
 if a.dump_pregrasp:r['pregrasp_positions']=plan['positions'].tolist();r['pregrasp_tcp_world_m']=[kin.fk(x)[:3,3].tolist() for x in plan['positions']]
 if not v['accepted']:raise RuntimeError('Dense pregrasp rejected')
 r['pregrasp']=True;q=plan['positions'][-1];world.allow_handle_contact=True
 for alpha in np.linspace(0,1,50)[1:]:
  target[:3,:3]=approach_rotation(approach,policy,float(alpha));target[:3,3]=handle-approach*policy.get('pregrasp_distance_m',.13)*(1-alpha);command=kin.ik(target,q,attempts=1);ok,detail=validate(command,0.,'approach_handle')
  if not ok:r['rejection']={'phase':'approach','alpha':float(alpha),**detail};raise RuntimeError('Approach rejected')
  q=command
 r['approach']=True;world.gripper_joint_position=closed['finger_joint']
 for angle in np.linspace(reference,goal,int(abs(goal-reference)/.003)+2):
  update_door(float(angle));command_angle=opening_command(float(angle),min(.006,policy['pull_lead_rad']),goal,lower);target=door_target_pose(hinge,axis,handle,R0,command_angle,reference);command=kin.ik(target,q,attempts=1);dq=command-q;command=q+dq/max(1.,float(np.max(np.abs(dq)))/.025);ok,detail=validate(command,1.,'pull_door')
  if not ok:r['rejection']={'phase':'pull','angle':float(angle),**detail};raise RuntimeError('Projected pull rejected')
  q=command
 r['pull']=True;r['accepted_geometry']=True
except Exception as e:r['error']=str(e)
finally:save()
print('APPROVED_PREFLIGHT',json.dumps(r),flush=True)
raise SystemExit(0 if r['accepted_geometry'] else 1)
