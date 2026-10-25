"""Bounded grasp/upper-route search with fixed hazards and unchanged dense collision checks."""
import numpy as np
from kitchen_safety.planning import transform


def preflight_grasp(planner,q,rotation,center,offset,pick_pose,pick,closed_grip=.383,contact_standoff_m=.02):
    world=planner.world;old_attachment=world.attachment;old_grip=getattr(world,'gripper_joint_position',None);old_phase=world.allowed_phase
    try:
        world.allowed_phase='approach';world.attachment=None;world.gripper_joint_position=0.
        hover=np.eye(4);hover[:3,:3]=rotation;hover[:3,3]=center+offset
        first=planner.plan_pose(q,hover,(pick,),0.,phase='transfer')
        checked=planner.validate_path(first['positions'],(pick,),0.,phase='transfer')
        if not checked['valid']:raise RuntimeError('Hover path rejected: '+str(checked))
        contact=hover.copy();contact[:3,3]=center+contact_standoff_m*offset/np.linalg.norm(offset)
        second=planner.plan_cartesian_line(first['positions'][-1],contact,(pick,),0.,phase='lift')
        checked2=planner.validate_path(second['positions'],(pick,),0.,phase='transfer')
        if not checked2['valid']:raise RuntimeError('Contact path rejected: '+str(checked2))
        end=second['positions'][-1];tcp=planner.kin.fk(end);world.attachment=np.linalg.inv(tcp)@transform(pick_pose[:3],pick_pose[3:7]);world.gripper_joint_position=closed_grip;world.allowed_phase='lift'
        lift=tcp.copy();lift[2,3]+=.12
        third=planner.plan_pose(end,lift,(pick,),1.,phase='lift')
        checked3=planner.validate_path(third['positions'],(pick,),1.,phase='lift')
        if not checked3['valid']:raise RuntimeError('Lift path rejected: '+str(checked3))
        return {'passed':True,'scope':'Planned hover/contact/lift only; physical bilateral grasp remains required','checks':[checked,checked2,checked3]}
    finally:
        world.attachment=old_attachment;world.gripper_joint_position=old_grip;world.allowed_phase=old_phase


def plan_transfer(planner,q,target,ignore,grip):
    failures=[]
    try:return planner.plan_pose(q,target,ignore,grip,phase='transfer')
    except RuntimeError as exc:failures.append({'route':'direct','error':str(exc)})
    start=planner.kin.fk(q)
    for rise in (.08,.16,.24):
        qstart=np.array(q,copy=True);legs=[]
        via_start=start.copy();via_start[2,3]+=rise;via_goal=target.copy();via_goal[2,3]+=rise
        try:
            for goal in (via_start,via_goal,target):
                leg=planner.plan_pose(qstart,goal,ignore,grip,phase='transfer');checked=planner.validate_path(leg['positions'],ignore,grip,phase='transfer')
                if not checked['valid']:raise RuntimeError(str(checked))
                legs.append(leg);qstart=leg['positions'][-1]
            positions=np.concatenate([leg['positions'] if i==0 else leg['positions'][1:] for i,leg in enumerate(legs)])
            checked=planner.validate_path(positions,ignore,grip,phase='transfer')
            if not checked['valid']:raise RuntimeError(str(checked))
            result={'positions':positions,'metadata':dict(legs[-1].get('metadata',{}))}
            result['metadata']=dict(result.get('metadata',{}),method='motion_gen_upper_route',samples=len(positions),upper_rise_m=rise,route_failures=failures,leg_metadata=[leg.get('metadata',{}) for leg in legs])
            return result
        except RuntimeError as exc:failures.append({'rise_m':rise,'error':str(exc)})
    raise RuntimeError('No direct or upper route at fixed scene: '+str(failures))

TOPDOWN_CANDIDATES=[(tilt,yaw,height) for height in (.04,.045,.05) for tilt,yaw in ((0,180),(0,0),(-15,180),(15,180),(-15,0),(15,0))]
TOPDOWN_POLICY={'name':'b03-topdown-v1','candidates':[list(x) for x in TOPDOWN_CANDIDATES],'pregrasp_world_z_m':.125,'contact_standoff_m':0.,'lift_m':.12,'closed_grip_preflight_rad':.383,'upper_route_rises_m':[.08,.16,.24]}


def search_topdown(world,kin,q,pick_pose,*,bias_x_m,finger_bias_m,dt,seed):
    from scipy.spatial.transform import Rotation
    from .planner import LimePlanner
    nominal=(transform(pick_pose[:3],pick_pose[3:7])@np.r_[world.object_center,1.])[:3];nominal[0]+=bias_x_m;attempts=[]
    for tilt,yaw,height in TOPDOWN_CANDIDATES:
        rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]],float)
        center=nominal+np.array([0,0,height])+finger_bias_m*rotation[:,1];offset=np.array([0.,0.,.125]);row={'tilt':tilt,'yaw':yaw,'height_offset_m':height}
        print('TOPDOWN_CANDIDATE',world.pick_object,tilt,yaw,height,flush=True)
        planner=LimePlanner(world,kin,dt=dt,seed=seed,max_joint_speed=.4)
        try:
            checked=preflight_grasp(planner,q,rotation,center,offset,pick_pose,world.pick_object,contact_standoff_m=0.)
            row.update(passed=True,preflight=checked);attempts.append(row)
            print('TOPDOWN_SELECTED',world.pick_object,tilt,yaw,height,flush=True)
            return {'rotation':rotation,'center':center,'offset':offset,**row},attempts
        except RuntimeError as exc:
            row.update(passed=False,error=str(exc));attempts.append(row);print('TOPDOWN_REJECTED',tilt,yaw,height,str(exc)[:300],flush=True)
        finally:
            del planner
    raise RuntimeError('All fixed-scene top-down grasp candidates rejected: '+str(attempts))
