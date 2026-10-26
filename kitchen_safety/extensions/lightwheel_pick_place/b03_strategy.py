"""Bounded grasp/upper-route search with fixed hazards and unchanged dense collision checks."""
import numpy as np
from kitchen_safety.planning import transform


def preflight_grasp(planner,q,rotation,center,offset,pick_pose,pick,closed_grip=.383,contact_standoff_m=.02,closed_grip_candidates=None,lift_m=.12):
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
        end=second['positions'][-1];tcp=planner.kin.fk(end);world.attachment=np.linalg.inv(tcp)@transform(pick_pose[:3],pick_pose[3:7]);world.allowed_phase='lift'
        lift=tcp.copy();lift[2,3]+=lift_m;lift_checks=[]
        for grip in (closed_grip_candidates or (closed_grip,)):
            world.gripper_joint_position=grip
            third=planner.plan_pose(end,lift,(pick,),1.,phase='lift')
            checked3=planner.validate_path(third['positions'],(pick,),1.,phase='lift')
            if not checked3['valid']:raise RuntimeError('Lift path rejected: '+str(checked3))
            lift_checks.append({'gripper_joint_rad':grip,'validation':checked3})
        return {'passed':True,'scope':'Planned hover/contact/lift only; physical bilateral grasp remains required','checks':[checked,checked2],'lift_checks':lift_checks}
    finally:
        world.attachment=old_attachment;world.gripper_joint_position=old_grip;world.allowed_phase=old_phase


def plan_transfer(planner,q,target,ignore,grip):
    failures=[]
    try:
        direct=planner.plan_pose(q,target,ignore,grip,phase='transfer')
        checked=planner.validate_path(direct['positions'],ignore,grip,phase='transfer')
        if not checked['valid']:raise RuntimeError('Direct route rejected by full geometry: '+str(checked))
        return direct
    except RuntimeError as exc:failures.append({'route':'direct','error':str(exc)})
    start=planner.kin.fk(q)
    offsets=[(0.,0.,z) for z in (.08,.16,.24)]
    if planner.world.pick_object=='green_ladle':offsets += [(0.,.12,0.),(.12,0.,0.),(0.,-.12,0.),(-.12,0.,0.)]
    if planner.world.pick_object in ('banana_1','banana_2','green_ladle'):
        offsets += REACHABLE_LATERAL_OFFSETS
    for delta in offsets:
        rise=delta[2]
        qstart=np.array(q,copy=True);legs=[]
        via_start=start.copy();via_start[:3,3]+=np.asarray(delta);via_goal=target.copy();via_goal[:3,3]+=np.asarray(delta)
        try:
            for goal in (via_start,via_goal,target):
                leg=planner.plan_pose(qstart,goal,ignore,grip,phase='transfer');checked=planner.validate_path(leg['positions'],ignore,grip,phase='transfer')
                if not checked['valid']:raise RuntimeError(str(checked))
                legs.append(leg);qstart=leg['positions'][-1]
            positions=np.concatenate([leg['positions'] if i==0 else leg['positions'][1:] for i,leg in enumerate(legs)])
            checked=planner.validate_path(positions,ignore,grip,phase='transfer')
            if not checked['valid']:raise RuntimeError(str(checked))
            result={'positions':positions,'metadata':dict(legs[-1].get('metadata',{}))}
            result['metadata']=dict(result.get('metadata',{}),method='motion_gen_upper_route',samples=len(positions),upper_rise_m=rise,route_offset_world_m=list(delta),route_failures=failures,leg_metadata=[leg.get('metadata',{}) for leg in legs])
            return result
        except RuntimeError as exc:failures.append({'route_offset_world_m':list(delta),'error':str(exc)})
    if planner.world.pick_object in ('banana_1','banana_2','green_ladle'):
        return plan_checked_cartesian_bypass(planner,q,target,ignore,grip,failures)
    raise RuntimeError('No direct or upper route at fixed scene: '+str(failures))

PAD_CALIBRATION_PATH='configs/extensions/lightwheel_pick_place/b03_pad_contact_calibration.json'
TOPDOWN_CANDIDATES=[(0,180,.0023,.011),(0,0,.0023,.011),(0,180,.002,0.),(0,0,.002,.03),(0,180,.002,.03),(0,0,.002,0.),(0,0,.003,-.03),(0,180,0.,.03),(0,180,.003,-.03),(15,180,0.,0.),(45,180,-.003,.03),(45,180,-.003,-.03),(45,180,-.003,0.),(60,180,-.003,.03),(75,180,.003,.03)]+[(tilt,yaw,z,y) for y in (.03,-.03,0.) for z in (-.003,0.,.003) for tilt,yaw in ((60,180),(70,180),(75,180),(80,180),(45,180),(-60,0),(-70,0)) if (tilt,yaw,z,y)!=(45,180,0.,0.)]
TOPDOWN_POLICY={'name':'b03-padcenter-v7-balanced-both','grasp_point':'Actual collision-mesh cross-section centroid aligned to measured bilateral pad center','candidates':[list(x) for x in TOPDOWN_CANDIDATES],'closed_grip_preflight_range_rad':[.30,.383,.46,.55],'pregrasp_world_z_m':.08,'contact_standoff_m':0.,'lift_m':.10,'upper_route_rises_m':[.08,.16,.24]}


def search_topdown(world,kin,q,pick_pose,*,bias_x_m,finger_bias_m,dt,seed):
    from scipy.spatial.transform import Rotation
    from .planner import LimePlanner
    import json,hashlib
    from pathlib import Path
    calibration_path=Path(PAD_CALIBRATION_PATH);calibration=json.loads(calibration_path.read_text());pad_offset=np.array(calibration['pad_center_tcp_m']);TOPDOWN_POLICY['pad_calibration_sha256']=hashlib.sha256(calibration_path.read_bytes()).hexdigest();attempts=[]
    for tilt,yaw,pinch_z,y_shift in TOPDOWN_CANDIDATES:
        if [tilt,yaw,pinch_z,y_shift] in TOPDOWN_POLICY.get('physical_exclusions',{}).get(world.pick_object,[]):
            attempts.append({'tilt':tilt,'yaw':yaw,'pinch_z_offset_m':pinch_z,'waist_y_shift_m':y_shift,'passed':False,'reason':'Excluded after a preserved real physical failure; never repeated unchanged'})
            continue
        local_grasp=collision_mesh_waist(world,y_shift=y_shift);nominal=(transform(pick_pose[:3],pick_pose[3:7])@np.r_[local_grasp,1.])[:3];nominal[0]+=bias_x_m
        rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]],float)
        center=nominal+np.array([0,0,pinch_z])-rotation@pad_offset+finger_bias_m*rotation[:,1];height=float(center[2]-nominal[2]);offset=np.array([0.,0.,.08]);row={'tilt':tilt,'yaw':yaw,'height_offset_m':height,'pinch_z_offset_m':pinch_z,'waist_y_shift_m':y_shift,'grasp_point_body_m':local_grasp.tolist()}
        print('TOPDOWN_CANDIDATE',world.pick_object,tilt,yaw,height,flush=True)
        planner=LimePlanner(world,kin,dt=dt,seed=seed,max_joint_speed=.4);planner.pose_ik_joint_fallback=True
        try:
            checked=preflight_grasp(planner,q,rotation,center,offset,pick_pose,world.pick_object,contact_standoff_m=0.,closed_grip_candidates=(.30,.383,.46,.55),lift_m=.10)
            row.update(passed=True,preflight=checked);attempts.append(row)
            print('TOPDOWN_SELECTED',world.pick_object,tilt,yaw,height,flush=True)
            return {'rotation':rotation,'center':center,'offset':offset,**row},attempts
        except RuntimeError as exc:
            row.update(passed=False,error=str(exc));attempts.append(row);print('TOPDOWN_REJECTED',tilt,yaw,height,str(exc)[:300],flush=True)
        finally:
            del planner
    raise RuntimeError('All fixed-scene top-down grasp candidates rejected: '+str(attempts))


def collision_mesh_waist(world,*,y_shift=0.):
    """Solid mid-length cross-section centroid; a curved banana's bounding-box center is empty space."""
    from scipy.spatial import ConvexHull
    root=(world.env_root or '/World/envs/env_0')+'/'+world.pick_object
    inverse=np.linalg.inv(world._root_reference_transforms[root]);parts=[]
    for (name,*_),(*_,mesh) in zip(world.objects,world._object_reference):
        if mesh is not None and world._matches(name,world.pick_object):
            vertices,faces=mesh;local=vertices@inverse[:3,:3].T+inverse[:3,3];parts.append((local,faces))
    all_vertices=np.concatenate([v for v,f in parts]);plane_y=float((all_vertices[:,1].min()+all_vertices[:,1].max())*.5+y_shift);crossings=[]
    for vertices,faces in parts:
        triangles=vertices[np.asarray(faces,int)]
        for edge in ((0,1),(1,2),(2,0)):
            a=triangles[:,edge[0]];b=triangles[:,edge[1]];dy=b[:,1]-a[:,1];mask=((a[:,1]-plane_y)*(b[:,1]-plane_y)<=0)&(np.abs(dy)>1e-10)
            points=a[mask]+((plane_y-a[mask,1])/dy[mask])[:,None]*(b[mask]-a[mask]);crossings.extend(points)
    points=np.asarray(crossings);xz=points[:,[0,2]];polygon=xz[ConvexHull(xz).vertices];next_vertices=np.roll(polygon,-1,axis=0);weights=polygon[:,0]*next_vertices[:,1]-next_vertices[:,0]*polygon[:,1];area_twice=weights.sum()
    if abs(area_twice)<1e-10:raise RuntimeError('Degenerate banana waist section')
    centroid=((polygon+next_vertices)*weights[:,None]).sum(axis=0)/(3*area_twice)
    return np.array([centroid[0],plane_y,centroid[1]])


def close_recenter_delta(world,kin,state,arm_ids,joint_names,contact_row,solution):
    """Use real one-sided contact gaps to recenter the robot, without moving the object."""
    import json
    from scipy.spatial.transform import Rotation
    q=state['q'][arm_ids];rotation=Rotation.from_quat(state['tcp_quat_xyzw']).as_matrix();axis=rotation[:,1]
    gripper={n:float(state['q'][joint_names.index(n)]) for n in world.GRIPPER_JOINT_NAMES}
    world.set_obstacle_pose(world.pick_object,state['lime'][:7]);body=[o for n,o,lo,hi in world.objects if world._matches(n,world.pick_object)];shapes=world.shapes(q,gripper_q=gripper);entries=[]
    for finger in ('left_inner_finger','right_inner_finger'):
        pieces=[x for x in shapes if world._matches(x[0],finger)]
        native=contact_row['fingers'][finger]
        if native['count']:gap=max(0.,float(min(native['separations_m'])))
        else:
            gap=min((max(0.,float(world.fcl.distance(piece[1],obj,world.fcl.DistanceRequest(),world.fcl.DistanceResult()))) for piece in pieces for obj in body),default=.003)
            gap=max(.003,gap)
        side=float(np.mean([np.dot((p[2]+p[3])*.5-state['tcp'],axis) for p in pieces])) if pieces else (1. if finger.startswith('left') else -1.)
        entries.append((side,gap,finger))
    entries.sort();delta=-.5*(entries[1][1]-entries[0][1])*axis
    if np.linalg.norm(delta)<.0005:
        point=np.asarray(solution['grasp_point_body_m']);desired=(transform(state['lime'][:3],state['lime'][3:7])@np.r_[point,1.])[:3];desired[2]+=solution.get('pinch_z_offset_m',0.)
        pad=np.asarray(json.load(open(PAD_CALIBRATION_PATH))['pad_center_tcp_m']);delta=desired-rotation@pad-state['tcp'];delta[2]=np.clip(delta[2],-.004,.004)
    length=float(np.linalg.norm(delta))
    if length>.012:delta*=.012/length
    return delta,{'finger_gaps_m':{n:g for side,g,n in entries},'delta_world_m':delta.tolist()}

TOPDOWN_POLICY.update(max_recenter_retries=2,recenter_max_step_m=.012,recenter_open_steps=12,recenter_close_steps=30,close_requires_bilateral_load=True)

TOPDOWN_POLICY.update(pose_ik_joint_goal_fallback={'enabled':True,'seed_scales':[0.,.2,.4,.7],'max_position_error_m':.002,'max_orientation_error_deg':1.},physical_exclusions={'banana_1':[[15,180,0.,0.]],'banana_2':[[0,0,.002,.03]]})

REACHABLE_LATERAL_OFFSETS=[(-.10,.10,0.),(-.15,.15,0.),(-.10,0.,.05),(-.15,.10,.06),(0.,.12,0.),(-.10,-.10,.05)]

def baseline_route_extension_proof(baseline):
    """Certify that new fallbacks cannot change a recorded direct baseline."""
    import ast,hashlib,json,subprocess
    from pathlib import Path
    baseline=Path(baseline);r=json.loads((baseline/'result.json').read_text())
    old=subprocess.check_output(['git','show','3997737:kitchen_safety/extensions/lightwheel_pick_place/b03_strategy.py'],text=True)
    new=Path(__file__).read_text()
    def function(text):
        return ast.get_source_segment(text,next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name=='plan_transfer'))
    old_function=function(old);new_function=function(new)
    block="    if planner.world.pick_object in ('banana_1','banana_2','green_ladle'):\n        offsets += REACHABLE_LATERAL_OFFSETS\n"
    cartesian_block="    if planner.world.pick_object in ('banana_1','banana_2','green_ladle'):\n        return plan_checked_cartesian_bypass(planner,q,target,ignore,grip,failures)\n"
    unchanged=new_function.replace(block,'').replace(cartesian_block,'')==old_function
    plans=[p for p in r.get('plans',[]) if p.get('executed')]
    direct=bool(plans) and all(p.get('metadata',{}).get('method') in ('motion_gen','cartesian_ik','cartesian_line_ik') and 'route_offset_world_m' not in p.get('metadata',{}) for p in plans)
    return {'passed':bool(unchanged and direct and r.get('success')),'baseline_result_sha256':hashlib.sha256((baseline/'result.json').read_bytes()).hexdigest(),'legacy_commit':'3997737','legacy_function_sha256':hashlib.sha256(old_function.encode()).hexdigest(),'current_function_sha256':hashlib.sha256(new_function.encode()).hexdigest(),'legacy_function_unchanged_except_appended_fallbacks':unchanged,'all_recorded_baseline_plans_direct_or_cartesian':direct,'added_offsets_world_m':[list(x) for x in REACHABLE_LATERAL_OFFSETS],'scope':'Old direct/legacy fallback code is unchanged; new offsets occur only after direct rejection. Every actual baseline plan succeeded without the new branch. Physical success and independent collision gates remain separately mandatory.'}

def plan_checked_cartesian_bypass(planner,q,target,ignore,grip,previous_failures):
    """Local IK bypass; cuRobo constraints and full dense FCL remain mandatory."""
    from scipy.spatial.transform import Rotation
    start=planner.kin.fk(q);failures=list(previous_failures)
    for delta in [(0.,0.,0.),*REACHABLE_LATERAL_OFFSETS]:
        a=start.copy();a[:3,3]+=delta;b=np.asarray(target).copy();b[:3,3]+=delta
        goals=[target] if delta==(0.,0.,0.) else [a,b,target]
        current=np.array(q,copy=True);legs=[]
        try:
            for goal in goals:
                leg=planner.plan_cartesian_line(current,goal,ignore,grip,phase='transfer')
                check=planner.validate_path(leg['positions'],ignore,grip,phase='transfer')
                if not check['valid']:raise RuntimeError(str(check))
                legs.append(leg);current=leg['positions'][-1]
            actual=planner.kin.fk(current)
            pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));re=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude()))
            if pe>.002 or re>1.:raise RuntimeError('Unchanged final TCP goal tolerance failed')
            positions=np.concatenate([x['positions'] if i==0 else x['positions'][1:] for i,x in enumerate(legs)])
            checked=planner.validate_path(positions,ignore,grip,phase='transfer')
            if not checked['valid']:raise RuntimeError(str(checked))
            return {'positions':positions,'metadata':{'backend':'local_ik_curobo_checked','method':'mixed_cartesian_bypass','samples':len(positions),'route_offset_world_m':list(delta),'goal_position_error_m':pe,'goal_orientation_error_deg':re,'leg_metadata':[x['metadata'] for x in legs],'route_failures':failures,'scope':'No MotionGen success claim; each local line and complete concatenated path pass cuRobo constraints and full FCL, followed by actual physics audits.'}}
        except RuntimeError as e:failures.append({'cartesian_route_offset_world_m':list(delta),'error':str(e)})
    raise RuntimeError('All checked Cartesian bypasses failed: '+str(failures))
