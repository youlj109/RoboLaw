"""Offline contact-approach lookahead for candidate screening; does not change collection policy."""
import argparse, json, hashlib
from pathlib import Path
import numpy as np
from scripts.extensions.doors.paired_planning import plan
from kitchen_safety.extensions.doors.reconstruction import reconstruct, DOOR
from scipy.spatial.transform import Rotation
from curobo.types.math import Pose
from kitchen_safety.curobo_backend import CuroboPlanner

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,required=True)
p.add_argument('--variant',choices=['human','property'],default='property')
p.add_argument('--output',type=Path,required=True)
p.add_argument('--baseline',type=Path,default=Path('datasets/doors/lwx_front_20261023/base_01'))
a=p.parse_args()
if a.output.exists():p.error('Refusing to overwrite evidence')
m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text());m['_start_full_q']=next(r['obs']['q'] for r in rows if r['phase']=='pregrasp')
spec=json.loads(a.config.read_text())[a.variant]
r=plan(np.asarray(m['_start_full_q'][:7]),m,spec,42,m['controller_parameters']['handle_height_candidates_m'])
report={'scope':'Offline IK/collision lookahead only; actual 120Hz contact and physics still required','config':str(a.config),'config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),'pregrasp':r,'approach_samples':[],'approach_passed':False}
if r['success']:
    stage,kin,world=reconstruct(m,spec);world.allow_handle_contact=True
    planner=CuroboPlanner(world,kin,dt=m['dt'],seed=42,max_joint_speed=.5)
    handles=tuple(n.rsplit('/',1)[-1] for n,*_ in world.objects if '/fridge_door_handle' in n)
    q=np.asarray(r['positions'][-1]);target=np.asarray(r['target_pose']);handle=target[:3,3]+[0.,.13,0.]
    for index,alpha in enumerate(np.linspace(0,1,50)[1:]):
        target[:3,3]=handle+[0.,-.13*(1-alpha),0.];command=kin.ik(target,q,attempts=1)
        planner._prepare_path_query(handles,0.,'approach_handle')
        curobo_valid=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all())
        fcl_valid=bool(world.valid(command,mode='execution',gripper_q={n:m['_start_full_q'][m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}))
        report['approach_samples'].append({'index':index,'alpha':float(alpha),'q':command.tolist(),'curobo_valid':curobo_valid,'fcl_valid':fcl_valid,'fcl_detail':world.last_clearance})
        if not curobo_valid or not fcl_valid:break
        q=command
    report['approach_passed']=len(report['approach_samples'])==49 and all(x['curobo_valid'] and x['fcl_valid'] for x in report['approach_samples'])
report['pull_preview_upper_door_angle_rad']=.32
report['pull_preview_passed']=False
report['pull_preview_samples']=[]
report['scope']='Offline pregrasp/approach and projected quasi-static door-angle sweep only; no physical stepping, contact stability or task-success claim'
if report['approach_passed']:
    reference=next(row['obs'] for row in rows if row['phase']=='pull_door')
    fingers={n:reference['q'][m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
    world.gripper_joint_position=fingers['finger_joint']
    hinge=np.asarray(m['hinge']);axis=np.asarray(m['axis']);R0=target[:3,:3].copy()
    for angle in np.linspace(0.,.32,115):
        rr=Rotation.from_rotvec(axis*angle).as_matrix();delta=np.eye(4);delta[:3,:3]=rr;delta[:3,3]=hinge-rr@hinge
        world.move_body(DOOR,delta)
        for j,(name,_,lo,hi) in enumerate(world.objects):
            if not name.startswith(DOOR+'/'):continue
            rot,pos,rlo,rhi,mesh=world._object_reference[j]
            if mesh is not None:T=planner.T_B_W@delta@kin.base
            else:
                Tb=np.eye(4);Tb[:3,3]=(rlo+rhi)/2;T=planner.T_B_W@delta@Tb
            planner.motion_gen.world_coll_checker.update_obstacle_pose(name,Pose.from_list(planner._pose_list(T),planner.args))
        rr_target=Rotation.from_rotvec(axis*min(angle+.018,.4)).as_matrix()
        target[:3,:3]=rr_target@R0;target[:3,3]=hinge+rr_target@(handle-hinge)
        command=kin.ik(target,q,attempts=1);dq=command-q;command=q+dq/max(1.,float(np.max(np.abs(dq)))/.025)
        planner._prepare_path_query(handles,1.,'pull_door')
        cv=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all())
        fv=bool(world.valid(command,mode='execution',gripper_q=fingers))
        report['pull_preview_samples'].append({'door_angle_rad':float(angle),'q':command.tolist(),'curobo_valid':cv,'fcl_valid':fv,'fcl_detail':world.last_clearance})
        if not cv or not fv:break
        q=command
    report['pull_preview_passed']=len(report['pull_preview_samples'])==115 and all(x['curobo_valid'] and x['fcl_valid'] for x in report['pull_preview_samples'])
a.output.write_text(json.dumps(report,indent=2)+'\n')
print('CONTACT_PREVIEW',r['success'],r.get('selected_offset_m'),report['approach_passed'],report['pull_preview_passed'],flush=True)
raise SystemExit(0 if report['approach_passed'] and report['pull_preview_passed'] else 1)
