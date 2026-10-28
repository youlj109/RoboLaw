#!/usr/bin/env python3
"""CPU IK and full-source FCL sampling for additional physical opening range."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
import numpy as np
from pxr import Usd,UsdPhysics
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.hard_execution_v1.person import PersonAsset,load_spec
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.planning import DroidKinematics,ROBOT_USD


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed-episode',type=Path,default=ROOT/'datasets/avoidance_trials/doors/human_attempt_02')
    p.add_argument('--config',type=Path,default=ROOT/'configs/extensions/doors/hard_execution_v1/open_fridge_25deg.json')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--step-deg',type=float,default=.1)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    m=json.loads((a.seed_episode/'metadata.json').read_text());rs=json.loads((a.seed_episode/'trajectory.json').read_text())
    spec=load_spec(a.config)
    if not np.isclose(m['selected_handle_offset_z_m'],spec['handle_offset_z_m']):raise ValueError('Seed uses a different handle height')
    initial=min(rs,key=lambda r:abs(np.degrees(r['next_obs']['door_q'])-18.))['next_obs']
    joints=dict(zip(m['robot_joint_names'],initial['q']))
    q=np.array([joints[f'panda_joint{i}'] for i in range(1,8)])
    kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg'])
    stage=Usd.Stage.CreateInMemory();kitchen=ROOT/'assets/kitchen/scene.usd'
    stage.DefinePrim('/World/envs/env_0/kitchen').GetReferences().AddReference(str(kitchen))
    world=DoorCollisionWorld(stage,kin,env_root='/World/envs/env_0',default_clearance=.002)
    world.allow_handle_contact=True
    person=PersonAsset(spec);person._source_shapes(167);person.bind_world(world)
    fingers={k:joints[k] for k in world.GRIPPER_JOINT_NAMES}
    hinge=np.asarray(m['hinge']);axis=np.asarray(m['axis']);handle=np.asarray(m['selected_handle_initial'])
    door='/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door'
    joint=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(door+'/fridge_door_joint'))
    door_limits=[float(joint.GetLowerLimitAttr().Get()),float(joint.GetUpperLimitAttr().Get())]
    if not (door_limits[0]<=18.<=np.degrees(spec['hard_completion_door_rad'])<=np.degrees(spec['pull_target_cap_rad'])<=door_limits[1]):
        raise ValueError('Proposed physical opening falls outside source door limits')
    r0=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    rows=[];previous=q.copy();failure=None
    for degree in np.arange(18.,np.degrees(spec['pull_target_cap_rad'])+a.step_deg/2,a.step_deg):
        angle=np.radians(degree);r=Rotation.from_rotvec(axis*angle).as_matrix()
        t=np.eye(4);t[:3,:3]=r@r0;t[:3,3]=hinge+r@(handle-hinge)
        try:q=kin.ik(t,previous,attempts=1)
        except Exception as error:failure={'door_deg':float(degree),'kind':'IK','error':repr(error)};break
        delta=np.eye(4);delta[:3,:3]=r;delta[:3,3]=hinge-r@hinge
        world.move_body(door,delta)
        valid=world.valid(q,mode='execution',gripper_q=fingers)
        gap=np.minimum(q-kin.limits[:,0],kin.limits[:,1]-q)
        actual=kin.fk(q)
        row={'door_deg':float(degree),'q':q.tolist(),'source_joint_limit_margin_rad':gap.tolist(),
             'max_joint_increment_rad':float(np.max(np.abs(q-previous))),
             'tcp_position_residual_m':float(np.linalg.norm(actual[:3,3]-t[:3,3])),
             'tcp_rotation_residual_rad':float(Rotation.from_matrix(t[:3,:3]@actual[:3,:3].T).magnitude()),
             'world_fcl_valid':bool(valid),'world_fcl_failure':world.last_clearance}
        rows.append(row)
        if not valid or np.any(gap<=0):failure={'door_deg':float(degree),'kind':'FCL_or_limit','details':row};break
        previous=q.copy()
    dependencies=[Path(__file__),a.config,kitchen,ROBOT_USD,ROOT/spec['human_asset'],
        ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/collision_planner.py',ROOT/'kitchen_safety/extensions/doors/collision.py']
    report={'scope':'CPU sampled kinematic/FCL proposal only; no simulator, cuRobo self-collision or physical grasp feasibility acceptance.',
        'passed_sampled_proposal':failure is None,'failure':failure,'human_frame':167,
        'handle_offset_z_m':spec['handle_offset_z_m'],'start_door_deg':18.,'end_door_deg':float(np.degrees(spec['pull_target_cap_rad'])),
        'sample_spacing_door_deg':a.step_deg,'original_native_open_threshold_deg':18.,
        'proposed_additional_measured_completion_deg':float(np.degrees(spec['hard_completion_door_rad'])),
        'source_joint_limits_rad':kin.limits.tolist(),'robot_base':m['robot_base'],'robot_yaw_deg':m['robot_yaw_deg'],
        'source_door_joint_limits_deg':door_limits,
        'seed_episode':str(a.seed_episode),'seed_door_rad':initial['door_q'],'gripper_q':fingers,
        'input_sha256':{name:hashlib.sha256((a.seed_episode/name).read_bytes()).hexdigest() for name in ('metadata.json','trajectory.json')},
        'source_dependency_sha256':{str(path.resolve().relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in dependencies},
        'limitations':['Finite angle sampling does not prove exact continuous swept clearance.',
                       'Robot self-collision remains checked by the native cuRobo execution path and independent review.',
                       'Finger angles are held at the measured prior physical grasp; actual contact loading must be measured in the new run.'],
        'samples':rows}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:report[k] for k in ('passed_sampled_proposal','failure','start_door_deg','end_door_deg')}))
    print('CPU_SAMPLES',len(rows))


if __name__=='__main__':main()
