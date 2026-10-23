"""Controls-local fixed endpoint policy for reproducible avoidance pairs."""
import numpy as np


def button_prepress_pose(config, roll_deg=0., tcp_x_offset=0., tcp_z_offset=-.006):
    from scipy.spatial.transform import Rotation
    from .task_contract import task_contract
    left_id=task_contract('left')['task_id']
    if config['task_id']!=left_id and 'button_center_world_m' not in config:
        raise ValueError('Non-left target requires an inspected center')
    center=config.get('button_center_world_m',[2.08087763,-.38513822422,1.03313965])
    pose=np.eye(4)
    pose[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()@Rotation.from_euler('x',roll_deg,degrees=True).as_matrix()
    pose[:3,3]=[center[0]+tcp_x_offset,-.44,center[2]+tcp_z_offset]
    return pose


def plan_prepress(planner, q, target, policy=None):
    return plan_control_pose(planner,q,target,policy,grip_binary=1,phase='prepress')


def plan_control_pose(planner,q,target,policy=None,*,grip_binary=0,phase='pregrasp'):
    policy = policy or {'method': 'curobo_pose'}
    if policy['method'] == 'curobo_pose':
        return planner.plan_pose(q, target, grip_binary=grip_binary, phase=phase)
    if policy['method'] != 'curobo_joint_goal':
        raise ValueError('Unsupported prepress policy: ' + policy['method'])
    # The redundant IK branch is fixed before switching hazards on/off.
    # Re-solve the original target; never add hazard-dependent waypoints.
    goal = planner.kin.ik(target, np.asarray(policy['ik_branch_seed']), attempts=1)
    plan = planner.plan_joint_goal(q, goal, grip_binary=grip_binary, phase=phase)
    plan['metadata']['prepress_policy'] = policy
    plan['metadata']['requested_tcp_pose'] = target.tolist()
    return plan
