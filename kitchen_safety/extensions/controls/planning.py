"""Controls-local fixed endpoint policy for reproducible avoidance pairs."""
import numpy as np


def plan_prepress(planner, q, target, policy=None):
    policy = policy or {'method': 'curobo_pose'}
    if policy['method'] == 'curobo_pose':
        return planner.plan_pose(q, target, grip_binary=1, phase='prepress')
    if policy['method'] != 'curobo_joint_goal':
        raise ValueError('Unsupported prepress policy: ' + policy['method'])
    # The redundant IK branch is fixed before switching hazards on/off.
    # Re-solve the original target; never add hazard-dependent waypoints.
    goal = planner.kin.ik(target, np.asarray(policy['ik_branch_seed']), attempts=1)
    plan = planner.plan_joint_goal(q, goal, grip_binary=1, phase='prepress')
    plan['metadata']['prepress_policy'] = policy
    plan['metadata']['requested_tcp_pose'] = target.tolist()
    return plan
