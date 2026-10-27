"""Bounded pre-contact endpoint selection; never generates a trajectory."""
import time

import numpy as np
from scipy.spatial.transform import Rotation


def single_policy_parameters(target, original_goal, phase, policy):
    """Describe one modification without solving or certifying anything."""
    if phase not in ('prepress', 'pregrasp') or set(policy) != {'change', 'value'}:
        raise ValueError('Expected one pre-contact change/value policy')
    pose = np.array(target, dtype=float, copy=True)
    seed = np.array(original_goal, dtype=float, copy=True)
    kind, value = policy['change'], policy['value']
    metadata = dict(strategy='single_'+kind, actual_contact_target_changed=False,
                    axial_roll_delta_deg=0., extra_standoff_m=0.)
    if kind == 'standoff':
        if not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 <= value <= .15:
            raise ValueError('Standoff change must be between 0 and 0.15 m')
        pose[1 if phase == 'prepress' else 0, 3] -= value
        metadata['extra_standoff_m'] = value
    elif kind == 'roll':
        if not isinstance(value, (int, float)) or not np.isfinite(value) or abs(value) > 90:
            raise ValueError('Axial roll change must be within +/-90 degrees')
        pose[:3, :3] = pose[:3, :3] @ Rotation.from_euler('x', value, degrees=True).as_matrix()
        metadata['axial_roll_delta_deg'] = value
    elif kind == 'ik_seed':
        seed = np.asarray(value, dtype=float)
        if seed.shape != (7,) or not np.isfinite(seed).all():
            raise ValueError('IK seed must contain seven finite joint values')
    elif kind == 'pregrasp_tilt_y':
        if phase != 'pregrasp' or not isinstance(value,(int,float)) or not np.isfinite(value) or abs(value)>60:
            raise ValueError('Free-space pregrasp tilt only, within +/-60 degrees')
        pose[:3,:3] = Rotation.from_euler('y',value,degrees=True).as_matrix() @ pose[:3,:3]
        metadata.update(world_y_tilt_deg=value, requires_alignment_before_contact=True)
    elif kind == 'pregrasp_yaw_z':
        if phase != 'pregrasp' or not isinstance(value, (int, float)) or not np.isfinite(value) or abs(value) > 60:
            raise ValueError('Free-space pregrasp yaw only, within +/-60 degrees')
        pose[:3, :3] = Rotation.from_euler('z', value, degrees=True).as_matrix() @ pose[:3, :3]
        metadata.update(world_z_yaw_deg=value, requires_alignment_before_contact=True)
    elif kind == 'pregrasp_height':
        if phase != 'pregrasp' or not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 < value <= .06:
            raise ValueError('Auxiliary pregrasp height change must be within (0, 0.06] m')
        pose[2, 3] += value
        metadata.update(auxiliary_pregrasp_height_m=value, approach_endpoint_changed=True,
                        requires_alignment_before_contact=True)
    elif kind == 'arrival_tcp_position':
        point=np.asarray(value,dtype=float)
        if phase!='pregrasp' or point.shape!=(3,) or not np.isfinite(point).all():
            raise ValueError('A nominal grasp-arrival endpoint requires three finite coordinates')
        pose[:3,3]=point
        metadata.update(target_kind='actual_grasp_arrival', approach_endpoint_changed=True)
    else:
        raise ValueError('Unknown single-change policy')
    return pose, seed, metadata


def single_policy_endpoint(kin, target, original_goal, phase, policy):
    """Apply exactly one declared change and solve IK once; no search loop."""
    pose, seed, metadata = single_policy_parameters(target, original_goal, phase, policy)
    goal = kin.ik(pose, seed, attempts=1)
    return dict(goal_q=goal, requested_tcp=pose, policy=metadata, candidate_index=0)


def pose_variants(target, phase):
    """Keep contact targets fixed; vary only pre-contact frame freedoms."""
    if phase not in ('prepress', 'pregrasp'):
        raise ValueError('Endpoint selection is restricted to pre-contact phases')
    for distance in (0., .03, .06):
        rolls = (0., -15., 15., -30., 30., -60., 60.) if distance == 0 else (0., -30., 30.)
        for roll in rolls:
            pose = np.array(target, dtype=float, copy=True)
            pose[:3, :3] = pose[:3, :3] @ Rotation.from_euler('x', roll, degrees=True).as_matrix()
            pose[1 if phase == 'prepress' else 0, 3] -= distance
            yield pose, dict(strategy=('redundant_ik_only' if roll == distance == 0 else
                                     'axial_roll' if distance == 0 else 'precontact_standoff'),
                             axial_roll_delta_deg=roll, extra_standoff_m=distance,
                             actual_contact_target_changed=False)


def select_endpoint(kin, planner, start, original_goal, target, phase,
                    seed=42, max_candidates=64, budget_s=25.):
    started = time.monotonic()
    rng = np.random.default_rng(seed)
    original_goal = np.asarray(original_goal, dtype=float)
    start = np.asarray(start, dtype=float)
    limits = kin.limits
    seeds = [original_goal, start]
    seeds += [np.clip(original_goal + rng.normal(0, .8, 7), limits[:, 0]+.01, limits[:, 1]-.01)
              for _ in range(6)]
    seeds += [rng.uniform(limits[:, 0]+.01, limits[:, 1]-.01) for _ in range(8)]
    records, seen = [], []
    selected = None
    for pose_index, (pose, policy) in enumerate(pose_variants(target, phase)):
        trial_seeds = seeds if pose_index == 0 else [original_goal, start, seeds[8], seeds[9]]
        for seed_index, ik_seed in enumerate(trial_seeds):
            if len(records) >= max_candidates or time.monotonic()-started >= budget_s:
                return selected, dict(candidates=records, budget_s=budget_s,
                                      budget_exhausted=True, seconds=time.monotonic()-started)
            entry = dict(index=len(records), pose_index=pose_index, ik_seed_index=seed_index,
                         policy=policy, requested_tcp=pose.tolist())
            records.append(entry)
            try:
                goal = kin.ik(pose, ik_seed, attempts=1)
                entry['goal_q'] = goal.tolist()
                if any(np.max(np.abs(goal-old)) < .005 for old in seen):
                    entry['status'] = 'duplicate_ik_solution'
                    continue
                seen.append(goal)
                valid = planner.motion_gen.graph_planner.mask_samples(
                    planner.args.to_device(goal[None])).detach().cpu().numpy().reshape(-1)
                entry['graph_endpoint_valid'] = bool(valid[0])
                entry['status'] = 'eligible' if valid[0] else 'endpoint_rejected'
                if valid[0]:
                    selected = dict(goal_q=goal, requested_tcp=pose, policy=policy,
                                    candidate_index=entry['index'])
                    return selected, dict(candidates=records, budget_s=budget_s,
                                          budget_exhausted=False, seconds=time.monotonic()-started)
            except RuntimeError as exc:
                entry.update(status='ik_failed', error=str(exc))
    return selected, dict(candidates=records, budget_s=budget_s,
                          budget_exhausted=False, seconds=time.monotonic()-started)
