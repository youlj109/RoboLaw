"""Bounded positive pull progress with unchanged dense GPU/FCL gates."""
import numpy as np
POLICY = dict(version='pull_lead_backoff_v1', lead_scales=[1., .6, .4, .2],
              max_joint_step_rad=.025, resolution_rad=.005, min_dense_samples=9,
              geometry='All dense samples must pass original GPU constraints and original FCL execution margins.',
              scope='Same fixed legal handle; reduce attempted door lead only. No state writes or added route waypoints.')

def select_pull_command(kin, q, target_for_lead, nominal_lead, gpu_valid, fcl_valid):
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(nominal_lead) or nominal_lead <= 0:
        raise ValueError('Finite seven-joint state and positive pull lead required')
    attempts = []
    for scale in POLICY['lead_scales']:
        lead = float(nominal_lead * scale)
        target = np.asarray(target_for_lead(lead), dtype=float)
        row = dict(lead_rad=lead, target_pose=target.tolist(), accepted=False)
        attempts.append(row)
        try:
            goal = np.asarray(kin.ik(target, q, attempts=1), dtype=float)
        except RuntimeError as exc:
            row['ik_failure'] = str(exc)
            continue
        if goal.shape != q.shape or not np.isfinite(goal).all():
            raise ValueError('IK returned malformed joints')
        dq = goal - q
        command = q + dq / max(1., float(np.max(np.abs(dq))) / POLICY['max_joint_step_rad'])
        row['command_q'] = command.tolist()
        if np.any(command < kin.limits[:, 0]) or np.any(command > kin.limits[:, 1]):
            row['failure'] = 'command_joint_limits'
            continue
        count = max(POLICY['min_dense_samples'], int(np.ceil(np.max(np.abs(command-q))/POLICY['resolution_rad']))+1)
        dense = q + np.linspace(0., 1., count)[:, None] * (command-q)
        valid = np.asarray(gpu_valid(dense), dtype=bool).reshape(-1)
        if len(valid) != count:
            raise RuntimeError('GPU validity count differs from dense path length')
        row['dense_samples'] = count
        row['gpu_failed_samples'] = np.flatnonzero(~valid).tolist()
        if not valid.all():
            continue
        row['fcl_checked_samples'] = 0
        for i, sample in enumerate(dense):
            row['fcl_checked_samples'] += 1
            if not fcl_valid(sample):
                row['fcl_failed_sample'] = i
                break
        else:
            row['accepted'] = True
            return command, dict(policy=POLICY.copy(), attempts=attempts, selected_lead_rad=lead)
    return None, dict(policy=POLICY.copy(), attempts=attempts, selected_lead_rad=None)
