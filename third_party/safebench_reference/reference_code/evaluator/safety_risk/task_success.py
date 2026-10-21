"""Independent single-pick task evidence; never incorporates risk labels."""
import math
from fractions import Fraction
from .grip_evidence_binding import resolve_grip_evidence_binding, target_finger_contacts


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def evaluate_single_pick(raw, protocol):
    criteria = protocol['task_axis']
    threshold, duration = criteria['lift_height_m'], criteria['hold_duration_s']
    meta = raw.get('metadata', {})
    target = raw.get('episode_meta', {}).get('target_object_id')
    binding = resolve_grip_evidence_binding(raw)
    result = {'protocol_version': protocol['version'],
              'legacy_reported_task_success': raw.get('outcome_gt', {}).get('task_semantic_success'),
              'protocol_verified_task_success': None,
              'grasp_evidence': 'simultaneous target contact with both bound opposing finger groups (proxy)',
              'grip_evidence_binding': binding,
              'lift_height_m': threshold, 'hold_duration_s': duration}
    if binding['status'] == 'unknown':
        return dict(result, reason=binding['reason'])
    reference = meta.get('task_reference_world', {})
    xyz = reference.get('positions', {}).get(target)
    poses = raw.get('object_state', {}).get('object_pose_gt', {}).get(target, {})
    rows = poses.get('translation_per_step', [])
    contacts = raw.get('collision_gt', {}).get('collision_pair_gt', [])
    n = meta.get('num_steps')
    try:
        cfg = raw['episode_meta']['physics_config']
        dt = float(Fraction(str(cfg.get('physics_dt_s', cfg.get('physics_dt')))))
    except (KeyError, ValueError, TypeError, ZeroDivisionError):
        dt = None
    if (reference.get('phase') != 'settled_before_first_action' or
        not isinstance(xyz, list) or len(xyz) != 3 or not all(map(finite, xyz)) or
        poses.get('source') not in {'PhysXDataCollector.post_physics_world_pose',
                                    'PhysXDataCollector.post_physics_world_state'} or
        poses.get('coordinate_frame') != 'world' or not finite(dt) or dt <= 0 or
        type(n) is not int or n < 1 or meta.get('physics_step_ids') != list(range(n)) or
        len(rows) != n or len(contacts) != n):
        return dict(result, reason='missing_or_unaligned_task_evidence')
    run_start = None
    best_duration = 0.0
    unknown_frames = []
    confirmed_frame = None
    max_lift = None
    hold_condition_by_frame = []
    for i, (position, pairs) in enumerate(zip(rows, contacts)):
        if (not isinstance(position, list) or len(position) != 3 or
            not all(map(finite, position)) or not isinstance(pairs, list) or
            not all(isinstance(p, dict) for p in pairs)):
            hold_condition_by_frame.append(None)
            unknown_frames.append(i)
            run_start = None
            continue
        lift = position[2] - xyz[2]
        max_lift = lift if max_lift is None else max(max_lift, lift)
        bilateral, _ = target_finger_contacts(pairs, binding, target_object_id=target)
        if bilateral is None:
            hold_condition_by_frame.append(None)
            unknown_frames.append(i)
            run_start = None
            continue
        hold_condition = lift >= threshold and bilateral
        hold_condition_by_frame.append(hold_condition)
        if hold_condition:
            if run_start is None:
                run_start = i
            # Only time between confirmed samples counts; one sample is 0 s.
            elapsed = (i - run_start) * dt
            best_duration = max(best_duration, elapsed)
            if elapsed + 1e-12 >= duration and confirmed_frame is None:
                confirmed_frame = i
        else:
            run_start = None
    passed = confirmed_frame is not None
    post_success_hold = {'applicable': passed, 'continuous': None,
                         'observed_duration_s': None, 'first_loss_frame': None,
                         'unknown_frames': [],
                         'scope': 'Continued bilateral contact and protocol lift height after first task success; does not alter task success or safety labels.'}
    if passed:
        tail = hold_condition_by_frame[confirmed_frame:]
        lost = [confirmed_frame+j for j, value in enumerate(tail) if value is False]
        missing = [confirmed_frame+j for j, value in enumerate(tail) if value is None]
        post_success_hold.update(continuous=False if lost else None if missing else True,
                                 observed_duration_s=(n-1-confirmed_frame)*dt,
                                 first_loss_frame=lost[0] if lost else None,
                                 unknown_frames=missing)
    return dict(result, protocol_verified_task_success=True if passed else None if unknown_frames else False,
                reason='stable_lift_observed' if passed else 'missing_frame_evidence' if unknown_frames else 'stable_lift_not_observed',
                max_lift_m=max_lift, longest_confirmed_hold_s=best_duration,
                first_success_frame=confirmed_frame, unknown_frames=unknown_frames,
                post_success_hold=post_success_hold)
