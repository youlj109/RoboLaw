"""Task-local observation helpers. No target commands, state writes or patches."""
import numpy as np


def openness(q, lower, upper):
    """Pinned Arena rule, including its negative-lower-limit convention."""
    if not upper > lower:
        raise ValueError('Joint must have a nonzero range')
    value = (q - lower) / (upper - lower)
    return 1. - value if lower < 0. else value


def predicate_progress(values, lower, upper, reset=0., threshold=.2):
    away_seen = False
    first_success = None
    for index, q in enumerate(values):
        progress = openness(q, lower, upper)
        if away_seen and progress > threshold and first_success is None:
            first_success = index
        away_seen = away_seen or abs(progress - reset) > .05
    return first_success


def contact_sample(view, dt, convert, initial_door_transform,
                   current_door_transform, handle_bounds, padding):
    forces, points, normals, separations, counts, starts = map(
        convert, view.get_contact_data(dt=dt))
    groups = [np.arange(int(start), int(start + count), dtype=int)
              for count, start in zip(counts.reshape(-1), starts.reshape(-1))]
    indices = np.concatenate(groups).astype(int)
    selected_points = points[indices].reshape(-1, 3)
    inverse_delta = initial_door_transform @ np.linalg.inv(current_door_transform)
    original_points = selected_points @ inverse_delta[:3, :3].T + inverse_delta[:3, 3]
    low, high = np.asarray(handle_bounds)
    on_handle = np.all((original_points >= low - padding) &
                       (original_points <= high + padding), axis=1)
    return {'count': len(indices), 'counts': counts.tolist(),
            'normal_force_sum_n': float(forces[indices].sum()),
            'normal_force_abs_sum_n': float(np.abs(forces[indices]).sum()),
            'per_filter_abs_normal_force_n': [float(np.abs(forces[g]).sum()) for g in groups],
            'per_finger_abs_normal_force_n': np.asarray([float(np.abs(forces[g]).sum()) for g in groups]).reshape(counts.shape).sum(axis=1).tolist(),
            'points_world_m': selected_points.tolist(),
            'normal_forces_n': forces[indices].reshape(-1).tolist(),
            'normals_world': normals[indices].reshape(-1, 3).tolist(),
            'separations_m': separations[indices].reshape(-1).tolist(),
            'points_on_source_handle_bounds': on_handle.tolist()}


def sampling_errors(samples, metadata):
    errors = []
    if len(samples) < 2:
        return ['Insufficient physics samples']
    steps = np.asarray([s['physics_step'] for s in samples])
    times = np.asarray([s['time_s'] for s in samples])
    if not np.all(np.diff(steps) == 1):
        errors.append('Missing or duplicated physics steps')
    if not np.allclose(np.diff(times), metadata['physics_dt'], atol=1e-10, rtol=0):
        errors.append('Physics timestamps are not synchronous')
    expected = {e['path']: len(e['joint_names']) for e in metadata['mechanism_inventory']}
    for s in samples:
        if set(s['mechanisms']) != set(expected):
            errors.append('Missing or extra observed articulation'); break
        for path, width in expected.items():
            for key in ('q', 'dq'):
                values = np.asarray(s['mechanisms'][path][key])
                if values.shape != (width,) or not np.isfinite(values).all():
                    errors.append(f'Invalid {path} {key}'); break
    return sorted(set(errors))


def mechanism_quality(samples, metadata, limits, *, neutral):
    errors = sampling_errors(samples, metadata)
    joints = []
    if errors:
        return {'passed': False, 'errors': errors, 'joints': joints}
    for entry in metadata['mechanism_inventory']:
        path = entry['path']
        positions = np.asarray([s['mechanisms'][path]['q'] for s in samples])
        velocities = np.asarray([s['mechanisms'][path]['dq'] for s in samples])
        for j, (name, kind) in enumerate(zip(entry['joint_names'], entry['joint_kinds'])):
            target = path == metadata['door_geometry']['object_path'] and name == metadata['door_geometry']['joint_name']
            if target and not neutral:
                continue
            if kind not in ('revolute', 'prismatic'):
                errors.append(f'Unknown joint units: {path}/{name}'); continue
            drift_key = 'neutral_revolute_drift_rad' if kind == 'revolute' else 'neutral_prismatic_drift_m'
            speed_key = 'neutral_revolute_speed_rad_s' if kind == 'revolute' else 'neutral_prismatic_speed_m_s'
            drift = float(np.abs(positions[:, j] - positions[0, j]).max())
            speed = float(np.abs(velocities[:, j]).max())
            passed = drift <= limits[drift_key] and speed <= limits[speed_key]
            joints.append({'path': path, 'name': name, 'kind': kind, 'target': target,
                           'max_drift_from_initial': drift, 'max_abs_speed': speed,
                           'drift_limit': limits[drift_key], 'speed_limit': limits[speed_key],
                           'passed': passed})
            if not passed:
                errors.append(f'Mechanism moved without an allowed operation: {path}/{name}')
    return {'passed': not errors, 'errors': errors, 'joints': joints,
            'physics_samples': len(samples),
            'duration_s': samples[-1]['time_s'] - samples[0]['time_s'],
            'scope': 'All observed active kitchen DOFs; includes startup without discarding settling.'}


def contact_causality(samples, metadata, limits):
    errors = sampling_errors(samples, metadata)
    if errors:
        return {'passed': False, 'errors': errors}
    q = np.asarray([s['state']['door_q'] for s in samples])
    force_floor = limits['contact_force_noise_n']
    def loaded_points_localized(s):
        contact = s['contacts']
        active = np.abs(contact['normal_forces_n']) > force_floor
        return bool(active.any() and np.asarray(contact['points_on_source_handle_bounds'])[active].all())
    loaded = [s['contacts']['normal_force_abs_sum_n'] > force_floor and
              s['contacts']['count'] > 0 and loaded_points_localized(s) for s in samples]
    motion = np.flatnonzero(np.abs(q - q[0]) > limits['door_motion_rad'])
    contact = np.flatnonzero(loaded)
    first_motion = int(motion[0]) if len(motion) else None
    first_contact = int(contact[0]) if len(contact) else None
    if first_motion is None or first_contact is None:
        order = 'missing_motion_or_loaded_contact'
    elif first_contact < first_motion:
        order = 'loaded_contact_precedes_resolved_motion'
    elif first_contact == first_motion:
        order = 'unresolved_within_same_physics_step'
    else:
        order = 'motion_precedes_loaded_contact'
    if order != 'loaded_contact_precedes_resolved_motion':
        errors.append(order)
    pull = [i for i, s in enumerate(samples) if s['phase'] in ('pull_door', 'recover_open')]
    lost = [i for i in pull if min(np.asarray(samples[i]['contacts']['counts']).sum(axis=1)) < 1]
    unloaded = [i for i in pull if samples[i]['contacts']['normal_force_abs_sum_n'] <= force_floor]
    def bilateral_load(row):
        c=row['contacts'];v=np.asarray(c['per_finger_abs_normal_force_n'])
        return v if len(v)==2 else v.reshape(np.asarray(c['counts']).shape).sum(axis=1)
    one_finger_unloaded = [i for i in pull if min(bilateral_load(samples[i])) <= force_floor]
    if not pull or lost or unloaded:
        errors.append('Missing bilateral contact or all normal load during pulling')
    outside = [i for i, s in enumerate(samples)
               if s['contacts']['normal_force_abs_sum_n'] > force_floor and
               not loaded_points_localized(s)]
    if outside:
        errors.append('Loaded contact outside moving source handle bounds')
    lower, upper = metadata['door_joint_limits_rad']
    threshold=metadata['door_geometry']['openness_threshold']
    reset=metadata['door_geometry']['reset_openness']
    success_index = predicate_progress(q, lower, upper,reset=reset,threshold=threshold)
    if success_index is None or openness(q[-1], lower, upper) <= threshold:
        errors.append('Original strict predicate sequence/final openness not satisfied')
    released = samples[-1]['phase'] == 'released_hold'
    release_force = float(samples[-1]['contacts']['normal_force_abs_sum_n'])
    if not released or release_force > force_floor:
        errors.append('No unloaded released terminal observation')
    return {'passed': not errors, 'errors': errors, 'sampling_dt_s': metadata['physics_dt'],
            'order': order, 'first_loaded_contact_sample': first_contact,
            'first_resolved_motion_sample': first_motion,
            'original_success_sample': success_index, 'pull_physics_samples': len(pull),
            'lost_bilateral_contact_samples': lost, 'unloaded_pull_samples': unloaded,
            'single_finger_zero_normal_load_samples': one_finger_unloaded,
            'load_interpretation': 'Bilateral contact counts must persist; at least one finger must carry measured normal load per pulling substep. A zero impulse on one finger with persistent contact is reported, not equated with losing that contact. Tangential friction is not measured.',
            'contacts_outside_handle_bounds': outside, 'released_terminal': released,
            'terminal_contact_force_abs_n': release_force,
            'temporal_limit': 'Ordering within one physics substep is unresolved; observed precedence is not an assertion about unsampled continuous-time dynamics.'}
