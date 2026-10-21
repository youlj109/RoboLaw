"""Replan a failed guide suffix against the current planner and actual goal.

Nothing here executes commands or changes collision margins. A valid guide
prefix is measured again: failed cuRobo validation may have skipped all FCL
samples, and a reported FCL failure need not be the first failure along a path.
"""
from copy import deepcopy

import numpy as np

from kitchen_safety.planning_checks import check_planned_path


def _path(values):
    result = np.asarray(values, dtype=float)
    if result.ndim != 2 or result.shape[1] != 7 or not len(result) or not np.isfinite(result).all():
        raise ValueError('Expected a finite nonempty (N, 7) joint path')
    return result.copy()


def _prefix_knots(geometry, path, cut):
    """Recover original connected corners, not another timing of every frame.

    retime_path_smooth retains its original edge endpoints exactly. Match them
    in order and append the chosen interior cut. Its skipped zero-length edges
    use the same 1e-12 numerical threshold. The whole result is revalidated.
    """
    result = [path[0].copy()]
    indices = [0]
    source_indices = [0]
    cursor = 0
    for source_index, point in enumerate(geometry[1:], 1):
        if np.max(np.abs(point - result[-1])) < 1e-12:
            continue
        matches = np.flatnonzero(np.all(path[cursor:cut+1] == point, axis=1))
        if not len(matches):
            break
        cursor += int(matches[0])
        result.append(point.copy())
        indices.append(cursor)
        source_indices.append(source_index)
    if not np.array_equal(result[-1], path[cut]):
        result.append(path[cut].copy())
    return np.asarray(result), indices, source_indices


def replan_guided_suffix(planner, world, path, connected, failed_check, goal,
                         ignore, grip, *, dt, apply_tilt_margin, guide_metadata,
                         report, on_progress=None):
    """Try at most two prefix cuts; return a fully checked trajectory or None.

    Callers restrict this fallback to supported human placement. Existing
    planner padding, grip, tilt cap, retiming and independent FCL predicates
    remain authoritative. Guide data selects initialization only.
    """
    path, geometry = _path(path), _path(connected)
    goal = _path(np.asarray(goal)[None])[0]
    report.update(method='guided_prefix_curobo_bridge', guide=deepcopy(guide_metadata),
                  source_samples=len(path), source_geometry_knots=len(geometry),
                  source_failed_validation=deepcopy(failed_check),
                  source_failed_positions=path.tolist(), source_geometry_positions=geometry.tolist(),
                  goal_q_arm=goal.tolist(), attempts=[], success=False,
                  maximum_prefix_attempts=2)

    def save():
        if on_progress is not None:
            on_progress()

    def validate(value):
        return planner.validate_path(value, ignore, grip, phase='above_bowl')

    if failed_check.get('valid') or not np.array_equal(path[-1], goal):
        report['error'] = 'Expected a failed guide ending at the exact actual candidate goal'
        save()
        return None
    failures = [value for key in ('curobo_failures', 'fcl_failures')
                for value in failed_check.get(key, []) if 'edge' in value]
    if not failures:
        report['error'] = 'Failed guide has no edge-localized validation diagnostic'
        save()
        return None
    failure = min(failures, key=lambda value: (int(value['edge']), float(value.get('fraction', 0.))))
    edge = int(failure['edge'])
    if not 0 <= edge < max(1, len(path)-1):
        report['error'] = 'Validation failure edge is outside the supplied guide path'
        save()
        return None
    first_cut = max(0, edge - (float(failure.get('fraction', 0.)) <= 0.))
    _, source_path_indices, _ = _prefix_knots(geometry, path, first_cut)
    earlier = [index for index in source_path_indices if index < first_cut]
    cuts = list(dict.fromkeys((first_cut, earlier[-1] if earlier else max(0, first_cut-1))))
    report.update(first_failure=deepcopy(failure), prefix_cut_indices=cuts)
    try:
        report['goal_validation'] = validate(goal[None])
        if not report['goal_validation']['valid']:
            report['error'] = 'Actual candidate goal failed current transport constraints'
            save()
            return None
    except RuntimeError as error:
        report['error'] = str(error)
        save()
        return None
    save()
    for cut in cuts:
        attempt = dict(prefix_last_sample=cut, prefix_samples=cut+1,
                       bridge_start_q_arm=path[cut].tolist(), success=False)
        report['attempts'].append(attempt)
        try:
            attempt['prefix_validation'] = validate(path[:cut+1])
            if not attempt['prefix_validation']['valid']:
                raise RuntimeError('Guide prefix failed current cuRobo/FCL validation')
            bridge = planner.plan_joint_goal(path[cut], goal, ignore, grip, phase='above_bowl')
            attempt['bridge_planning'] = deepcopy(bridge.get('metadata', {}))
            bridge_path = _path(bridge.get('raw_positions', bridge['positions']))
            start_error = float(np.max(np.abs(bridge_path[0] - path[cut])))
            goal_error = float(np.max(np.abs(bridge_path[-1] - goal)))
            attempt.update(bridge_start_error_rad=start_error, bridge_goal_error_rad=goal_error)
            # Same endpoint tolerance enforced by CuroboPlanner.plan_joint_goal.
            if max(start_error, goal_error) > 1e-4:
                raise RuntimeError('Bridge planner changed its requested start or goal')
            bridge_path[0], bridge_path[-1] = path[cut], goal
            prefix, _, retained_source_indices = _prefix_knots(geometry, path, cut)
            joined = np.vstack([prefix, bridge_path[1:]])
            attempt['prefix_geometry_knots'] = len(prefix)
            attempt['retained_source_geometry_indices'] = retained_source_indices
            attempt['joined_geometry_validation'] = validate(joined)
            if not attempt['joined_geometry_validation']['valid']:
                raise RuntimeError('Joined prefix/bridge geometry failed validation')
            attempt['joined_independent_fcl_valid'] = bool(check_planned_path(world, joined, ignore))
            if not attempt['joined_independent_fcl_valid']:
                raise RuntimeError(f'Joined prefix/bridge failed independent FCL: {world.last_collision}')
            timed = planner.retime_path_smooth(joined)
            trajectory = dict(positions=timed, metadata=dict(
                method='guided_prefix_curobo_bridge', samples=len(timed), dt=dt,
                guide=deepcopy(guide_metadata), prefix_last_source_sample=cut,
                prefix_geometry_knots=len(prefix), bridge_planning=deepcopy(bridge.get('metadata', {})),
                max_joint_speed_rad_s=.2, max_joint_acceleration_rad_s2=.4,
                timing='quintic_rest_to_rest'))
            trajectory = apply_tilt_margin(trajectory)
            final = _path(trajectory['positions'])
            attempt.update(start_exactly_unchanged=np.array_equal(final[0], path[0]),
                           end_exactly_unchanged=np.array_equal(final[-1], goal))
            if not attempt['start_exactly_unchanged'] or not attempt['end_exactly_unchanged']:
                raise RuntimeError('Retiming or tilt correction changed the actual transport endpoints')
            attempt['final_validation'] = validate(final)
            if not attempt['final_validation']['valid']:
                raise RuntimeError('Timed/corrected prefix/bridge failed complete validation')
            attempt['final_independent_fcl_valid'] = bool(check_planned_path(world, final, ignore))
            if not attempt['final_independent_fcl_valid']:
                raise RuntimeError(f'Timed/corrected bridge failed independent FCL: {world.last_collision}')
            attempt['success'] = report['success'] = True
            report['selected_prefix_last_sample'] = cut
            report['retained_source_geometry_indices'] = retained_source_indices
            trajectory['metadata']['guided_suffix_replanning'] = deepcopy(report)
            save()
            return trajectory
        except RuntimeError as error:
            attempt['error'] = str(error)
            save()
    return None


def resolve_guided_transport_phases(trajectory, kin, attachment):
    """Map explicit source phases only while their original prefix survives."""
    from kitchen_safety.transport_phases import map_guide_phase_boundaries

    metadata = trajectory['metadata']
    guide = metadata.get('guide', {})
    mapping = map_guide_phase_boundaries(trajectory['positions'], kin, attachment, guide)
    suffix = metadata.get('guided_suffix_replanning')
    if mapping['applied'] and suffix is not None:
        source = map_guide_phase_boundaries(
            suffix.get('source_failed_positions', []), kin, attachment, guide)
        geometry = map_guide_phase_boundaries(
            suffix.get('source_geometry_positions', []), kin, attachment, guide)
        mapping['suffix_source_path_mapping'] = source
        mapping['suffix_source_geometry_mapping'] = geometry
        cut = suffix.get('selected_prefix_last_sample', -1)
        retained = suffix.get('retained_source_geometry_indices', [])
        preserved = (source['applied'] and geometry['applied']
                     and all(item['path_index'] <= cut for item in source['boundaries'])
                     and all(item['path_index'] in retained for item in geometry['boundaries']))
        mapping['source_prefix_preserved'] = bool(preserved)
        if not preserved:
            mapping.update(applied=False, boundaries=[], reason='source_phase_anchor_not_in_retained_prefix')
    if not mapping['applied'] and guide.get('phase_boundaries'):
        # Preserve source evidence, while removing an inactive execution tag.
        mapping['inactive_source_phase_boundaries'] = deepcopy(guide.pop('phase_boundaries'))
    metadata['transport_phase_mapping'] = mapping
    return mapping


def transport_command_segments(mapping, path_samples, handoff_samples):
    """An arrival command belongs to the preceding phase, without duplication.

    The handoff reaches path[0]; subsequent commands are path[1:]. Segment
    command ranges are half-open; path endpoint ranges are inclusive.
    """
    if not mapping or not mapping.get('applied'):
        return []
    result = []
    start = 0
    before = mapping['boundaries'][0]['before_phase']
    for boundary in mapping['boundaries']:
        end = int(boundary['path_index'])
        result.append(dict(phase=before, path_start_index=start, path_end_index=end,
                           command_start_index=0 if start == 0 else handoff_samples+start,
                           command_end_index=handoff_samples+end,
                           constraint_phase=boundary.get('constraint_phase', 'above_bowl')))
        start, before = end, boundary['after_phase']
    result.append(dict(phase=before, path_start_index=start, path_end_index=path_samples-1,
                       command_start_index=handoff_samples+start,
                       command_end_index=handoff_samples+path_samples-1,
                       constraint_phase=mapping['boundaries'][-1].get('constraint_phase', 'above_bowl')))
    return result
