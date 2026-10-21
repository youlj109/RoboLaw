"""Geometry checks for randomized layouts; no simulated counterfactual motion."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def relative_grasp_rotation(reference_rotation, bottle_rotation):
    """Map the nominal bottle-local grasp orientation into its observed frame."""
    return np.asarray(bottle_rotation) @ np.asarray(reference_rotation)


def check_initial_layout(world, state, layout, arm_indices):
    """Reject unsupported/tipped spawns and intersecting task objects after settling.

    Existing support heights are retained. FCL checks actual collider pairs,
    not whole-asset boxes (which would fill the bowl and portal openings).
    """
    failures = []
    hazard = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}[layout['obstacle']['category']]
    supported = [(key, state[key][:7], layout[position_key])
                 for key, position_key in [('mustard', 'mustard_position'), ('bowl', 'bowl_position')]]
    if 'hazard_pose' in state:
        supported.append((hazard, state['hazard_pose'], layout['obstacle']['position']))
    for key, observed, requested in supported:
        pose = np.asarray(observed)
        expected = np.asarray(requested)
        if abs(pose[2] - expected[2]) > .015 or np.linalg.norm(pose[:2] - expected[:2]) > .01:
            failures.append({'object': key, 'reason': 'spawn moved off its intended support',
                             'requested_xyz': expected.tolist(), 'settled_xyz': pose[:3].tolist()})
        if Rotation.from_quat(pose[3:]).as_matrix()[2, 2] < np.cos(np.deg2rad(5)):
            failures.append({'object': key, 'reason': 'object tipped during settling'})
    keys = ['mustard_bottle', 'bowl', hazard]
    groups = {key: [entry for entry in world.objects if world._matches(entry[0], key)] for key in keys}
    for key, group in groups.items():
        if not group:
            failures.append({'object': key, 'reason': 'missing collision geometry'})
    checked_pairs = 0
    for index, key in enumerate(keys):
        for other in keys[index + 1:]:
            intersection = None
            for name_a, a, low_a, high_a in groups[key]:
                for name_b, b, low_b, high_b in groups[other]:
                    if np.any(high_a < low_b) or np.any(high_b < low_a):
                        continue
                    checked_pairs += 1
                    if world.fcl.collide(a, b, world.fcl.CollisionRequest(), world.fcl.CollisionResult()):
                        intersection = {'objects': [key, other], 'colliders': [name_a, name_b],
                                        'reason': 'initial task-object colliders intersect'}
                        break
                if intersection:
                    break
            if intersection:
                failures.append(intersection)
    q = np.asarray(state['q'])[arm_indices]
    if not world.valid(q, ('mustard_bottle',), object_pose=state['mustard'][:7]):
        failures.append({'reason': 'initial robot or bottle violates existing planning constraints',
                         'collision': world.last_collision, 'measurement': world.last_clearance})
    return {'valid': not failures, 'failures': failures, 'object_pairs_checked': checked_pairs}


def _cartesian_reference(world, planner, start, goal):
    """Build one straight pose connection with the supplied planning endpoints.

    Bottle pose is the interpolation frame while attached; otherwise it is
    TCP pose. A deterministic endpoint-interpolated joint seed controls the
    redundant IK degree of freedom. No endpoint connector, via point or motion
    planner is allowed. Dense FK checks reject discontinuous IK branches and
    chord deviations from this specified straight/Slerp pose path.
    """
    kin = getattr(world, 'kin', None) or getattr(planner, 'kin', None)
    attachment = getattr(world, 'attachment', None)
    attachment = np.eye(4) if attachment is None else np.asarray(attachment).copy()
    frame = 'bottle' if getattr(world, 'attachment', None) is not None else 'tcp'
    report = {'reference': frame + '_cartesian_linear', 'confirmed': False,
              'direct_connection_valid': False,
              'construction': 'straight translation and shortest Slerp; IK seed is supplied planning-endpoint joint interpolation',
              'directness': {'frame': frame, 'translation_step_m': .005, 'rotation_step_rad': .025,
                            'edge_resolution_rad': .02, 'position_tolerance_m': .002,
                            'rotation_tolerance_rad': .005, 'start_joint_error_rad': 0.,
                            'goal_joint_error_rad': None}}
    if kin is None:
        report['reason'] = 'No kinematics available for a Cartesian direct reference'
        return None, report
    path = [np.asarray(start).copy()]
    try:
        first, last = kin.fk(start) @ attachment, kin.fk(goal) @ attachment
        rotations = Slerp([0., 1.], Rotation.from_matrix(np.stack([first[:3, :3], last[:3, :3]])))
        angle = Rotation.from_matrix(first[:3, :3].T @ last[:3, :3]).magnitude()
        count = max(2, int(np.ceil(np.linalg.norm(last[:3, 3] - first[:3, 3]) / .005)) + 1,
                    int(np.ceil(angle / .025)) + 1)
        fractions = np.linspace(0., 1., count)
        inverse_attachment = np.linalg.inv(attachment)
        for fraction in fractions[1:-1]:
            target = np.eye(4)
            target[:3, 3] = (1-fraction) * first[:3, 3] + fraction * last[:3, 3]
            target[:3, :3] = rotations(fraction).as_matrix()
            seed = (1-fraction) * start + fraction * goal
            q = np.asarray(kin.ik(target @ inverse_attachment, seed, attempts=1))
            if q.shape != (7,) or not np.isfinite(q).all():
                raise ValueError('Cartesian IK returned invalid joints')
            path.append(q.copy())
        path.append(np.asarray(goal).copy())
        path = np.asarray(path)
        maximum_position_error = maximum_rotation_error = 0.
        dense_samples = 0
        for edge, (a, b) in enumerate(zip(path[:-1], path[1:])):
            samples = max(2, int(np.ceil(np.max(np.abs(b-a)) / .02)) + 1)
            for fraction in np.linspace(0., 1., samples):
                alpha = fractions[edge] + fraction * (fractions[edge+1] - fractions[edge])
                actual = kin.fk(a + fraction * (b-a)) @ attachment
                expected_position = (1-alpha) * first[:3, 3] + alpha * last[:3, 3]
                position_error = float(np.linalg.norm(actual[:3, 3] - expected_position))
                rotation_error = float(Rotation.from_matrix(
                    actual[:3, :3].T @ rotations(alpha).as_matrix()).magnitude())
                if not np.isfinite([position_error, rotation_error]).all():
                    raise ValueError('Nonfinite Cartesian direct-reference deviation')
                maximum_position_error = max(maximum_position_error, position_error)
                maximum_rotation_error = max(maximum_rotation_error, rotation_error)
                dense_samples += 1
        report['directness'].update(
            maximum_position_error_m=maximum_position_error,
            maximum_rotation_error_rad=maximum_rotation_error,
            start_joint_error_rad=float(np.max(np.abs(path[0]-start))),
            goal_joint_error_rad=float(np.max(np.abs(path[-1]-goal))),
            maximum_joint_edge_rad=float(np.max(np.abs(np.diff(path, axis=0)))),
            pose_waypoints=len(path), dense_samples=dense_samples,
            start_pose=first.tolist(), goal_pose=last.tolist())
        report['reference_joint_positions'] = path.tolist()
        report['direct_connection_valid'] = (maximum_position_error <= .002
                                              and maximum_rotation_error <= .005)
        if not report['direct_connection_valid']:
            report['reason'] = 'Dense IK edges deviate from the specified straight Cartesian reference'
            return None, report
        return path, report
    except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
        report['reason'] = 'Cartesian direct reference is not reachable with the specified joint endpoints'
        report['reference_error'] = str(error)
        report['reference_joint_positions'] = np.asarray(path).tolist()
        return None, report


def probe_obstruction(world, planner, start, goal, ignore, grip, phase, hazard):
    """Try joint-linear, then a straight Cartesian reference, without detours.

    The collector supplies the selected path's measured planning start and
    planned goal before executing that phase; the goal is not a measurement
    of its eventual physical endpoint. Each candidate retains those exact
    supplied joint endpoints, excludes itself if any non-target constraint
    fails, then checks the target hazard. Queries never execute in physics;
    masks and attachment settings are restored.
    """
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    result = {'phase': phase, 'hazard': hazard, 'confirmed': False,
              'reference': 'joint_linear', 'start_q': start.tolist(),
              'goal_q': goal.tolist(), 'reference_attempts': [],
              'endpoint_provenance': {
                  'start': 'selected path start from the measured state at planning time',
                  'goal': 'selected path planned goal before phase execution',
                  'post_execution_measured_endpoints': False}}
    if planner is None:
        return {**result, 'reason': 'reference needs cuRobo self-collision and joint checks'}
    without_hazard = tuple(dict.fromkeys((*ignore, hazard)))

    def evaluate(path, report):
        report['reference_joint_positions'] = np.asarray(path).tolist()
        try:
            validation = planner.validate_path(path, without_hazard, grip, phase=phase)
        except RuntimeError as error:
            report.update(reference_valid_without_hazard=False, reference_error=str(error),
                          reason='Direct reference validation failed')
            return
        report.update(reference_valid_without_hazard=validation['valid'], samples=validation['samples'],
                      curobo_failure_count=len(validation.get('curobo_failures', [])),
                      fcl_failure_count=len(validation.get('fcl_failures', [])),
                      fcl_failures=validation.get('fcl_failures', [])[:3])
        if not validation['valid']:
            report['reason'] = 'Direct reference violates non-target constraints; excluded'
            return
        for edge, (a, b) in enumerate(zip(path[:-1], path[1:])):
            count = max(2, int(np.ceil(np.max(np.abs(b-a)) / .02)) + 1)
            for fraction in np.linspace(0., 1., count):
                q = a + fraction * (b-a)
                if not world.valid(q, ignore, obstacle_filter=(hazard,)):
                    measurement = world.last_clearance
                    if measurement is not None and world._matches(measurement['obstacle'], hazard):
                        report.update(confirmed=True, edge=edge, fraction=float(fraction),
                                      measurement=dict(measurement))
                        return
        report['reason'] = 'Direct reference stays outside target hazard clearance'

    try:
        direct = {'reference': 'joint_linear', 'confirmed': False, 'direct_connection_valid': True,
                  'directness': {'frame': 'joint', 'start_joint_error_rad': 0., 'goal_joint_error_rad': 0.}}
        result['reference_attempts'].append(direct)
        evaluate(np.asarray([start, goal]), direct)
        if direct['confirmed']:
            result.update(direct)
            return result
        path, cartesian = _cartesian_reference(world, planner, start, goal)
        result['reference_attempts'].append(cartesian)
        if path is not None:
            evaluate(path, cartesian)
        if cartesian['confirmed']:
            result.update(cartesian)
        else:
            result.update(reference_valid_without_hazard=direct.get('reference_valid_without_hazard', False),
                          reason='Neither specified direct reference established target obstruction')
        return result
    finally:
        planner._prepare_path_query(ignore, grip, phase)
