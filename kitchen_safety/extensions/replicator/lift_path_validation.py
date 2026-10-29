"""Verify the returned TCP curve before permitting a real grasp lift.

These are task-specific geometric limits, not proof of contact stability or
collision safety. They supplement cuRobo's soft pose cost with an actual FK gate.
"""
import numpy as np

FIELDS = {'reference_phase', 'max_xy_deviation_m', 'max_orientation_deviation_deg', 'min_z_offset_m'}


def checked_limits(recipe):
    limits = recipe.get('lift_path_validation')
    if limits is None:
        return None
    if not isinstance(limits, dict) or set(limits) != FIELDS or limits['reference_phase'] != 'approach':
        raise ValueError('Lift gate requires the actual approach endpoint reference')
    for key in FIELDS - {'reference_phase'}:
        if type(limits[key]) not in (int, float) or not np.isfinite(limits[key]):
            raise ValueError('Invalid finite lift limit: ' + key)
    if not (0 < limits['max_xy_deviation_m'] <= .02
            and 0 < limits['max_orientation_deviation_deg'] <= 10
            and -.005 <= limits['min_z_offset_m'] <= 0):
        raise ValueError('Lift limits outside supported local corridor')
    return dict(limits)


def measure_lift(poses, reference, limits):
    poses, reference = np.asarray(poses), np.asarray(reference)
    if (poses.ndim != 3 or poses.shape[1:] != (4, 4) or not len(poses)
            or reference.shape != (4, 4) or not np.isfinite(poses).all() or not np.isfinite(reference).all()):
        raise ValueError('Lift FK samples/reference must be finite rigid poses')
    delta = poses[:, :3, 3] - reference[:3, 3]
    xy = float(np.linalg.norm(delta[:, :2], axis=1).max())
    # Execution preflight runs before Kit. Keep this module NumPy-only: loading
    # SciPy's native libraries here changes Kit's plugin initialization order.
    relative = poses[:, :3, :3] @ reference[:3, :3].T
    skew = np.stack((relative[:,2,1]-relative[:,1,2],relative[:,0,2]-relative[:,2,0],
        relative[:,1,0]-relative[:,0,1]),axis=1)
    sine = .5*np.linalg.norm(skew,axis=1)
    cosine = np.clip((np.trace(relative,axis1=1,axis2=2)-1)*.5,-1.,1.)
    angle = float(np.rad2deg(np.arctan2(sine,cosine)).max())
    low = float(delta[:, 2].min())
    return dict(reference_world=reference.tolist(), thresholds=dict(limits), source_samples=len(poses),
        max_xy_deviation_m=xy, max_orientation_deviation_deg=angle, min_z_offset_m=low,
        final_z_offset_m=float(delta[-1, 2]),
        passed=(xy <= limits['max_xy_deviation_m'] + 1e-9
                and angle <= limits['max_orientation_deviation_deg'] + 1e-9
                and low >= limits['min_z_offset_m'] - 1e-9),
        scope='Returned FK samples relative to actual approach endpoint; not physical grasp or safety acceptance')


def validate_round9_policy(recipe):
    if recipe.get('retry_round') != 9 or recipe.get('retained_scene_revision') is not True or recipe.get('scene_settings_changed_this_round') is not False:
        raise ValueError('Round 9 recipe must retain the exact round 8 layout')
    policy = recipe.get('planning_path_policy', {})
    if policy.get('previous_straight_line_gate_removed') is not True or recipe.get('joint_space_target_q_by_phase') or recipe.get('segment_deviation_limits_m'):
        raise ValueError('Round 9 cannot restore the old all-phase line gate or use joint-space goals')
    limits = checked_limits(recipe)
    if recipe.get('payload_name') == 'mustard_bottle' and limits is None:
        raise ValueError('Round 9 mustard repair requires the explicit lift FK gate')
    if limits is not None:
        if (policy.get('kind') != 'collision_checked_lift_corridor'
                or 'lift' not in recipe.get('hold_orientation_phases', [])
                or 'lift' not in recipe.get('linear_motion_phases', [])):
            raise ValueError('Lift corridor requires an explicit lift pose cost and FK gate')
        targets = recipe['target_poses_world']
        ref = np.asarray(targets['approach'])
        points = recipe.get('phase_waypoints_world', {}).get('lift', []) + [targets['lift']]
        heights = []
        for point in points:
            p = np.asarray(point)
            if not np.allclose(p[:3, :3], ref[:3, :3], atol=1e-7, rtol=0) or not np.allclose(p[:2, 3], ref[:2, 3], atol=1e-7, rtol=0):
                raise ValueError('Frozen lift goals must keep approach XY and orientation')
            heights.append(float(p[2, 3] - ref[2, 3]))
        if not heights or heights[0] <= 0 or any(b <= a for a, b in zip(heights, heights[1:])):
            raise ValueError('Lift waypoint heights must increase monotonically')
    elif policy.get('kind') != 'collision_checked_free_path':
        raise ValueError('Unknown round 9 planning policy')
    return limits


def array_lift_gate(data, recipe):
    limits = checked_limits(recipe)
    if limits is None:
        return None
    phase = np.asarray(data['phase'])
    reference = data['tcp_world'][np.flatnonzero(phase == 'approach')[-1]]
    return measure_lift(data['tcp_world'][phase == 'lift'], reference, limits)
