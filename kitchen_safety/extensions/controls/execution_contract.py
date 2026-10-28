"""Shared matching contract for controller-only revisions of frozen scenes."""

FROZEN_FIELDS = ('task_id', 'source_sha256', 'robot_pose_xyzw', 'variants',
                 'mechanism_joint', 'button_joint', 'button_center_world_m',
                 'original_target_level', 'original_reset_level', 'knob_axis_world',
                 'grasp_tcp_world_m', 'joint_limits_rad', 'button_joint_limits_m')


def frozen_task_differences(source, revised):
    return [key for key in FROZEN_FIELDS if source.get(key) != revised.get(key)]


def controller_signature(config):
    keys = ('task_id', 'source_sha256', 'robot_pose_xyzw', 'prepress_planning',
            'button_center_world_m', 'grasp_tcp_world_m', 'pregrasp_tcp_world_m',
            'initial_roll_deg', 'pregrasp_roll_deg', 'grasp_rotation_world',
            'target_angle_interior_deg', 'repair_oven_racks', 'balance_temperature_knob',
            'planning_gripper_geometry', 'retreat_offset_world_m', 'planning_fixture_cylinder_segments')
    keys+=('pregrasp_rotation_world','knob_contact_mode','friction_radius_m','friction_preload_n',
           'friction_start_x_offset_m','friction_max_axial_advance_m','friction_min_axial_advance_m',
           'friction_lead_deg','friction_terminal_policy','friction_alignment_world_m','friction_start_angle_deg',
           'friction_connection_world_m')
    signature = {key: config[key] for key in keys if key in config}
    if 'prepress_planning' in signature:
        policy = signature['prepress_planning']
        signature['prepress_planning'] = {
            key: policy[key] for key in ('method', 'ik_branch_seed') if key in policy}
    return signature


def controller_digest(config):
    """Cache semantically equal JSON numbers identically, without rounding."""
    import hashlib
    import json
    import math
    def canonical(value):
        if isinstance(value, dict):return {key:canonical(v) for key,v in value.items()}
        if isinstance(value, list):return [canonical(v) for v in value]
        if isinstance(value, float):
            if not math.isfinite(value):raise ValueError('Controller values must be finite')
            if value.is_integer():return int(value)
        return value
    return hashlib.sha256(json.dumps(canonical(controller_signature(config)),sort_keys=True).encode()).hexdigest()


def root_pose_match(measured_xyzw,expected_xyzw):
    """Bind the pinned Arena/IsaacLab xyz/xyzw root-pose contract in physical units."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    measured=np.asarray(measured_xyzw,dtype=float);expected=np.asarray(expected_xyzw,dtype=float)
    if measured.shape!=(7,) or expected.shape!=(7,) or not np.isfinite([measured,expected]).all():
        raise ValueError('Root poses must be finite xyz/xyzw seven-vectors')
    position=float(np.linalg.norm(measured[:3]-expected[:3]))
    angle=float((Rotation.from_quat(measured[3:]).inv()*Rotation.from_quat(expected[3:])).magnitude())
    return dict(passed=position<=1e-6 and angle<=1e-6,position_error_m=position,
        rotation_error_rad=angle,position_tolerance_m=1e-6,rotation_tolerance_rad=1e-6,
        quaternion_order='xyzw')


def paired_coverage_gates(runs,seeds,variants):
    expected={(seed,variant) for seed in seeds for variant in variants}
    observed=[(row['seed'],row['variant']) for row in runs]
    complete=bool(expected) and len(observed)==len(expected) and set(observed)==expected
    hazards=[row for row in runs if row['variant']!='base']
    return dict(complete_pairs=complete,
        all_plans_valid=complete and all(row.get('success') is True for row in runs),
        all_hazard_off_paths_require_adjustment=complete and bool(hazards) and all(
            row.get('true_shape_counterfactual',{}).get('violations') for row in hazards),
        all_counterfactual_risks_robust=complete and bool(hazards) and all(
            row.get('true_shape_counterfactual',{}).get('robust_violation') is True for row in hazards),
        all_counterfactual_endpoints_safe=complete and bool(hazards) and all(
            row.get('true_shape_counterfactual',{}).get('initial_and_goal_safe') is True for row in hazards),
        all_hazard_on_source_shapes_safe=complete and bool(hazards) and all(
            row.get('true_shape_actual',{}).get('samples',0)>0 and
            not row['true_shape_actual']['violations'] for row in hazards))
