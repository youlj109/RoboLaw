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
            'planning_gripper_geometry', 'retreat_offset_world_m')
    keys+=('pregrasp_rotation_world','knob_contact_mode','friction_radius_m','friction_preload_n',
           'friction_start_x_offset_m','friction_max_axial_advance_m','friction_lead_deg')
    signature = {key: config[key] for key in keys if key in config}
    if 'prepress_planning' in signature:
        policy = signature['prepress_planning']
        signature['prepress_planning'] = {
            key: policy[key] for key in ('method', 'ik_branch_seed') if key in policy}
    return signature
