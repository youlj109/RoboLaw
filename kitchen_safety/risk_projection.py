"""Comparable raw-frame projection used only to audit the Arena feature adapter.

Explicit semantic labels are supplied here: we do not ask legacy substring
heuristics to infer Robot/person/mustard identities from arbitrary USD paths.
"""
from kitchen_safety.risk_profile_loader import create_evaluator  # imports reference package
from safety_risk.iso_input import raw_frame


def reference_projection(state,distances,contacts,arm_ids,dt,human):
    hs = distances.get('hs_geometry', {}).get('features', {})
    mapping={'robot_human_distance_matrix_gt':hs.get('d_robot_h_min_gt_m') if human else None,
             'ee_human_distance_gt':hs.get('d_ee_h_min_gt_m') if human else None,
             'object_human_distance_gt':hs.get('d_obj_h_min_gt_m') if human else None}
    def label(path):
        if '/person/' in path:return 'human/person'
        if '/Robot/' in path:return 'robot/'+path.rsplit('/',1)[-1]
        if '/mustard_bottle' in path:return 'object/mustard_bottle'
        return 'environment/'+path.rsplit('/',1)[-1]
    pairs=[{**p,'bodyA':label(p['bodyA']),'bodyB':label(p['bodyB'])} for p in contacts.get('pairs',[])]
    limits=state.get('joint_pos_limits') or []
    efforts=state.get('joint_effort_limits') or []
    raw={'metadata':{'num_steps':1},'episode_meta':{'target_object_id':'mustard_bottle','physics_config':{
        'physics_dt_s':dt,'joint_position_limits_rad_by_index':{str(j):{'lower_rad':v[0],'upper_rad':v[1]} for j,v in enumerate(limits)},
        'joint_torque_limits_nm_by_index':{str(j):{'limit_nm':v} for j,v in enumerate(efforts)}}},
        'distance_gt':{**{k:[v] for k,v in mapping.items()},'_provenance':{k:{'metric':'surface_clearance', 'source':distances.get('hs_geometry',{}).get('source')} for k in mapping}},
        'robot_state':{'joint_position_q_gt':[state.get('q')],'joint_torque_gt':[state.get('applied_torque')],
                       'joint_state_metadata':{'risk_metric_indices':arm_ids}},
        'collision_gt':{'collision_pair_gt':[pairs] if contacts.get('coverage') else [None],
                        'contact_coverage_per_frame':[contacts.get('coverage') is True]}}
    return raw_frame(raw)['features']
