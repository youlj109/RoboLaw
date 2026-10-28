import json, pathlib, hashlib
root=pathlib.Path('/root/autodl-tmp/RoboLaw-lgd/datasets/skill_trials/lightwheel_pick_place')
groups={'B03':['property_lgd_b03_checkpoint_replay_20261027T064923Z','robot_lgd_b03_checkpoint_replay_20261027T033440Z'],'B04':['human_lgd_b04_round09_20261027T081735Z','property_lgd_b04_round16_20261027T095059Z','robot_lgd_b04_round36_20261027T152820Z']}
keys=['seed','placement_seed','requested_grasp_tilt_deg','requested_grasp_yaw_deg','requested_grasp_height_offset_m','smooth_lift','allowed_grasp_fallback','selected_grasp','dt','pregrasp_offset_m','grasp_lateral_x_m','grasp_finger_bias_m','grasp_local_approach','grasp_point_body_m','transfer_route_policy','commanded_initial_lift_m','pregrasp_planner_mode','task_motion_parameters','planning_attachment_policy','planning_attachment_updates','source_support_contact_model','initial_state_layout_source','place_lower']
report={'scope':'Read-only actual stored controller fields; no acceptance waiver; missing fields remain missing','groups':{}}
for g,names in groups.items():
 rows=[]
 for n in names:
  p=root/n;r=json.load(open(p/'result.json'));a=json.load(open(p/'avoidance_audit.current_review.json'));bp=pathlib.Path(a['baseline_episode']);b=json.load(open(bp/'result.json'))
  diffs={k:{'baseline':b.get(k),'variant':r.get(k),'missing':k not in b or k not in r} for k in keys if b.get(k)!=r.get(k) or k not in b or k not in r}
  gates={k:v for k,v in a.items() if k in ['accepted','baseline_original_success_verified','same_source_layout','same_control_parameters','same_attachment_update_policy','same_source_support_policy','source_settling_nuisance_verified','hazard_environment_legal','avoidance_effective','initial_and_goal_feasible','baseline_audit_gates','augmented_audit_gates']}
  rows.append({'variant_episode':str(p),'baseline_episode':str(bp),'gates':gates,'stored_field_differences':diffs})
  print(g,n,'FAILED:',[k for k,v in gates.items() if v is False],flush=True)
 report['groups'][g]=rows
out=pathlib.Path('/root/autodl-tmp/B-group-review/strict_acceptance_gaps_20261028.json');out.write_text(json.dumps(report,indent=2));print(out)
