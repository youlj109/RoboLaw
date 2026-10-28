"""Hash-backed parameter-recipe comparison. Does not replace native/audit results."""
import argparse,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--variant',choices=['robot','property'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
b=read(a.baseline/'result.json');profile=b['explicit_b03_stage_profile']['profile'];target=Path(profile['stages']['banana_2']['source']['path']);r=read(target);first_path=Path(profile['stages']['banana_1']['source']['path']);first=read(first_path)
checks={'baseline_original_reset':not any(b.get(k) for k in ('prefix_replay','development_only','second_grasp_replay')),'profile_target_variant':profile['matching_target_variant']==a.variant,'profile_source_hashes':all(sha(Path(x['path']))==x['sha256'] for x in profile['source_results']),'baseline_collector_snapshot_hash':sha(a.baseline/'collector_source_snapshot.py')==b['collector_source_sha256'],'executed_second_stage':b.get('executed_stage_control_profiles',{}).get('banana_2')==profile['stages']['banana_2']}
provenance=read(Path('/root/autodl-tmp/B03-review/replay_control_provenance_review_20261028.json'))[target.parent.name]
checks['variant_first_source_commands_verified']=all(x['actual_recorded_actions_and_phases_identical'] and sha(Path(x['episode'])/'result.json')==x['result_sha256'] for x in provenance['verified_replay_chain']) and Path(provenance['first_pick_source']).resolve()==first_path.parent.resolve()
checks['variant_second_grasp_commands_verified']=all(provenance['second_grasp_prefix'].get(k) is True for k in ('sha256_matches','actual_commands_match'))
keys=('seed','placement_seed','requested_grasp_tilt_deg','requested_grasp_yaw_deg','requested_grasp_height_offset_m','smooth_lift','allowed_grasp_fallback','dt','grasp_lateral_x_m','grasp_finger_bias_m','grasp_local_approach','pregrasp_planner_mode','planning_attachment_policy')
checks['first_input_controls']=all(k in b and k in first and b[k]==first[k] for k in keys)
first_motion=dict(first['task_motion_parameters']);first_motion.setdefault('gentle_supported_placement',False);first_motion.setdefault('placement_feedback_lower',False)
checks['first_motion_parameters']=b['task_motion_parameters']==first_motion
mapping={'gentle_supported_placement':'gentle_supported_placement','round10_second_placement':'round10_second_placement','release_ramp_steps':'release_ramp_steps','dual_first_place_offset_x':'dual_first_place_offset_x_m','dual_second_place_offset_x':'dual_second_place_offset_x_m','dual_first_place_offset_y':'dual_first_place_offset_y_m','align_second_grasp':'align_second_grasp','free_release':'free_release'}
stage=profile['stages']['banana_2'];checks['second_motion_overrides']=all(stage['args'][k]==r['task_motion_parameters'][v] for k,v in mapping.items())
checks['second_body_goal_file']=sha(Path(stage['placement_goals']))==stage['placement_goals_sha256']==r['task_motion_parameters']['placement_goals_sha256']
checks['explicit_second_center_offset']=stage['center_offset_xy_m']==([.045,-.035] if a.variant=='property' else [.035,0.])
checks['same_grasp_search_policy']=b.get('grasp_search_policy')==first.get('grasp_search_policy')
report={'scope':'Per-pick parameter recipe provenance only. Archived variant command execution differs from fresh baseline planning; not a claim of bit-identical online runtime. Native success, physical initial-state matching, full safety and counterfactual evidence are separate mandatory gates.','baseline':str(a.baseline),'target_episode':str(target.parent),'input_hashes':{'baseline_result':sha(a.baseline/'result.json'),'variant_result':sha(target),'first_source_result':sha(first_path)},'checks':checks,'parameter_recipe_match':all(checks.values()),'baseline_native_success':b.get('success'),'strict_acceptance_upgraded':False,'failure_details':{k:False for k,v in checks.items() if not v}}
a.output.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k in ('parameter_recipe_match','baseline_native_success','failure_details','strict_acceptance_upgraded')}))
