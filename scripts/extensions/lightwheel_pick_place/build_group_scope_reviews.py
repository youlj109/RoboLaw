"""Aggregate evidence and unresolved gates; never rewrite individual audit results."""
import json,hashlib,datetime,subprocess
from pathlib import Path
subprocess.run(['/root/autodl-tmp/conda_envs/robolaw_data/bin/python','scripts/extensions/lightwheel_pick_place/build_delivery_inventory.py'],check=True)
base=Path('/root/autodl-tmp');inventory=json.loads((base/'B-group-review/current_delivery_inventory.json').read_text());stamp=datetime.datetime.now(datetime.timezone.utc).isoformat()
def reference(path):
 path=Path(path)
 if not path.exists():return {'path':str(path),'exists':False}
 with path.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
 r=json.loads(path.read_text());return {'path':str(path.resolve()),'sha256':digest,'exists':True,'passed':r.get('passed',r.get('accepted'))}
for task in ('B01','B03','B04'):
 rows=inventory['tasks'][task]['episodes'];native=all(x['original_native_success'] and x['videos_exist'] for x in rows.values());core=all(all(x['saved_report_results'].get(k) is True for k in ('validation.json','full_world_audit.json','self_collision_audit.json')) for x in rows.values())
 report={'as_of_utc':stamp,'task':task,'scope':'Evidence aggregation separates native collection, actual safety, strict cross-episode comparisons and planning-only necessity. Existing false reports are not upgraded.','native_success_and_original_videos_complete':native,'validation_world_self_complete':core,'episodes':rows,'passed':False,'full_review_gates_passed':False,'github_status':'Preparing lgd branch and verified evidence bundle; not pushed','remaining':[]}
 if task=='B01':
  variants=[x for k,x in rows.items() if k!='base'];comparisons=all(x.get('avoidance_audit.json',{}).get('accepted') is True for x in variants);neutral=inventory['tasks'][task]['historical_neutral_passed'];paired=inventory['tasks'][task]['historical_paired_necessity_passed'];fixed=all(x['saved_report_results']['fixed_base_hazard_audit.json'] is True for x in variants)
  report.update(passed=bool(native and core and comparisons and neutral and paired and fixed),historical_neutral_passed=neutral,historical_paired_necessity_passed=paired,strict_variant_comparison_passed=comparisons,base_fixed_hazard_report={'applicable':False,'raw_passed':rows['base']['saved_report_results']['fixed_base_hazard_audit.json'],'reason':'No added hazard in baseline; preserved raw report says KeyError:base, not physical collision'},paired_scope='Historical tested-seed planning necessity; hazard-on solver outcomes and free-transfer coverage remain as source report, not full-task physical success rate')
  report['full_review_gates_passed']=False;report['remaining']=['Full-review scope is not inferred from historical booleans; source reports and user exclusions remain authoritative']
 else:
  comparisons={}
  for v,x in rows.items():
   ep=Path(x['episode']);candidate=ep/'avoidance_audit.current_review.json'
   for name in ('avoidance_audit.fresh_matched.policy_review.json','avoidance_audit.fresh_matched.current_review.json','avoidance_audit.fresh_matched.json','avoidance_audit.current_review.json','avoidance_audit.json'):
    if (ep/name).exists():candidate=ep/name;break
   if candidate.exists():
    r=json.loads(candidate.read_text());comparisons[v]={'reference':reference(candidate),**{k:r.get(k) for k in ('accepted','same_control_parameters','same_attachment_update_policy','source_settling_nuisance_verified','hazard_environment_legal','avoidance_effective')}}
  report['cross_episode_comparisons']=comparisons
  report['strict_cross_episode_comparison_complete']=bool(len(comparisons)==3 and all(x.get('accepted') for x in comparisons.values()))
  if task=='B03':
   report['paired_evidence']={'first_transport':reference(base/'B03-review/paired_transportreview_080313_leg3_20261027/paired_planning.json'),'second_transport':reference(base/'B03-review/paired_transportreview_080313_leg9_20261027/paired_planning.json')};report['paired_necessity_primary_passed']=report['paired_evidence']['first_transport']['passed'] is True;report['scene_support_context_review']=reference(base/'B03-review/property_native_support_context_review.json')
  else:
   report['paired_evidence']={v:reference(base/'B04-review'/name/'paired_planning.json') for v,name in {'human':'paired_actual_human09_policy_v2_20261028','property':'paired_current_property_20261027','robot':'paired_current_robot_20261027'}.items()};report['paired_necessity_primary_passed']=all(x.get('passed') is True for x in report['paired_evidence'].values());report['matching_baseline_attempt']=reference(Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_matched_human_20261027T162825Z/result.json'));report['matching_baseline_attempt']['original_native_success']=False;report['additional_matching_baseline_attempt']=reference(Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_matched_historical_20261027T165151Z/result.json'));report['additional_matching_baseline_attempt']['original_native_success']=False;report['new_shared_grasp_baseline_proposal']={'executed':False,'status':'Automatically rejected: explicit user authorization required before modifying collector and starting this additional costly cross-variant prefix capture'}
  report['passed']=bool(native and core and report['strict_cross_episode_comparison_complete'] and report['paired_necessity_primary_passed'])
  current=base/(task+'-review')/'current_selected_acceptance_20261028.json'
  if current.exists():
   selected=json.loads(current.read_text());report['current_selected_acceptance']=reference(current);report['passed']=bool(selected['passed']);report['selected_remaining']=selected['remaining'];report['current_paired_evidence']=selected.get('paired_planning',{'human':selected.get('human_primary_paired_planning'),'robot_property':selected.get('fresh_robot_property_primary_paired_planning')})
  if not report['strict_cross_episode_comparison_complete']:report['remaining'].append('Strict physically collected baseline control/attachment/source nuisance matching incomplete; planning-only matched toggles do not automatically satisfy this gate')
  report['scope_notes']=['Unpassed auxiliary diagnostics and user-excluded background investigations remain explicitly separate; historical raw reports preserved']
 out=base/(task+'-review')/'scope_review_manifest.json';out.parent.mkdir(exist_ok=True);out.write_text(json.dumps(report,indent=2));print(task,'native',native,'core',core,'scope_passed',report['passed'],flush=True)
