"""Aggregate selected current B03 evidence without modifying any source report."""
import datetime,hashlib,json
from pathlib import Path
root=Path('/root/autodl-tmp/RoboLaw-lgd/datasets/skill_trials/lightwheel_pick_place')
review=Path('/root/autodl-tmp/B04-review')
def ref(p):
 p=Path(p)
 if not p.exists():return {'path':str(p),'exists':False}
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return {'path':str(p),'exists':True,'sha256':h.hexdigest()}
def report(p,key='passed'):
 r=ref(p)
 if r['exists']:r[key]=json.loads(Path(p).read_text()).get(key)
 return r
selected={'human':('human_lgd_b04_fresh_matched_20261028T104647Z','base_lgd_b04_fresh_matched_20261028T102807Z','avoidance_audit.fresh_matched.policy_review.json'),'property':('property_lgd_b04_fresh_matched_20261028T114946Z','base_lgd_b04_fresh_matched_20261028T102807Z','avoidance_audit.fresh_matched.json'),'robot':('robot_lgd_b04_fresh_matched_20261028T111828Z','base_lgd_b04_fresh_matched_20261028T102807Z','avoidance_audit.fresh_matched.policy_review.json')}
rows={};required=[]
for v,(episode,baseline,audit) in selected.items():
 p=root/episode;b=root/baseline;r=json.loads((p/'result.json').read_text());br=json.loads((b/'result.json').read_text())
 core={n:report(p/n) for n in ['validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','fixed_base_hazard_audit.json']}
 basecore={n:report(b/n) for n in ['validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json']}
 a=report(p/audit,'accepted');videos={n:ref(p/(n+'.mp4')) for n in ['external_camera','external_camera_2','wrist_camera']+(['overview_camera'] if v=='human' else [])}
 validation=json.loads((p/'validation.json').read_text());decoded=validation.get('checks',{}).get('videos') is True and (v!='human' or validation.get('checks',{}).get('overview_videos') is True)
 plots={suffix:ref(review/('fresh_'+v+'_final')/('path_comparison.'+suffix)) for suffix in ['png','pdf']}
 passed=bool(r.get('success') and br.get('success') and a.get('accepted') is True and all(x.get('passed') is True for x in list(core.values())+list(basecore.values())) and all(x['exists'] for x in videos.values()) and decoded and all(x['exists'] for x in plots.values()))
 rows[v]={'episode':str(p),'baseline':str(b),'result':ref(p/'result.json'),'baseline_result':ref(b/'result.json'),'native_success':r.get('success'),'original_terminal_terms':r.get('terminal_terms'),'actual_duration_seconds':r['frames']*r['dt'],'baseline_native_success':br.get('success'),'core':core,'baseline_core':basecore,'comparison':a,'videos':videos,'complete_video_decode_verified':decoded,'trajectory':ref(p/'trajectory.h5'),'images':ref(p/'images.h5'),'source_scene':ref(p/'scene.usda'),'path_comparison':plots,'passed':passed}
 required.append(passed)
neutral=report(review/'task_neutral_90_extendedwindow_20261026/neutral_hold.json')
paired_names={'human':'paired_fresh_common_human102807_execution_policy_20261028','property':'paired_fresh_common_property_102807_20261028','robot':'paired_fresh_common_robot_102807_20261028'}
paired={v:report(review/name/'paired_planning.json') for v,name in paired_names.items()}
for v,name in paired_names.items():
 path=review/name/'paired_planning.json'
 if path.exists():
  j=json.loads(path.read_text());row=j.get('variant_reports',{}).get(v,{});paired[v]['all_on_plans_valid']=all(x['success'] and x['actual_violations']==0 for x in row.get('seeds',[])) and len(row.get('seeds',[]))==3
  required.append(paired[v].get('passed') is True and paired[v]['all_on_plans_valid'])
 else:required.append(False)
nj=json.loads((review/'task_neutral_90_extendedwindow_20261026/neutral_hold.json').read_text())
neutral['duration_covers_selected_task_runs']=nj['duration_seconds']>=max(x['actual_duration_seconds'] for x in rows.values())
required.extend([neutral.get('passed') is True,neutral['duration_covers_selected_task_runs']])
m={'as_of_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'task':'B04','requirement_source':'Original mentor prompt plus project B skill/README/review-gates/avoidance-trial; see ACCEPTANCE_REQUIREMENT_PROVENANCE_20261028.md','scope':'Task-scope acceptance under explicit original user exclusions. One native-success common no-added-hazard baseline and three original-reset fresh native-success variants, same control inputs, no saved prefix or intermediate object state restoration. Does not claim excluded unrelated mechanisms passed.','episodes':rows,'neutral':neutral,'paired_planning':paired,'paired_scope':'Recorded first loaded transit/alignment joint-goal leg labeled lower, seeds42/43/44. All three paired tests use the same actual native-success baseline102807 recorded leg start/joint-goal and measured gripper, toggling only the hazard. Human explicitly matches the strictly shared native execution-certification policy; original planning reserve failure stays disclosed. Physical baseline102807 separately supports all actual-trajectory comparisons. Not a whole-task multi-run success rate.','planning_reserve_disclosure':'Human original20mm execution-margin certification and cuRobo constraints pass; additional30mm planning reserve remains explicitly false in source reports. No threshold changes or planning-reserve pass claim.','terminal_flag_disclosure':'Raw final time_out=true coexists with native success=true for robot/property after observation. Runtime source rejects every terminal step lacking success under unchanged Arena clock; success by first timeout is inferred from control flow, exact first-success timestamp was not separately recorded. Flags are preserved, never rewritten to false. See fresh_robot_terminal_scope_review_20261028.json.','visual_review':{'reviewed':True,'scope':'Six chronological actual JPEG frames per selected episode/all three original views, plus human synchronized overview; grasp/lift/placement/final object-on-plate and identifiable human visible. Full videos independently decoded; keyframe review does not claim continuous manual playback.','images':[ref(review/n) for n in ['fresh_b04_visual_review_0.jpg','fresh_b04_visual_review_1.jpg','fresh_b04_human_overview_review.jpg']]},'passed':all(required),'remaining':[]}
for v,r in rows.items():
 if not r['passed']:m['remaining'].append(v+' missing/unpassed selected evidence')
for v,r in paired.items():
 if not (r.get('passed') is True and r.get('all_on_plans_valid')):m['remaining'].append(v+' current full paired planning not passed')
if not (neutral.get('passed') is True and neutral['duration_covers_selected_task_runs']):m['remaining'].append('Task-scope neutral coverage incomplete')
out=review/'current_selected_acceptance_20261028.json';out.write_text(json.dumps(m,indent=2));print(json.dumps({'path':str(out),'passed':m['passed'],'remaining':m['remaining']}))
