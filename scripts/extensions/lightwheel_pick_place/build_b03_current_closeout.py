"""Aggregate selected current B03 evidence without modifying any source report."""
import datetime,hashlib,json
from pathlib import Path
root=Path('/root/autodl-tmp/RoboLaw-lgd/datasets/skill_trials/lightwheel_pick_place')
review=Path('/root/autodl-tmp/B03-review')
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
selected={'human':('human_lgd_b03_fixedscene_20261026T092630Z','base_lgd_b03_fixedscene_20261026T080313Z','avoidance_audit.json'),'robot':('robot_lgd_b03_standard_stage_property_20261028T074115Z','base_lgd_b03_matched_property_20261028T063733Z','avoidance_audit.fresh_matched.policy_review.json'),'property':('property_lgd_b03_standard_stage_property_20261028T082633Z','base_lgd_b03_matched_property_20261028T063733Z','avoidance_audit.fresh_matched.json')}
rows={};required=[]
for v,(episode,baseline,audit) in selected.items():
 p=root/episode;b=root/baseline;r=json.loads((p/'result.json').read_text());br=json.loads((b/'result.json').read_text())
 core={n:report(p/n) for n in ['validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','fixed_base_hazard_audit.json']}
 basecore={n:report(b/n) for n in ['validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json']}
 a=report(p/audit,'accepted');videos={n:ref(p/(n+'.mp4')) for n in ['external_camera','external_camera_2','wrist_camera']+(['overview_camera'] if v=='human' else [])}
 validation=json.loads((p/'validation.json').read_text());decoded=validation.get('checks',{}).get('videos') is True
 plots={suffix:ref(review/('fresh_'+v+'_final')/('path_comparison.'+suffix)) for suffix in ['png','pdf']}
 passed=bool(r.get('success') and br.get('success') and a.get('accepted') is True and all(x.get('passed') is True for x in list(core.values())+list(basecore.values())) and all(x['exists'] for x in videos.values()) and decoded and all(x['exists'] for x in plots.values()))
 rows[v]={'episode':str(p),'baseline':str(b),'result':ref(p/'result.json'),'baseline_result':ref(b/'result.json'),'native_success':r.get('success'),'baseline_native_success':br.get('success'),'core':core,'baseline_core':basecore,'comparison':a,'videos':videos,'complete_video_decode_verified':decoded,'trajectory':ref(p/'trajectory.h5'),'images':ref(p/'images.h5'),'source_scene':ref(p/'scene.usda'),'path_comparison':plots,'passed':passed}
 required.append(passed)
neutral=report(review/'task_neutral_120_20261026/neutral_hold.json');oldpaired=report(review/'paired_transportreview_080313_leg3_20261027/paired_planning.json');freshpaired=report(review/'paired_fresh_property_controller_leg3_20261028/paired_planning.json')
required.extend([neutral.get('passed') is True,oldpaired.get('passed') is True,freshpaired.get('passed') is True])
m={'as_of_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'task':'B03','requirement_source':'Mentor prompt and student_skills/robolaw-lightwheel-pick-place/{SKILL.md,references/review-gates.md,references/avoidance-trial.md}; see ACCEPTANCE_REQUIREMENT_PROVENANCE_20261028.md','scope':'Original task plus three variants. Each variant links its own matching successful physical baseline. Human retains existing accepted evidence; robot/property use fresh common controller. Native success, strict actual comparison, neutral observation and paired-planning scopes remain separate. User-approved unrelated-background exclusions remain explicit in original neutral report.','episodes':rows,'neutral':neutral,'human_primary_paired_planning':oldpaired,'fresh_robot_property_primary_paired_planning':freshpaired,'paired_scope':'First free-transfer segment, seeds42/43/44 only; not all paths, whole-task planning success rate or 200-run data collection.','visual_keyframe_review':{'image':ref(review/'fresh_robot_property_visual_review.jpg'),'reviewed':True,'scope':'Six chronological actual JPEG frames per new episode, all three views; first/second pick progression and both fruit destination visible. Full MP4 decode separately validated; keyframe review does not claim full playback.'},'passed':all(required),'remaining':[]}
if not freshpaired.get('passed'):m['remaining'].append('Current matching-controller primary paired planning is pending or failed; inspect original report.')
for v,r in rows.items():
 if not r['passed']:m['remaining'].append(v+' selected evidence has missing/unpassed requirement; inspect row.')
if not neutral.get('passed'):m['remaining'].append('Neutral task-scope report not passed.')
out=review/'current_selected_acceptance_20261028.json';out.write_text(json.dumps(m,indent=2));print(json.dumps({'path':str(out),'passed':m['passed'],'remaining':m['remaining']}))
