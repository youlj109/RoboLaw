"""At most three offline candidates, then one real variant/control and full audits."""
import argparse,json,subprocess,hashlib
from pathlib import Path
from matched_scene_lwx import require_matching_scene
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--variant',required=True,choices=['human','property','robot']);p.add_argument('--search',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--attempt',default='01');p.add_argument('--report-root',type=Path,default=Path('reports/doors/lwx_C04_C08_20261024'));p.add_argument('--data-root',type=Path,default=Path('datasets/doors/lwx_C04_C08_20261024'));p.add_argument('--config',type=Path);p.add_argument('--screen-contact-approach',action='store_true');a=p.parse_args();label=a.label;variant=a.variant;r=a.report_root;data=a.data_root;ep=data/(label+'_'+variant+'_'+a.attempt);neutral=data/(label+'_'+variant+'_neutral_'+a.attempt);statepath=r/(ep.name+'_campaign01.json');config=a.config or Path(json.loads((a.baseline/'metadata.json').read_text())['variant_config'])
if statepath.exists() or ep.exists():raise FileExistsError(ep)
state={'stage':'prepared','baseline':str(a.baseline),'search':str(a.search),'search_sha256':hashlib.sha256(a.search.read_bytes()).hexdigest(),'offline_candidate_budget':3,'physical_trial_budget':1,'small_detour_limits':{'max_joint_length_ratio':1.6,'max_added_tcp_length_m':.15},'complete':False,'candidates':[]};required=[('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
def save():statepath.write_text(json.dumps(state,indent=2)+'\n')
def run(name,script,args,gpu=False,allow_failure=False):
 state['stage']=name;save();cmd=['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/'+script,*map(str,args)]
 with (r/(ep.name+'_'+name+'_campaign01.log')).open('x') as f:code=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 if code and not allow_failure:raise RuntimeError(name+' exited '+str(code))
 return code
try:
 save();gates={f:bool(json.loads((a.baseline/f).read_text()).get(k,False)) for f,k in required};state['baseline_gates']=gates
 if not all(gates.values()):raise RuntimeError('Baseline independent gates failed')
 m=json.loads((a.baseline/'metadata.json').read_text());cp=m['controller_parameters'];review=next((p for p in [Path('configs/extensions/doors')/(label.lower()+'_review_lwx.json'),Path('configs/extensions/doors/review_gates_lwx.json')] if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==m['review_config_sha256']),None)
 if review is None:raise RuntimeError('Cannot recover exact predeclared baseline review config')
 rank=r/(ep.name+'_candidate_rank01.json');run('rank','rank_assigned_candidates_lwx.py',['--baseline',a.baseline,'--config',config,'--variant',variant,'--search',a.search,'--output',rank]);ranking=json.loads(rank.read_text());safe=[x for x in ranking if x['initial_sphere']['slack_m']>.003][:3];choices=[]
 for ri,item in enumerate(safe):
  idx=item['candidate'];cfg=Path('configs/extensions/doors')/(label.lower()+'_'+variant+'_candidate_'+a.attempt+'_'+str(idx).zfill(2)+'.json');paired=r/(ep.name+'_candidate_'+str(idx).zfill(2)+'_paired01');run('prepare_'+str(idx),'prepare_task_candidate_lwx.py',['--config',config,'--search',a.search,'--variant',variant,'--index',idx,'--output',cfg]);require_matching_scene(m,cfg);code=run('paired_'+str(idx),'paired_tasks_lwx.py',['--baseline',a.baseline,'--config',cfg,'--variant',variant,'--output',paired],gpu=True,allow_failure=True)
  entry={'candidate_index':idx,'config':str(cfg),'paired_report':str(paired/'report.json'),'exit_code':code,'eligible':False};state['candidates'].append(entry)
  if (paired/'report.json').exists():
   report=json.loads((paired/'report.json').read_text());pairs=report['pairs'];actual_hits=all(any(v.get('intersecting',False) for v in pair['off'].get('risk_with_added_geometry',{}).get('violations',[])) for pair in pairs);ratios=[];extra=[]
   for pair in pairs:
    if not (pair['off'].get('success') and pair['on'].get('success')):ratios.append(999.);extra.append(999.);continue
    off=pair['off']['path_metrics'];on=pair['on']['path_metrics'];ratios.append(on['joint_l2_length_rad']/max(off['joint_l2_length_rad'],1e-9));extra.append(on['tcp_length_m']-off['tcp_length_m'])
   entry.update(actual_source_intersections_all_seeds=actual_hits,max_joint_length_ratio=max(ratios),max_added_tcp_length_m=max(extra));entry['eligible']=bool(not code and report['stable_policy_necessity_in_tested_seeds'] and report['fixed_endpoint_rerouting_supported'] and actual_hits and max(ratios)<=1.6 and max(extra)<=.15)
   if entry['eligible'] and a.screen_contact_approach:
    diagnostic=r/(ep.name+'_candidate_'+str(idx).zfill(2)+'_approach01.json')
    run('approach_screen_'+str(idx),'diagnose_paired_approach_lwx.py',['--baseline',a.baseline,'--config',cfg,'--variant',variant,'--paired',paired/'report.json','--output',diagnostic])
    entry['contact_approach_diagnostic']=str(diagnostic)
    entry['eligible']=bool(json.loads(diagnostic.read_text())['all_predicted_approaches_discovery_eligible'])
   if entry['eligible']:choices.append(entry)
  save()
  if entry['eligible'] and entry['max_added_tcp_length_m']<=.05 and entry['max_joint_length_ratio']<=1.15:
   state['selection_rule']='First eligible candidate with <=.05m added TCP length and <=1.15 joint ratio; otherwise minimum added length among at most three eligible candidates';save();break
 if not choices:raise RuntimeError('No candidate established actual obstruction and a small safe reroute across all seeds')
 choice=min(choices,key=lambda x:(x['max_added_tcp_length_m'],x['max_joint_length_ratio']));state['selected_candidate']=choice;cfg=Path(choice['config']);save()
 for mode,out in [('collect',ep),('neutral',neutral)]:
  args=['--task-id',m['task'],'--variant',variant,'--variant-config',cfg,'--output',out,'--mode',mode,'--handle-offset-y',cp['handle_offset_y'],'--handle-offset-z',cp['handle_offset_z'],'--handle-normal-offset',cp['handle_normal_offset_m'],'--grasp-roll-deg',cp['grasp_roll_deg'],'--grasp-pitch-deg',cp.get('grasp_pitch_deg',0.),'--pull-lead-rad',cp['pull_lead_rad'],'--pull-steps',cp['pull_steps'],'--ik-limit-margin-rad',cp.get('ik_limit_margin_rad',0.),'--max-steps','1200','--validated-pregrasp','--render-warmup','--review-config',review];
  if cp.get('planner_hazard_extra_padding_m'):args+=['--planner-hazard-extra-padding-m',str(cp['planner_hazard_extra_padding_m'])]
  if cp.get('release_retreat_m'):args+=['--release-retreat-m',str(cp['release_retreat_m'])]
  if cp.get('release_tilt_deg'):args+=['--release-tilt-deg',str(cp['release_tilt_deg'])]
  if cp.get('opening_reserve_rad'):args+=['--opening-reserve-rad',str(cp['opening_reserve_rad'])]
  if cp.get('release_command_policy','full_pose')!='full_pose':args+=['--release-command-policy',cp['release_command_policy']]
  if cp.get('release_settle_steps'):args+=['--release-settle-steps',str(cp['release_settle_steps'])]
  if cp.get('pregrasp_shortcut_policy','none')!='none':args+=['--pregrasp-shortcut-policy',cp['pregrasp_shortcut_policy']]
  if cp.get('release_retreat_profile'):args+=['--release-retreat-profile',cp['release_retreat_profile']]
  if mode=='neutral':args+=['--neutral-seconds',str(max(40.,json.loads((ep/'result.json').read_text())['frames']*m['dt']+2.))]
  run(mode,'collect_tasks_lwx.py',args,gpu=True)
 state['stage']='independent_reviews';save()
 with (r/(ep.name+'_reviews_campaign01.log')).open('x') as f:subprocess.run(['python3','-u','scripts/extensions/doors/supervise_task_reviews_lwx.py',str(ep),'--neutral',str(neutral),'--report-root',str(r),'--run-label','02'],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
 gates={f:bool(json.loads((ep/f).read_text()).get(k,False)) for f,k in required};state['variant_gates']=gates;save()
 if not all(gates.values()):raise RuntimeError('Physical variant independent gates rejected')
 run('avoidance','audit_task_avoidance_lwx.py',[ep,'--baseline',a.baseline]);run('comparison','make_task_comparison_lwx.py',[ep,'--baseline',a.baseline,'--run-label','03']);run('source_risk_illustration','make_source_risk_illustration_lwx.py',[ep,'--baseline',a.baseline,'--output',r/(ep.name+'_source_risk01.jpg')]);state.update(stage='ready_for_visual_review',complete=True,visual_review_pending=True,episode=str(ep),neutral=str(neutral));save()
except Exception as exc:state['error']=repr(exc);save();raise
