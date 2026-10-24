"""One safely planned physical pilot for user visual review; no automatic visible-detour acceptance."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
from matched_scene_lwx import require_matching_scene
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--paired',type=Path,required=True);p.add_argument('--attempt',required=True);a=p.parse_args();a.necessity_mode='source_intersection';variant='robot';r=Path('reports/doors/lwx_C07_obvious_demo_20261025');data=Path('datasets/doors/lwx_C07_obvious_demo_20261025');ep=data/('C07_robot_'+a.attempt);neutral=data/('C07_robot_neutral_'+a.attempt);statepath=r/(ep.name+'_campaign01.json');cfg=a.config;m=json.loads((a.baseline/'metadata.json').read_text());cp=m['controller_parameters'];paired=json.loads(a.paired.read_text())
if statepath.exists() or ep.exists():raise FileExistsError(ep)
require_matching_scene(m,cfg)
assert paired['config_sha256']==hashlib.sha256(cfg.read_bytes()).hexdigest()
assert paired['baseline_sha256']==hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest()
assert paired['controller_parameters']==cp
assert paired['stable_policy_necessity_in_tested_seeds'] and paired['fixed_endpoint_rerouting_supported']
assert all(any(v.get('intersecting',False) for v in pair['off']['risk_with_added_geometry']['violations']) for pair in paired['pairs'])
required=[('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
assert all(json.loads((a.baseline/f).read_text()).get(k,False) for f,k in required)
review=next(p for p in [Path('configs/extensions/doors/c07_review_lwx.json'),Path('configs/extensions/doors/review_gates_lwx.json')] if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==m['review_config_sha256'])
state={'stage':'prepared','complete':False,'baseline':str(a.baseline),'config':str(cfg),'paired':str(a.paired),'physical_trial_budget':1,'visual_review_pending':True,'automatic_visible_detour_threshold_passed':False,'scope':'Safe physical pilot for manual visual review. Three-seed maximum TCP deviations approximately .0952/.1005/.0991m; strict all-seed .10m automatic discovery screen was not met. Safety gates remain unchanged.'}
def save():statepath.write_text(json.dumps(state,indent=2)+'\n')
def run(name,script,args,gpu=False,allow_failure=False):
 state['stage']=name;save();cmd=['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/'+script,*map(str,args)]
 with (r/(ep.name+'_'+name+'_campaign01.log')).open('x') as f:code=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 if code and not allow_failure:raise RuntimeError(name+' exited '+str(code))
 return code
try:
 save();approach=r/(ep.name+'_approach01.json');run('approach_screen','diagnose_paired_approach_lwx.py',['--baseline',a.baseline,'--config',cfg,'--variant',variant,'--paired',a.paired,'--output',approach]);assert json.loads(approach.read_text())['all_predicted_approaches_discovery_eligible']
 pull=r/(ep.name+'_pull_proxy01.json');run('pull_proxy','diagnose_paired_pull_proxy_lwx.py',['--baseline',a.baseline,'--config',cfg,'--variant',variant,'--paired',a.paired,'--approach',approach,'--output',pull]);assert json.loads(pull.read_text())['all_projected_pulls_discovery_eligible']
 for mode,out in [('collect',ep),('neutral',neutral)]:
  args=['--task-id',m['task'],'--variant',variant,'--variant-config',cfg,'--output',out,'--mode',mode,'--handle-offset-y',cp['handle_offset_y'],'--handle-offset-z',cp['handle_offset_z'],'--handle-normal-offset',cp['handle_normal_offset_m'],'--grasp-roll-deg',cp['grasp_roll_deg'],'--grasp-pitch-deg',cp.get('grasp_pitch_deg',0.),'--pull-lead-rad',cp['pull_lead_rad'],'--pull-steps',cp['pull_steps'],'--ik-limit-margin-rad',cp.get('ik_limit_margin_rad',0.),'--max-steps','1200','--validated-pregrasp','--render-warmup','--review-config',review];
  if cp.get('planner_hazard_extra_padding_m'):args+=['--planner-hazard-extra-padding-m',str(cp['planner_hazard_extra_padding_m'])]
  if cp.get('release_retreat_m'):args+=['--release-retreat-m',str(cp['release_retreat_m'])]
  if cp.get('release_tilt_deg'):args+=['--release-tilt-deg',str(cp['release_tilt_deg'])]
  if cp.get('opening_reserve_rad'):args+=['--opening-reserve-rad',str(cp['opening_reserve_rad'])]
  if cp.get('release_command_policy','full_pose')!='full_pose':args+=['--release-command-policy',cp['release_command_policy']]
  if cp.get('release_settle_steps'):args+=['--release-settle-steps',str(cp['release_settle_steps'])]
  if cp.get('pregrasp_shortcut_policy','none')!='none':args+=['--pregrasp-shortcut-policy',cp['pregrasp_shortcut_policy']]
  if cp.get('pregrasp_settle_steps'):args+=['--pregrasp-settle-steps',str(cp['pregrasp_settle_steps'])]
  if cp.get('release_retreat_profile'):args+=['--release-retreat-profile',cp['release_retreat_profile']]
  if mode=='neutral':args+=['--neutral-seconds',str(max(40.,json.loads((ep/'result.json').read_text())['frames']*m['dt']+2.))]
  run(mode,'collect_tasks_lwx.py',args,gpu=True)
 state['stage']='independent_reviews';save()
 with (r/(ep.name+'_reviews_campaign01.log')).open('x') as f:subprocess.run(['python3','-u','scripts/extensions/doors/supervise_task_reviews_lwx.py',str(ep),'--neutral',str(neutral),'--report-root',str(r),'--run-label','02'],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
 gates={f:bool(json.loads((ep/f).read_text()).get(k,False)) for f,k in required};state['variant_gates']=gates;save()
 if not all(gates.values()):raise RuntimeError('Physical variant independent gates rejected')
 run('avoidance','audit_task_avoidance_lwx.py',[ep,'--baseline',a.baseline]);run('comparison','make_task_comparison_lwx.py',[ep,'--baseline',a.baseline,'--run-label','03']);run('source_risk_illustration','make_source_risk_illustration_lwx.py',[ep,'--baseline',a.baseline,'--output',r/(ep.name+'_source_risk01.jpg')]+(['--necessity-mode',a.necessity_mode] if a.necessity_mode!='source_intersection' else []));state.update(stage='ready_for_visual_review',complete=True,visual_review_pending=True,episode=str(ep),neutral=str(neutral));save()
except Exception as exc:state['error']=repr(exc);save();raise
