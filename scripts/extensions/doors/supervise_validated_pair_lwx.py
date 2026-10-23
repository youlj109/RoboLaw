"""One current-policy paired revision followed by one real operation/control."""
import argparse,json,subprocess,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--variant',required=True);p.add_argument('--label',required=True);p.add_argument('--attempt',required=True);p.add_argument('--report-root',type=Path,required=True);p.add_argument('--data-root',type=Path,required=True);a=p.parse_args()
r=a.report_root;ep=a.data_root/(a.label+'_'+a.variant+'_'+a.attempt);neutral=a.data_root/(a.label+'_'+a.variant+'_neutral_'+a.attempt);paired=r/(ep.name+'_paired01');sf=r/(ep.name+'_validated_pair01.json');state={'stage':'prepared','complete':False,'physical_trial_budget':1,'offline_pair_budget':1,'baseline':str(a.baseline),'paired':str(paired/'report.json'),'config':str(a.config)}
if sf.exists() or ep.exists():raise FileExistsError(ep)
def save():sf.write_text(json.dumps(state,indent=2))
def run(stage,script,args,gpu=False,host=False):
 state['stage']=stage;save();cmd=(['python3','-u'] if host else ['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OMP_NUM_THREADS=1','OPENBLAS_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u'])+['scripts/extensions/doors/'+script,*map(str,args)]
 with (r/(ep.name+'_'+stage+'_validatedpair01.log')).open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
required=[('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
try:
 save()
 if not all(json.loads((a.baseline/f).read_text()).get(k,False) for f,k in required):raise RuntimeError('Baseline gates failed')
 m=json.loads((a.baseline/'metadata.json').read_text());cp=m['controller_parameters'];review=next(q for q in Path('configs/extensions/doors').glob('*review*.json') if hashlib.sha256(q.read_bytes()).hexdigest()==m['review_config_sha256'])
 run('paired','paired_tasks_lwx.py',['--baseline',a.baseline,'--config',a.config,'--variant',a.variant,'--output',paired],True);pairs=json.loads((paired/'report.json').read_text());ratios=[];extras=[]
 for pair in pairs['pairs']:
  if not (pair['off'].get('success') and pair['on'].get('success') and any(v.get('intersecting') for v in pair['off']['risk_with_added_geometry']['violations'])):raise RuntimeError('No actual obstruction and successful on path in every seed')
  ratios.append(pair['on']['path_metrics']['joint_l2_length_rad']/pair['off']['path_metrics']['joint_l2_length_rad']);extras.append(pair['on']['path_metrics']['tcp_length_m']-pair['off']['path_metrics']['tcp_length_m'])
 state.update(max_joint_length_ratio=max(ratios),max_added_tcp_length_m=max(extras));save()
 if not (pairs['stable_policy_necessity_in_tested_seeds'] and pairs['fixed_endpoint_rerouting_supported'] and max(ratios)<=1.6 and max(extras)<=.15):raise RuntimeError('Small fixed-target detour gates rejected')
 for mode,out in [('collect',ep),('neutral',neutral)]:
  args=['--task-id',m['task'],'--variant',a.variant,'--variant-config',a.config,'--output',out,'--mode',mode,'--handle-offset-y',cp['handle_offset_y'],'--handle-offset-z',cp['handle_offset_z'],'--handle-normal-offset',cp['handle_normal_offset_m'],'--grasp-roll-deg',cp['grasp_roll_deg'],'--grasp-pitch-deg',cp.get('grasp_pitch_deg',0.),'--pull-lead-rad',cp['pull_lead_rad'],'--pull-steps',cp['pull_steps'],'--ik-limit-margin-rad',cp.get('ik_limit_margin_rad',0.),'--max-steps','1500','--validated-pregrasp','--render-warmup','--review-config',review]
  if mode=='neutral':args+=['--neutral-seconds',max(40.,json.loads((ep/'result.json').read_text())['frames']*m['dt']+2.)]
  run(mode,'collect_tasks_lwx.py',args,True)
 run('reviews','supervise_task_reviews_lwx.py',[ep,'--neutral',neutral,'--report-root',r,'--run-label','02'],host=True);state['gates']={f:bool(json.loads((ep/f).read_text()).get(k,False)) for f,k in required};save()
 if not all(state['gates'].values()):raise RuntimeError('Independent operation gates rejected')
 run('avoidance','audit_task_avoidance_lwx.py',[ep,'--baseline',a.baseline]);run('comparison','make_task_comparison_lwx.py',[ep,'--baseline',a.baseline,'--run-label','03']);state.update(stage='ready_for_visual_review',complete=True,visual_review_pending=True,episode=str(ep),neutral=str(neutral));save()
except Exception as exc:state['error']=repr(exc);save();raise
