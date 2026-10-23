"""Host: start one matched baseline/control after its own source geometry screen."""
import argparse,json,time,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('label',choices=['C01','C02','C03','C04','C05','C06','C07','C08']);p.add_argument('--screen-labels',default='01');p.add_argument('--attempt',default='01');p.add_argument('--run-label',default='01');p.add_argument('--config',type=Path);p.add_argument('--report-root',type=Path);p.add_argument('--data-root',type=Path);p.add_argument('--review-config',type=Path);p.add_argument('--release-retreat-m',type=float,default=0.);p.add_argument('--release-tilt-deg',type=float,default=0.);p.add_argument('--pull-steps',type=int,default=600);a=p.parse_args();label=a.label
if not 1<=a.pull_steps<=1200 or not 0<=a.release_retreat_m<=.04 or not -60<=a.release_tilt_deg<=60:p.error('Policy revision outside existing collector bounds')
if a.release_tilt_deg and not a.release_retreat_m:p.error('Release tilt needs a nonzero retreat')
root=a.report_root or Path('reports/doors/lwx_C04_C08_20261024');data=a.data_root or Path('datasets/doors/lwx_C04_C08_20261024');statefile=root/(label+'_baseline_driver'+a.run_label+'.json')
if statefile.exists():raise FileExistsError(statefile)
state={'label':label,'stage':'waiting_for_contact_screen','history':[],'complete':False};deadline=time.monotonic()+18000
def save():statefile.write_text(json.dumps(state,indent=2)+'\n')
def run(name,cmd):
 state['stage']=name;save()
 with (root/(label+'_'+name+'_driver'+a.run_label+'.log')).open('x') as f:code=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 state['history'].append({'stage':name,'exit_code':code});save()
 if code:raise RuntimeError(name+' exited '+str(code))
try:
 save()
 while True:
  screen={}
  for screen_label in a.screen_labels.split(','):
   path=root/(label+'_contact_screen'+screen_label+'.json')
   try:current=json.loads(path.read_text())
   except (FileNotFoundError,json.JSONDecodeError):continue
   if current.get('first_feasible'):
    screen=current;state['selected_screen']=str(path);break
  if screen.get('first_feasible'):break
  attempted=[root/(label+'_contact_screen'+tag+'.json') for tag in a.screen_labels.split(',')]
  if all(q.exists() and json.loads(q.read_text()).get('search_complete') for q in attempted):raise RuntimeError('All offline candidates rejected; diagnose before a new screen')
  if time.monotonic()>deadline:raise TimeoutError('No fully feasible offline grasp candidate; inspect retained candidates')
  time.sleep(20)
 candidate=screen['first_feasible'];config=a.config or Path('configs/extensions/doors')/(label.lower()+'_variants_lwx_01.json');task=json.loads(config.read_text()).get('_task_scope',screen['task']);state['chosen_candidate']=candidate;save()
 for mode,name in [('collect',label+'_base_'+a.attempt),('neutral',label+'_base_neutral_'+a.attempt)]:
  cmd=['docker','exec','robolaw-wenxuanli','flock','/workspace/reports/skill_trials/gpu.lock','env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OMP_NUM_THREADS=2','OPENBLAS_NUM_THREADS=2','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/collect_tasks_lwx.py','--task-id',task,'--variant','base','--variant-config',str(config),'--output',str(data/name),'--mode',mode,'--handle-offset-z',str(candidate['offset_z']),'--handle-normal-offset',str(candidate['normal_offset_m']),'--grasp-roll-deg',str(candidate['roll_deg']),'--grasp-pitch-deg',str(candidate.get('pitch_deg',0.)),'--pull-lead-rad',str(screen['pull_lead_rad']),'--pull-steps',str(a.pull_steps),'--max-steps',str(max(1000,a.pull_steps+400)),'--validated-pregrasp','--render-warmup','--ik-limit-margin-rad',str(screen['ik_command_margin_rad']),'--review-config',str(a.review_config or Path('configs/extensions/doors')/(label.lower()+'_review_lwx.json'))]
  if a.release_retreat_m:cmd+=['--release-retreat-m',str(a.release_retreat_m)]
  if a.release_tilt_deg:cmd+=['--release-tilt-deg',str(a.release_tilt_deg)]
  if mode=='neutral':
   operation=json.loads((data/(label+'_base_'+a.attempt)/'result.json').read_text());cmd+=['--neutral-seconds',str(max(40.,operation['frames']/15.+2.))]
  run(name,cmd)
 run('independent_reviews',['python3','-u','scripts/extensions/doors/supervise_task_reviews_lwx.py',str(data/(label+'_base_'+a.attempt)),'--neutral',str(data/(label+'_base_neutral_'+a.attempt)),'--report-root',str(root),'--run-label','02'])
 gates={}
 for filename,key in [('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]:
  gates[filename]=json.loads((data/(label+'_base_'+a.attempt)/filename).read_text()).get(key,False)
 state['gates']=gates;state['baseline_audits_passed']=all(gates.values());state['complete']=True;state['stage']='baseline_reviewed';save()
except Exception as exc:
 state['error']=repr(exc);save();raise
