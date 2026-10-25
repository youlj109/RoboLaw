"""Host supervisor for already authorized one-trial audits; preserve each report."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--neutral',type=Path,required=True);p.add_argument('--wait-seconds',type=int,default=10800);p.add_argument('--run-label',default='02');p.add_argument('--report-root',type=Path,default=Path('reports/doors/lwx_C02_C03_20261023'));p.add_argument('--task-review-name',default='task_review_audit.json');a=p.parse_args()
root=a.report_root;state_path=root/(a.episode.name+'_review_pipeline_'+a.run_label+'.json')
if state_path.exists():raise FileExistsError(state_path)
state={'episode':str(a.episode),'neutral':str(a.neutral),'stages':[],'complete':False};deadline=time.monotonic()+a.wait_seconds
prefix=['docker','exec','robolaw-wenxuanli'];env=['env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python']
def save():state_path.write_text(json.dumps(state,indent=2)+'\n')
def wait_result(ep):
 while not (ep/'result.json').exists():
  if time.monotonic()>deadline:raise TimeoutError(str(ep))
  time.sleep(20)
 result=json.loads((ep/'result.json').read_text())
 if not (result.get('neutral_completed') and result.get('failure') is None if result.get('mode')=='neutral' else result.get('accepted')):raise RuntimeError('Collector rejected '+str(ep)+': '+str(result.get('failure')))
def run(name,script,args,report,gpu=False):
 if report.exists():state['stages'].append({'name':name,'report':str(report),'existing':True});save();return
 cmd=prefix+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+env+['scripts/extensions/doors/'+script,*map(str,args)]
 logfile=root/(a.episode.name+'_'+name+'_pipeline'+a.run_label+'.log')
 with logfile.open('x') as f:code=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 state['stages'].append({'name':name,'exit_code':code,'report':str(report),'report_exists':report.exists(),'log':str(logfile)});save();print(name,code,flush=True)
try:
 save();wait_result(a.episode);wait_result(a.neutral)
 run('neutral_review','audit_task_review_lwx.py',[a.neutral],a.neutral/'task_review_audit.json')
 run('task_review','audit_task_review_lwx.py',[a.episode,'--neutral',a.neutral,'--output',a.episode/a.task_review_name],a.episode/a.task_review_name)
 run('full_validation','validate_task_trial_lwx.py',[a.episode],a.episode/'task_validation_full.json')
 run('source_contact','audit_task_contact_lwx.py',[a.episode],a.episode/'source_contact_audit.json')
 run('visual_extract','review_task_success_lwx.py',[a.episode,'--output-dir',root],root/(a.episode.name+'_visual.jpg'))
 run('self_collision','audit_self_collision.py',[a.episode],a.episode/'self_collision_audit.json',gpu=True)
 state['complete']=True;save()
except Exception as exc:
 state['error']=repr(exc);save();raise
