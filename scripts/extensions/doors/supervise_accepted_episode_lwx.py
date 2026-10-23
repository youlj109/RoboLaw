"""Finish audits after a replacement neutral; never repeat accepted physical action."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--neutral',type=Path,required=True);p.add_argument('--neutral-state',type=Path,required=True);p.add_argument('--report-root',type=Path,required=True);p.add_argument('--run-label',required=True);a=p.parse_args();r=a.report_root;ep=a.episode;sf=r/(ep.name+'_recovered_'+a.run_label+'.json');state={'complete':False,'stage':'waiting_neutral_replacement'}
if sf.exists():raise FileExistsError(sf)
def save():sf.write_text(json.dumps(state,indent=2)+'\n')
def run(name,script,args,gpu=False,host=False):
 state['stage']=name;save()
 prefix=['python3','-u'] if host else ['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u']
 with (r/(ep.name+'_'+name+'_recovered'+a.run_label+'.log')).open('x') as f:subprocess.run(prefix+['scripts/extensions/doors/'+script,*map(str,args)],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
try:
 save();deadline=time.monotonic()+18000
 if not json.loads((ep/'result.json').read_text())['accepted']:raise RuntimeError('No accepted original operation')
 while True:
  d=json.loads(a.neutral_state.read_text()) if a.neutral_state.exists() else {}
  if d.get('error'):raise RuntimeError('Replacement neutral rejected: '+d['error'])
  if d.get('complete'):break
  if time.monotonic()>deadline:raise TimeoutError('Replacement neutral not ready')
  time.sleep(20)
 run('reviews','supervise_task_reviews_lwx.py',[ep,'--neutral',a.neutral,'--report-root',r,'--run-label',a.run_label],host=True)
 required=[('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
 state['gates']={n:bool(json.loads((ep/n).read_text()).get(k)) for n,k in required};save()
 if not all(state['gates'].values()):raise RuntimeError('Independent gates rejected')
 for name,script,args,output in [('avoidance','audit_task_avoidance_lwx.py',[ep,'--baseline',a.baseline],ep/'avoidance_audit.json'),('comparison','make_task_comparison_lwx.py',[ep,'--baseline',a.baseline,'--run-label','03'],ep/'baseline_comparison_external_camera_2_03.json'),('source_risk','make_source_risk_illustration_lwx.py',[ep,'--baseline',a.baseline,'--output',r/(ep.name+'_source_risk01.jpg')],r/(ep.name+'_source_risk01.json'))]:
  if not output.exists():run(name,script,args)
 state.update(complete=True,stage='ready_for_visual_review',episode=str(ep),neutral=str(a.neutral));save()
except Exception as exc:state['error']=repr(exc);save();raise
