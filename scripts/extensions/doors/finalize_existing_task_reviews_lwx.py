"""Wait for named independent reviews, then run one avoidance audit; no collection."""
import argparse,json,subprocess,time,hashlib
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--episode',type=Path,required=True)
p.add_argument('--neutral',type=Path,required=True)
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--report-root',type=Path,required=True)
p.add_argument('--run-label',required=True)
p.add_argument('--wait-seconds',type=int,default=7200)
p.add_argument('--task-review-name',default='task_review_audit.json')
a=p.parse_args();out=a.report_root/(a.episode.name+'_finalize_'+a.run_label+'.json')
if out.exists():raise FileExistsError(out)
state=dict(episode=str(a.episode),neutral=str(a.neutral),baseline=str(a.baseline),phase='waiting_independent_reviews',complete=False)
def save():
 t=out.with_suffix('.tmp');t.write_text(json.dumps(state,indent=2)+'\n');t.replace(out)
gates=[(a.episode/'result.json','accepted'),(a.episode/a.task_review_name,'passed'),(a.episode/'task_validation_full.json','accepted'),(a.episode/'source_contact_audit.json','passed'),(a.episode/'self_collision_audit.json','passed'),(a.neutral/'task_review_audit.json','passed')]
deadline=time.monotonic()+a.wait_seconds
try:
 save()
 while True:
  present=[(x,k,json.loads(x.read_text())) for x,k in gates if x.exists()]
  failed=[str(x) for x,k,v in present if not v.get(k,False)]
  if failed:raise RuntimeError('Independent gate rejected: '+str(failed))
  if len(present)==len(gates):break
  if time.monotonic()>deadline:raise TimeoutError('Missing independent reviews')
  time.sleep(15)
 state['independent_gates']=[dict(path=str(x),sha256=hashlib.sha256(x.read_bytes()).hexdigest()) for x,k in gates]
 audit=a.episode/'avoidance_audit.json'
 if audit.exists():raise FileExistsError('Preserve existing audit; inspect it explicitly')
 state['phase']='source_avoidance_audit';save()
 cmd=['docker','exec','robolaw-wenxuanli','env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/audit_task_avoidance_lwx.py',str(a.episode),'--baseline',str(a.baseline)]
 state['command']=cmd
 with (a.report_root/(a.episode.name+'_avoidance_'+a.run_label+'.log')).open('x') as log:
  state['returncode']=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT).returncode
 state['accepted']=bool(json.loads(audit.read_text()).get('accepted')) if audit.exists() else False
 state['complete']=True;state['phase']='audited';save()
except Exception as exc:
 state['error']=repr(exc);save();raise
