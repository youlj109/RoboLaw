"""C01 source inspection then fixed-target baseline, before redesigning rejected variants."""
import json,time,subprocess
from pathlib import Path
r=Path('reports/doors/lwx_C01_revision_20261024');data=Path('datasets/doors/lwx_C01_revision_20261024');source=data/'C01_source_neutral_01';statepath=r/'C01_revision_driver01.json';state={'stage':'waiting_source','complete':False}
if statepath.exists():raise FileExistsError(statepath)
def save():statepath.write_text(json.dumps(state,indent=2))
def run(stage,script,args,gpu=False,host=False):
 state['stage']=stage;save();cmd=(['python3','-u'] if host else ['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OMP_NUM_THREADS=1','OPENBLAS_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u'])+['scripts/extensions/doors/'+script,*map(str,args)]
 with (r/(stage+'_driver01.log')).open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
try:
 save();deadline=time.monotonic()+18000
 while not (source/'result.json').exists():
  if time.monotonic()>deadline:raise TimeoutError('Source incomplete')
  time.sleep(20)
 run('source_review','audit_source_neutral_lwx.py',[source,'--review-config','configs/extensions/doors/c01_review_lwx.json'])
 if not json.loads((source/'source_neutral_audit.json').read_text())['passed']:raise RuntimeError('Source neutral rejected')
 run('contact_screen','screen_assigned_contact_lwx.py',['--metadata',source/'metadata.json','--config','configs/extensions/doors/c01_variants_lwx_01.json','--output',r/'C01_contact_screen01.json','--offsets=.18,.12,.06','--rolls=0','--normal-offsets=0,-.005','--pull-lead-rad','.03','--check-release','--check-release-settled'],True)
 run('baseline','supervise_assigned_baseline_lwx.py',['C01','--attempt','01','--screen-labels','01','--run-label','01','--report-root',r,'--data-root',data],host=True)
 state.update(stage='baseline_ready_for_source_obstruction_design',complete=True);save()
except Exception as exc:state['error']=repr(exc);save();raise
