"""Repeat only the neutral observation for a longer successful operation, preserving reviews."""
import argparse,json,subprocess,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--neutral',type=Path,required=True);p.add_argument('--report-root',type=Path,required=True);p.add_argument('--revision',default='02');a=p.parse_args()
m=json.loads((a.episode/'metadata.json').read_text());cp=m['controller_parameters'];r=a.report_root;statepath=r/(a.episode.name+'_extended_neutral'+a.revision+'.json');state={'complete':False,'stage':'prepared','episode':str(a.episode),'neutral':str(a.neutral)}
if statepath.exists() or a.neutral.exists():raise FileExistsError(a.neutral)
def save():statepath.write_text(json.dumps(state,indent=2))
def run(stage,script,args,gpu=False):
 state['stage']=stage;save();cmd=['docker','exec','robolaw-wenxuanli']+(['flock','/workspace/reports/skill_trials/gpu.lock'] if gpu else [])+['env','-u','ISAACSIM_ROOT_PATH','-u','ISAACLAB_PATH','-u','OMNI_SERVER','PYTHONPATH=/workspace','HEADLESS=1','OMP_NUM_THREADS=1','OPENBLAS_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/'+script,*map(str,args)]
 with (r/(a.episode.name+'_'+stage+'_extended'+a.revision+'.log')).open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
try:
 review=next(q for q in Path('configs/extensions/doors').glob('*review*.json') if hashlib.sha256(q.read_bytes()).hexdigest()==m['review_config_sha256'])
 duration=json.loads((a.episode/'result.json').read_text())['frames']*m['dt']+2.
 args=['--task-id',m['task'],'--variant',m['variant'],'--variant-config',m['variant_config'],'--output',a.neutral,'--mode','neutral','--neutral-seconds',max(40.,duration),'--max-steps',max(1200,int(duration/m['dt'])+30),'--handle-offset-y',cp['handle_offset_y'],'--handle-offset-z',cp['handle_offset_z'],'--handle-normal-offset',cp['handle_normal_offset_m'],'--grasp-roll-deg',cp['grasp_roll_deg'],'--grasp-pitch-deg',cp.get('grasp_pitch_deg',0),'--pull-lead-rad',cp['pull_lead_rad'],'--pull-steps',cp['pull_steps'],'--ik-limit-margin-rad',cp.get('ik_limit_margin_rad',0),'--validated-pregrasp','--render-warmup','--review-config',review]
 run('neutral','collect_tasks_lwx.py',args,True);run('neutral_review','audit_task_review_lwx.py',[a.neutral])
 revised=a.episode/('task_review_audit_'+a.revision+'.json');run('operation_review','audit_task_review_lwx.py',[a.episode,'--neutral',a.neutral,'--output',revised]);d=json.loads(revised.read_text())
 if not d['passed']:raise RuntimeError('Extended control did not establish all gates')
 old=a.episode/'task_review_audit.json';archive=a.episode/('task_review_audit_before_'+a.revision+'.json')
 if archive.exists():raise FileExistsError(archive)
 old.rename(archive);revised.rename(old);state.update(complete=True,stage='accepted_with_extended_control',prior_rejected_review=str(archive));save()
except Exception as exc:state['error']=repr(exc);save();raise
