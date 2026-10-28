#!/usr/bin/env python3
"""Run one bounded G-shape source-task attempt and verify/export its evidence."""
import argparse,fcntl,json,os,subprocess,sys,time
from pathlib import Path
from trial_evidence import acceptance
ROOT=Path(__file__).resolve().parents[3]
VARIANTS={'base':'','human':'human_default_mpfb','property':'property_vase','robot':'robot_projecting_crossbar'}
def main():
 p=argparse.ArgumentParser();p.add_argument('--trial-root',choices=['skill_trials','avoidance_trials'],default='skill_trials');p.add_argument('--episode-spec');p.add_argument('--baseline-episode');p.add_argument('--layout-json');p.add_argument('--control-profile',choices=['base','human','property','robot']);p.add_argument('--variant',choices=VARIANTS,default='base');p.add_argument('--seed',type=int,default=42);p.add_argument('--attempt',default='attempt_01');p.add_argument('--max-steps',type=int,default=0);p.add_argument('--dry-run',action='store_true');p.add_argument('--no-rgb',action='store_true');p.add_argument('--overview-camera',action='store_true');a=p.parse_args()
 out=ROOT/'datasets'/a.trial_root/'replicator'/a.variant/a.attempt;reports=ROOT/'reports'/a.trial_root/'replicator'/a.variant/a.attempt
 cmd=[sys.executable,str(ROOT/'scripts/extensions/replicator/collect.py'),'--variant',a.variant,'--output',str(out),'--seed',str(a.seed),'--planner','curobo','--device','cpu','--torch-threads','2','--physics-threads','2','--execution-checks','deferred','--camera-owner','writer','--cpu-rgb-copy','--max-steps',str(a.max_steps),'--episode-length-s','180']
 if a.control_profile:cmd+=['--control-profile',a.control_profile]
 if a.layout_json:cmd+=['--layout-json',a.layout_json]
 if a.episode_spec:cmd+=['--episode-spec',a.episode_spec]
 if VARIANTS[a.variant]:cmd+=['--obstacle_id',VARIANTS[a.variant]]
 if not a.no_rgb:cmd+=['--record-rgb','--background-rgb']
 if a.overview_camera:
  if a.no_rgb:raise ValueError('Overview requires RGB collection')
  cmd+=['--overview-camera']
 print(json.dumps(dict(task='kitchen_bench_replicator_g_shape_mustard_bowl',variant=a.variant,command=cmd,output=str(out)),indent=2),flush=True)
 if a.dry_run:return 0
 if out.exists():raise RuntimeError('Output exists; use a new attempt ID to preserve prior evidence')
 reports.mkdir(parents=True,exist_ok=True);started=time.time()
 lock=ROOT/'reports/skill_trials/gpu.lock';lock.parent.mkdir(parents=True,exist_ok=True)
 with lock.open('a') as handle:
  fcntl.flock(handle,fcntl.LOCK_EX)
  with (reports/'collection.log').open('w') as log:
   result=subprocess.run(cmd,cwd=ROOT,env={**os.environ,'HEADLESS':'1'},stdout=log,stderr=subprocess.STDOUT,timeout=2400)
  fcntl.flock(handle,fcntl.LOCK_UN)
 statuses={'collection':result.returncode}
 if (out/'result.json').exists():
  checks=[('source_relations',[sys.executable,'scripts/extensions/replicator/audit_source_relation.py','--episode',str(out)])]
  if a.variant!='base':checks.append(('fixed_base',[sys.executable,'scripts/extensions/replicator/audit_fixed_base.py','--episode',str(out),'--report',str(reports/'fixed_base_hazard_audit.json')]))
  for name,command in checks:
   with (reports/(name+'.log')).open('w') as log:statuses[name]=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=1200).returncode
 if (out/'trajectory.h5').exists() and not a.no_rgb:
  steps=[('export',[sys.executable,'scripts/export_collection_previews.py',str(out),'--report',str(reports/'videos.json')]),('validation',[sys.executable,'scripts/validate_collection.py',str(out),'--require-rgb','--require-previews','--report',str(reports/'validation.json')])]
  if a.variant!='base':
   steps.append(('audit',[sys.executable,'scripts/extensions/replicator/audit.py',str(out),'--report',str(reports/'audit.json')]))
  steps.append(('self_collision',[sys.executable,'scripts/extensions/replicator/audit_self_collision.py',str(out)]))
  if a.overview_camera:steps.append(('overview',[sys.executable,'scripts/extensions/replicator/validate_overview.py',str(out)]))
  if a.trial_root=='avoidance_trials' and a.variant!='base':
   comparison=[sys.executable,'scripts/extensions/replicator/audit_avoidance.py',str(out)]
   if a.baseline_episode:comparison+=['--baseline',a.baseline_episode]
   background=[sys.executable,'scripts/extensions/replicator/audit_background_consistency.py',str(out)]
   if a.baseline_episode:background+=['--baseline',a.baseline_episode]
   steps.append(('background_consistency',background))
   steps.append(('avoidance',comparison))
  for name,command in steps:
   with (reports/(name+'.log')).open('w') as log:statuses[name]=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=1200).returncode
 summary=dict(task='kitchen_bench_replicator_g_shape_mustard_bowl',variant=a.variant,attempt=a.attempt,exit_codes=statuses,wall_seconds=time.time()-started,fixed_only=True,**acceptance(out,reports,statuses,require_rgb=not a.no_rgb))
 (reports/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
 return int(not summary['passed'])
if __name__=='__main__':raise SystemExit(main())
