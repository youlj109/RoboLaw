#!/usr/bin/env python3
"""Plan exactly one frozen candidate per remaining failure, then bind execution.

Recipes may be supplied in independent groups. Completed or interrupted scene
attempts are never rerun; --finalize freezes the 48-entry physical test plan.
"""
import argparse
import fcntl
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import sha,stamp,atomic_json,relative

PREVIOUS='exports/replicator_physical_retry_round1_20261028/manifest.json'
OUT=ROOT/'reports/replicator_physical_retry_round2_planning_20261028'
PLAN=ROOT/'configs/extensions/replicator/physical_retry_round2_20261028.json'

def ref(path):
    path=(ROOT/path).resolve();return dict(path=relative(path),sha256=sha(path))

def save(progress):
    progress['counts']={s:sum(r['status']==s for r in progress['entries']) for s in ('pending','running','generated','failed')}
    progress['stage']='complete' if not progress['counts']['pending'] and not progress['counts']['running'] else 'preparing'
    progress['updated_at']=stamp();atomic_json(OUT/'progress.json',progress)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--recipes',type=Path)
    p.add_argument('--finalize',action='store_true');p.add_argument('--initialize-only',action='store_true')
    p.add_argument('--recover-unstarted',action='store_true',help='Retain CUDA startup errors with zero solver calls and resume the exact frozen recipe')
    args=p.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    # Serialize the read-modify-write of the progress file, not only CUDA.
    with (OUT/'.prepare.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return prepare(args)

def prepare(args):
    previous=json.loads((ROOT/PREVIOUS).read_text());rows=[r for r in previous['entries'] if r['execution_status']=='failed']
    if previous['stage']!='complete' or len(rows)!=48:raise ValueError('Expected exact 48 round1 failures')
    OUT.mkdir(parents=True,exist_ok=True)
    if (OUT/'progress.json').exists():
        progress=json.loads((OUT/'progress.json').read_text())
        if progress['source_previous_retry_manifest']!=ref(PREVIOUS):raise ValueError('Previous evidence changed')
    else:
        progress=dict(kind='physical_retry_round2_planning',total=48,retry_round=2,
            source_previous_retry_manifest=ref(PREVIOUS),created_at=stamp(),
            entries=[dict(scene_id=r['scene_id'],task_id=r['task_id'],variant_id=r['variant_id'],
                config_sha256=r['config_sha256'],status='pending',attempts=0) for r in rows])
        save(progress)
    if args.initialize_only:return
    if args.recipes:
        import torch
        if not torch.cuda.is_available():raise RuntimeError('CUDA preflight failed before any scene attempt')
        torch.zeros(1,device='cuda');torch.cuda.synchronize()
        recipe_path=(ROOT/args.recipes).resolve();recipe_ref=ref(recipe_path)
        recipes=json.loads(recipe_path.read_text())['recipes'];indexed={r['scene_id']:r for r in rows}
        if not set(recipes)<=set(indexed):raise ValueError('Recipe outside remaining cohort')
        from kitchen_safety.extensions.replicator.physical_retry_round2 import run_candidate,retime_generated
        if args.recover_unstarted:
            for row in progress['entries']:
                if row['scene_id'] not in recipes or row['status']!='failed':continue
                previous_result=ROOT/row['candidate_planning_result']['path']
                result=json.loads(previous_result.read_text())
                if (result.get('phases')!=[] or result.get('artifacts')!={} or result.get('stage')!='initializing'
                        or result.get('failure_reason')!="RuntimeError('No CUDA GPUs are available')"
                        or row['recipe_source']!=recipe_ref or result['recipe']!=recipes[row['scene_id']]):
                    raise ValueError('Recovery only permits the identical recipe after zero-solver CUDA startup failure')
                row.setdefault('initialization_errors',[]).append(row['candidate_planning_result'])
                row.update(status='pending',attempts=0,initialization_recovery_reason='Restarted idle project container after NVML/CUDA failure; zero solver calls or physics episodes in retained startup error.')
            save(progress)
        with (ROOT/'reports/skill_trials/gpu.lock').open('a+') as lock:
            print('WAITING_PLANNER_LOCK',flush=True);fcntl.flock(lock,fcntl.LOCK_EX)
            for row in progress['entries']:
                sid=row['scene_id']
                if sid not in recipes or row['status'] in ('generated','failed'):continue
                if row['attempts'] or row['status']=='running':raise ValueError('Interrupted candidate retained: '+sid)
                if ref(recipe_path)!=recipe_ref:raise ValueError('Frozen recipes changed')
                row.update(status='running',attempts=1,recipe_source=recipe_ref,started_at=stamp());save(progress)
                print('ROUND2_PLAN_START',sid,flush=True)
                candidate_output=OUT/('startup_recovery_'+str(len(row['initialization_errors'])+1)) if row.get('initialization_errors') else OUT
                result,result_path=run_candidate(indexed[sid],recipes[sid],candidate_output)
                row.update(status=result['status'],candidate_planning_result=ref(result_path),
                    failure_reason=result.get('failure_reason'),finished_at=stamp(),recipe=recipes[sid])
                if result['status']=='generated':
                    path=result_path.parent/'retimed_trajectory.npz'
                    layout=json.loads((ROOT/indexed[sid]['config_path']).read_text())
                    row['timing_validation']=retime_generated(result,path,layout)
                    row['trajectory']=ref(path)
                save(progress);print('ROUND2_PLAN_DONE',sid,row['status'],row.get('failure_reason'),flush=True)
    if args.finalize:
        if any(r['status'] not in ('generated','failed') for r in progress['entries']):raise ValueError('Planning cohort incomplete')
        if PLAN.exists():raise FileExistsError('Plan already frozen')
        from kitchen_safety.extensions.replicator.physical_retry_plan import EXECUTION_SETTINGS
        entries=[];indexed={r['scene_id']:r for r in rows}
        for row in progress['entries']:
            old=indexed[row['scene_id']]
            entry={k:old[k] for k in ('scene_id','task_id','variant_id','result_path','result_sha256',
                'config_path','config_sha256','source_scene_path','source_scene_sha256','source_trajectory_path','source_trajectory_sha256')}
            recipe=row['recipe'];evidence=recipe['evidence']
            entry.update(planning_status=row['status'],candidate_planning_result=row['candidate_planning_result'],
                previous_retry_result=dict(path=old['execution_result_path'],sha256=old['execution_result_sha256']),
                physical_attempt_number=old['physical_attempt_number']+1,
                trajectory_path=row.get('trajectory',{}).get('path'),trajectory_sha256=row.get('trajectory',{}).get('sha256'),
                execution_overrides=recipe.get('execution_overrides',{}),
                adjustment=dict(kind='grasp_placement_orientation_round2',description=recipe['description'],
                    evidence_path=evidence['path'],evidence_sha256=evidence['sha256'],candidate_count=1,
                    recipe_source=row['recipe_source'],root_cause_confirmed=False),
                timing_validation=row.get('timing_validation'))
            if row['status']=='failed':entry['failure_reason']=row['failure_reason']
            entries.append(entry)
        plan=dict(kind='physical_failure_retry_plan',schema_version=2,retry_round=2,
            source_first_pass_manifest=previous['source_first_pass_manifest'],
            source_camera_retake_manifest=previous['source_camera_retake_manifest'],
            source_previous_retry_manifest=ref(PREVIOUS),expected_scene_count=48,candidate_count_per_scene=1,
            physical_retries_per_scene=1,layout_changed=False,success_predicate_changed=False,
            execution_settings=EXECUTION_SETTINGS,entries=entries,created_at=stamp(),
            planning_progress=ref(OUT/'progress.json'),scope='One fixed-layout grasp/placement adjustment and one no-render native test per generated candidate; failed planning consumes zero physical attempts.')
        with PLAN.open('x') as f:json.dump(plan,f,ensure_ascii=False,indent=2);f.write('\n')
        print('ROUND2_PLAN_FROZEN',relative(PLAN),progress['counts'],flush=True)
if __name__=='__main__':main()
