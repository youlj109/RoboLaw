#!/usr/bin/env python3
"""Plan exactly one frozen candidate per remaining failure, then bind execution.

Recipes may be supplied in independent groups. Completed or interrupted scene
attempts are never rerun; --finalize freezes the 2-entry physical test plan.
"""
import argparse
import fcntl
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import sha,stamp,atomic_json,relative

PREVIOUS='exports/replicator_physical_retry_round9_20261029/manifest.json'
OUT=ROOT/'reports/replicator_physical_retry_round10_planning_20261029'
PLAN=ROOT/'configs/extensions/replicator/physical_retry_round10_20261029.json'

def ref(path):
    path=(ROOT/path).resolve();return dict(path=relative(path),sha256=sha(path))

def save(progress):
    progress['counts']={s:sum(r['status']==s for r in progress['entries']) for s in ('pending','running','generated','failed')}
    progress['stage']='complete' if not progress['counts']['pending'] and not progress['counts']['running'] else 'preparing'
    progress['updated_at']=stamp();atomic_json(OUT/'progress.json',progress)

def freeze_revision(old):
    """Retain the exact previous layout and its historical revision identity."""
    from copy import deepcopy
    from kitchen_safety.extensions.replicator.scene_revision_round8 import validate_scene_revision
    validate_scene_revision(ROOT,old['scene_revision'],expected_scene_id=old['scene_id'],current=old)
    revised=deepcopy(old)
    revised.update(retained_scene_revision=True,scene_settings_changed_this_round=False,scene_settings_changed_once=True)
    return revised


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
    previous=json.loads((ROOT/PREVIOUS).read_text());rows=[r for r in previous['entries'] if r['execution_status'] in ('failed','planning_failed')]
    if previous['stage']!='complete' or len(rows)!=2:raise ValueError('Expected exact 2 round9 unresolved scenes')
    OUT.mkdir(parents=True,exist_ok=True)
    if (OUT/'progress.json').exists():
        progress=json.loads((OUT/'progress.json').read_text())
        if progress['source_previous_retry_manifest']!=ref(PREVIOUS):raise ValueError('Previous evidence changed')
    else:
        progress=dict(kind='physical_retry_round10_planning',total=2,retry_round=10,
            source_previous_retry_manifest=ref(PREVIOUS),created_at=stamp(),
            entries=[dict(scene_id=r['scene_id'],task_id=r['task_id'],variant_id=r['variant_id'],
                config_path=r['config_path'],config_sha256=r['config_sha256'],layout_changed=False,status='pending',attempts=0) for r in rows])
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
                revised=freeze_revision(indexed[sid])
                row.update({key:revised[key] for key in ('config_path','config_sha256','source_scene_path','source_scene_sha256','scene_revision','layout_changed','retained_scene_revision','scene_settings_changed_this_round','scene_settings_changed_once')})
                row.update(status='running',attempts=1,recipe_source=recipe_ref,started_at=stamp());save(progress)
                print('ROUND10_PLAN_START',sid,flush=True)
                candidate_output=OUT/('startup_recovery_'+str(len(row['initialization_errors'])+1)) if row.get('initialization_errors') else OUT
                result,result_path=run_candidate(revised,recipes[sid],candidate_output)
                row.update(status=result['status'],candidate_planning_result=ref(result_path),
                    failure_reason=result.get('failure_reason'),finished_at=stamp(),recipe=recipes[sid])
                if result['status']=='generated':
                    path=result_path.parent/'retimed_trajectory.npz'
                    layout=json.loads((ROOT/revised['config_path']).read_text())
                    row['timing_validation']=retime_generated(result,path,layout)
                    from kitchen_safety.extensions.replicator.lift_path_validation import array_lift_gate
                    import numpy as np
                    with np.load(path,allow_pickle=False) as data:
                        gate=array_lift_gate(data,result['recipe'])
                    if gate is not None:
                        row['timing_validation']['retimed_lift_path_validation']=gate
                    if gate is not None and not gate['passed']:
                        # Preserve the rejected output as evidence; never execute it.
                        result.update(status='failed',trajectory_complete=False,failed_phase='lift',
                            failure_reason='Retimed FK path rejected: LIFT_PATH_CONSTRAINT_LIMIT',
                            retimed_lift_path_validation=gate,rejected_retimed_trajectory=ref(path))
                        atomic_json(result_path,result)
                        row.update(status='failed',failure_reason=result['failure_reason'],candidate_planning_result=ref(result_path))
                    else:
                        row['trajectory']=ref(path)
                save(progress);print('ROUND10_PLAN_DONE',sid,row['status'],row.get('failure_reason'),flush=True)
    if args.finalize:
        if any(r['status'] not in ('generated','failed') for r in progress['entries']):raise ValueError('Planning cohort incomplete')
        if PLAN.exists():raise FileExistsError('Plan already frozen')
        from kitchen_safety.extensions.replicator.physical_retry_plan import EXECUTION_SETTINGS
        entries=[];indexed={r['scene_id']:r for r in rows}
        for row in progress['entries']:
            old=indexed[row['scene_id']]
            entry={k:old[k] for k in ('scene_id','task_id','variant_id','result_path','result_sha256',
                'config_path','config_sha256','source_scene_path','source_scene_sha256','source_trajectory_path','source_trajectory_sha256')}
            entry.update({key:row[key] for key in ('config_path','config_sha256','source_scene_path','source_scene_sha256','scene_revision','layout_changed','retained_scene_revision','scene_settings_changed_this_round','scene_settings_changed_once')})
            recipe=row['recipe'];evidence=recipe['evidence']
            entry.update(planning_status=row['status'],candidate_planning_result=row['candidate_planning_result'],
                previous_retry_result=(dict(path=old['execution_result_path'],sha256=old['execution_result_sha256']) if old['executed'] else dict(path=old['prior_physical_attempts'][-1]['result_path'],sha256=old['prior_physical_attempts'][-1]['result_sha256'])),
                physical_attempt_number=old['physical_attempt_number']+int(old['executed']),
                trajectory_path=row.get('trajectory',{}).get('path'),trajectory_sha256=row.get('trajectory',{}).get('sha256'),
                execution_overrides=recipe.get('execution_overrides',{}),
                adjustment=dict(kind='retained_layout_joint_goal_repair_round10',description=recipe['description'],
                    evidence_path=evidence['path'],evidence_sha256=evidence['sha256'],candidate_count=1,
                    recipe_source=row['recipe_source'],root_cause_confirmed=False),
                timing_validation=row.get('timing_validation'))
            if row['status']=='failed':entry['failure_reason']=row['failure_reason']
            entries.append(entry)
        plan=dict(kind='physical_failure_retry_plan',schema_version=2,retry_round=10,
            source_first_pass_manifest=previous['source_first_pass_manifest'],
            source_camera_retake_manifest=previous['source_camera_retake_manifest'],
            source_previous_retry_manifest=ref(PREVIOUS),expected_scene_count=2,candidate_count_per_scene=1,
            physical_retries_per_scene=1,layout_changed=False,layout_changed_from_original=True,success_predicate_changed=False,
            execution_settings=EXECUTION_SETTINGS,entries=entries,created_at=stamp(),
            planning_progress=ref(OUT/'progress.json'),scope='Retain exact round 8 scene, adjust only one frozen trajectory candidate per unresolved scene. Targeted FK lift corridor where declared; one no-render native test per complete plan. Original task/assets/predicate unchanged; all historical attempts retained.')
        with PLAN.open('x') as f:json.dump(plan,f,ensure_ascii=False,indent=2);f.write('\n')
        print('ROUND10_PLAN_FROZEN',relative(PLAN),progress['counts'],flush=True)
if __name__=='__main__':main()
