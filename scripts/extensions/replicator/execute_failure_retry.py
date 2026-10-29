#!/usr/bin/env python3
"""Execute each whitelisted physical failure exactly once, with no rendering.

--dry-run validates frozen inputs without starting Kit or writing output.
Use --task with separate invocations to get a fresh Kit process per kitchen task.
--resume selects only never-attempted pending rows; terminal rows never repeat.
"""
import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator import retry_physical_execution as execution

TERMINAL = {'success','failed','error','planning_failed'}
DEFAULTS = dict(device='cpu', seed=42, physics_threads=8, torch_threads=4,
    episode_seconds=200., max_actions=6000, initial_settle_seconds=5.,
    close_seconds=2., release_seconds=2., final_settle_seconds=3.)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan', type=Path, default=ROOT/'configs/extensions/replicator/physical_retry_round1_20261028.json')
    p.add_argument('--output', type=Path, default=ROOT/'exports/replicator_physical_retry_round1_20261028')
    p.add_argument('--scene', default='')
    p.add_argument('--task', default='')
    p.add_argument('--limit', type=int, default=0)
    p.add_argument('--resume', action='store_true')
    p.add_argument('--allow-runtime-revision', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--initialize-only', action='store_true')
    return p


def apply_settings(args, plan):
    values = dict(DEFAULTS)
    settings = plan.get('execution_settings', {})
    if set(settings)-set(DEFAULTS): raise ValueError('Unknown execution setting in plan')
    values.update(settings)
    for key, expected in dict(device='cpu', seed=42, physics_threads=8,
            initial_settle_seconds=5.,close_seconds=2.,release_seconds=2.,final_settle_seconds=3.).items():
        if values[key] != expected: raise ValueError('This retry keeps the original controller/hold settings: '+key)
    if type(values['max_actions']) is not int or values['max_actions']<1 or values['episode_seconds']<=0:
        raise ValueError('Invalid physical execution budget')
    if type(values['torch_threads']) is not int or values['torch_threads']<1:
        raise ValueError('Invalid Torch thread count')
    for key,value in values.items(): setattr(args,key,value)
    # Explicit AppLauncher kwargs; no camera, GUI, livestream, XR, or video.
    args.headless=True; args.enable_cameras=False; args.visualizer=['none']
    args.livestream=0; args.xr=False; args.video=False; args.experience=''
    return args


def select_pending(manifest, args):
    ids={r['scene_id'] for r in manifest['entries']}; tasks={r['task_id'] for r in manifest['entries']}
    if args.scene and args.scene not in ids: raise ValueError('Scene not in the fixed retry whitelist')
    if args.task and args.task not in tasks: raise ValueError('Task not in the fixed retry whitelist')
    selected=[]
    for row in manifest['entries']:
        if row['execution_status'] not in TERMINAL | {'pending','running'}:
            raise ValueError('Unknown execution state')
        if row['execution_status']=='planning_failed' and (row.get('planning_status') != 'failed'
                or row.get('attempts') != 0 or row.get('executed') is not False or row.get('task_success') is not False):
            raise ValueError('Planning failure cannot consume a physical attempt')
        if row['execution_status'] in {'success','failed','error','running'} and row.get('attempts') != 1:
            raise ValueError('Attempted scene must consume exactly one physical attempt')
        if row['execution_status']=='pending':
            if row.get('attempts') != 0 or row.get('executed') is not False:
                raise ValueError('Pending scene has a prior or uncertain attempt')
            if (not args.scene or row['scene_id']==args.scene) and (not args.task or row['task_id']==args.task):
                selected.append(row)
    return selected[:args.limit] if args.limit else selected


def check_existing(manifest, source, args):
    if manifest.get('kind') != 'physical_failure_retry' or manifest.get('retry_plan_sha256') != execution.sha(args.plan):
        raise ValueError('Output belongs to a different execution/plan')
    if execution.local(manifest['retry_plan_path']) != args.plan:
        raise ValueError('Output plan path differs')
    old=execution.rows_by_id(manifest); new={r['scene_id']:r for r in source}
    if set(old)!=set(new): raise ValueError('Output whitelist differs from plan')
    mutable={'execution_status','attempts','executed','task_success','video_path','video_sha256'}
    for sid,row in old.items():
        for key,value in new[sid].items():
            if key not in mutable and row.get(key)!=value: raise ValueError('Output source binding differs: '+sid+' '+key)
        if row['execution_status'] in TERMINAL and row.get('execution_result_path'):
            if execution.sha(execution.local(row['execution_result_path'])) != row['execution_result_sha256']:
                raise ValueError('Retained retry result changed')
    prior=manifest['runtime_revisions'][-1]
    if prior['source_sha256'] != execution.runtime_hashes() or prior['settings'] != execution.runtime_settings(args):
        if not args.allow_runtime_revision:
            raise ValueError('Runtime code/settings changed; explicit --allow-runtime-revision required for pending rows')
    return prior


def save(output, manifest):
    manifest['updated_at']=execution.stamp()
    manifest['counts']={s:sum(e['execution_status']==s for e in manifest['entries'])
        for s in ('pending','running','success','failed','error','planning_failed')}
    manifest['stage']='complete' if not manifest['counts']['pending'] and not manifest['counts']['running'] else 'executing'
    execution.write_json(output/'manifest.json',manifest)


def main(argv=None):
    args=parser().parse_args(argv)
    args.plan=execution.local(args.plan); args.output=execution.local(args.output)
    if args.limit<0: raise ValueError('Negative scene limit')
    plan,source=execution.validate_plan(args.plan); apply_settings(args,plan)
    source_keys = ['source_first_pass_manifest','source_camera_retake_manifest']
    if execution.round_two(plan): source_keys.append('source_previous_retry_manifest')
    for key in source_keys:
        oldroot=execution.local(plan[key]['path']).parent
        if args.output==oldroot or args.output.is_relative_to(oldroot):
            raise ValueError('Retry output cannot be inside retained first-pass/camera evidence')
    candidate=dict(kind='physical_failure_retry',schema_version=plan.get('schema_version',1),total=len(source),entries=source,
        retry_plan_path=str(args.plan.relative_to(ROOT)),retry_plan_sha256=execution.sha(args.plan),
        source_first_pass_manifest=plan['source_first_pass_manifest'],
        source_camera_retake_manifest=plan['source_camera_retake_manifest'],
        created_at=execution.stamp(),rendering_enabled=False,video_expected=False,one_additional_attempt_per_scene=True,
        scope=(f'{len(source)} frozen failures, one candidate per scene and at most one real physical test of each generated candidate; '
               'planning failures are retained without physical execution, zero cameras/rendering, original layouts/predicates retained.'),
        runtime_revisions=[dict(id=0,created_at=execution.stamp(),source_sha256=execution.runtime_hashes(),
                               settings=execution.runtime_settings(args))])
    if execution.round_two(plan):
        candidate.update(retry_round=plan['retry_round'], source_previous_retry_manifest=plan['source_previous_retry_manifest'])
    if plan.get('retry_round') in (7, 8):
        candidate.update(layout_changed=True, success_predicate_changed=False,
            scope='Separately versioned added-hazard poses; one frozen candidate and at most one actual no-render episode each. Original task assets/predicate and all prior-layout histories retained.')
    mp=args.output/'manifest.json'
    existing=json.loads(mp.read_text()) if mp.exists() else None
    if existing:
        if not args.resume and not args.dry_run: raise ValueError('Existing attempts retained; use --resume')
        check_existing(existing,source,args)
    manifest=existing or candidate
    selected=select_pending(manifest,args)
    for row in selected:
        _,_,data=execution.load_retry_inputs(row)
        execution.validate_budget(data,execution.entry_settings(row,args))
    if args.dry_run:
        print(json.dumps(dict(cpu_preflight_passed=True,whitelist_scenes=len(source),
            selected_pending=len(selected),planning_failed=sum(r['execution_status']=='planning_failed' for r in manifest['entries']),
            scene_ids=[r['scene_id'] for r in selected],
            runtime_settings=execution.runtime_settings(args),physics_launched=False,
            rendering_launched=False,source_plan_sha256=execution.sha(args.plan)),ensure_ascii=False))
        return
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'.retry_execution.lock').open('a') as output_lock:
        try: fcntl.flock(output_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as error: raise ValueError('Another process holds this retry output') from error
        # Read again under lock; never use a stale pending selection.
        if mp.exists():
            manifest=json.loads(mp.read_text()); prior=check_existing(manifest,source,args)
            if prior['source_sha256']!=execution.runtime_hashes() or prior['settings']!=execution.runtime_settings(args):
                manifest['runtime_revisions'].append(dict(id=prior['id']+1,created_at=execution.stamp(),
                    source_sha256=execution.runtime_hashes(),settings=execution.runtime_settings(args),
                    reason='Explicit runtime revision for never-attempted pending rows only'))
        for row in manifest['entries']:
            if row['execution_status']=='running':
                row.update(execution_status='error',task_success=False,
                    failure_reason='Interrupted attempt retained; no automatic retry')
        selected=select_pending(manifest,args); save(args.output,manifest)
        if args.initialize_only or not selected:
            print('INITIALIZED_ONLY' if args.initialize_only else 'NO_PENDING_SCENES'); return
        # XR has an environment fallback even with xr=False; explicitly disable it.
        os.environ['XR']='0'
        os.environ.setdefault('ISAACLAB_ARENA_FORCE_EXIT_ON_COMPLETE','1')
        lock=ROOT/'reports/skill_trials/gpu.lock';lock.parent.mkdir(parents=True,exist_ok=True)
        with lock.open('a+') as stream:
            print('WAITING_KIT_LOCK',flush=True);fcntl.flock(stream,fcntl.LOCK_EX)
            from isaaclab_arena.utils.isaaclab_utils.simulation_app import (SimulationAppContext,
                teardown_simulation_app,collect_garbage_and_clear_cuda_cache)
            with SimulationAppContext(args):
                for row in selected:
                    execution.validate_entry_lineage(row,args)
                    row.update(execution_status='running',attempts=1,started_at=execution.stamp(),
                        runtime_revision_id=manifest['runtime_revisions'][-1]['id'])
                    save(args.output,manifest)
                    print('PHYSICAL_RETRY_START',row['scene_id'],flush=True)
                    result,path=execution.execute_one(row,args.output,args)
                    for key in ('execution_status','task_success','attempts','executed','failure_reason','action_count','elapsed_seconds'):
                        row[key]=result.get(key)
                    row.update(execution_result_path=str(path.relative_to(ROOT)),execution_result_sha256=execution.sha(path),
                        finished_at=execution.stamp())
                    save(args.output,manifest)
                    print('PHYSICAL_RETRY_DONE',row['scene_id'],result['execution_status'],result.get('failure_reason'),flush=True)
                    teardown_simulation_app(suppress_exceptions=True)
                    collect_garbage_and_clear_cuda_cache()
    print('PHYSICAL_RETRY_BATCH_FINISHED',manifest['counts'],flush=True)


if __name__=='__main__': main()
