#!/usr/bin/env python3
"""One native physical execution per frozen scene; never replan or animate poses.

Use --dry-run for CPU-only source validation. Actual execution needs Isaac/RTX;
--device cpu selects CPU PhysX, not CPU rendering. Completed/failed episodes are
never repeated by --resume. A single --scene is the initial smoke trial.
"""
from pathlib import Path
import fcntl
import json
import os
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.planned_execution import (local, load_inputs, sha, stamp,
    write_json, runtime_hashes, runtime_settings, load_camera_overrides, validate_physical_attempt)


def source_entries(source):
    progress_path = source/'progress.json'; progress = json.loads(progress_path.read_text())
    if progress['stage'] != 'complete' or progress['counts']['generated'] != 154:
        raise ValueError('Expected the terminal 154 generated planning scenes')
    entries = []
    for row in progress['entries']:
        rp = local(row['result_path']); result = json.loads(rp.read_text())
        trajectory = result['artifacts']['trajectory']; scene = rp.with_name('source_scene.usda')
        entry = dict(scene_id=row['scene_id'], task_id=row['task_id'], variant_id=row['variant_id'],
            result_path=str(rp.relative_to(ROOT)), result_sha256=sha(rp), config_path=row['config_path'],
            config_sha256=row['config_sha256'], trajectory_path=trajectory['path'], trajectory_sha256=trajectory['sha256'],
            source_scene_path=str(scene.relative_to(ROOT)), source_scene_sha256=result['composition']['scene_sha256'],
            planning_status='generated', execution_status='pending', task_success=False, attempts=0, executed=False,
            video_path=None, video_sha256=None)
        load_inputs(entry)
        entries.append(entry)
    if len(entries) != 154 or len({e['scene_id'] for e in entries}) != 154:
        raise ValueError('Expected 154 unique scenes')
    return progress_path, entries


def save_manifest(output, manifest):
    manifest['updated_at'] = stamp()
    manifest['counts'] = {s: sum(e['execution_status']==s for e in manifest['entries'])
                          for s in ('pending', 'running', 'success', 'failed', 'error')}
    manifest['video_count'] = sum(bool(e.get('video_sha256')) for e in manifest['entries'])
    manifest['stage'] = 'complete' if not manifest['counts']['pending'] and not manifest['counts']['running'] else 'executing'
    write_json(output/'manifest.json', manifest)

def retake_entries(source, args):
    if not args.camera_retake_plan: return source
    path = local(args.camera_retake_plan); plan = json.loads(path.read_text())
    if plan.get('kind') != 'physical_camera_retake' or plan.get('physical_attempt_number') != 2:
        raise ValueError('Invalid explicit camera-retake plan')
    rows = plan['entries']; lookup = {row['scene_id']: row for row in source}
    if not rows or len({row['scene_id'] for row in rows}) != len(rows):
        raise ValueError('Camera-retake whitelist must be nonempty and unique')
    selected = []
    for row in rows:
        entry = dict(lookup[row['scene_id']])
        for key in ('task_id','variant_id','config_sha256','trajectory_sha256','source_scene_sha256'):
            if row[key] != entry[key]: raise ValueError('Retake source binding differs: '+key)
        for key in ('config_path','trajectory_path','source_scene_path'):
            if local(row[key]) != local(entry[key]): raise ValueError('Retake source path differs: '+key)
        entry.update(physical_attempt_number=2, first_pass=row['first_pass'],
            camera_retake_plan_path=str(path.relative_to(ROOT)), camera_retake_plan_sha256=sha(path))
        validate_physical_attempt(entry, args, args.output)
        selected.append(entry)
    return selected


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--source-run', type=Path, default=ROOT/'reports/replicator_trajectory_retry10_20261028')
    parser.add_argument('--output', type=Path, default=ROOT/'exports/replicator_154_physical_demos_20261028')
    parser.add_argument('--scene', default='')
    parser.add_argument('--task', default='', help='Filter pending scenes by their complete original task_id')
    parser.add_argument('--camera-overrides', type=Path)
    parser.add_argument('--camera-retake-plan', type=Path,
        help='Explicit source-bound whitelist for a separate second physical camera recording')
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--allow-runtime-revision', action='store_true',
        help='Explicitly record changed code/settings for remaining unattempted scenes; never repeats prior episodes')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=640)
    parser.add_argument('--video-stride', type=int, default=2)
    parser.add_argument('--torch-threads', type=int, default=4)
    parser.add_argument('--physics-threads', type=int, default=8,
        help='Explicit native PhysX worker count, set before each environment is constructed')
    parser.add_argument('--episode-seconds', type=float, default=120.)
    parser.add_argument('--max-actions', type=int, default=3000)
    parser.add_argument('--initial-settle-seconds', type=float, default=5.)
    parser.add_argument('--close-seconds', type=float, default=2.)
    parser.add_argument('--release-seconds', type=float, default=2.)
    parser.add_argument('--final-settle-seconds', type=float, default=3.)
    args = parser.parse_args()
    args.output, args.source_run = args.output.resolve(), args.source_run.resolve()
    if not args.output.is_relative_to(ROOT): parser.error('Output must be inside repository')
    if args.physics_threads < 1 or args.torch_threads < 1:
        parser.error('Physics and Torch thread counts must be positive')
    if args.width % 2 or args.height % 2 or args.video_stride < 1 or 30 % args.video_stride:
        parser.error('Use even dimensions and a positive video stride dividing 30')
    if min(args.initial_settle_seconds,args.close_seconds,args.release_seconds,args.final_settle_seconds) < 0:
        parser.error('Negative physical hold duration')
    pp, source = source_entries(args.source_run)
    load_camera_overrides(args.camera_overrides)
    source = retake_entries(source, args)
    if args.scene and args.scene not in {r['scene_id'] for r in source}: parser.error('Unknown scene')
    if args.task and args.task not in {r['task_id'] for r in source}: parser.error('Unknown task')
    if args.dry_run and (args.output/'manifest.json').exists():
        existing = json.loads((args.output/'manifest.json').read_text())
        expected_plan = str(local(args.camera_retake_plan).relative_to(ROOT)) if args.camera_retake_plan else None
        if existing.get('camera_retake_plan_path') != expected_plan:
            raise ValueError('Existing recording requires the same explicit retake plan')
        for row in existing['entries']:
            if row['execution_status']=='pending' and (not args.scene or row['scene_id']==args.scene) and (not args.task or row['task_id']==args.task):
                load_inputs(row); validate_physical_attempt(row, args, args.output)
    if args.dry_run:
        print(json.dumps(dict(cpu_preflight_passed=True, scenes=len(source), source_progress_sha256=sha(pp),
            selected_scene=args.scene or None, selected_task=args.task or None,
            runtime_settings=runtime_settings(args), physics_launched=False, planning_launched=False,
            controller='PhysX actuator position targets', payload_pose_writes=False), ensure_ascii=False)); return
    mp = args.output/'manifest.json'
    if mp.exists():
        if not args.resume: parser.error('Existing execution retained; use --resume (no completed scene repeats)')
        manifest = json.loads(mp.read_text())
        if manifest['kind'] != 'physical_execution' or manifest['source_progress_sha256'] != sha(pp):
            raise ValueError('Execution/source identity changed')
        expected_plan = str(local(args.camera_retake_plan).relative_to(ROOT)) if args.camera_retake_plan else None
        if manifest.get('camera_retake_plan_path') != expected_plan or manifest.get('camera_retake_plan_sha256') != (sha(local(args.camera_retake_plan)) if args.camera_retake_plan else None):
            raise ValueError('Existing recording requires the same explicit retake plan')
        if {row['scene_id'] for row in manifest['entries']} != {row['scene_id'] for row in source}:
            raise ValueError('Recording scene set differs from its source/retake whitelist')
        prior_revision = manifest['runtime_revisions'][-1]
        if prior_revision['source_sha256'] != runtime_hashes() or prior_revision['settings'] != runtime_settings(args):
            if not args.allow_runtime_revision:
                raise ValueError('Runtime code/settings changed; require explicit --allow-runtime-revision to record mixed execution versions')
            manifest['runtime_revisions'].append(dict(id=prior_revision['id']+1, created_at=stamp(),
                source_sha256=runtime_hashes(), settings=runtime_settings(args),
                reason='Explicit operator revision for still-unattempted scenes; earlier trials remain unchanged'))
        # Interrupted episodes retain their one attempt and are never repeated.
        for row in manifest['entries']:
            if row['execution_status'] == 'running':
                row.update(execution_status='error', failure_reason='Previous process interrupted; one attempt retained, no automatic repeat')
    else:
        args.output.mkdir(parents=True, exist_ok=True)
        manifest = dict(schema_version=1, kind='physical_execution', created_at=stamp(), total=len(source),
            source_run=str(args.source_run.relative_to(ROOT)), source_progress_sha256=sha(pp), entries=source,
            scope='Native one-shot PhysX execution of cuRobo joint targets. Payload never attached or teleported. Original task predicates retained; safety acceptance separate.',
            one_attempt_per_scene=not bool(args.camera_retake_plan), planning_rerun=False,
            camera_retake_plan_path=str(local(args.camera_retake_plan).relative_to(ROOT)) if args.camera_retake_plan else None,
            camera_retake_plan_sha256=sha(local(args.camera_retake_plan)) if args.camera_retake_plan else None,
            initial_settings=runtime_settings(args), initial_execution_source_sha256=runtime_hashes(),
            runtime_revisions=[dict(id=0, created_at=stamp(), source_sha256=runtime_hashes(), settings=runtime_settings(args))])
        if args.camera_retake_plan:
            manifest['scope'] = 'Additional second physical executions for camera visibility only; original first-pass outcomes and videos retained, no task optimization or replanning.'
        save_manifest(args.output, manifest)
    selected = [r for r in manifest['entries'] if r['execution_status']=='pending' and (not args.scene or r['scene_id']==args.scene) and (not args.task or r['task_id']==args.task)]
    if args.limit: selected = selected[:args.limit]
    for row in selected:
        load_inputs(row); validate_physical_attempt(row, args, args.output)
    if not selected:
        save_manifest(args.output, manifest); print('NO_PENDING_SCENES'); return
    args.enable_cameras = True
    os.environ.setdefault('ISAACLAB_ARENA_FORCE_EXIT_ON_COMPLETE', '1')
    lock = ROOT/'reports/skill_trials/gpu.lock'; lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a+') as stream:
        print('WAITING_GPU_LOCK', flush=True); fcntl.flock(stream, fcntl.LOCK_EX)
        from isaaclab_arena.utils.isaaclab_utils.simulation_app import (SimulationAppContext,
            teardown_simulation_app, collect_garbage_and_clear_cuda_cache)
        from kitchen_safety.extensions.replicator.planned_execution import execute_one
        with SimulationAppContext(args):
            for row in selected:
                row['runtime_revision_id'] = manifest['runtime_revisions'][-1]['id']
                row.update(execution_status='running', attempts=1, started_at=stamp()); save_manifest(args.output, manifest)
                print('PHYSICAL_EXECUTION_START', row['scene_id'], flush=True)
                result, path = execute_one(row, args.output, args)
                for key in ('execution_status','task_success','attempts','physical_attempt_number','executed','video_sha256','frame_count','fps','width','height','duration_seconds','failure_reason'):
                    row[key] = result.get(key)
                row['video_path'] = str(local(result['video_path']).relative_to(args.output)) if result.get('video_path') else None
                row.update(execution_result_path=str(path.relative_to(ROOT)), execution_result_sha256=sha(path), finished_at=stamp())
                save_manifest(args.output, manifest)
                print('PHYSICAL_EXECUTION_DONE', row['scene_id'], result['execution_status'], result.get('failure_reason'), flush=True)
                teardown_simulation_app(suppress_exceptions=True)
                collect_garbage_and_clear_cuda_cache()
    print('PHYSICAL_BATCH_FINISHED', manifest['counts'], flush=True)


if __name__ == '__main__':
    main()
