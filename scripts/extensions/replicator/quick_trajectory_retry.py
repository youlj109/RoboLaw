#!/usr/bin/env python3
"""Retry each previously failed scene exactly once, preserving the fixed layouts."""
from __future__ import annotations

import argparse
from copy import deepcopy
import fcntl
import importlib.util
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import atomic_json, relative, sha, stamp

spec = importlib.util.spec_from_file_location('quick_generation', Path(__file__).with_name('quick_trajectory_generate.py'))
generation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generation)
DEFAULT = ROOT / 'reports/replicator_trajectory_retry_20261026'
ORIGINAL = ROOT / 'reports/replicator_trajectory_generation_20261026'
RECIPES = ROOT / 'configs/extensions/replicator/trajectory_retry_v1.json'


def save(output, progress):
    rows = [row for row in progress['entries'] if row.get('retry_round') == progress.get('retry_round', 1)]
    progress['retry_counts'] = {state: sum(row['status'] == state for row in rows) for state in generation.STATES}
    progress['round_actions'] = dict(
        planned=sum(row.get('attempts_this_round', 0) == 1 for row in rows),
        annotated=sum(row.get('annotation_only') is True and row['status'] == 'failed' for row in rows))
    generation.save_progress(output, progress)


def validate_annotation_recipe(recipe, scene_id):
    """Reject incomplete annotation work before any queue action is started."""
    if recipe.get('action', 'plan') != 'annotate_only':
        return
    note = recipe.get('review_note')
    if not isinstance(note, str) or not note.strip():
        raise ValueError('Annotation-only action requires a nonempty review note: '+scene_id)
    evidence = recipe.get('annotation_evidence')
    path = evidence.get('path') if isinstance(evidence, dict) else None
    digest = evidence.get('sha256') if isinstance(evidence, dict) else None
    if (not isinstance(path, str) or not path.strip() or
            not isinstance(digest, str) or len(digest) != 64 or
            any(character not in '0123456789abcdef' for character in digest) or
            not (ROOT/path).is_file() or sha(ROOT/path) != digest):
        raise ValueError('Correction evidence missing or changed: '+scene_id+' annotation_evidence')


def initialize(output, source, recipes_path):
    from kitchen_safety.extensions.replicator.quick_trajectory_retry import load_retry_recipes
    recipes = load_retry_recipes(recipes_path)
    original = json.loads((source/'progress.json').read_text())
    failed = {row['scene_id'] for row in original['entries'] if row['status'] == 'failed'}
    if len(original['entries']) != 154 or not failed or set(recipes) != failed:
        raise ValueError('Retry recipes must match exactly the previous round failed scenes')
    retry_round = int(original.get('retry_round', 0)) + 1
    from kitchen_safety.extensions.replicator.scene_retry_once import validate_round_authorization
    authorization = validate_round_authorization(ROOT, json.loads(recipes_path.read_text()), source, original, failed, output)
    frozen = {}
    if authorization:
        frozen[authorization['path']] = authorization['sha256']
    rows = []
    adjusted_count = 0
    for old in original['entries']:
        sid = old['scene_id']
        result_path = source/'scenes'/sid/'result.json'
        result = json.loads(result_path.read_text())
        if result['status'] != old['status']:
            raise ValueError('Original result/progress status mismatch: '+sid)
        files = {relative(result_path): sha(result_path), old['config_path']: old['config_sha256'],
                 **result['composition']['source_paths_sha256']}
        for path, digest in files.items():
            if path in frozen and frozen[path] != digest:
                raise ValueError('Conflicting original source hashes: '+path)
            frozen[path] = digest
        row = deepcopy(old)
        if row.get('scene_adjustment'):
            row['prior_scene_adjustment'] = row.pop('scene_adjustment')
        row.update(attempts_this_round=0, actions_this_round=0, annotation_only=False, recipe={})
        row.update(prior_status=old['status'], prior_stage=old['stage'],
                   prior_result_path=relative(result_path), prior_result_sha256=sha(result_path),
                   carry_forward=old['status']=='generated', retry_round=0 if old['status']=='generated' else retry_round)
        if row['retry_round'] == retry_round:
            recipe = recipes[sid]
            if recipe.get('action', 'plan') not in ('plan', 'annotate_only'):
                raise ValueError('Unknown round action: '+sid)
            validate_annotation_recipe(recipe, sid)
            if recipe['config_sha256'] != old['config_sha256'] or recipe['result_sha256'] != sha(result_path):
                raise ValueError('Recipe does not bind original scene/result: '+sid)
            if recipe.get('scene_adjustment') is not None:
                if (original.get('scene_settings_changed_once') or result.get('scene_settings_changed_once') or old.get('scene_adjustment')) and not authorization:
                    raise ValueError('This scene already used its one authorized scene adjustment: '+sid)
                from kitchen_safety.extensions.replicator.scene_retry_once import validate_adjustment
                change = validate_adjustment(ROOT, recipe, old['config_path'], old['config_sha256'])
                row.update(config_path=change['config_path'], config_sha256=change['config_sha256'],
                           previous_config_sha256=old['config_sha256'], scene_adjustment=change,
                           previous_views=deepcopy(old.get('views', [])), views=[])
                frozen[change['config_path']] = change['config_sha256']
                adjusted_count += 1
            row.update(status='pending', stage='queued', recipe=recipe, attempts_this_round=0,
                       actions_this_round=0, annotation_only=False,
                       failure_code=None, failure_reason=None, failed_phase=None, result_path=None,
                       elapsed_seconds=None, trajectory_complete=False, trajectory_samples=None,
                       artifacts={}, suggestion=('本景仅调整一次新增物体摆位，冻结后仅规划一次，失败不再调整。' if row.get('scene_adjustment') else
                       '布局固定；按本景修改提示选择一个抓点/姿态/路点方案，失败直接下一个。'), updated_at=stamp())
        rows.append(row)
    for path, digest in frozen.items():
        if sha(ROOT/path) != digest:
            raise ValueError('Original layout/source/result changed before retry: '+path)
    if output.exists():
        raise FileExistsError('Existing retry run retained: '+str(output))
    output.mkdir(parents=True)
    for row in rows:
        if row['carry_forward']:
            destination = output/'scenes'/row['scene_id']
            shutil.copytree(source/'scenes'/row['scene_id'], destination)
            row['result_path'] = relative(destination/'result.json')
    atomic_json(output/'frozen_inputs.json', dict(created_at=stamp(), files=frozen,
        recipes_path=relative(recipes_path), recipes_sha256=sha(recipes_path)))
    snapshot = dict(schema_version=1, created_at=stamp(), entries=deepcopy(rows), total=154)
    atomic_json(output/'scene_snapshot.json', snapshot)
    progress = dict(schema_version=1, total=154, retry_total=len(failed),
        carried_forward=sum(row['carry_forward'] for row in rows), retry_round=retry_round,
        stage='preparing', scope=generation.SCOPE+(' One frozen candidate (hazard pose or target sequence) and one planning pass per failed scene.'
            if adjusted_count else ' Fixed layout, one selected retry candidate per previously failed scene.'),
        scene_settings_changed_once=bool(adjusted_count), scene_adjustment_count=adjusted_count,
        target_only_retry_count=sum(not row.get('scene_adjustment') for row in rows if not row['carry_forward']),
        scene_adjustment_authorization=authorization,
        created_at=stamp(), source_run=relative(source), recipe_path=relative(recipes_path),
        recipe_sha256=sha(recipes_path), source_snapshot_path=relative(output/'scene_snapshot.json'),
        source_snapshot_sha256=sha(output/'scene_snapshot.json'), entries=rows,
        checks_not_performed=original['checks_not_performed'])
    save(output, progress)
    return progress


def error_result(row, output, code, reason):
    result = {key: row.get(key) for key in ('scene_id','task_id','variant_id','category','config_path','config_sha256',
                                          'prior_result_path','prior_result_sha256','retry_round')}
    destination = output/'scenes'/row['scene_id']
    result.update(status='failed', stage='retry_worker', failed_phase='retry_worker', failure_code=code,
                  failure_reason=reason, updated_at=stamp(), attempts=1, per_stage_max_attempts=1,
                  planning_only=True, executed=False, task_success_verified=False, trajectory_complete=False,
                  artifacts={}, phases=[], result_path=relative(destination/'result.json'),
                  suggestion='布局固定；本轮已尝试，保留失败记录并继续下一个，不自动再试。')
    prior = ROOT / row['prior_result_path'] if row.get('prior_result_path') else None
    if prior is not None and prior.is_file() and sha(prior) == row.get('prior_result_sha256'):
        previous = json.loads(prior.read_text())
        for key in ('composition', 'start_q', 'start_tcp_world', 'targets',
                    'nominal_attachment', 'nominal_gripper_joint_rad', 'payload_name', 'grasp_info'):
            if key in previous:
                result[key] = deepcopy(previous[key])
        result['source_binding_origin'] = 'Retained verified previous result; no new solver evidence implied'
    if row.get('scene_adjustment'):
        result.update(scene_adjustment=deepcopy(row['scene_adjustment']), scene_settings_changed_once=True,
                      layout_fixed=False, new_layout_frozen=True)
        if 'composition' in result:
            result['previous_composition'] = result.pop('composition')
        current_path = destination/'result.json'
        if current_path.is_file():
            current = json.loads(current_path.read_text())
            composition = current.get('composition', {})
            if (current.get('config_sha256') == row['config_sha256'] and
                    composition.get('layout_sha256') == row['config_sha256']):
                result['composition'] = composition
        result['suggestion'] = '本景已经消耗本轮一次场景调整和规划机会；保留工作器失败，不再调整或重试。'
    atomic_json(destination/'result.json', result)
    return result


def update_row(row, result):
    for key in ('status','stage','failed_phase','failure_code','failure_reason','suggestion','elapsed_seconds',
                'updated_at','result_path','artifacts','trajectory_complete','trajectory_samples'):
        row[key] = result.get(key)


def finish_integrity(output, progress):
    frozen = json.loads((output/'frozen_inputs.json').read_text())
    changes = [path for path, digest in frozen['files'].items() if sha(ROOT/path) != digest]
    recipe_changed = sha(ROOT/frozen['recipes_path']) != frozen['recipes_sha256']
    retried = [row for row in progress['entries'] if row['retry_round'] == progress.get('retry_round', 1)]
    attempts_valid = all(row['attempts_this_round'] == (0 if row.get('recipe', {}).get('action') == 'annotate_only' else 1)
                         for row in retried)
    actions_valid = all(row.get('actions_this_round', row['attempts_this_round']) == 1 for row in retried)
    document = dict(created_at=stamp(), frozen_file_count=len(frozen['files']), changed_files=changes,
                    recipe_changed=recipe_changed,
                    exactly_one_attempt_per_failed_scene=all(row['attempts_this_round'] == 1 for row in retried),
                    action_attempt_counts_valid=attempts_valid, exactly_one_action_per_failed_scene=actions_valid,
                    retry_total=len(retried), passed=not changes and not recipe_changed and attempts_valid and actions_valid)
    if progress.get('scene_settings_changed_once'):
        document.update(scope='New hazard-pose configs frozen after one authorized adjustment; previous configs retained.',
                        scene_adjustment_count=progress['scene_adjustment_count'], original_layout_unchanged=False,
                        new_layout_unchanged_during_planning=not changes)
        atomic_json(output/'scene_adjustment_integrity.json', document)
    atomic_json(output/'fixed_layout_integrity.json', document)
    return document


def bind_correction_evidence(output, progress, frozen):
    """Verify and freeze the exact evidence for exceptional target/contact fixes."""
    changed = False
    for row in progress['entries']:
        if row.get('carry_forward'): continue
        if row.get('scene_adjustment'):
            from kitchen_safety.extensions.replicator.scene_retry_once import validate_adjustment
            validate_adjustment(ROOT, row['recipe'])
        if (row['status'] not in ('generated', 'failed', 'skipped') and
                not row.get('actions_this_round', 0) and not row.get('attempts_this_round', 0)):
            validate_annotation_recipe(row.get('recipe', {}), row['scene_id'])
        for key in ('source_contact_evidence', 'goal_correction_evidence', 'annotation_evidence', 'scene_adjustment_evidence', 'target_adjustment_evidence'):
            evidence = row.get('recipe', {}).get(key)
            if evidence is None: continue
            path = evidence.get('path') if isinstance(evidence, dict) else None
            digest = evidence.get('sha256') if isinstance(evidence, dict) else None
            if not path or not digest or sha(ROOT/path) != digest:
                raise ValueError('Correction evidence missing or changed: '+row['scene_id']+' '+key)
            if path in frozen['files'] and frozen['files'][path] != digest:
                raise ValueError('Correction evidence conflicts with frozen inputs')
            if path not in frozen['files']:
                frozen['files'][path] = digest; changed = True
    if changed: atomic_json(output/'frozen_inputs.json', frozen)


def annotate_only(row, output):
    """One evidence-bound annotation action; no fresh solver or invented path."""
    recipe = row['recipe']
    if not recipe.get('annotation_evidence') or not recipe.get('review_note'):
        raise ValueError('Annotation-only action requires exact evidence and a concrete review note')
    prior_path = ROOT/row['prior_result_path']
    if sha(prior_path) != row['prior_result_sha256']:
        raise ValueError('Previous result changed before annotation')
    previous = json.loads(prior_path.read_text())
    if previous['status'] != 'failed': raise ValueError('Only unresolved failures may be annotated')
    result = deepcopy(previous)
    history, history_path = previous, prior_path
    history_chain = [dict(path=relative(prior_path), sha256=sha(prior_path))]
    seen = {prior_path.resolve()}
    while history.get('stage') == 'retry_worker':
        ancestor = ROOT/history['prior_result_path']
        if ancestor.resolve() in seen or sha(ancestor) != history['prior_result_sha256']:
            raise ValueError('Historical planning result changed or contains a cycle')
        seen.add(ancestor.resolve())
        candidate = json.loads(ancestor.read_text())
        if candidate.get('status') != 'failed' or any(candidate.get(key) != previous.get(key)
                for key in ('scene_id', 'config_sha256', 'start_q', 'composition')):
            raise ValueError('Historical planning result does not match the fixed scene')
        history, history_path = candidate, ancestor
        history_chain.append(dict(path=relative(ancestor), sha256=sha(ancestor)))
    destination = output/'scenes'/row['scene_id']; destination.mkdir(parents=True, exist_ok=True)
    artifacts = {}
    for key, item in history.get('artifacts', {}).items():
        source = ROOT/item['path']
        if sha(source) != item['sha256']: raise ValueError('Previous partial artifact changed')
        target = destination/source.name
        shutil.copyfile(source, target)
        artifacts[key] = dict(path=relative(target), sha256=sha(target))
    for key in ('stage', 'failed_phase', 'failure_code', 'trajectory_samples', 'partial_trajectory'):
        if key in history:
            result[key] = deepcopy(history[key])
    result.update(retry_round=row['retry_round'], annotation_only=True, annotation_recorded=True, planning_attempted=False,
        attempts=0, per_stage_max_attempts=0, actions_this_round=1, artifacts=artifacts,
        previous_phases=history.get('phases') or history.get('previous_phases', []), phases=[],
        historical_planning_result=dict(path=relative(history_path), sha256=sha(history_path)),
        historical_result_chain=history_chain,
        prior_result_path=row['prior_result_path'], prior_result_sha256=row['prior_result_sha256'],
        failure_reason='本轮仅标注，未重新求解：'+recipe['review_note'],
        suggestion=recipe['review_note'], annotation_evidence=recipe['annotation_evidence'],
        failure_code_source='Retained previous solver failure; this round performs one annotation only',
        started_at=stamp(), updated_at=stamp(), elapsed_seconds=0.,
        result_path=relative(destination/'result.json'))
    atomic_json(destination/'result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ORIGINAL)
    parser.add_argument('--output', type=Path, default=DEFAULT)
    parser.add_argument('--recipes', type=Path, default=RECIPES)
    parser.add_argument('--phase-timeout', type=float, default=6.)
    parser.add_argument('--initialize-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--limit', type=int)
    args = parser.parse_args()
    output = args.output.resolve()
    if (output/'progress.json').exists():
        if not args.resume: parser.error('Existing round retained; use --resume (never reruns completed or interrupted attempts)')
        progress = json.loads((output/'progress.json').read_text())
    else:
        progress = initialize(output, args.source.resolve(), args.recipes.resolve())
    if args.initialize_only:
        print(json.dumps(dict(output=relative(output), counts=progress['counts'], retry_counts=progress['retry_counts'])), flush=True)
        return
    frozen = json.loads((output/'frozen_inputs.json').read_text())
    bind_correction_evidence(output, progress, frozen)
    if sha(ROOT/frozen['recipes_path']) != frozen['recipes_sha256']:
        raise ValueError('Frozen recipe file changed; refusing mixed planner candidates')
    for path, digest in frozen['files'].items():
        if sha(ROOT/path) != digest: raise ValueError('Frozen input changed: '+path)
    lock_path = ROOT/'reports/skill_trials/gpu.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a+') as lock:
        print('WAITING_GPU_LOCK', flush=True)
        fcntl.flock(lock, fcntl.LOCK_EX)
        print('GPU_LOCK_ACQUIRED', flush=True)
        progress.update(stage='running', started_at=progress.get('started_at') or stamp())
        save(output, progress)
        count = 0
        for row in progress['entries']:
            if row['retry_round'] != progress.get('retry_round', 1) or row['status'] in ('generated','failed','skipped'): continue
            if row.get('actions_this_round', 0) or row.get('attempts_this_round', 0):
                result_path = output/'scenes'/row['scene_id']/'result.json'
                result = json.loads(result_path.read_text()) if result_path.exists() else {}
                if result.get('status') not in ('generated','failed'):
                    result = error_result(row, output, 'INTERRUPTED_ONCE', 'Interrupted attempt is retained as failure; no second attempt')
                    if row['recipe'].get('action') == 'annotate_only':
                        result.update(attempts=0, per_stage_max_attempts=0, annotation_only=True, planning_attempted=False, actions_this_round=1)
                        atomic_json(result_path, result)
                result['result_path'] = relative(result_path)
                update_row(row, result); save(output, progress)
                continue
            if args.limit is not None and count >= args.limit: break
            if row['recipe'].get('action') == 'annotate_only':
                row.update(status='running', stage='annotating', actions_this_round=1,
                           attempts_this_round=0, annotation_only=True, updated_at=stamp())
                save(output, progress)
                try:
                    result = annotate_only(row, output)
                except Exception as error:
                    result = error_result(row, output, type(error).__name__, str(error))
                    result.update(attempts=0, per_stage_max_attempts=0, annotation_only=True, planning_attempted=False, actions_this_round=1)
                    atomic_json(output/'scenes'/row['scene_id']/'result.json', result)
                update_row(row, result); save(output, progress)
                print('SCENE_ANNOTATED', row['scene_id'], json.dumps(progress['retry_counts']), flush=True)
                count += 1
                continue
            row.update(status='running', stage='planning', attempts_this_round=1, actions_this_round=1, updated_at=stamp())
            save(output, progress)
            print('SCENE_START', row['scene_id'], flush=True)
            try:
                result = generation.run_scene(row, output, args.phase_timeout, retry_recipe=row['recipe'])
                old = json.loads((ROOT/row['prior_result_path']).read_text())
                if result.get('start_q') != old.get('start_q'):
                    raise ValueError('Fixed initial joint state changed')
                if result.get('composition', {}).get('source_paths_sha256') != old['composition']['source_paths_sha256']:
                    raise ValueError('Fixed source geometry binding changed')
            except Exception as error:
                result = error_result(row, output, type(error).__name__, str(error))
            update_row(row, result)
            save(output, progress)
            print('SCENE_DONE', row['scene_id'], row['status'], row['stage'], row.get('failure_code'),
                  json.dumps(progress['retry_counts']), flush=True)
            count += 1
        if all(row['status'] in ('generated','failed','skipped') for row in progress['entries']):
            progress.update(stage='complete', finished_at=stamp())
            progress['fixed_layout_integrity'] = finish_integrity(output, progress)
        save(output, progress)


if __name__ == '__main__':
    main()
