"""Trajectory repair on the exact round 8 scenes; actual episodes remain separate."""
from collections import Counter
from copy import deepcopy
from . import retry_physical_execution as execution


def previous_histories(plan):
    previous = execution.bound_json(plan['source_previous_retry_manifest'])
    if (previous.get('kind') != 'physical_failure_retry' or previous.get('stage') != 'complete'
            or previous.get('retry_round') != 8 or previous.get('schema_version') != 2
            or Counter(r['execution_status'] for r in previous['entries']) != {'failed': 1, 'planning_failed': 1}):
        raise ValueError('Expected frozen round 8: one physical failure and one planning failure')
    binding = dict(path=previous['retry_plan_path'], sha256=previous['retry_plan_sha256'])
    execution.bound_json(binding)
    prior_plan, validated = execution.validate_plan(binding['path'])
    validated = {r['scene_id']: r for r in validated}
    previous_rows = execution.rows_by_id(previous)
    if set(previous_rows) != set(validated):
        raise ValueError('Previous cohort changed')
    for key in ('source_first_pass_manifest', 'source_camera_retake_manifest'):
        if previous[key] != plan[key] or prior_plan[key] != plan[key]:
            raise ValueError('Historical manifest binding changed')
    first = execution.rows_by_id(execution.bound_json(plan['source_first_pass_manifest']))
    mutable = {'execution_status', 'attempts', 'executed', 'task_success', 'video_path', 'video_sha256'}
    histories = {}
    for sid, prior in previous_rows.items():
        for key, value in validated[sid].items():
            if key not in mutable and prior.get(key) != value:
                raise ValueError('Previous frozen input changed: ' + sid + ' ' + key)
        history = deepcopy(prior['prior_physical_attempts'])
        if prior['execution_status'] == 'failed':
            # The prior episode really used the revised scene. Validate that
            # chain first, then reuse the strict physical-evidence checks with
            # the exact revised source, retaining the original task/trajectory.
            from .scene_revision_round8 import validate_scene_revision
            checked = validate_scene_revision(execution.ROOT, prior['scene_revision'], expected_scene_id=sid, current=prior)
            source = deepcopy(first[sid])
            for key in execution.SOURCE_FIELDS:
                if key in ('config_path', 'config_sha256', 'source_scene_path', 'source_scene_sha256'):
                    source[key] = prior[key]
                elif prior[key] != source[key]:
                    raise ValueError('Revised prior changed original identity: ' + key)
            result = execution.bound_json(dict(path=prior['execution_result_path'], sha256=prior['execution_result_sha256']))
            if (result.get('scene_revision') != prior['scene_revision'] or result.get('layout_changed') is not True
                    or result.get('original_planning_result') != dict(path=prior['result_path'], sha256=prior['result_sha256'])):
                raise ValueError('Previous physical trial revision/original plan binding differs')
            for key in ('source_scene_path', 'source_scene_sha256'):
                if result.get(key) != prior[key]: raise ValueError('Previous physical scene differs')
            execution.load_retry_inputs(prior)
            history.append(execution.validate_previous_retry(prior, source, history, previous))
        else:
            if (prior.get('attempts') != 0 or prior.get('executed') is not False
                    or prior.get('task_success') is not False or prior.get('execution_result_path')):
                raise ValueError('Previous planning failure cannot contain a physical episode')
            execution.load_retry_inputs(prior)
        histories[sid] = history
    return previous_rows, histories


def validate_round9_plan(path, plan):
    for key, value in dict(schema_version=2, retry_round=9, expected_scene_count=2,
            candidate_count_per_scene=1, physical_retries_per_scene=1,
            layout_changed=False, layout_changed_from_original=True, success_predicate_changed=False).items():
        if plan.get(key) != value: raise ValueError('Round 9 scope differs: ' + key)
    previous, histories = previous_histories(plan)
    rows = execution.rows_by_id(plan)
    if set(rows) != set(histories) or len(rows) != 2:
        raise ValueError('Round 9 must contain exactly the two unresolved scenes')
    output = []
    for sid, row in rows.items():
        prior, history = previous[sid], histories[sid]
        for key in execution.SOURCE_FIELDS + ('source_trajectory_path', 'source_trajectory_sha256', 'scene_revision', 'layout_changed'):
            if row.get(key) != prior.get(key): raise ValueError('Round 9 must retain the exact prior source: ' + key)
        if (row.get('retained_scene_revision') is not True or row.get('scene_settings_changed_this_round') is not False
                or row.get('scene_settings_changed_once') is not True):
            raise ValueError('Round 9 must declare the retained layout')
        last = history[-1]
        if row['previous_retry_result'] != dict(path=last['result_path'], sha256=last['result_sha256']):
            raise ValueError('Latest actual failure binding differs')
        if row['physical_attempt_number'] != len(history) + 1:
            raise ValueError('Planning failures must not increment physical attempt count')
        entry = deepcopy(row)
        entry.update(retry_schema_version=2, retry_plan_path=str(path.relative_to(execution.ROOT)),
            retry_plan_sha256=execution.sha(path), prior_physical_attempts=history,
            execution_status='planning_failed' if row['planning_status']=='failed' else 'pending',
            attempts=0, executed=False, task_success=False, video_path=None, video_sha256=None)
        candidate,_,_=execution.load_retry_inputs(entry)
        from .lift_path_validation import validate_round9_policy
        validate_round9_policy(candidate['recipe'])
        recipe_source=execution.bound_json(row['adjustment']['recipe_source'])
        if recipe_source.get('recipes',{}).get(sid) != candidate['recipe']:
            raise ValueError('Candidate differs from the frozen recipe source')
        execution.entry_settings(entry, type('Settings', (), {})())
        output.append(entry)
    return plan, output
