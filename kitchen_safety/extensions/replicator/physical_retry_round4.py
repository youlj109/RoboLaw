"""Bind round four to the 6 unresolved scenes without inventing episodes."""
from collections import Counter
from copy import deepcopy
from . import retry_physical_execution as execution


def previous_histories(plan):
    previous = execution.bound_json(plan['source_previous_retry_manifest'])
    if (previous.get('kind') != 'physical_failure_retry' or previous.get('stage') != 'complete'
            or previous.get('retry_round') != 3 or previous.get('schema_version') != 2
            or Counter(r['execution_status'] for r in previous['entries']) !=
                {'success': 23, 'failed': 4, 'planning_failed': 2}):
        raise ValueError('Expected frozen round 3: 23 success, 4 physical failure, 2 planning failure')
    prior_plan_binding = dict(path=previous['retry_plan_path'], sha256=previous['retry_plan_sha256'])
    execution.bound_json(prior_plan_binding)
    prior_plan, validated = execution.validate_plan(prior_plan_binding['path'])
    validated = {r['scene_id']: r for r in validated}
    previous_rows = execution.rows_by_id(previous)
    if set(previous_rows) != set(validated): raise ValueError('Previous cohort changed')
    for key in ('source_first_pass_manifest', 'source_camera_retake_manifest'):
        if previous[key] != plan[key] or prior_plan[key] != plan[key]:
            raise ValueError('Historical manifest binding changed')
    histories = {}
    mutable = {'execution_status', 'attempts', 'executed', 'task_success', 'video_path', 'video_sha256'}
    first = execution.rows_by_id(execution.bound_json(plan['source_first_pass_manifest']))
    for sid, prior in previous_rows.items():
        for key, value in validated[sid].items():
            if key not in mutable and prior.get(key) != value:
                raise ValueError('Previous frozen input changed: '+sid+' '+key)
        if prior['execution_status'] == 'success': continue
        history = deepcopy(prior['prior_physical_attempts'])
        if prior['execution_status'] == 'failed':
            history.append(execution.validate_previous_retry(prior, first[sid], history, previous))
        else:
            if (prior.get('attempts') != 0 or prior.get('executed') is not False
                    or prior.get('task_success') is not False or prior.get('execution_result_path')):
                raise ValueError('Previous planning failure cannot contain a physical episode')
            execution.load_retry_inputs(prior)
        histories[sid] = history
    return previous_rows, histories


def validate_round4_plan(path, plan):
    for key, value in dict(schema_version=2, retry_round=4, expected_scene_count=6,
            candidate_count_per_scene=1, physical_retries_per_scene=1,
            layout_changed=False, success_predicate_changed=False).items():
        if plan.get(key) != value: raise ValueError('Round 4 scope differs: '+key)
    previous, histories = previous_histories(plan)
    rows = execution.rows_by_id(plan)
    if set(rows) != set(histories) or len(rows) != 6:
        raise ValueError('Round 4 must contain exactly all 6 unresolved scenes')
    output = []
    for sid, row in rows.items():
        prior, history = previous[sid], histories[sid]
        for key in (*execution.SOURCE_FIELDS, 'source_trajectory_path', 'source_trajectory_sha256'):
            if row[key] != prior[key]: raise ValueError('Original source changed: '+key)
        last = history[-1]
        if row['previous_retry_result'] != dict(path=last['result_path'], sha256=last['result_sha256']):
            raise ValueError('Latest actual failure binding differs')
        if row['physical_attempt_number'] != len(history)+1:
            raise ValueError('Planning failures must not increment physical attempt count')
        entry = deepcopy(row)
        entry.update(retry_schema_version=2, retry_plan_path=str(path.relative_to(execution.ROOT)),
            retry_plan_sha256=execution.sha(path), prior_physical_attempts=history,
            execution_status='planning_failed' if row['planning_status']=='failed' else 'pending',
            attempts=0, executed=False, task_success=False, video_path=None, video_sha256=None)
        execution.load_retry_inputs(entry)
        execution.entry_settings(entry, type('Settings', (), {})())
        output.append(entry)
    return plan, output
