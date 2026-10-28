"""Synthetic retained initialization files for CPU-only provenance tests."""
from copy import deepcopy
import hashlib
import json


def initialization_fixture(root, demos, entry, physical, count=1):
    sid = entry['scene_id']
    base = demos/'initialization_history'/sid
    history = []

    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else json.dumps(value).encode())
        return dict(path=str(path.relative_to(root)), sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    source = write(base/'shared_prior.py', b"# Synthetic source checkpoint fixture\nresult['status'] = 'executing'\n")
    log = write(base/'shared_batch.log', b'Synthetic initializing log, no actuator command\n')
    for number in range(1, count+1):
        folder = base/f'initialization_{number:02d}'
        old_path = demos/'executions'/sid
        if number > 1:
            old_path /= f'initialization_retry_{number-1:02d}'
        old = deepcopy(physical)
        old.update(status='initializing', execution_status='error', executed=False, task_success=False,
                   video_path=None, video_sha256=None, frame_count=0, action_count=0,
                   initialization_history=deepcopy(history),
                   execution_source_sha256_start={'kitchen_safety/extensions/replicator/planned_execution.py':source['sha256']})
        old_ref = write(old_path/'result.json', old)
        prior_row = deepcopy(entry)
        prior_row.update(execution_status='running', executed=False, task_success=False,
                         video_path=None, video_sha256=None, initialization_history=deepcopy(history))
        before = write(folder/'prior_manifest.json', dict(kind='physical_execution', entries=[prior_row]))
        stop = write(folder/'operator_stop.json', dict(pid=1234, result_sha256=old_ref['sha256'],
            status_before_signal='initializing', pid_absent_after_escalation=True))
        diagnosis = write(folder/'diagnosis.json', dict(scene_id=sid, assessment='Synthetic test fixture'))
        event = dict(schema_version=1, kind='initialization_interruption', scene_id=sid,
            execution_started=False, prior_process_id=1234, prior_process_confirmed_stopped=True,
            no_actuator_commands_verified=True, reason='Synthetic initialization interruption',
            checkpoint_basis='Synthetic checkpoint before actuator commands',
            counter_interpretation='Initialization start is not a physical episode', prior_result=old_ref,
            prior_source=source, prior_log=log, prior_manifest=before, operator_stop=stop, diagnosis=diagnosis)
        ref = write(folder/'recovery_event.json', event)
        history.append(dict(kind=event['kind'], execution_started=False, reason=event['reason'],
            prior_result_path=old_ref['path'], prior_result_sha256=old_ref['sha256'],
            recovery_event_path=ref['path'], recovery_event_sha256=ref['sha256']))
    subdirectory = f'initialization_retry_{count:02d}'
    entry.update(initialization_history=history, execution_subdirectory=subdirectory,
                 execution_result_path=f'executions/{sid}/{subdirectory}/result.json')
    physical.update(initialization_history=deepcopy(history), execution_subdirectory=subdirectory)
    return history
