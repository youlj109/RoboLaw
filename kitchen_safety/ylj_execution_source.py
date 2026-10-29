"""Select an explicitly frozen ylj adopted revision without falling back to old NPZs.

Selection and source gates only: no simulator, planner, GPU query or risk grading.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

LEGACY_INDEX = 'configs/extensions/replicator/final_154_scenes_20261028.json'
SOURCE_LOCK = 'configs/risk_integration/ylj_execution_source.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local(root, value):
    root = Path(root).resolve()
    path = (root / str(value).removeprefix('/workspace/')).resolve()
    if root not in path.parents:
        raise ValueError('ylj input outside checkout: ' + str(value))
    return path


def bound(root, ref):
    path = local(root, ref['path'])
    if sha(path) != ref['sha256']:
        raise ValueError('ylj source SHA differs: ' + str(path))
    return json.loads(path.read_text())


def execution_index(root, source='legacy'):
    root = Path(root)
    original = json.loads((root / LEGACY_INDEX).read_text())
    if source == 'legacy':
        return original
    if source != 'current':
        raise ValueError('Unknown ylj source revision')
    lock = json.loads((root / SOURCE_LOCK).read_text())
    if sha(root / LEGACY_INDEX) != lock['legacy_index_sha256']:
        raise ValueError('Historical input index changed')
    published = bound(root, lock['publication'])
    if published.get('partial_preview') is not False:
        raise ValueError('Adopted revision publication is incomplete')
    base = {x['scene_id']: x for x in original['entries']}
    rows = published['entries']
    if len(base) != 154 or len(rows) != 154 or {x['scene_id'] for x in rows} != set(base):
        raise ValueError('Adopted revision must cover the same exact 154 scenes')
    output = []
    for row in rows:
        sid = row['scene_id']
        if row['task_id'] != base[sid]['task_id'] or row['variant_id'] != base[sid]['variant_id']:
            raise ValueError('Adopted revision changed native task/variant identity')
        result_ref = row['current_revision_result']
        layers = [('first_pass', row.get('first_pass')), ('camera_retake', row.get('camera_retake')),
                  ('retry', row.get('retry'))]
        layers += [('retry_round' + str(n), row.get('retry_round' + str(n))) for n in range(2, 14)]
        matches = [(name, layer) for name, layer in layers if layer and layer.get('result') == result_ref]
        if len(matches) != 1:
            raise ValueError('Adopted result does not resolve to one exact source layer: ' + sid)
        layer_name, layer = matches[0]
        if layer.get('executed') is not True or layer.get('execution_status') != 'success' or layer.get('task_success') is not True:
            raise ValueError('Published adopted source is not an executed success: ' + sid)
        entry = deepcopy(base[sid])
        binding = dict(source='current', source_commit=lock['source_commit'], publication=lock['publication'],
                       selected_layer=layer_name, adopted_result=result_ref,
                       adopted_config=row['current_revision_config'], scene_revision=row.get('current_revision'),
                       safety_acceptance_inferred=False)
        if layer_name not in ('first_pass', 'camera_retake'):
            number = 1 if layer_name == 'retry' else int(layer_name.removeprefix('retry_round'))
            plan_ref = lock['retry_plans'][str(number)]
            plan = bound(root, plan_ref)
            selected = [x for x in plan['entries'] if x['scene_id'] == sid]
            if len(selected) != 1:
                raise ValueError('Missing unique frozen retry candidate: ' + sid)
            retry = deepcopy(selected[0])
            if retry['task_id'] != row['task_id'] or retry['variant_id'] != row['variant_id']:
                raise ValueError('Frozen retry candidate changed identity')
            trajectory_ref = layer['adjusted_trajectory']
            if trajectory_ref != dict(path=retry['trajectory_path'], sha256=retry['trajectory_sha256']):
                raise ValueError('Published trajectory and frozen plan disagree')
            if number >= 2:
                retry['retry_schema_version'] = 2
                if retry.get('planning_status') != 'generated' or retry.get('candidate_planning_result') != layer.get('candidate_planning_result'):
                    raise ValueError('Published candidate and frozen plan disagree')
                result = retry['candidate_planning_result']
            else:
                result = dict(path=retry['result_path'], sha256=retry['result_sha256'])
            if row.get('current_revision') != retry.get('scene_revision'):
                raise ValueError('Published layout revision and candidate disagree')
            entry.update(config_path=retry['config_path'], config_sha256=retry['config_sha256'],
                         source_result_path=result['path'], source_result_sha256=result['sha256'],
                         source_scene=dict(path=retry['source_scene_path'], sha256=retry['source_scene_sha256']))
            entry['artifacts']['trajectory'] = trajectory_ref
            entry['ylj_retry_entry'] = retry
            binding['retry_plan'] = plan_ref
            binding['execution_settings'] = deepcopy(plan['execution_settings'])
        else:
            if layer.get('config', row['current_revision_config']) != row['current_revision_config']:
                raise ValueError('First-pass adopted config differs')
        if row['current_revision_config'] != dict(path=entry['config_path'], sha256=entry['config_sha256']):
            raise ValueError('Adopted config does not match selected source')
        entry['ylj_source_binding'] = binding
        output.append(entry)
    return dict(entries=output, source='current', source_commit=lock['source_commit'])


def required_source_bindings(entry):
    refs = [dict(path=entry[k + '_path'], sha256=entry[k + '_sha256'])
            for k in ('config', 'result', 'trajectory', 'source_scene')]
    retry = entry.get('ylj_retry_entry')
    binding = entry.get('ylj_source_binding')
    if binding:
        refs.append(binding['publication'])
        if binding.get('retry_plan'): refs.append(binding['retry_plan'])
    if retry:
        for role in ('result', 'source_trajectory'):
            refs.append(dict(path=retry[role + '_path'], sha256=retry[role + '_sha256']))
        refs.append(dict(path=retry['adjustment']['evidence_path'], sha256=retry['adjustment']['evidence_sha256']))
        if retry.get('scene_revision'): refs.append(retry['scene_revision'])
    return {x['path']: x['sha256'] for x in refs}


def load_selected_inputs(root, entry):
    if entry.get('ylj_source_binding'):
        from kitchen_safety.ylj_risk_capture import select_entry
        expected = select_entry(execution_index(root, 'current'), entry['scene_id'])
        if expected != entry:
            raise ValueError('Selected ylj source changed after preflight')
    if entry.get('ylj_retry_entry'):
        from kitchen_safety.extensions.replicator.retry_physical_execution import load_retry_inputs
        return load_retry_inputs(entry['ylj_retry_entry'])
    from kitchen_safety.extensions.replicator.planned_execution import load_inputs
    return load_inputs(entry)


def apply_execution_overrides(entry, args):
    if not entry.get('ylj_retry_entry'):
        return args
    from kitchen_safety.extensions.replicator.retry_physical_execution import entry_settings
    return entry_settings(entry['ylj_retry_entry'], args)
