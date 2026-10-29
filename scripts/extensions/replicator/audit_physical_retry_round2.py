#!/usr/bin/env python3
"""Independent read-only CPU audit of the 48-scene second physical retry.

Only the requested audit JSON is written. A planning failure must have zero
physical attempts. Completed native episodes are checked against their new
candidate, exact actuator commands, original predicates, and all prior trials.
No Kit, physics, planning, rendering, USD stage, or GPU is initialized here.
"""
import argparse
import math
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DEMOS = 'exports/replicator_physical_retry_round2_20261028'
DEFAULT_OUTPUT = 'reports/replicator_physical_retry_round2_20261028/audit_execution.json'
TERMINAL = {'success', 'failed', 'error', 'planning_failed'}
PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')
SOURCE_KEYS = ('scene_id', 'task_id', 'variant_id', 'result_path', 'result_sha256',
               'config_path', 'config_sha256', 'source_scene_path', 'source_scene_sha256')
HASHES = {}


def require(value, message):
    if not value:
        raise ValueError(message)


def local(value):
    path = (ROOT / str(value).removeprefix('/workspace/')).resolve()
    require(path.is_relative_to(ROOT), 'Path outside checkout: ' + str(path))
    return path


def digest(path):
    path = Path(path)
    stat = path.stat()
    key = (str(path), stat.st_size, stat.st_mtime_ns)
    if key not in HASHES:
        with path.open('rb') as stream:
            HASHES[key] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return HASHES[key]


def bound(path, expected):
    path = local(path)
    require(path.is_file() and path.stat().st_size > 0, 'Missing/empty file: ' + str(path))
    require(digest(path) == expected, 'SHA mismatch: ' + str(path))
    return path


def read(path):
    return json.loads(Path(path).read_text())


def ref(path):
    path = local(path)
    return dict(path=str(path.relative_to(ROOT)), sha256=digest(path), bytes=path.stat().st_size)


def file_binding(path):
    """Use the exact two-field binding expected by downstream publishers."""
    value = ref(path)
    return {key: value[key] for key in ('path', 'sha256')}


def bound_json(value):
    return read(bound(value['path'], value['sha256']))


def rows_by_id(document):
    rows = document['entries']
    if isinstance(rows, dict):
        rows = list(rows.values())
    require(len({row['scene_id'] for row in rows}) == len(rows), 'Duplicate scene IDs')
    return {row['scene_id']: row for row in rows}


def merged_settings(settings, overrides):
    require(isinstance(overrides, dict) and not set(overrides) - {'close_seconds', 'release_seconds'},
            'Only close/release duration overrides are authorized')
    values = dict(settings)
    for key, value in overrides.items():
        require(not isinstance(value, bool) and isinstance(value, (int, float))
                and np.isfinite(value) and .1 <= value <= 10., 'Invalid hold duration override')
        values[key] = value
    return values


def prior_binding(row):
    return dict(result_path=row['execution_result_path'], result_sha256=row['execution_result_sha256'],
        execution_status=row['execution_status'], task_success=row['task_success'], executed=row['executed'],
        physical_attempt_number=row.get('physical_attempt_number', 1), video_sha256=row.get('video_sha256'))


def audit_history(row, recipe, context):
    """Read small bound result JSONs only; old native arrays are not reread."""
    sid = row['scene_id']
    original = context['original'][sid]
    previous = context['previous'][sid]
    prior_rows = [original]
    if sid in context['retakes']:
        prior_rows.append(context['retakes'][sid])
    prior_rows.append(previous)
    history = [prior_binding(prior) for prior in prior_rows]
    require(row['prior_physical_attempts'] == history, 'Prior physical history differs from bound manifests')
    require(previous['prior_physical_attempts'] == history[:-1], 'Round 1 omitted earlier physical history')
    require(row['previous_retry_result'] == recipe['previous_retry_result'] == {
        'path': previous['execution_result_path'], 'sha256': previous['execution_result_sha256']},
        'Wrong previous-round result binding')
    require(row['physical_attempt_number'] == recipe['physical_attempt_number'] == len(history) + 1,
            'Next physical attempt number omits history')
    for key in SOURCE_KEYS:
        require(row[key] == recipe[key] == original[key] == previous[key], 'Historical scene source differs: ' + key)
    require(row['source_trajectory_path'] == original['trajectory_path']
            and row['source_trajectory_sha256'] == original['trajectory_sha256'], 'Original trajectory identity differs')
    prior_results = []
    for number, (prior, saved) in enumerate(zip(prior_rows, history), 1):
        result = read(bound(saved['result_path'], saved['result_sha256']))
        require(result.get('physical_attempt_number', 1) == saved['physical_attempt_number'] == number,
                'Physical lineage is not consecutive')
        require(result['scene_id'] == sid and result['status'] == 'complete' and result['executed'] is True
                and result['execution_status'] == 'failed' and result['task_success'] is False,
                'History contains an unexecuted, incomplete, or successful trial')
        for key in ('task_id', 'variant_id', 'config_path', 'config_sha256', 'trajectory_path', 'trajectory_sha256'):
            require(result[key] == prior[key], 'Prior result differs from its own manifest: ' + key)
        require(result['config_sha256'] == row['config_sha256'], 'Prior layout differs')
        require(result['planning_result_path'] == original['result_path']
                and result['planning_result_sha256'] == original['result_sha256'], 'Prior planning source differs')
        if prior is previous:
            require(result['kind'] == 'physical_failure_retry_execution' and result['attempts'] == 1,
                    'Previous retry is not one native trial')
            require(result['prior_physical_attempts'] == history[:-1], 'Previous result history differs')
            require(result['source_trajectory_path'] == row['source_trajectory_path']
                    and result['source_trajectory_sha256'] == row['source_trajectory_sha256'],
                    'Previous retry original trajectory binding differs')
            for key in ('retry_plan_path', 'retry_plan_sha256'):
                require(result[key] == prior[key] == context['previous_manifest'][key], 'Previous retry plan differs')
        else:
            require(result['kind'] == 'physical_execution', 'Unexpected original physical result kind')
            require(result['trajectory_sha256'] == row['source_trajectory_sha256'], 'Original/camera trajectory differs')
            if number == 2:
                require(prior['first_pass']['result_sha256'] == history[0]['result_sha256'], 'Camera history omits first trial')
        # Binding each old trajectory is small and does not load old physics arrays.
        bound(result['trajectory_path'], result['trajectory_sha256'])
        prior_results.append(result)
    return prior_results


def audit_candidate(row, recipe):
    """The new planner output is distinct from the retained original result."""
    for key, value in recipe.items():
        require(row.get(key) == value, 'Manifest differs from frozen retry recipe: ' + key)
    require(row.get('retry_schema_version') == 2, 'Wrong retry entry schema')
    for path_key, hash_key in (('result_path', 'result_sha256'), ('config_path', 'config_sha256'),
            ('source_scene_path', 'source_scene_sha256'), ('source_trajectory_path', 'source_trajectory_sha256')):
        bound(row[path_key], row[hash_key])
    adjustment = recipe['adjustment']
    require(adjustment['candidate_count'] == 1 and adjustment['description'], 'Missing single-candidate adjustment')
    bound(adjustment['evidence_path'], adjustment['evidence_sha256'])
    recipe_document = bound_json(adjustment['recipe_source'])
    declared = recipe_document['recipes'][row['scene_id']]
    candidate = bound_json(row['candidate_planning_result'])
    old = read(local(row['result_path']))
    require(local(row['candidate_planning_result']['path']) != local(row['result_path']), 'New candidate overwrites historical planning result')
    for key in ('scene_id', 'task_id', 'variant_id', 'config_path', 'config_sha256'):
        require(candidate[key] == row[key], 'Candidate scene identity differs: ' + key)
    require(candidate['recipe'] == declared, 'Candidate recipe differs from its frozen source')
    require(candidate['prior_result_path'] == row['result_path']
            and candidate['prior_result_sha256'] == row['result_sha256'], 'Candidate omits old planning source')
    for key, value in dict(planning_only=True, executed=False, attempts=1, per_stage_max_attempts=1,
                           candidate_count=1, layout_fixed=True, task_success_verified=False).items():
        require(candidate.get(key) == value, 'Candidate scope differs: ' + key)
    require(candidate.get('scene_settings_changed_once', False) is False, 'Candidate changed scene settings')
    require(np.asarray(candidate['start_q']).shape == (7,)
            and np.allclose(candidate['start_q'], old['start_q'], atol=1e-5, rtol=0), 'Candidate changed frozen start q')
    require(candidate['composition']['source_paths_sha256'] == old['composition']['source_paths_sha256'],
            'Candidate source geometry differs')
    for path, sha in candidate['composition']['source_paths_sha256'].items():
        bound(path, sha)
    require(candidate.get('collision_geometry_unchanged') is True and candidate.get('added_hazard_masks') is False,
            'Candidate changed collision geometry or added hazard masks')
    phases = candidate['phases']
    require(tuple(phase['phase'] for phase in phases) == PHASES[:len(phases)] and len(phases) <= len(PHASES),
            'Planner repeated or reordered phases')
    old_masks = {phase['phase']: phase['contact_masks'] for phase in old['phases']}
    for phase in phases:
        require(phase['contact_masks'] == old_masks[phase['phase']], 'Candidate changed phase contact masks')
    require(row['execution_overrides'] == declared.get('execution_overrides', {}), 'Execution override differs from declared recipe')
    merged_settings({}, row['execution_overrides'])
    require(candidate['status'] == row['planning_status'], 'Planning state mismatch')
    if row['planning_status'] == 'failed':
        require(candidate['trajectory_complete'] is False and row.get('failure_reason')
                and row['trajectory_path'] is None and row['trajectory_sha256'] is None,
                'Failed candidate has an executable trajectory or no reason')
        return candidate
    require(row['planning_status'] == 'generated' and candidate['trajectory_complete'] is True, 'Incomplete generated candidate')
    require(tuple(phase['phase'] for phase in phases) == PHASES
            and all(phase['solver_success'] is True and phase['status'] == 'generated' for phase in phases),
            'Candidate lacks six successful phases')
    grip = candidate['nominal_gripper_joint_rad']
    require(0 < grip <= .785 and grip == declared['nominal_gripper_joint_rad'], 'Invalid nominal execution gripper')
    planning_grip = candidate['planning_gripper_joint_rad']
    require(0 < planning_grip <= .785
            and planning_grip == declared.get('planning_gripper_joint_rad', grip), 'Invalid estimated planning gripper')
    raw_ref = candidate['artifacts']['trajectory']
    raw_path = bound(raw_ref['path'], raw_ref['sha256'])
    retimed_path = bound(row['trajectory_path'], row['trajectory_sha256'])
    require(len({raw_path, retimed_path, local(row['source_trajectory_path'])}) == 3, 'Raw/new/original trajectory paths overlap')
    with np.load(raw_path, allow_pickle=False) as raw, np.load(retimed_path, allow_pickle=False) as retimed:
        audit_retiming(raw, retimed, grip, candidate['start_q'])
    return candidate


def audit_retiming(raw, retimed, grip, start_q):
    """Independently reconstruct the new retimed path from its source edges."""
    for data in (raw, retimed):
        n = len(data['q'])
        require(data['q'].shape == (n, 7) and data['gripper'].shape == data['phase'].shape == (n,)
                and data['tcp_world'].shape == (n, 4, 4), 'Malformed candidate arrays')
        require(n >= 6 and tuple(dict.fromkeys(data['phase'].tolist())) == PHASES, 'Candidate phase order differs')
        for key in ('q', 'gripper', 'tcp_world'):
            require(np.isfinite(data[key]).all(), 'Nonfinite candidate arrays: ' + key)
        require(np.isclose(float(data['dt']), 1/30, atol=1e-12, rtol=0), 'Candidate control dt differs')
        carried = np.isin(data['phase'], ('lift', 'transfer', 'lower'))
        require(np.allclose(data['gripper'][carried], grip, atol=1e-8, rtol=0)
                and np.all(data['gripper'][~carried] == 0), 'NPZ does not use actual nominal execution grip')
    require(np.allclose(raw['q'][0], start_q, atol=1e-5, rtol=0), 'Raw candidate starts from a changed q')
    a, b, f, vertices = (retimed[key] for key in ('source_edge_start_index', 'source_edge_end_index',
                                               'source_edge_fraction', 'source_vertex_output_index'))
    n = len(retimed['q'])
    require(a.shape == b.shape == f.shape == (n,) and vertices.shape == (len(raw['q']),), 'Retiming index lengths differ')
    require(all(index.dtype.kind in 'iu' for index in (a, b, vertices)), 'Noninteger retiming indices')
    require(np.all(a >= 0) and np.all(b < len(raw['q'])) and np.all((b-a >= 0) & (b-a <= 1))
            and np.isfinite(f).all() and np.all((f >= 0) & (f <= 1))
            and np.all(vertices >= 0) and np.all(vertices < n) and np.all(np.diff(vertices) > 0),
            'Invalid retiming edge/vertex mapping')
    location = a + f*(b-a)
    require(location[0] == 0 and location[-1] == len(raw['q'])-1 and np.all(np.diff(location) >= -1e-12),
            'Retiming skips or reverses the source polyline')
    # The frozen retiming policy promotes source float32 joints before arithmetic.
    source_q = np.asarray(raw['q'], dtype=np.float64)
    expected = source_q[a] + f[:, None]*(source_q[b]-source_q[a])
    require(np.max(np.abs(retimed['q']-expected)) <= 1e-12
            and np.array_equal(retimed['q'][vertices], raw['q']), 'Retiming changes candidate joints/vertices')
    require(np.array_equal(retimed['phase'], raw['phase'][b])
            and np.array_equal(retimed['gripper'], raw['gripper'][b]), 'Retiming changes phase/grip')
    require(np.allclose(retimed['tcp_world'][vertices], raw['tcp_world'], atol=2e-6, rtol=0), 'Retiming changes source TCP vertices')
    for q in (retimed['q'], retimed['q'].astype(np.float32)):
        velocity = np.diff(np.vstack([q[0], q, q[-1]]).astype(float), axis=0)*30
        acceleration = np.diff(velocity, axis=0)*30
        require(np.max(np.abs(velocity)) <= .4+1e-9 and np.max(np.abs(acceleration)) <= .8+1e-9,
                'Retimed actuator velocity/acceleration exceeds original limits')


def expected_command_stream(plan,data,settings):
    actions=[]
    def add(q,g,phase,index=-1):
        actions.append(dict(phase=phase,source_trajectory_index=index,target=np.r_[q,g].astype(np.float32)))
    def hold(q,g,phase,key):
        for _ in range(int(math.ceil(settings[key]/(1/30)))): add(q,g,phase)
    hold(plan['start_q'],0.,'initial_settle','initial_settle_seconds')
    previous=None
    for i,(q,phase,g) in enumerate(zip(data['q'],data['phase'],data['gripper'])):
        if phase!=previous:
            if phase=='lift': hold(data['q'][i-1],float(g),'close','close_seconds')
            if phase=='retreat': hold(data['q'][i-1],0.,'release','release_seconds')
        add(q,g,str(phase),i);previous=phase
    hold(data['q'][-1],0.,'final_settle','final_settle_seconds')
    return actions


def predicate_components(data,metadata):
    # Same pinned xyzw inverse-rotation algebra, independently evaluated from measured arrays.
    destination=data['destination_pose'];vector=data['object_centroid_world']-destination[:,:3]
    xyz=destination[:,3:6]; t=np.cross(xyz,vector)*2
    local_center=vector-destination[:,6:7]*t+np.cross(xyz,t)
    box=metadata['actual_spawned_local_bounds'][metadata['destination_location']]
    low=np.asarray(box['min'],dtype=local_center.dtype);high=np.asarray(box['max'],dtype=local_center.dtype)
    within=np.all((local_center[:,:2]>=low[:2]) & (local_center[:,:2]<=high[:2]),axis=1) & (local_center[:,2]>=low[2])
    threshold=metadata['thresholds']; force=data['destination_contact_force']; magnitude=np.linalg.norm(force,axis=1)
    support=(magnitude>=threshold['force_threshold']) & (force[:,2]>0) & (force[:,2]>=magnitude*math.cos(threshold['support_cone_half_angle_rad']))
    slow=np.linalg.norm(data['object_linear_velocity'],axis=1)<threshold['velocity_threshold']
    return dict(within_destination=within,upward_support=support,slow=slow)


def audit_entry(row, recipe, revisions, demos, retry_plan, context):
    # The manifest, not a guessed scene directory, selects recovery child results.
    prior_results = audit_history(row, recipe, context)
    plan = audit_candidate(row, recipe)
    rp = bound(row['execution_result_path'], row['execution_result_sha256'])
    require(rp.is_relative_to(demos / 'executions'), 'Physical result outside execution directory')
    result = read(rp)
    for key in ('scene_id', 'task_id', 'variant_id', 'config_path', 'config_sha256',
                'trajectory_path', 'trajectory_sha256', 'execution_status', 'task_success',
                'attempts', 'executed', 'runtime_revision_id', 'physical_attempt_number',
                'source_trajectory_path', 'source_trajectory_sha256', 'prior_physical_attempts',
                'retry_plan_path', 'retry_plan_sha256', 'retry_schema_version', 'candidate_planning_result',
                'previous_retry_result', 'execution_overrides', 'adjustment', 'planning_status'):
        require(row.get(key) == result.get(key), 'Manifest/result mismatch: ' + key)
    require(result['kind'] == 'physical_failure_retry_execution' and result['status'] == 'complete', 'Not a terminal physical result')
    require(result['executed'] is True and type(result['attempts']) is int and result['attempts'] == 1,
            'Expected one actual physical episode')
    require(result['physical_execution'] is True, 'Missing native execution flag')
    for key in ('runtime_state_setters_used', 'physical_weld_used', 'payload_attachment_used', 'planning_rerun'):
        require(result.get(key) is False, 'Forbidden physical substitution: ' + key)
    require(result.get('source_inputs_unchanged') is True and result.get('execution_sources_unchanged') is True,
            'Inputs or code changed during execution')
    revision = revisions[result['runtime_revision_id']]
    require(result['execution_source_sha256_start'] == result['execution_source_sha256_end'] == revision['source_sha256'],
            'Execution code differs from its recorded runtime revision')
    require(result['runtime_settings'] == merged_settings(revision['settings'], recipe['execution_overrides']),
            'Runtime settings do not match revision plus declared close/release overrides')
    # Revision 0 predates the gate summary fields. Audit actual states below in
    # every revision; never manufacture a missing legacy runtime gate result.
    if 'reset_state_validation' in result:
        require(result['reset_state_validation']['passed'] is True, 'Frozen initial state gate failed')
    for path_key, hash_key in (('result_path', 'result_sha256'), ('config_path', 'config_sha256'),
                               ('trajectory_path', 'trajectory_sha256'), ('source_scene_path', 'source_scene_sha256')):
        bound(row[path_key], row[hash_key])
    require(result['original_planning_result'] == {'path': row['result_path'], 'sha256': row['result_sha256']},
            'Result omits original historical planning source')
    require(local(result['planning_result_path']) == local(row['candidate_planning_result']['path'])
            and result['planning_result_sha256'] == row['candidate_planning_result']['sha256'],
            'Actual execution does not bind its new candidate planning result')
    for path, sha in plan['composition']['source_paths_sha256'].items():
        bound(path, sha)
    overlay = result['physical_overlay_binding']
    require(overlay['unchanged'] is True and overlay['sha256_start'] == overlay['sha256_end'], 'Overlay changed')
    bound(overlay['path'], overlay['sha256_start'])
    physics = result['physics_evidence']
    require(physics['continuous'] is True and physics['complete'] is True, 'Incomplete native physics evidence')
    if 'physics_evidence_gate' in result:
        require(result['physics_evidence_gate'] == {'required': True, 'passed': True}, 'Native physics gate failed')
    qp = bound(physics['q_path'], physics['q_sha256'])
    mp = bound(physics['metadata_path'], physics['metadata_sha256'])
    require(qp.parent == mp.parent == rp.parent, 'Physics files do not belong to selected episode')
    metadata = read(mp)
    for key in ('runtime_state_setters_used', 'physical_weld_used', 'payload_attachment_used'):
        require(metadata.get(key) is False, 'Native recorder reports forbidden substitution: ' + key)
    actions_path = rp.parent / 'actions.json'
    actions = read(actions_path)
    require(len(actions) == result['action_count'] > 0, 'Missing or mismatched actuator commands')
    before = np.array([a['before_physics_step'] for a in actions], dtype=np.int64)
    after = np.array([a['after_physics_step'] for a in actions], dtype=np.int64)
    require([a['action_index'] for a in actions] == list(range(len(actions))), 'Non-contiguous action indices')
    require(np.all(after - before == 4) and np.array_equal(before[1:], after[:-1]), 'Missing/duplicated physics substeps between actions')
    require(before[0] == metadata['reset_physics_step'], 'Action stream does not start at recorded reset')
    expected = 1 + int(np.sum(after - before))
    with np.load(qp, allow_pickle=False) as data:
        steps = data['physics_step']
        n = len(steps)
        require(n == expected == physics['samples'] == physics['expected_post_reset_samples'] == metadata['samples'], 'Physics sample count mismatch')
        require(steps[0] == before[0] and steps[-1] == after[-1] and np.all(np.diff(steps) == 1), 'Native physics stream is discontinuous')
        require(data['q'].shape == data['dq'].shape == (n, len(metadata['joint_names'])), 'Joint record shape mismatch')
        for key in data.files:
            values = data[key]
            require(values.shape[0] == n, 'Physics field sample count mismatch: ' + key)
            if values.dtype.kind in 'fiu':
                require(np.isfinite(values).all(), 'Nonfinite physics values: ' + key)
        physics_action_index = data['action_index'].copy()
        physics_phase = data['phase'].copy()
        physics_source_index = data['source_trajectory_index'].copy()
        require(physics_action_index[0] == physics_source_index[0] == -1 and physics_phase[0] == 'reset',
                'Native stream does not begin with the sole reset observation')
        native_object_states = data['task_object_states'].copy()
        native_destination_force = data['destination_contact_force'][:, 0, 0].copy()
        arm_ids = [metadata['joint_names'].index('panda_joint' + str(i)) for i in range(1, 8)]
        require(np.allclose(data['q'][0, arm_ids], result['reset_observed_arm_q'], atol=1e-7, rtol=0), 'Reset q does not match physical record')
        initial_q_error = float(np.max(np.abs(data['q'][0, arm_ids] - np.asarray(plan['start_q']))))
        require(initial_q_error <= .01, 'Recorded initial q differs from frozen planning q')
        layout = read(local(row['config_path']))
        for name, state in zip(metadata['object_names'], data['task_object_states'][0]):
            spec = layout['objects'][name]
            half = np.deg2rad(spec.get('yaw_deg', 0.)) / 2
            expected_quat = np.array([0., 0., np.sin(half), np.cos(half)])
            quat = state[3:7]
            require(.99 < np.linalg.norm(quat) < 1.01, 'Invalid initial object quaternion')
            angle = float(np.rad2deg(2*np.arccos(np.clip(abs(np.dot(quat/np.linalg.norm(quat), expected_quat)), 0., 1.))))
            require(np.linalg.norm(state[:3]-spec['position']) <= .002 and angle <= 1.,
                    'Recorded initial object pose differs from frozen scene: ' + name)
    require(metadata['continuous_substeps'] is True and metadata['callback_errors'] == [], 'Native recorder errors')
    require(abs(metadata['physics_dt_s'] - 1/120) < 1e-12 and abs(metadata['control_dt_s'] - 1/30) < 1e-12, 'Unexpected execution timing')
    with np.load(local(row['trajectory_path']), allow_pickle=False) as planned:
        expected_actions = expected_command_stream(plan, planned, result['runtime_settings'])
        require(len(actions) <= len(expected_actions), 'Commands exceed the single planned attempt')
        for action, expected_action in zip(actions, expected_actions):
            require(action['phase'] == expected_action['phase'] and action['source_trajectory_index'] == expected_action['source_trajectory_index']
                    and np.array_equal(np.asarray(action['target'],dtype=np.float32),expected_action['target']),
                    'Actual command/hold differs from adjusted NPZ and original holds')
        if result.get('sequence_complete'):
            require(len(actions) == len(expected_actions), 'Complete episode omitted commands/holds')
        source_actions = [a for a in actions if a['source_trajectory_index'] >= 0]
        require([a['source_trajectory_index'] for a in source_actions] == list(range(len(source_actions))), 'Planned action sequence skipped/repeated')
        for action in source_actions:
            index = action['source_trajectory_index']
            target = np.r_[planned['q'][index], planned['gripper'][index]].astype(np.float32)
            require(action['phase'] == str(planned['phase'][index])
                    and np.array_equal(np.asarray(action['target'], dtype=np.float32), target), 'Executed target differs from frozen trajectory')
        if result.get('sequence_complete'):
            require(len(source_actions) == len(planned['q']), 'Complete sequence omitted planning samples')
    predicate_metadata_path = rp.parent / 'task_predicate_metadata.json'
    predicate_metadata = read(predicate_metadata_path)
    require(predicate_metadata['task_id'] == row['task_id'], 'Wrong task predicate metadata')
    for name, sha in predicate_metadata['input_sha256'].items():
        bound(rp.parent / name, sha)
    with np.load(rp.parent / 'task_predicate_actions.npz', allow_pickle=False) as predicates:
        require(np.array_equal(predicates['physics_step'], after), 'Action predicates not synchronized to native steps')
        require(np.array_equal(predicates['action_index'], np.arange(len(actions))), 'Predicate action indices differ')
        observed = bool(np.any(predicates['terminal_success']))
        final = bool(predicates['original_object_on_destination'][-1])
        if 'original_task_success' in result:
            require(result['original_task_success'] is observed, 'Original task success differs from recorded predicate')
        if 'final_original_destination_predicate' in result:
            require(result['final_original_destination_predicate'] is final, 'Final destination predicate mismatch')
        require(result['task_success'] is (result['execution_status'] == 'success'), 'Success/status contradiction')
        if result['task_success']:
            require(observed and final and result.get('sequence_complete') is True, 'Success lacks original terminal/final predicate')
    with np.load(rp.parent / 'task_predicate_physics.npz', allow_pickle=False) as predicates:
        require(np.array_equal(predicates['physics_step'], steps), 'Physics predicate timeline differs')
    require(np.array_equal(physics_action_index[1:], np.repeat(np.arange(len(actions)),4)), 'Physics/action indices differ')
    require(np.array_equal(physics_phase[1:], np.repeat([x['phase'] for x in actions],4)), 'Physics phase differs from command')
    require(np.array_equal(physics_source_index[1:], np.repeat([x['source_trajectory_index'] for x in actions],4)), 'Physics source index differs')
    for key in ('source_first_pass_manifest','source_camera_retake_manifest','source_previous_retry_manifest'):
        require(result[key] == retry_plan[key], 'Original manifest binding differs')
        bound(result[key]['path'],result[key]['sha256'])
    for key,value in recipe.items():
        require(row.get(key) == value, 'Manifest differs from frozen retry recipe: '+key)
    bound(row['source_trajectory_path'],row['source_trajectory_sha256'])
    evidence = result['evidence']
    for name,binding in evidence.items():
        require(local(binding['path']).parent == rp.parent and local(binding['path']).name == name, 'Evidence path outside selected episode')
        bound(binding['path'],binding['sha256'])
    required_evidence={'actions.json','physics_states.npz','physics_metadata.json','grasp_contacts.json',
        'task_predicate_actions.npz','task_predicate_physics.npz','task_predicate_metadata.json'}
    require(required_evidence <= set(evidence), 'Incomplete evidence file manifest')
    first = prior_results[0]
    require(result['physical_attempt_number']==len(result['prior_physical_attempts'])+1,'New attempt number omits history')
    require(result['controller']==first['controller'],'Controller/physics settings changed')
    require(result['initial_object_pose_reference']==first['initial_object_pose_reference'],'Object initial poses changed')
    require(result['physical_overlay_binding']['sha256_start']==first['physical_overlay_binding']['sha256_start'],'Physical kitchen overlay changed')
    first_metadata_path = local(result['prior_physical_attempts'][0]['result_path']).with_name('task_predicate_metadata.json')
    first_metadata=read(first_metadata_path)
    previous_metadata_binding = prior_results[-1]['evidence']['task_predicate_metadata.json']
    previous_metadata = bound_json(previous_metadata_binding)
    require(predicate_metadata['thresholds']==first_metadata['thresholds'],'Native predicate thresholds changed')
    require(predicate_metadata['actual_spawned_local_bounds']==first_metadata['actual_spawned_local_bounds'],'Native predicate geometry changed')
    require(predicate_metadata['sensor_identity']==predicate_metadata['sensor_identity_at_end']==first_metadata['sensor_identity'],'Original contact target/filter identity changed')
    for key in ('thresholds', 'actual_spawned_local_bounds', 'sensor_identity', 'pick_up_object', 'destination_location'):
        require(predicate_metadata[key] == previous_metadata[key], 'Predicate differs from source-bound round 1 evidence: ' + key)
    for source, expected_sha in first['execution_source_sha256_start'].items():
        if '/tasks/' in source or source.endswith('task_collection_evidence.py'):
            require(result['execution_source_sha256_start'].get(source) == expected_sha,
                    'Original task/predicate source changed: ' + source)
    for source, expected_sha in prior_results[-1]['execution_source_sha256_start'].items():
        if source.endswith(('planned_execution.py', 'task_collection_evidence.py')):
            require(result['execution_source_sha256_start'].get(source) == expected_sha,
                    'Round 1 native recorder/controller source changed: ' + source)
    recomputed_samples=0
    for name in ('actions','physics'):
        with np.load(rp.parent/('task_predicate_'+name+'.npz'),allow_pickle=False) as predicate_data:
            indexes = np.arange(n) if name == 'physics' else after-steps[0]
            for key in predicate_data.files:
                values = predicate_data[key]
                require(values.shape[0] == len(indexes), 'Predicate sample count differs: ' + key)
                if values.dtype.kind in 'fiub':
                    require(np.isfinite(values).all(), 'Nonfinite predicate samples: ' + key)
            require(np.array_equal(predicate_data['phase'], physics_phase[indexes])
                    and np.array_equal(predicate_data['action_index'], physics_action_index[indexes]),
                    'Predicate phase/action differs from native physics')
            payload_index = metadata['object_names'].index(predicate_metadata['pick_up_object'])
            measured = native_object_states[indexes, payload_index]
            for key, native in (('object_pose', measured[:, :7]), ('object_linear_velocity', measured[:, 7:10]),
                                ('destination_contact_force', native_destination_force[indexes])):
                require(predicate_data[key].shape == native.shape
                        and np.allclose(predicate_data[key], native, atol=1e-6, rtol=0),
                        'Predicate field differs from native physics: ' + name + '/' + key)
            if predicate_metadata['destination_location'] in metadata['object_names']:
                destination_index = metadata['object_names'].index(predicate_metadata['destination_location'])
                require(np.allclose(predicate_data['destination_pose'], native_object_states[indexes, destination_index, :7],
                                    atol=1e-6, rtol=0), 'Destination pose differs from native physics')
            box = predicate_metadata['actual_spawned_local_bounds'][predicate_metadata['pick_up_object']]
            center = (np.asarray(box['min']) + np.asarray(box['max'])) / 2
            quaternion = measured[:, 3:7]
            t = 2*np.cross(quaternion[:, :3], center)
            centroid = measured[:, :3] + center + quaternion[:, 3:4]*t + np.cross(quaternion[:, :3], t)
            require(np.allclose(centroid, predicate_data['object_centroid_world'], atol=1e-6, rtol=0),
                    'Predicate centroid differs from native pose and spawned bounds')
            components=predicate_components(predicate_data,predicate_metadata)
            on=components['within_destination'] & components['upward_support'] & components['slow']
            require(np.array_equal(on,predicate_data['original_object_on_destination']),'Recomputed native destination predicate differs: '+name)
            recomputed_samples+=len(on)
            above = predicate_data['has_original_rest_pose'] & (predicate_data['object_pose'][:, 2]
                    > predicate_data['original_rest_position'][:, 2]+.01)
            require(np.array_equal(above, predicate_data['original_above_rest']), 'Original strict lift predicate differs')
            if name=='actions':
                final_components={k:bool(v[-1]) for k,v in components.items()}
                audit_terminal_order(predicate_data, predicate_metadata)
    if result.get('sequence_complete'):
        require(result['task_success'] is (observed and final),'Complete task outcome does not equal original ever-success AND final placement')
    for key in ('rendering_enabled','camera_entities_enabled','video_expected'):
        require(result[key] is False,'Rendering/video was enabled: '+key)
    require(row.get('video_path') is None and result.get('video_path') is None and result.get('video_sha256') is None,'Unexpected video')
    for name in ('render_state_start','render_state_end'):
        state=result[name]
        require(state['passed'] is True and not any(state[k] for k in ('is_rendering','render_enabled','has_gui','rtx_sensors','offscreen',
            'visualizer_types','visualizer_count','camera_sensors','render_generation')),'Nonzero rendering/camera state: '+name)
    require(result['render_generation_delta']==0,'Render generation advanced')
    require(not any(p.suffix.lower() in ('.mp4','.png','.jpg','.jpeg') for p in rp.parent.rglob('*')),'Unexpected rendered frame/video')
    for source,expected_sha in result['execution_source_sha256_end'].items():bound(source,expected_sha)
    bound(row['execution_result_path'],row['execution_result_sha256'])
    return dict(scene_id=row['scene_id'],task_id=row['task_id'],status='passed',execution_status=result['execution_status'],
        executed=True,attempts=1,physical_attempt_number=result['physical_attempt_number'],task_success=result['task_success'],
        runtime_revision_id=result['runtime_revision_id'],physics_samples=n,actions=len(actions),
        recorded_initial_state_independently_checked=True,initial_q_error_rad=initial_q_error,
        adjusted_commands=len(source_actions),full_expected_actions=len(expected_actions),
        original_task_terminal_observed=observed,final_original_destination_predicate=final,
        final_native_predicate_components=final_components,native_predicate_samples_recomputed=recomputed_samples,
        rendered_frames=0,no_render_gate_passed=True,source_geometry_unchanged=True,controller_unchanged=True,
        result=file_binding(rp),planning_status=row['planning_status'],candidate_planning_result=row['candidate_planning_result'],
        previous_retry_result=row['previous_retry_result'],execution_overrides=row['execution_overrides'],
        nominal_gripper_joint_rad=plan['nominal_gripper_joint_rad'],
        planning_gripper_joint_rad=plan['planning_gripper_joint_rad'],
        runtime_substitution_flags_verified=True,
        physics=ref(qp),actions_evidence=ref(actions_path),predicates_evidence=ref(predicate_metadata_path))


def audit_terminal_order(data, metadata):
    """Check the recorded original settle/lift/placement sequence, not a new task."""
    resting = np.flatnonzero(data['has_original_rest_pose'])
    if len(resting):
        first = int(resting[0])
        require(data['has_original_rest_pose'][first:].all(), 'Original rest state was cleared')
        require(np.allclose(data['original_rest_position'][first:], data['original_rest_position'][first],
                            atol=1e-7, rtol=0), 'Original rest reference moved')
    success = np.flatnonzero(data['terminal_success'])
    if not len(success):
        return
    rise = np.flatnonzero(data['original_above_rest'])
    require(len(resting) and len(rise) and resting[0] <= rise[0] <= success[0],
            'Native success lacks original settle then lift ordering')
    count = metadata['thresholds']['placement_consecutive_steps']
    require(type(count) is int and count > 0, 'Invalid original placement duration')
    end = int(success[0])
    require(end-count+1 >= rise[0] and data['original_object_on_destination'][end-count+1:end+1].all(),
            'First native success lacks original consecutive destination samples')


def audit_planning_failure(row, recipe, demos, context):
    require(row['execution_status'] == 'planning_failed' and row['planning_status'] == 'failed',
            'Planning-failure status mismatch')
    require(type(row['attempts']) is int and row['attempts'] == 0 and row['executed'] is False
            and row['task_success'] is False, 'Failed planning consumed a physical attempt')
    require(not row.get('execution_result_path') and not row.get('execution_result_sha256')
            and row.get('video_path') is None and row.get('video_sha256') is None,
            'Planning failure has unexpected physical result/video')
    require(not (demos/'executions'/row['scene_id']).exists(), 'Planning failure has an unexpected physical episode directory')
    audit_history(row, recipe, context)
    candidate = audit_candidate(row, recipe)
    return dict(scene_id=row['scene_id'], task_id=row['task_id'], status='passed',
        planning_status='failed', execution_status='planning_failed', executed=False, attempts=0,
        task_success=False, physical_attempt_number=None,
        reserved_next_physical_attempt_number=row['physical_attempt_number'],
        candidate_planning_result=row['candidate_planning_result'], previous_retry_result=row['previous_retry_result'],
        failure_reason=candidate['failure_reason'], physics_samples=0, native_predicate_samples_recomputed=0,
        no_render_gate_passed=True, no_physical_episode_created=True,
        scope='A single failed planning candidate; no physical runtime, action, or render was created.')


def audit_manifest(manifest, plan):
    require(manifest['kind'] == 'physical_failure_retry' and plan['kind'] == 'physical_failure_retry_plan', 'Wrong retry kind')
    require(manifest['schema_version'] == plan['schema_version'] == 2
            and manifest['retry_round'] == plan['retry_round'] == 2, 'Wrong physical retry round/schema')
    for key, value in dict(expected_scene_count=48, candidate_count_per_scene=1, physical_retries_per_scene=1,
                           layout_changed=False, success_predicate_changed=False).items():
        require(plan.get(key) == value, 'Frozen authorization differs: ' + key)
    original_document = bound_json(plan['source_first_pass_manifest'])
    retake_document = bound_json(plan['source_camera_retake_manifest'])
    previous_document = bound_json(plan['source_previous_retry_manifest'])
    original = rows_by_id(original_document)
    retakes = rows_by_id(retake_document)
    previous = rows_by_id(previous_document)
    original_failed = {sid for sid, row in original.items() if row['execution_status'] == 'failed'}
    failed = {sid for sid, row in previous.items() if row['execution_status'] == 'failed'}
    require(len(original) == 154 and len(original_failed) == 84, 'Unexpected original 154-scene history')
    require(previous_document['stage'] == 'complete' and previous_document['schema_version'] == 1
            and previous_document['kind'] == 'physical_failure_retry' and len(previous) == 84
            and set(previous) == original_failed
            and Counter(row['execution_status'] for row in previous.values()) == {'success': 36, 'failed': 48},
            'Expected complete round 1 with 36 successes and 48 failures')
    bound(previous_document['retry_plan_path'], previous_document['retry_plan_sha256'])
    require(retake_document['stage'] == 'complete', 'Camera retake history is not complete')
    rows, recipes = rows_by_id(manifest), rows_by_id(plan)
    require(len(rows) == len(recipes) == manifest['total'] == 48 and set(rows) == set(recipes) == failed,
            'Round 2 whitelist differs from exactly the 48 round 1 failures')
    for key in ('source_first_pass_manifest', 'source_camera_retake_manifest', 'source_previous_retry_manifest'):
        require(manifest[key] == plan[key], 'Manifest historical source binding differs: ' + key)
    for key in ('source_first_pass_manifest', 'source_camera_retake_manifest'):
        require(previous_document[key] == plan[key], 'Previous round historical source differs')
    require(manifest['rendering_enabled'] is False and manifest['video_expected'] is False
            and manifest['one_additional_attempt_per_scene'] is True, 'Manifest runtime scope differs')
    counts = Counter(row['execution_status'] for row in rows.values())
    require(set(counts) <= TERMINAL | {'pending', 'running'}, 'Unknown execution state')
    require(all(manifest['counts'].get(key) == counts[key]
                for key in ('pending', 'running', 'success', 'failed', 'error', 'planning_failed')), 'Manifest count mismatch')
    for row in rows.values():
        require(row['retry_plan_path'] == manifest['retry_plan_path']
                and row['retry_plan_sha256'] == manifest['retry_plan_sha256'], 'Row retry-plan binding differs')
        if row['execution_status'] in ('pending', 'planning_failed'):
            require(row['attempts'] == 0 and row['executed'] is False, 'Unexecuted candidate consumed a trial')
        else:
            require(row['attempts'] == 1, 'Attempted candidate must consume exactly one trial')
        for key, value in recipes[row['scene_id']].items():
            require(row.get(key) == value, 'Manifest differs from frozen plan: ' + key)
    revisions = manifest['runtime_revisions']
    require(len(revisions) == len({revision['id'] for revision in revisions}) > 0, 'Missing/duplicate runtime revision')
    for revision in revisions:
        settings = revision['settings']
        for key, value in dict(device='cpu', headless=True, enable_cameras=False, visualizers=[],
                               rendering_enabled=False, physics_dt=1/120, control_dt=1/30).items():
            require(settings.get(key) == value, 'Revision runtime scope differs: ' + key)
        require(settings['retry_plan_path'] == manifest['retry_plan_path']
                and settings['retry_plan_sha256'] == manifest['retry_plan_sha256'], 'Revision retry-plan binding differs')
        for key, value in plan['execution_settings'].items():
            require(settings[key] == value, 'Revision changes frozen base execution settings: ' + key)
    return dict(original=original, retakes=retakes, previous=previous,
                previous_manifest=previous_document, recipes=recipes, counts=counts)


def main(argv=None):
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demos', default=DEFAULT_DEMOS)
    parser.add_argument('--output', default=DEFAULT_OUTPUT)
    parser.add_argument('--require-complete', action='store_true')
    parser.add_argument('--scene', default='')
    args = parser.parse_args(argv)
    demos = local(args.demos)
    manifest_path = demos/'manifest.json'
    raw = manifest_path.read_bytes()
    manifest_sha = hashlib.sha256(raw).hexdigest()
    manifest = json.loads(raw)
    errors, checked = [], []
    plan = None
    context = None
    counts = Counter(row.get('execution_status') for row in manifest.get('entries', []))
    try:
        plan = read(bound(manifest['retry_plan_path'], manifest['retry_plan_sha256']))
        context = audit_manifest(manifest, plan)
        if args.require_complete:
            require(manifest['stage'] == 'complete' and sum(counts[key] for key in TERMINAL) == 48,
                    'All 48 candidates are not terminal')
        if args.scene:
            require(args.scene in context['recipes'], 'Unknown audit scene')
    except Exception as error:
        errors.append(dict(scope='manifest', error=f'{type(error).__name__}: {error}'))
    revisions = {revision['id']: revision for revision in manifest.get('runtime_revisions', [])}
    if context is not None:
        for row in manifest['entries']:
            if row['execution_status'] not in TERMINAL or (args.scene and row['scene_id'] != args.scene):
                continue
            try:
                recipe = context['recipes'][row['scene_id']]
                if row['execution_status'] == 'planning_failed':
                    checked.append(audit_planning_failure(row, recipe, demos, context))
                else:
                    checked.append(audit_entry(row, recipe, revisions, demos, plan, context))
            except Exception as error:
                errors.append(dict(scene_id=row['scene_id'], error=f'{type(error).__name__}: {error}'))
    if args.scene and not checked:
        errors.append(dict(scope='selection', error='Requested scene is not a fully audited terminal outcome'))
    changed = digest(manifest_path) != manifest_sha
    if changed:
        errors.append(dict(scope='manifest', error='Manifest changed during audit; this snapshot cannot certify the new manifest'))
    require_complete_coverage = args.require_complete and not args.scene
    if require_complete_coverage and len(checked) != 48:
        errors.append(dict(scope='coverage', error='Complete audit must cover all 48 candidates, including planning failures'))
    report = dict(schema_version=2, kind='physical_retry_round2_independent_audit',
        created_at=datetime.now(timezone.utc).isoformat(), elapsed_seconds=time.monotonic()-started, passed=not errors,
        scope=('Independent CPU read-only audit: exact 48 prior-failure cohort; full physical history; one new candidate and at most one native trial; '
               'actual new-grip/hold commands; continuous 120Hz state and original predicates; frozen geometry/controller; zero render/camera runtime; source SHA. '
               'No new physics/GPU, collision validation, or safety acceptance.'),
        manifest_snapshot={'path': str(manifest_path.relative_to(ROOT)), 'sha256': manifest_sha},
        manifest_snapshot_sha256=manifest_sha, manifest_changed_during_audit=changed,
        require_complete=args.require_complete, selected_scene=args.scene or None, manifest_snapshot_counts=dict(counts),
        audited_scenes=len(checked), physical_executions=sum(entry['executed'] for entry in checked),
        planning_failures=sum(entry['execution_status'] == 'planning_failed' for entry in checked),
        physical_successes=sum(entry['task_success'] for entry in checked),
        native_physics_samples=sum(entry['physics_samples'] for entry in checked),
        native_predicate_samples_recomputed=sum(entry['native_predicate_samples_recomputed'] for entry in checked),
        all48_audited=len(checked) == 48 and not errors, errors=errors, entries=checked,
        auditor_source=file_binding(Path(__file__)))
    if plan is not None:
        report.update(original_first_pass_manifest=plan['source_first_pass_manifest'],
            original_camera_retake_manifest=plan['source_camera_retake_manifest'],
            source_previous_retry_manifest=plan['source_previous_retry_manifest'])
    output = local(args.output)
    require(not output.is_relative_to(demos), 'Audit output must not modify physical execution evidence')
    require(output.suffix == '.json' and output not in {Path(key[0]) for key in HASHES},
            'Audit output must be a separate JSON, never a bound input or source file')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix+'.tmp')
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(output)
    print(json.dumps({key: value for key, value in report.items() if key != 'entries'}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
