"""One additional physical test per explicitly whitelisted prior failure, without rendering.

Frozen scenes and historical evidence are never changed. Only the supplied,
source-bound adjusted command trajectory differs. No planner or camera is used.
"""
from pathlib import Path
import copy
import json
import math
import time
import traceback
import numpy as np
from . import planned_execution as base
from .planned_execution import (ROOT, PHASES, local, sha, stamp, write_json,
    validate_planned_arrays, reset_state_gate, enforce_physics_evidence_gate,
    configure_physics_threads, build_environment, PhysicalRecorder)

SETTING_KEYS = ('device', 'seed', 'torch_threads', 'physics_threads', 'episode_seconds',
    'max_actions', 'initial_settle_seconds', 'close_seconds', 'release_seconds', 'final_settle_seconds')
SOURCES = tuple(dict.fromkeys(base.RUNTIME_SOURCES + (
    'scripts/extensions/replicator/execute_failure_retry.py',
    'kitchen_safety/extensions/replicator/retry_physical_execution.py',
    'kitchen_safety/extensions/replicator/physical_retry_plan.py',
    'kitchen_safety/extensions/replicator/physical_retry_round3.py',
    'third_party/IsaacLab-Arena/isaaclab_arena/utils/isaaclab_utils/simulation_app.py',
    'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/app/app_launcher.py',
    'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env.py',
    'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/sim/simulation_context.py',
)))
SOURCE_FIELDS = ('scene_id','task_id','variant_id','result_path','result_sha256','config_path',
    'config_sha256','source_scene_path','source_scene_sha256')
PREDICATE_SOURCES = tuple(p for p in base.RUNTIME_SOURCES if
    '/tasks/' in p or p.endswith(('planned_execution.py', 'task_collection_evidence.py')))


def round_two(plan):
    version = plan.get('schema_version', 1)
    if version not in (1, 2): raise ValueError('Unsupported retry schema')
    if version == 2 and plan.get('retry_round') not in (2, 3):
        raise ValueError('Unsupported source-bound grasp retry round')
    return version == 2


def entry_settings(entry, args):
    """Only bounded close/release durations may differ for a frozen candidate."""
    settings = entry.get('execution_overrides', {})
    if not isinstance(settings, dict) or set(settings) - {'close_seconds', 'release_seconds'}:
        raise ValueError('Only close/release hold overrides are permitted')
    if settings and entry.get('retry_schema_version', 1) != 2:
        raise ValueError('Round 1 hold settings remain frozen')
    result = copy.copy(args)
    for key, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or not .1 <= value <= 10.:
            raise ValueError('Close/release override must be finite and within 0.1..10 seconds')
        setattr(result, key, value)
    return result


def planning_result_bindings(entry):
    original=dict(path=entry['result_path'],sha256=entry['result_sha256'])
    candidate=entry['candidate_planning_result'] if entry.get('retry_schema_version',1)==2 else original
    result=dict(planning_result_path=candidate['path'],planning_result_sha256=candidate['sha256'])
    if entry.get('retry_schema_version',1)==2:result['original_planning_result']=original
    return result


def runtime_hashes():
    return {p: sha(ROOT/p) for p in SOURCES}


def runtime_settings(args):
    return {**{k: getattr(args, k) for k in SETTING_KEYS}, 'headless': True,
        'enable_cameras': False, 'visualizers': [], 'rendering_enabled': False,
        'physics_dt': 1/120, 'control_dt': 1/30, 'retry_plan_path': str(local(args.plan).relative_to(ROOT)),
        'retry_plan_sha256': sha(local(args.plan))}


def bound_json(binding):
    path = local(binding['path'])
    if sha(path) != binding['sha256']: raise ValueError('Bound source changed: '+str(path))
    return json.loads(path.read_text())


def rows_by_id(document):
    rows = document['entries']
    if isinstance(rows, dict): rows = list(rows.values())
    if len({r['scene_id'] for r in rows}) != len(rows): raise ValueError('Duplicate scene IDs')
    return {r['scene_id']: r for r in rows}


def prior_binding(row):
    return dict(result_path=row['execution_result_path'], result_sha256=row['execution_result_sha256'],
        execution_status=row['execution_status'], task_success=row['task_success'], executed=row['executed'],
        physical_attempt_number=row.get('physical_attempt_number', 1),
        video_sha256=row.get('video_sha256'))


def validate_prior(row, source, *, number):
    result_path = local(row['execution_result_path'])
    if sha(result_path) != row['execution_result_sha256']: raise ValueError('Prior result changed')
    result = json.loads(result_path.read_text())
    if (result.get('kind') != 'physical_execution' or result.get('status') != 'complete'
        or result.get('executed') is not True or result.get('execution_status') != 'failed'
        or result.get('task_success') is not False or result.get('physical_attempt_number',1) != number):
        raise ValueError('Retry lineage must be a completed real failed physical trial')
    for key in ('scene_id','task_id','variant_id','config_path','config_sha256','trajectory_path','trajectory_sha256'):
        if row[key] != source[key] or result[key] != row[key]:
            raise ValueError('Prior source identity differs: '+key)
    if result['planning_result_path'] != source['result_path'] or result['planning_result_sha256'] != source['result_sha256']:
        raise ValueError('Prior planning provenance differs')
    for key in ('executed','task_success','execution_status','video_sha256'):
        if result.get(key) != row.get(key): raise ValueError('Prior outcome differs: '+key)
    return prior_binding(row)


def validate_previous_retry(row, original, history, manifest):
    """Bind the completed round-1 failure without rereading its large recordings."""
    result = bound_json(dict(path=row['execution_result_path'], sha256=row['execution_result_sha256']))
    number = len(history) + 1
    if (result.get('kind') != 'physical_failure_retry_execution' or result.get('status') != 'complete'
            or result.get('execution_status') != 'failed' or result.get('task_success') is not False
            or result.get('executed') is not True or result.get('attempts') != 1
            or result.get('physical_attempt_number') != number):
        raise ValueError('Previous retry must be one completed real failed physical trial')
    for key in SOURCE_FIELDS:
        if row[key] != original[key]: raise ValueError('Previous retry original source differs: '+key)
    for key in ('scene_id','task_id','variant_id','config_path','config_sha256','trajectory_path','trajectory_sha256',
                'source_trajectory_path','source_trajectory_sha256','physical_attempt_number',
                'executed','task_success','execution_status','attempts','prior_physical_attempts'):
        if result.get(key) != row.get(key): raise ValueError('Previous retry result differs: '+key)
    planning = planning_result_bindings(row)
    if (result['planning_result_path'] != planning['planning_result_path']
            or result['planning_result_sha256'] != planning['planning_result_sha256']
            or result['prior_physical_attempts'] != history
            or result['source_trajectory_path'] != original['trajectory_path']
            or result['source_trajectory_sha256'] != original['trajectory_sha256']):
        raise ValueError('Previous retry lineage differs')
    for key in ('retry_plan_path', 'retry_plan_sha256'):
        if result[key] != manifest[key] or row[key] != manifest[key]:
            raise ValueError('Previous retry plan binding differs')
    if sha(local(row['trajectory_path'])) != row['trajectory_sha256']:
        raise ValueError('Previous retry trajectory changed')
    if (result.get('source_inputs_unchanged') is not True or result.get('execution_sources_unchanged') is not True
            or result.get('rendering_enabled') is not False or result.get('render_generation_delta') != 0
            or result.get('render_state_start', {}).get('passed') is not True
            or result.get('render_state_end', {}).get('passed') is not True
            or any(result.get(k) is not False for k in
                ('runtime_state_setters_used','physical_weld_used','payload_attachment_used'))):
        raise ValueError('Previous retry provenance/no-render gate differs')
    for path in PREDICATE_SOURCES:
        if result['execution_source_sha256_start'].get(path) != sha(ROOT/path):
            raise ValueError('Original physical success predicate source changed: '+path)
    return prior_binding(row)


def load_grasp_candidate(entry, original_plan, layout):
    """Read a new cuRobo candidate and verify the unchanged retiming policy."""
    from .physical_retry_plan import validate_candidate, validate_source
    candidate = bound_json(entry['candidate_planning_result'])
    for key in ('scene_id','task_id','variant_id','config_path','config_sha256'):
        if candidate.get(key) != entry[key]: raise ValueError('Candidate planning identity differs: '+key)
    if (candidate.get('planning_only') is not True or candidate.get('executed') is not False
            or candidate.get('attempts') != 1 or candidate.get('per_stage_max_attempts') != 1
            or candidate.get('scene_settings_changed_once', False) is not False):
        raise ValueError('Candidate must be one planning-only attempt with unchanged layout')
    status = entry.get('planning_status')
    if status == 'failed':
        if (candidate.get('status') != 'failed' or candidate.get('trajectory_complete') is not False
                or not entry.get('failure_reason') or entry.get('trajectory_path') is not None
                or entry.get('trajectory_sha256') is not None):
            raise ValueError('Failed planning may not supply an executable trajectory')
        return candidate, layout, None
    if status != 'generated' or candidate.get('status') != 'generated' or candidate.get('trajectory_complete') is not True:
        raise ValueError('New candidate must be generated or explicitly planning_failed')
    if (np.asarray(candidate.get('start_q')).shape != (7,)
            or not np.allclose(candidate['start_q'], original_plan['start_q'], atol=1e-5, rtol=0)):
        raise ValueError('New candidate changed the frozen robot start')
    old_assets = original_plan['composition']['source_paths_sha256']
    new_assets = candidate['composition']['source_paths_sha256']
    if old_assets != new_assets: raise ValueError('New candidate changed source asset bindings')
    for path, digest in new_assets.items():
        if sha(local(path)) != digest: raise ValueError('Candidate source asset changed: '+path)
    binding = candidate['artifacts']['trajectory']
    raw_path, path = local(binding['path']), local(entry['trajectory_path'])
    if len({raw_path, path, local(entry['source_trajectory_path'])}) != 3:
        raise ValueError('Raw/retimed/original trajectories must have distinct paths')
    if sha(raw_path) != binding['sha256'] or sha(path) != entry['trajectory_sha256']:
        raise ValueError('Candidate raw/retimed trajectory changed')
    with np.load(raw_path, allow_pickle=False) as archive: raw = dict(archive)
    with np.load(path, allow_pickle=False) as archive: data = dict(archive)
    validate_planned_arrays(raw); validate_planned_arrays(data); validate_source(raw)
    if not np.allclose(raw['q'][0], original_plan['start_q'], atol=1e-5, rtol=0):
        raise ValueError('New trajectory changed frozen start q')
    if np.any(raw['gripper'] < 0) or np.any(raw['gripper'] > .8):
        raise ValueError('Candidate gripper command outside 0..0.8 rad')
    grip = np.unique(raw['gripper'][np.isin(raw['phase'], ('lift','transfer','lower'))])
    if (len(grip) != 1 or not np.isclose(grip[0], candidate['nominal_gripper_joint_rad'], atol=1e-8, rtol=0)):
        raise ValueError('Candidate nominal gripper differs from raw trajectory')
    validate_candidate(raw, data)
    if not np.allclose(data['tcp_world'][data['source_vertex_output_index']], raw['tcp_world'], atol=2e-6, rtol=0):
        raise ValueError('Retimed candidate TCP vertices differ')
    return candidate, layout, data


def load_retry_inputs(entry):
    original = dict(entry, trajectory_path=entry['source_trajectory_path'],
                    trajectory_sha256=entry['source_trajectory_sha256'])
    plan, layout, source_data = base.load_inputs(original)
    adjustment = entry['adjustment']
    if not adjustment.get('kind') or not adjustment.get('description'): raise ValueError('Missing adjustment explanation')
    bound_json(dict(path=adjustment['evidence_path'], sha256=adjustment['evidence_sha256']))
    if entry.get('retry_schema_version', 1) == 2:
        if adjustment.get('candidate_count') != 1: raise ValueError('Exactly one new candidate is authorized')
        return load_grasp_candidate(entry, plan, layout)
    path = local(entry['trajectory_path'])
    if sha(path) != entry['trajectory_sha256']: raise ValueError('Adjusted trajectory changed')
    data = dict(np.load(path, allow_pickle=False)); validate_planned_arrays(data)
    if path == local(entry['source_trajectory_path']): raise ValueError('Adjusted output may not overwrite original trajectory')
    # This round is timing-only: six original phase endpoints and nominal grip
    # are immutable. Full path/velocity evidence is bound in the plan adjustment.
    for phase in PHASES:
        old = source_data['phase'] == phase; new = data['phase'] == phase
        if not np.array_equal(data['q'][new][[0,-1]], source_data['q'][old][[0,-1]]):
            raise ValueError('Timing-only adjustment changed phase endpoints: '+phase)
        if not np.array_equal(np.unique(data['gripper'][new]), np.unique(source_data['gripper'][old])):
            raise ValueError('Timing-only adjustment changed nominal gripper: '+phase)
    return plan, layout, data


def validate_plan(path):
    path = local(path); plan = json.loads(path.read_text())
    if plan.get('kind') != 'physical_failure_retry_plan': raise ValueError('Explicit failure retry plan required')
    if plan.get('retry_round') == 3:
        from .physical_retry_round3 import validate_round3_plan
        return validate_round3_plan(path, plan)
    second = round_two(plan)
    first = rows_by_id(bound_json(plan['source_first_pass_manifest']))
    retakes = rows_by_id(bound_json(plan['source_camera_retake_manifest']))
    selected = rows_by_id(plan)
    failures = {sid for sid, row in first.items() if row['execution_status']=='failed'}
    if len(first) != 154 or len(failures) != 84:
        raise ValueError('Whitelist must equal all 84 frozen first-pass failures')
    previous = None; previous_rows = {}
    if second:
        for key, expected in dict(expected_scene_count=48,candidate_count_per_scene=1,physical_retries_per_scene=1,
                                  layout_changed=False,success_predicate_changed=False).items():
            if plan.get(key) != expected: raise ValueError('Round 2 authorization differs: '+key)
        previous = bound_json(plan['source_previous_retry_manifest'])
        previous_rows = rows_by_id(previous)
        if (previous.get('kind') != 'physical_failure_retry' or previous.get('schema_version') != 1
                or previous.get('stage') != 'complete' or set(previous_rows) != failures
                or sum(r['execution_status']=='success' for r in previous_rows.values()) != 36
                or sum(r['execution_status']=='failed' for r in previous_rows.values()) != 48):
            raise ValueError('Expected completed round 1: 36 successes and 48 failures')
        for key in ('source_first_pass_manifest','source_camera_retake_manifest'):
            if previous[key] != plan[key]: raise ValueError('Previous round historical source differs')
        bound_json(dict(path=previous['retry_plan_path'],sha256=previous['retry_plan_sha256']))
        failures = {sid for sid, row in previous_rows.items() if row['execution_status']=='failed'}
    if set(selected) != failures:
        raise ValueError('Whitelist must equal all '+('48 previous-round' if second else '84 frozen first-pass')+' failures')
    output = []
    for sid, row in selected.items():
        original = first[sid]
        for key in SOURCE_FIELDS:
            if row[key] != original[key]: raise ValueError('Frozen scene/planning source changed: '+sid+' '+key)
        for key in ('path','sha256'):
            if row['source_trajectory_'+key] != original['trajectory_'+key]:
                raise ValueError('Original trajectory binding differs')
        history = [validate_prior(original, original, number=1)]
        if sid in retakes:
            history.append(validate_prior(retakes[sid], original, number=2))
            if retakes[sid]['first_pass']['result_sha256'] != original['execution_result_sha256']:
                raise ValueError('Camera history does not bind first physical attempt')
        if second:
            prior = previous_rows[sid]
            expected = dict(path=prior['execution_result_path'],sha256=prior['execution_result_sha256'])
            if row.get('previous_retry_result') != expected: raise ValueError('Previous retry result binding differs')
            history.append(validate_previous_retry(prior, original, history, previous))
        if row['physical_attempt_number'] != len(history)+1: raise ValueError('Physical attempt number does not include camera history')
        entry = copy.deepcopy(row)
        if second: entry['retry_schema_version'] = 2
        entry.update(retry_plan_path=str(path.relative_to(ROOT)), retry_plan_sha256=sha(path),
            prior_physical_attempts=history, execution_status='pending', attempts=0, executed=False,
            task_success=False, video_path=None, video_sha256=None, planning_status='generated')
        if second:
            entry['planning_status'] = row.get('planning_status')
            if entry['planning_status'] == 'failed': entry['execution_status'] = 'planning_failed'
        load_retry_inputs(entry)
        # Validate overrides before initialization; durations are applied per row.
        entry_settings(entry, type('Settings', (), {})())
        output.append(entry)
    if not second and sum(e['physical_attempt_number']==3 for e in output) != 4:
        raise ValueError('Expected exactly four failed U-banana scenes with two prior physical episodes')
    return plan, output


def validate_entry_lineage(entry, args):
    if local(entry['retry_plan_path']) != local(args.plan) or sha(local(args.plan)) != entry['retry_plan_sha256']:
        raise ValueError('Retry plan changed')
    plan = json.loads(local(args.plan).read_text()); row = rows_by_id(plan)[entry['scene_id']]
    bound_json(plan['source_first_pass_manifest']); bound_json(plan['source_camera_retake_manifest'])
    if round_two(plan):
        previous = bound_json(plan['source_previous_retry_manifest'])
        bound_json(dict(path=previous['retry_plan_path'],sha256=previous['retry_plan_sha256']))
        if entry.get('retry_schema_version') != 2: raise ValueError('Retry schema changed')
    for key, value in row.items():
        if entry.get(key) != value: raise ValueError('Retry entry differs from frozen plan: '+key)
    for binding in entry['prior_physical_attempts']:
        if sha(local(binding['result_path'])) != binding['result_sha256']: raise ValueError('Prior physical result changed')
    load_retry_inputs(entry)


def configure_no_render(cfg):
    cfg.observations.camera_obs = None
    cfg.sim.visualizer_cfgs = []; cfg.sim.default_visualizer_cfg = None
    cfg.rerender_on_reset = False; cfg.num_rerenders_on_reset = 0; cfg.wait_for_textures = False
    # Reject a camera in the composed config instead of silently retaining it.
    for name, value in vars(cfg.scene).items():
        if value is not None and ('camera' in type(value).__name__.lower() or 'camera' in name.lower()):
            raise ValueError('Camera entity forbidden in no-render execution: '+name)


def render_state(env):
    sim = env.sim
    state = dict(is_rendering=bool(sim.is_rendering), render_enabled=bool(env.render_enabled),
        has_gui=bool(sim.has_gui), rtx_sensors=bool(sim.get_setting('/isaaclab/render/rtx_sensors')),
        offscreen=bool(sim.get_setting('/isaaclab/render/offscreen')),
        visualizer_types=list(sim.resolve_visualizer_types()), visualizer_count=len(sim.visualizers),
        camera_sensors=[name for name, value in env.scene.sensors.items() if 'camera' in type(value).__name__.lower()],
        render_generation=int(sim.render_generation))
    state['passed'] = not any((state['is_rendering'],state['render_enabled'],state['has_gui'],
        state['rtx_sensors'],state['offscreen'],state['visualizer_types'],state['visualizer_count'],state['camera_sensors'],
        state['render_generation']))
    return state


def validate_budget(data, args):
    holds = sum(math.ceil(getattr(args,k)*30) for k in
        ('initial_settle_seconds','close_seconds','release_seconds','final_settle_seconds'))
    count = len(data['q'])+holds
    if count > args.max_actions or count/30 >= args.episode_seconds:
        raise ValueError(f'Adjusted command count {count} exceeds action/time budget')
    return count

def execute_one(entry, output_root, args):
    """Exactly one physical episode; initialization/runtime failures are retained."""
    if entry.get('planning_status') != 'generated': raise ValueError('Cannot execute a failed planning candidate')
    args = entry_settings(entry, args)
    import torch
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from .startup_recorder import array
    from .task_collection_evidence import TaskPredicateRecorder
    validate_entry_lineage(entry, args)
    out = Path(output_root)/'executions'/entry['scene_id']; out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    result = dict(kind='physical_failure_retry_execution', scene_id=entry['scene_id'], task_id=entry['task_id'], variant_id=entry['variant_id'],
        config_path=entry['config_path'], config_sha256=entry['config_sha256'], planning_result_path=entry['result_path'],
        planning_result_sha256=entry['result_sha256'], trajectory_path=entry['trajectory_path'], trajectory_sha256=entry['trajectory_sha256'],
        planning_status='generated', execution_status='error', task_success=False, executed=False, attempts=1,
        physical_attempt_number=entry.get('physical_attempt_number', 1),
        status='initializing', started_at=stamp(), physical_execution=True, runtime_state_setters_used=False,
        physical_weld_used=False, payload_attachment_used=False, planning_rerun=False, accepted=False,
        safety_certified=False, long_term_stability_verified=False, video_path=None, video_sha256=None,
        runtime_revision_id=entry.get('runtime_revision_id', 0), runtime_settings=runtime_settings(args),
        execution_source_sha256_start=runtime_hashes(),
        scope='One native PhysX execution of frozen planned joint targets; physical task success measured separately, no safety acceptance.')
    result.update(planning_result_bindings(entry))
    for key in ('source_trajectory_path','source_trajectory_sha256','adjustment','prior_physical_attempts',
                'retry_plan_path','retry_plan_sha256'):
        result[key] = copy.deepcopy(entry[key])
    for key in ('retry_schema_version','candidate_planning_result','previous_retry_result','execution_overrides'):
        if key in entry: result[key] = copy.deepcopy(entry[key])
    retry_plan = json.loads(local(args.plan).read_text())
    for key in ('source_first_pass_manifest','source_camera_retake_manifest'):
        result[key] = copy.deepcopy(retry_plan[key])
    if round_two(retry_plan): result['source_previous_retry_manifest'] = copy.deepcopy(retry_plan['source_previous_retry_manifest'])
    result.update(rendering_enabled=False, camera_entities_enabled=False, video_expected=False,
        scope=('One real physical test of one new source-bound grasp/trajectory candidate; frozen layout and original success predicate, '
               'zero render/camera/video, no runtime planning or object state writes; prior attempts retained.' if round_two(retry_plan) else
               'One additional real physical test of an adjusted timing-only trajectory. No render/camera/video, no planning or scene change; previous attempts retained.'))
    write_json(out/'result.json', result)
    env = recorder = predicates = None
    original_reset = None
    actions, terminal_events = [], []
    physical_arrays = None
    try:
        plan, layout, data = load_retry_inputs(entry)
        result['expected_action_count'] = validate_budget(data, args)
        torch.set_num_threads(args.torch_threads)
        result['runtime_threads'] = configure_physics_threads(args.physics_threads)
        result['runtime_threads'].update(requested_torch_threads=args.torch_threads,
            live_torch_threads=torch.get_num_threads(), requested_physics_device=args.device)
        write_json(out/'result.json', result)
        definition = build_environment(plan, layout, output_root/'physical_overlays', args.episode_seconds)
        result['background_physics'] = definition.background_physics_provenance
        background_overlay_path = definition.scene.assets['kitchen'].usd_path
        result['physical_overlay_binding'] = dict(path=str(background_overlay_path),
            sha256_start=sha(background_overlay_path))
        result['initialization_scope'] = 'One native environment reset to frozen start_q and layout; no later state resets/writes'
        builder = ArenaEnvBuilder(definition, ArenaEnvBuilderCfg(num_envs=1, seed=args.seed, device=args.device,
            presets='physx', solve_relations=False, resolve_on_reset=False))
        cfg, kwargs = builder.compose_manager_cfg()
        configure_no_render(cfg)
        cfg.sim.dt = 1/120; cfg.decimation = 4; cfg.sim.render_interval = 4
        cfg.scene.robot_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0., history_length=1)
        result['controller'] = dict(kind='Arena absolute joint-position action through PhysX drives',
            arm_stiffness=400., arm_damping=80., gripper_command='NPZ nominal finger_joint radians through original source mimic',
            source_droid_config_path='third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py',
            source_droid_config_sha256=sha(ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py'),
            robot_disable_gravity=cfg.scene.robot.spawn.rigid_props.disable_gravity,
            enabled_self_collisions=cfg.scene.robot.spawn.articulation_props.enabled_self_collisions,
            physics_dt=cfg.sim.dt, control_dt=cfg.sim.dt*cfg.decimation)
        env = builder.make_registered(cfg, kwargs, render_mode=None)
        env.unwrapped.render_enabled = False
        env.reset()
        e, robot = env.unwrapped, env.unwrapped.scene['robot']
        result['render_state_start'] = render_state(e)
        if not result['render_state_start']['passed']:
            raise RuntimeError('No-render runtime gate failed before actuator commands')
        result['runtime_threads']['actual_environment_device'] = str(e.device)
        if abs(e.step_dt-float(data['dt'])) > 1e-8: raise ValueError('Native control interval differs from planning interval')
        predicates = TaskPredicateRecorder(e, definition)
        recorder = PhysicalRecorder(e, definition, predicates)
        arm_ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
        result['reset_observed_arm_q'] = recorder.rows[0]['q'][arm_ids]
        result['reset_q_max_error_rad'] = float(np.max(np.abs(result['reset_observed_arm_q']-plan['start_q'])))
        result['reset_observed_object_states'] = recorder.rows[0]['task_object_states']
        result['initial_object_pose_reference'] = layout['objects']
        result['native_quaternion_convention'] = 'xyzw; Arena Pose and native recorder conventions match this pinned checkout'
        initial_differences = []
        for name, state in zip(recorder.names, recorder.rows[0]['task_object_states']):
            spec = layout['objects'][name]; half = math.radians(spec.get('yaw_deg', 0.))/2
            expected_quat = np.array([0., 0., math.sin(half), math.cos(half)])
            quat = state[3:7]; norm = float(np.linalg.norm(quat))
            if not .99 < norm < 1.01: raise RuntimeError('Invalid native xyzw quaternion readback')
            initial_differences.append(dict(name=name,
                position_error_m=float(np.linalg.norm(state[:3]-spec['position'])),
                orientation_error_deg=float(np.rad2deg(2*np.arccos(np.clip(abs(np.dot(quat/norm, expected_quat)), 0., 1.))))))
        result['reset_pose_comparison'] = initial_differences
        result['reset_state_validation'] = reset_state_gate(result['reset_q_max_error_rad'], initial_differences)
        original_reset = e._reset_idx
        def no_autoreset(ids):
            if len(ids):
                terms = {name: bool(array(e.termination_manager.get_term(name))[0]) for name in e.termination_manager.active_terms}
                terminal_events.append(dict(physics_step=int(e.sim.get_physics_step_count()), phase=recorder.phase, terms=terms))
            # Keep actual terminal state; no state reset after the sole initialization.
        e._reset_idx = no_autoreset
        if not result['reset_state_validation']['passed']:
            raise RuntimeError('Frozen initial-state gate failed: ' + ', '.join(result['reset_state_validation']['failures']))
        result.update(status='executing', command_started=True); write_json(out/'result.json', result)
        def step(q, grip, phase, source_index=-1):
            if len(actions) >= args.max_actions: raise RuntimeError('Maximum action count reached')
            recorder.phase, recorder.action_index, recorder.source_index = phase, len(actions), int(source_index)
            before = int(e.sim.get_physics_step_count())
            command = np.r_[q, grip].astype(np.float32)
            action = dict(action_index=len(actions), phase=phase, source_trajectory_index=int(source_index),
                target=command.tolist(), before_physics_step=before, after_physics_step=None)
            actions.append(action)
            try:
                env.step(torch.tensor(command[None], dtype=torch.float32, device=e.device))
                result['executed'] = True
                predicates.sample_action(phase, len(actions)-1)
            finally:
                action['after_physics_step'] = int(e.sim.get_physics_step_count())
                if action['after_physics_step'] > before: result['executed'] = True
            if predicates.actions[-1]['terminal_failure']:
                raise RuntimeError('Original source task failure/timeout termination')
        def hold(q, grip, phase, seconds):
            for _ in range(int(math.ceil(seconds/e.step_dt))): step(q, grip, phase)
        hold(np.asarray(plan['start_q']), 0., 'initial_settle', args.initial_settle_seconds)
        previous = None
        for i, (q, phase, grip) in enumerate(zip(data['q'], data['phase'], data['gripper'])):
            if phase != previous:
                if phase == 'lift': hold(data['q'][i-1], float(grip), 'close', args.close_seconds)
                if phase == 'retreat': hold(data['q'][i-1], 0., 'release', args.release_seconds)
            step(q, float(grip), str(phase), i)
            previous = phase
        hold(data['q'][-1], 0., 'final_settle', args.final_settle_seconds)
        result['sequence_complete'] = True
        observed_success = any(x['terminal_success'] for x in predicates.actions)
        final_on = predicates.actions[-1]['original_object_on_destination']
        result.update(original_task_success=observed_success, final_original_destination_predicate=bool(final_on),
            task_success=bool(observed_success and final_on), execution_status='success' if observed_success and final_on else 'failed',
            status='complete', failure_reason=None if observed_success and final_on else 'Original source PickAndPlaceTask success/final destination predicate not satisfied')
    except Exception as error:
        result.update(execution_status='failed' if result['executed'] else 'error', status='complete', task_success=False,
            failure_reason=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    finally:
        result.update(finished_at=stamp(), elapsed_seconds=time.monotonic()-started, action_count=len(actions), terminal_events=terminal_events)
        if recorder is not None:
            try: physical_arrays = recorder.close(out)
            except Exception as error: result['recording_error'] = repr(error)
        if predicates is not None:
            try: predicates.finish(out)
            except Exception as error: result['predicate_recording_error'] = repr(error)
        write_json(out/'actions.json', actions)
        if physical_arrays is not None:
            expected = 1 + sum(max(0, (x['after_physics_step'] or x['before_physics_step'])-x['before_physics_step']) for x in actions)
            result['physics_evidence'] = dict(samples=len(physical_arrays['q']), expected_post_reset_samples=expected,
                continuous=recorder.metadata['continuous_substeps'], complete=len(physical_arrays['q'])==expected and not recorder.errors,
                q_path=str((out/'physics_states.npz').relative_to(ROOT)), q_sha256=sha(out/'physics_states.npz'),
                metadata_path=str((out/'physics_metadata.json').relative_to(ROOT)), metadata_sha256=sha(out/'physics_metadata.json'))
        enforce_physics_evidence_gate(result)
        try:
            validate_entry_lineage(entry, args)
            if runtime_settings(args) != result['runtime_settings']:
                raise ValueError('Runtime settings/retry source changed during episode')
            result['source_inputs_unchanged'] = True
        except Exception as error: result.update(source_inputs_unchanged=False, source_binding_error=repr(error))
        result['execution_source_sha256_end'] = runtime_hashes()
        result['execution_sources_unchanged'] = result['execution_source_sha256_start'] == result['execution_source_sha256_end']
        if not result['execution_sources_unchanged']:
            result['source_binding_error'] = 'Execution source changed during physical trial'
        if result.get('physical_overlay_binding'):
            binding = result['physical_overlay_binding']; binding['sha256_end'] = sha(binding['path'])
            binding['unchanged'] = binding['sha256_end'] == binding['sha256_start']
            if not binding['unchanged']: result['source_binding_error'] = 'Physical overlay changed during trial'
        if any(result.get(k) for k in ('recording_error','predicate_recording_error','source_binding_error')):
            result.update(execution_status='error', task_success=False)
        if env is not None:
            try:
                result['render_state_end'] = render_state(env.unwrapped)
                result['render_generation_delta'] = (result['render_state_end']['render_generation']-
                    result.get('render_state_start', {}).get('render_generation',0))
                if not result['render_state_end']['passed'] or result['render_generation_delta'] != 0:
                    raise RuntimeError('Rendering occurred or was enabled during no-render physical test')
            except Exception as error:
                result.update(rendering_gate_error=repr(error), execution_status='error', task_success=False)
            if original_reset is not None: env.unwrapped._reset_idx = original_reset
            try: env.close()
            except Exception as error: result['environment_close_error'] = repr(error)
        result['evidence'] = {p.name: dict(path=str(p.relative_to(ROOT)), sha256=sha(p))
            for p in out.iterdir() if p.is_file() and p.name != 'result.json'}
        write_json(out/'result.json', result)
    return result, out/'result.json'
