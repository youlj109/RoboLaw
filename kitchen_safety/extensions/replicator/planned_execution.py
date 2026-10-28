"""Execute frozen cuRobo joint targets through native Arena/PhysX actuators.

No runtime object/robot pose writes, FK animation, planning, weld, or attachment.
The sole reset initializes the frozen authored scene. Native task predicates
and all post-reset physics substeps are observed; failed trials remain evidence.
"""
from pathlib import Path
from datetime import datetime, timezone
import copy
import hashlib
import json
import math
import shutil
import time
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')
RESET_TOLERANCES = {'arm_q_max_error_rad': .01, 'object_position_error_m': .002,
                    'object_orientation_error_deg': 1.}
RUNTIME_SOURCES = (
    'scripts/extensions/replicator/execute_planned_trajectories.py',
    'kitchen_safety/extensions/replicator/planned_execution.py',
    'kitchen_safety/extensions/replicator/background_physics.py',
    'kitchen_safety/extensions/replicator/mustard_family_background.py',
    'kitchen_safety/extensions/replicator/local_droid.py',
    'kitchen_safety/extensions/replicator/task_collection_evidence.py',
    'kitchen_safety/extensions/replicator/task_contacts.py',
    'kitchen_safety/extensions/replicator/startup_recorder.py',
    'kitchen_safety/extensions/replicator/overview.py',
    'kitchen_safety/dataset.py',
    'third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py',
    'third_party/IsaacLab-Arena/isaaclab_arena/tasks/pick_and_place_task.py',
    'third_party/IsaacLab-Arena/isaaclab_arena/tasks/predicates/spatial.py',
    'third_party/IsaacLab-Arena/isaaclab_arena/tasks/predicates/object_settling.py',
    'third_party/IsaacLab-Arena/isaaclab_arena/tasks/predicates/temporal.py',
)
SETTING_KEYS = ('device', 'seed', 'width', 'height', 'video_stride', 'torch_threads', 'physics_threads',
    'episode_seconds', 'max_actions', 'initial_settle_seconds', 'close_seconds',
    'release_seconds', 'final_settle_seconds')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def stamp():
    return datetime.now(timezone.utc).isoformat()

def runtime_hashes():
    return {p: sha(ROOT/p) for p in RUNTIME_SOURCES}

def runtime_settings(args):
    values = {k: getattr(args, k) for k in SETTING_KEYS}
    path = getattr(args, 'camera_overrides', None)
    values.update(camera_overrides_path=str(local(path).relative_to(ROOT)) if path else None,
        camera_overrides_sha256=sha(local(path)) if path else None)
    retake = getattr(args, 'camera_retake_plan', None)
    values.update(camera_retake_plan_path=str(local(retake).relative_to(ROOT)) if retake else None,
        camera_retake_plan_sha256=sha(local(retake)) if retake else None)
    return values

RENDER_SETTING_KEYS = ('/rtx/post/tonemap/op', '/rtx/post/tonemap/exposureTime',
    '/rtx/post/tonemap/filmIso', '/rtx/post/tonemap/fNumber',
    '/rtx/post/tonemap/responsivity', '/rtx/post/histogram/enabled')

def load_camera_overrides(path):
    if not path: return None
    path = local(path); data = json.loads(path.read_text())
    if data.get('kind') != 'physical_execution_camera_overrides' or not isinstance(data.get('tasks'), dict):
        raise ValueError('Invalid camera override document')
    allowed = {'eye','target','focal_length_mm','horizontal_aperture_mm','vertical_aperture_mm',
        'clipping_range_m','renderer_settings','evidence_path','evidence_sha256',
        'image_path','image_sha256','evidence_config_sha256'}
    for task, row in data['tasks'].items():
        if not task.startswith('kitchen_bench_replicator_') or set(row) != allowed:
            raise ValueError('Only the declared camera/exposure fields are permitted')
        eye, target = np.asarray(row['eye'], dtype=float), np.asarray(row['target'], dtype=float)
        if eye.shape != (3,) or target.shape != (3,) or not np.isfinite([eye,target]).all() or np.linalg.norm(eye-target) < 1e-6:
            raise ValueError('Invalid camera eye/target')
        for key in ('focal_length_mm','horizontal_aperture_mm','vertical_aperture_mm'):
            if not np.isfinite(row[key]) or row[key] <= 0: raise ValueError('Invalid camera optics')
        clip = np.asarray(row['clipping_range_m'], dtype=float)
        if clip.shape != (2,) or not np.isfinite(clip).all() or not 0 < clip[0] < clip[1]:
            raise ValueError('Invalid camera clipping range')
        render = row['renderer_settings']
        if set(render) != set(RENDER_SETTING_KEYS) or type(render['/rtx/post/histogram/enabled']) is not bool:
            raise ValueError('Only the declared exposure settings are permitted')
        if any(not np.isfinite(render[k]) or render[k] <= 0 for k in RENDER_SETTING_KEYS[:-1]):
            raise ValueError('Invalid exposure value')
        for field in ('evidence','image'):
            if sha(local(row[field+'_path'])) != row[field+'_sha256']:
                raise ValueError('Camera screenshot evidence changed')
    return dict(path=str(path.relative_to(ROOT)), sha256=sha(path), tasks=data['tasks'])

def apply_render_override(values):
    import carb
    settings = carb.settings.get_settings()
    before = {key: settings.get(key) for key in RENDER_SETTING_KEYS}
    if any(value is None for value in before.values()):
        raise RuntimeError('Cannot preserve an absent exposure setting')
    try:
        for key, value in values.items(): settings.set(key, value)
        actual = {key: settings.get(key) for key in RENDER_SETTING_KEYS}
        if any(not np.isclose(actual[k], values[k], rtol=1e-6, atol=1e-12) for k in values):
            raise RuntimeError('Exposure readback mismatch')
    except Exception:
        for key, value in before.items(): settings.set(key, value)
        raise
    return dict(original=before, requested=copy.deepcopy(values), applied=actual, restored=False)

def validate_physical_attempt(entry, args, output_root):
    """Additional physical runs require an exact, source-bound camera-retake whitelist."""
    number = entry.get('physical_attempt_number', 1)
    if type(number) is not int or number not in (1, 2): raise ValueError('Invalid physical attempt number')
    if number == 1:
        if any(k in entry for k in ('first_pass','camera_retake_plan_path','camera_retake_plan_sha256')):
            raise ValueError('Retake provenance cannot be labelled first physical attempt')
        return
    first = entry['first_pass']; rp = local(first['result_path']); vp = local(first['video_path'])
    if sha(rp) != first['result_sha256'] or sha(vp) != first['video_sha256']:
        raise ValueError('First physical result/video changed')
    prior = json.loads(rp.read_text())
    if prior.get('kind') != 'physical_execution' or prior.get('scene_id') != entry['scene_id'] or prior.get('executed') is not True:
        raise ValueError('A real first physical execution is required')
    if first.get('physical_attempt_number') != 1 or prior.get('physical_attempt_number', 1) != 1:
        raise ValueError('Only a second physical attempt is authorized')
    if prior.get('status') != 'complete' or prior.get('execution_status') not in ('success','failed'):
        raise ValueError('First physical attempt is not a completed recorded trial')
    for key in ('execution_status','task_success','executed'):
        if first[key] != prior[key]: raise ValueError('First-pass outcome differs')
    if local(prior['video_path']) != vp or prior['video_sha256'] != first['video_sha256']:
        raise ValueError('First-pass video identity differs')
    for key in ('config','trajectory'):
        if local(prior[key+'_path']) != local(entry[key+'_path']) or prior[key+'_sha256'] != entry[key+'_sha256']:
            raise ValueError('Retake must retain original configuration and trajectory')
    if local(prior['planning_result_path']) != local(entry['result_path']) or prior['planning_result_sha256'] != entry['result_sha256']:
        raise ValueError('Retake planning result differs')
    if execution_output_path(output_root, entry) == rp.parent or Path(output_root)/'videos'/(entry['scene_id']+'.mp4') == vp:
        raise ValueError('Retake output would overwrite first physical evidence')
    plan_path = local(entry['camera_retake_plan_path'])
    if sha(plan_path) != entry['camera_retake_plan_sha256']: raise ValueError('Camera retake plan changed')
    selected_plan = getattr(args, 'camera_retake_plan', None)
    if not selected_plan or local(selected_plan) != plan_path:
        raise ValueError('Retake requires the same explicit --camera-retake-plan')
    plan = json.loads(plan_path.read_text())
    if plan.get('kind') != 'physical_camera_retake': raise ValueError('Invalid camera-retake plan kind')
    rows = [row for row in plan['entries'] if row['scene_id'] == entry['scene_id']]
    if len(rows) != 1: raise ValueError('Scene is not uniquely whitelisted for camera retake')
    row = rows[0]
    for key in ('scene_id','config_sha256','trajectory_sha256','first_pass','physical_attempt_number'):
        if row[key] != entry[key]: raise ValueError('Camera retake whitelist differs: '+key)
    for key in ('config_path','trajectory_path'):
        if local(row[key]) != local(entry[key]): raise ValueError('Camera retake source path differs')
    settings = runtime_settings(args)
    if row.get('locked_execution_settings') != prior['runtime_settings']:
        raise ValueError('Locked execution settings differ from first physical evidence')
    for key in SETTING_KEYS:
        if settings[key] != prior['runtime_settings'][key]:
            raise ValueError('Camera-only retake cannot change execution settings: '+key)
    if local(plan['camera_overrides_path']) != local(settings['camera_overrides_path']) or plan['camera_overrides_sha256'] != settings['camera_overrides_sha256']:
        raise ValueError('Retake camera configuration differs from the whitelist')
    camera = load_camera_overrides(getattr(args, 'camera_overrides', None))
    if not camera or entry['task_id'] not in camera['tasks']: raise ValueError('Retake requires a task camera override')

def configure_physics_threads(requested):
    """Set native worker scheduling before scene initialization, with live readback."""
    if not isinstance(requested, int) or isinstance(requested, bool) or requested < 1:
        raise ValueError('physics_threads must be a positive integer')
    import carb
    import omni.physx
    from omni.physx.bindings._physx import SETTING_NUM_THREADS
    settings = carb.settings.get_settings()
    previous = settings.get(SETTING_NUM_THREADS)
    settings.set_int(SETTING_NUM_THREADS, requested)
    omni.physx.get_physx_interface().set_thread_count(requested)
    actual = settings.get(SETTING_NUM_THREADS)
    if actual != requested:
        raise RuntimeError(f'PhysX thread setting readback differs: requested={requested}, actual={actual}')
    return dict(requested_physics_threads=requested, previous_persistent_physics_threads=previous,
        live_physics_threads=actual, native_set_thread_count_called=True,
        setting_path=SETTING_NUM_THREADS,
        readback_scope='Live Carb setting after native set_thread_count; not an OS worker-thread census')

def execution_output_path(output_root, entry):
    """Only an explicitly archived initialization may use the one named child path."""
    subdirectory = entry.get('execution_subdirectory', '')
    if subdirectory not in ('', 'initialization_retry_01'):
        raise ValueError('Unsupported execution_subdirectory')
    out = Path(output_root)/'executions'/entry['scene_id']
    return out/subdirectory if subdirectory else out

def local(path):
    p = (ROOT / str(path).removeprefix('/workspace/')).resolve()
    if not p.is_relative_to(ROOT):
        raise ValueError('Input outside repository')
    return p

def write_json(path, value):
    def convert(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        if isinstance(x, Path): return str(x)
        raise TypeError(type(x).__name__)
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=convert, allow_nan=False) + '\n')
    temporary.replace(path)

def validate_planned_arrays(data):
    """Reject misaligned streams before zip can truncate commanded motion."""
    n = len(data['q'])
    shapes = {'q': (n, 7), 'tcp_world': (n, 4, 4), 'phase': (n,), 'gripper': (n,), 'dt': ()}
    if n < 6 or any(np.asarray(data[k]).shape != shape for k, shape in shapes.items()):
        raise ValueError('Invalid or misaligned planning arrays: ' + str({k: np.asarray(data[k]).shape for k in shapes}))
    if tuple(dict.fromkeys(data['phase'].tolist())) != PHASES or not all(np.isfinite(data[k]).all() for k in ('q', 'tcp_world', 'gripper', 'dt')):
        raise ValueError('Invalid phase sequence or finite state')
    if abs(float(data['dt']) - 1/30) > 1e-9:
        raise ValueError('This executor retains the source 30Hz command interval')


def reset_state_gate(q_error, object_differences):
    """These are initialization identity tolerances, never task-success rules."""
    failures = []
    if not np.isfinite(q_error) or q_error > RESET_TOLERANCES['arm_q_max_error_rad']:
        failures.append('robot_initial_q_mismatch')
    if not object_differences: failures.append('missing_initial_object_comparison')
    for row in object_differences:
        for field, limit in (('position_error_m', 'object_position_error_m'),
                             ('orientation_error_deg', 'object_orientation_error_deg')):
            value = row[field]
            if not np.isfinite(value) or value > RESET_TOLERANCES[limit]:
                failures.append(row['name'] + ':' + field)
    return dict(passed=not failures, failures=failures, tolerances=RESET_TOLERANCES.copy(),
        scope='Post-reset native initial state versus frozen source; no actuator command if failed')


def enforce_physics_evidence_gate(result):
    """Incomplete measured evidence can never retain a physical success flag."""
    evidence = result.get('physics_evidence')
    required = bool(result.get('executed') or result.get('task_success') or result.get('execution_status') == 'success')
    passed = bool(isinstance(evidence, dict) and evidence.get('complete') is True
                  and evidence.get('continuous') is True)
    result['physics_evidence_gate'] = dict(required=required, passed=passed if required else None)
    if required and not passed:
        result.update(execution_status='error', task_success=False,
            recording_error='Native physics evidence missing, incomplete, or non-continuous')


def load_inputs(entry):
    for pathkey, hashkey in (('result_path', 'result_sha256'), ('config_path', 'config_sha256'),
                             ('trajectory_path', 'trajectory_sha256'), ('source_scene_path', 'source_scene_sha256')):
        if sha(local(entry[pathkey])) != entry[hashkey]:
            raise ValueError('Frozen input changed: ' + entry[pathkey])
    result = json.loads(local(entry['result_path']).read_text())
    layout = json.loads(local(entry['config_path']).read_text())
    if result['status'] != 'generated' or not result['trajectory_complete']:
        raise ValueError('A complete planning result is required')
    if result['scene_id'] != entry['scene_id'] or layout['task_id'] != entry['task_id']:
        raise ValueError('Scene identity differs')
    data = dict(np.load(local(entry['trajectory_path']), allow_pickle=False))
    validate_planned_arrays(data)
    for path, digest in result['composition']['source_paths_sha256'].items():
        if sha(local(path)) != digest: raise ValueError('Source asset changed: ' + path)
    return result, layout, data


def build_environment(result, layout, asset_output, episode_seconds, *, background_name='kitchen'):
    """Generalize the existing source-PickAndPlace factories without changing goals."""
    import yaml
    from isaaclab.envs.common import ViewerCfg
    from isaaclab.envs.mdp.actions import JointPositionActionCfg
    from isaaclab_arena.assets.background import Background
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_reference import ObjectReference
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.assets.registries import AssetRegistry
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.relations.relations import IsAnchor, NextTo, On, RotateAroundSolution
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.tasks.pick_and_place_task import PickAndPlaceTask
    from isaaclab_arena.utils.pose import Pose
    from .local_droid import LocalDroid
    from .mustard_family_background import family_mounted_kitchen
    assignment = json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())
    item = next(x for x in assignment['tasks'] if x['id'] == layout['task_id'])
    source_yaml = ROOT/item['source_yaml']
    if sha(source_yaml) != item['source_sha256']: raise ValueError('Original YAML changed')
    original = yaml.safe_load(source_yaml.read_text())
    subs = original['task']['subtasks']
    if original['task']['composition'] != 'atomic' or len(subs) != 1 or subs[0]['kind'] != 'PickAndPlaceTask':
        raise ValueError('Unsupported original task; no predicate substitution allowed')
    background_path, overlay = family_mounted_kitchen(ROOT/layout['kitchen_usd'], asset_output)
    def pose(position, yaw=0.):
        half = math.radians(yaw)/2
        # This pinned Arena/IsaacLab checkout explicitly uses xyzw Pose.
        return Pose(tuple(position), (0., 0., math.sin(half), math.cos(half)))
    kitchen = Background(name=background_name, usd_path=str(background_path), object_min_z=-.2,
        initial_pose=Pose.identity(), reset_nested_physics=True, **original['background']['params'])
    objects = {name: Object(name=name, usd_path=str(ROOT/spec['usd_path']), object_type=ObjectType.RIGID,
        initial_pose=pose(spec['position'], spec.get('yaw_deg', 0.))) for name, spec in layout['objects'].items()}
    references = {row['id']: ObjectReference(name=row['id'], parent_asset=kitchen,
        prim_path=kitchen.get_prim_path()+'/'+row['prim_path'], object_type=ObjectType.BASE,
        **row['params']) for row in original['object_references']}
    base = list(layout['robot_position']); base[2] += layout['stand_height']
    robot = LocalDroid(enable_cameras=False, stand_height_m=layout['stand_height'],
        initial_pose=pose(base, layout['robot_yaw_deg']), initial_joint_pose=list(result['start_q'])+[0.]*6)
    robot.event_config.randomize_franka_joint_state.params['std'] = 0.
    # Absolute continuous actuator target, not a state setter. PhysX source mimic
    # joints remain intact. The NPZ nominal closure is not assumed to grasp.
    robot.action_config.gripper_action = JointPositionActionCfg(asset_name='robot',
        joint_names=['finger_joint'], preserve_order=True, use_default_offset=False)
    entities = {'kitchen': kitchen, 'droid': robot, **objects, **references}
    relation_types = {'is_anchor': IsAnchor, 'next_to': NextTo, 'on': On, 'rotate_around_solution': RotateAroundSolution}
    for row in original['relations']:
        args = [entities[row['reference']]] if 'reference' in row else []
        entities[row['subject']].add_relation(relation_types[row['kind']](*args, **row['params']))
    params = copy.deepcopy(subs[0]['params'])
    for name in ('pick_up_object', 'destination_location', 'background_scene', 'destination_object'):
        if name in params: params[name] = entities[params[name]]
    task = PickAndPlaceTask(**params, task_description=original['task']['description'], episode_length_s=episode_seconds)
    hazard = layout['obstacle']
    hazard_object = Object(name='hazard', usd_path=str(ROOT/hazard['usd_path']),
        object_type=ObjectType.BASE if hazard['static'] else ObjectType.RIGID,
        initial_pose=pose(hazard['position'], hazard.get('yaw_deg', 0.)))
    registry = AssetRegistry()
    assets = [kitchen, *objects.values(), *references.values(), hazard_object,
              registry.get_asset_by_name('light')(), registry.get_asset_by_name('directional_light')()]
    eye = result['composition']['camera_eye']; target = result['composition']['camera_target']
    def viewer(cfg):
        cfg.viewer = ViewerCfg(eye=tuple(eye), lookat=tuple(target)); return cfg
    definition = IsaacLabArenaEnvironment(name=layout['task_id']+'_planned_physical_execution',
        embodiment=robot, scene=Scene(assets=assets), task=task, env_cfg_callback=viewer)
    definition.resolved_task_layout = layout
    definition.source_task_spec = original
    definition.background_physics_provenance = overlay
    definition.source_relation_audit = {'source_yaml': item['source_yaml'], 'source_yaml_sha256': item['source_sha256'],
        'layout_used_without_relation_solving': True, 'physical_relations_verified': False}
    return definition


class PhysicalRecorder:
    """Generic read-only recorder covering every native post-reset substep."""
    def __init__(self, e, definition, predicates):
        from .startup_recorder import array
        from .task_contacts import ObjectGraspContacts
        self.e, self.scene, self.robot, self.predicates = e, e.scene, e.scene['robot'], predicates
        self.names = list(definition.resolved_task_layout['objects'])
        self.payload = definition.task.pick_up_object.name
        self.phase, self.action_index, self.source_index = 'reset', -1, -1
        self.rows, self.errors = [], []
        view = self.scene['robot_contacts']._physics_sim_view
        self.contacts = ObjectGraspContacts(view, e.physics_dt, self.payload)
        self.all_bodies = view.create_rigid_body_view('/World/envs/env_0/**')
        self.task = definition.task
        self.metadata = dict(joint_names=list(self.robot.joint_names), robot_body_names=list(self.robot.body_names),
            object_names=self.names, payload_name=self.payload, contact_body_names=list(self.scene['robot_contacts'].body_names),
            all_rigid_body_paths=list(self.all_bodies.prim_paths), quaternion_convention='xyzw (pinned Arena/IsaacLab checkout)',
            physics_dt_s=float(e.physics_dt), control_dt_s=float(e.step_dt), reset_physics_step=int(e.sim.get_physics_step_count()),
            runtime_state_setters_used=False, physical_weld_used=False, payload_attachment_used=False,
            scope='Native measured post-reset state and each scene.update substep; reset-internal states not claimed')
        self.original_update = self.scene.update
        self.capture()
        def update(dt, *args, **kwargs):
            result = self.original_update(dt, *args, **kwargs)
            step = int(e.sim.get_physics_step_count())
            if step != self.rows[-1]['physics_step']:
                try: self.capture()
                except Exception as error:
                    self.errors.append(dict(physics_step=step, error=repr(error))); raise
            return result
        self.scene.update = update

    def capture(self):
        from .startup_recorder import array
        robot = self.robot
        states = np.stack([array(self.scene[name].data.root_state_w)[0] for name in self.names])
        row = dict(physics_step=int(self.e.sim.get_physics_step_count()), phase=self.phase,
            action_index=self.action_index, source_trajectory_index=self.source_index,
            q=array(robot.data.joint_pos)[0], dq=array(robot.data.joint_vel)[0],
            robot_body_pose_w=np.concatenate((array(robot.data.body_pos_w)[0], array(robot.data.body_quat_w)[0]), axis=-1),
            task_object_states=states, robot_contact_force=array(self.scene['robot_contacts'].data.net_forces_w)[0],
            destination_contact_force=array(self.scene[self.task.contact_sensor_name].data.force_matrix_w)[0],
            all_rigid_body_pose_w=array(self.all_bodies.get_transforms()),
            all_rigid_body_velocity_w=array(self.all_bodies.get_velocities()))
        if not all(np.isfinite(v).all() for v in row.values() if isinstance(v, np.ndarray)):
            raise RuntimeError('Nonfinite native physics state')
        self.rows.append(row)
        self.contacts.sample(self.phase, self.action_index, states[self.names.index(self.payload), :7], physics_step=row['physics_step'])
        self.predicates.sample_physics(self.phase, self.action_index)
        return row

    def close(self, output):
        self.scene.update = self.original_update
        arrays = {k: np.asarray([row[k] for row in self.rows]) for k in self.rows[0]}
        self.metadata.update(samples=len(self.rows), callback_errors=self.errors,
            continuous_substeps=bool(np.all(np.diff(arrays['physics_step']) == 1)))
        np.savez_compressed(output/'physics_states.npz', **arrays)
        write_json(output/'physics_metadata.json', self.metadata)
        write_json(output/'grasp_contacts.json', self.contacts.records)
        return arrays


def execute_one(entry, output_root, args):
    """Exactly one physical episode; initialization/runtime failures are retained."""
    import torch
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from .startup_recorder import array
    from .task_collection_evidence import TaskPredicateRecorder
    from .overview import configure_overview
    from kitchen_safety.dataset import CameraWriter, decode_video
    validate_physical_attempt(entry, args, output_root)
    out = execution_output_path(output_root, entry); out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    result = dict(kind='physical_execution', scene_id=entry['scene_id'], task_id=entry['task_id'], variant_id=entry['variant_id'],
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
    if entry.get('execution_subdirectory'):
        result['execution_subdirectory'] = entry['execution_subdirectory']
    if 'initialization_history' in entry:
        result['initialization_history'] = copy.deepcopy(entry['initialization_history'])
    for key in ('first_pass','camera_retake_plan_path','camera_retake_plan_sha256'):
        if key in entry: result[key] = copy.deepcopy(entry[key])
    write_json(out/'result.json', result)
    env = recorder = writer = predicates = None
    original_reset = None
    actions, camera_frames, terminal_events = [], [], []
    physical_arrays = None
    render_override = None
    try:
        plan, layout, data = load_inputs(entry)
        camera_document = load_camera_overrides(getattr(args, 'camera_overrides', None))
        camera = camera_document['tasks'].get(entry['task_id']) if camera_document else None
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
        cfg.observations.camera_obs = None
        cfg.sim.dt = 1/120; cfg.decimation = 4; cfg.sim.render_interval = 4
        cfg.scene.robot_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0., history_length=1)
        original_eye, original_target = plan['composition']['camera_eye'], plan['composition']['camera_target']
        overview = configure_overview(cfg.scene, eye=camera['eye'] if camera else original_eye,
            target=camera['target'] if camera else original_target)
        if camera:
            spawn = cfg.scene.overview_camera.spawn
            original_optics = dict(focal_length_mm=spawn.focal_length,
                horizontal_aperture_mm=spawn.horizontal_aperture, vertical_aperture_mm=spawn.vertical_aperture,
                clipping_range_m=list(spawn.clipping_range))
            spawn.focal_length = camera['focal_length_mm']
            spawn.horizontal_aperture = camera['horizontal_aperture_mm']
            spawn.vertical_aperture = camera['vertical_aperture_mm']
            spawn.clipping_range = tuple(camera['clipping_range_m'])
            overview.update({k: camera[k] for k in original_optics})
            result['camera_override'] = dict(path=camera_document['path'], sha256=camera_document['sha256'],
                task_id=entry['task_id'], original=dict(eye=original_eye, target=original_target, **original_optics),
                applied=copy.deepcopy(camera), scope='Camera optics/pose and renderer exposure only; all physical task inputs retained')
        cfg.scene.overview_camera.width = args.width; cfg.scene.overview_camera.height = args.height
        overview.update(width=args.width, height=args.height)
        result['overview'] = overview
        result['controller'] = dict(kind='Arena absolute joint-position action through PhysX drives',
            arm_stiffness=400., arm_damping=80., gripper_command='NPZ nominal finger_joint radians through original source mimic',
            source_droid_config_path='third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py',
            source_droid_config_sha256=sha(ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py'),
            robot_disable_gravity=cfg.scene.robot.spawn.rigid_props.disable_gravity,
            enabled_self_collisions=cfg.scene.robot.spawn.articulation_props.enabled_self_collisions,
            physics_dt=cfg.sim.dt, control_dt=cfg.sim.dt*cfg.decimation)
        env = builder.make_registered(cfg, kwargs); env.reset()
        e, robot = env.unwrapped, env.unwrapped.scene['robot']
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
        writer = CameraWriter(out, e.scene, ['overview_camera'], e.step_dt*args.video_stride, previews=True)
        if camera:
            render_override = apply_render_override(camera['renderer_settings'])
            result['camera_renderer'] = render_override
        result['appearance_warmup'] = writer.warm_up(e.sim, min_frames=32, max_frames=128, sample_every=8, stable_windows=2)
        if int(e.sim.get_physics_step_count()) != recorder.metadata['reset_physics_step']:
            raise RuntimeError('Camera warmup advanced unrecorded physics')
        if not result['reset_state_validation']['passed']:
            writer.write('initialization_mismatch')
            camera_frames.append(dict(frame=writer.frames-1, action_index=-1,
                physics_step=int(e.sim.get_physics_step_count()), phase='initialization_mismatch',
                source_trajectory_index=-1))
            raise RuntimeError('Frozen initial-state gate failed: ' + ', '.join(result['reset_state_validation']['failures']))
        result['status'] = 'executing'; write_json(out/'result.json', result)
        def step(q, grip, phase, source_index=-1):
            if len(actions) >= args.max_actions: raise RuntimeError('Maximum action count reached')
            recorder.phase, recorder.action_index, recorder.source_index = phase, len(actions), int(source_index)
            before = int(e.sim.get_physics_step_count())
            if len(actions) % args.video_stride == 0:
                writer.write(phase)
                camera_frames.append(dict(frame=writer.frames-1, action_index=len(actions), physics_step=before, phase=phase, source_trajectory_index=int(source_index)))
                if int(e.sim.get_physics_step_count()) != before: raise RuntimeError('Camera capture advanced physics')
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
        if writer is not None:
            try: writer.close()
            except Exception as error: result['video_error'] = repr(error)
        if recorder is not None:
            try: physical_arrays = recorder.close(out)
            except Exception as error: result['recording_error'] = repr(error)
        if predicates is not None:
            try: predicates.finish(out)
            except Exception as error: result['predicate_recording_error'] = repr(error)
        write_json(out/'actions.json', actions); write_json(out/'camera_frames.json', camera_frames)
        if physical_arrays is not None:
            expected = 1 + sum(max(0, (x['after_physics_step'] or x['before_physics_step'])-x['before_physics_step']) for x in actions)
            result['physics_evidence'] = dict(samples=len(physical_arrays['q']), expected_post_reset_samples=expected,
                continuous=recorder.metadata['continuous_substeps'], complete=len(physical_arrays['q'])==expected and not recorder.errors,
                q_path=str((out/'physics_states.npz').relative_to(ROOT)), q_sha256=sha(out/'physics_states.npz'),
                metadata_path=str((out/'physics_metadata.json').relative_to(ROOT)), metadata_sha256=sha(out/'physics_metadata.json'))
        enforce_physics_evidence_gate(result)
        native_video = out/'overview_camera.mp4'
        if native_video.exists() and writer is not None:
            try:
                decode_video(native_video, writer.frames, expected_size=(args.width, args.height), expected_dt=(1/30)*args.video_stride)
                target = output_root/'videos'/(entry['scene_id']+'.mp4'); target.parent.mkdir(exist_ok=True)
                if target.exists(): raise FileExistsError(target)
                shutil.copyfile(native_video, target)
                result.update(video_path=str(target.relative_to(ROOT)), video_sha256=sha(target), frame_count=writer.frames,
                    fps=30/args.video_stride, width=args.width, height=args.height, duration_seconds=writer.frames*args.video_stride/30,
                    video_verified=True)
            except Exception as error: result['video_error'] = repr(error)
        else: result['video_missing'] = True
        try:
            load_inputs(entry); validate_physical_attempt(entry, args, output_root)
            if runtime_settings(args) != result['runtime_settings']:
                raise ValueError('Runtime settings/camera/retake source changed during episode')
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
        if any(result.get(k) for k in ('video_error','recording_error','predicate_recording_error','source_binding_error')):
            result.update(execution_status='error', task_success=False)
        if env is not None:
            if original_reset is not None: env.unwrapped._reset_idx = original_reset
            try: env.close()
            except Exception as error: result['environment_close_error'] = repr(error)
        if render_override is not None:
            try:
                import carb
                settings = carb.settings.get_settings()
                for key, value in render_override['original'].items(): settings.set(key, value)
                render_override['restored_values'] = {key: settings.get(key) for key in RENDER_SETTING_KEYS}
                render_override['restored'] = all(np.isclose(render_override['restored_values'][k], v, rtol=1e-6, atol=1e-12)
                    for k, v in render_override['original'].items())
                if not render_override['restored']: raise RuntimeError('Renderer exposure restore mismatch')
            except Exception as error:
                result.update(renderer_restore_error=repr(error), execution_status='error', task_success=False)
        write_json(out/'result.json', result)
    return result, out/'result.json'
