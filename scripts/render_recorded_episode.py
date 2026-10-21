"""Render measured obs[t] without executing actions or advancing physics.

Requires a successful source collected with --record-render-state. A new
output directory receives RGB and provenance only; source trajectories and
success flags are never copied or modified. Imports of Kit remain in main.
"""
import json
from contextlib import nullcontext
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera')
KITCHEN_SHAPES = {'kitchen_root_state': (21, 13), 'kitchen_joint_pos': (48,),
                  'kitchen_joint_vel': (48,), 'kitchen_link_pose': (87, 7),
                  'kitchen_link_velocity': (87, 6)}


def render_provenance(args, source_result):
    purpose = getattr(args, 'collection_purpose', 'diagnostic')
    return {'collection_purpose': purpose,
            'collection_source': getattr(args, 'collection_source', None),
            'source_collection_purpose': source_result.get('collection_purpose', 'diagnostic'),
            'source_collection_source': source_result.get('collection_source'),
            'source_collection_job': source_result.get('collection_job'),
            'acceptance_status': 'pending_batch_verification' if purpose == 'production' else 'diagnostic',
            'production_quota_eligible': False,
            'acceptance_authority': 'verify_attempt after original physics, safety, RGB and obstruction checks'}


def select_indices(count, requested=None):
    values = list(range(count)) if requested is None else list(requested)
    if (count <= 0 or not values or any(type(i) is not int or i < 0 or i >= count for i in values)
            or len(set(values)) != len(values) or values != sorted(values)):
        raise ValueError('Obs indices must be nonempty, unique, increasing and in range')
    return values


def _read_json(path):
    return json.loads(Path(path).read_text())


def load_source(source, requested=None):
    """CPU-only loader: refuse legacy/reset-only states before launching Kit."""
    import h5py
    import numpy as np

    source = Path(source)
    required_files = ('result.json', 'episode_spec.json', 'trajectory.h5',
                      'render_state_metadata.json', 'camera_config.json')
    missing = [name for name in required_files if not (source / name).is_file()]
    if missing:
        raise ValueError('Source requires --record-render-state; missing: ' + ', '.join(missing))
    result, spec = _read_json(source / 'result.json'), _read_json(source / 'episode_spec.json')
    if result.get('success') is not True and not (source / 'safety_observer_binding.json').exists():
        raise ValueError('Source must be successful or an explicitly recorded safety observer attempt')
    metadata = _read_json(source / 'render_state_metadata.json')
    cameras = _read_json(source / 'camera_config.json')
    if not isinstance(metadata.get('kitchen'), dict) or metadata.get('quaternion_order') != 'xyzw':
        raise ValueError('Missing kitchen ordering or unsupported quaternion order')
    if set(CAMERAS) - set(cameras):
        raise ValueError('Source must describe all three original cameras')
    for name in CAMERAS:
        cfg = cameras[name]
        if (cfg.get('width'), cfg.get('height')) != (1280, 720):
            raise ValueError(f'{name}: expected original 1280x720 resolution')
        for key in ('prim_path', 'offset', 'spawn'):
            if key not in cfg:
                raise ValueError(f'{name}: missing authored camera {key}')
    with h5py.File(source / 'trajectory.h5', 'r') as handle:
        count = len(handle['action'])
        indices = select_indices(count, requested)
        joint_names = json.loads(handle.attrs['joint_names'])
        body_names = json.loads(handle.attrs['robot_body_names'])
        if len(body_names) != 18 or len(joint_names) != 13:
            raise ValueError('Expected original robot with 18 bodies and 13 joints')
        if len(set(body_names)) != len(body_names) or len(set(joint_names)) != len(joint_names):
            raise ValueError('Recorded robot names must be unique')
        for key, names in [('body_names', body_names), ('joint_names', joint_names)]:
            if metadata.get('robot', {}).get(key) != names:
                raise ValueError(f'Render metadata and trajectory robot {key} disagree')
        coordinate = str(handle.attrs.get('coordinate_frame', 'world'))
        if coordinate not in ('world', 'environment_local') or metadata.get('coordinate_frame') != coordinate:
            raise ValueError('Trajectory and render-state coordinate frames disagree')
        dynamic = not spec['layout']['obstacle']['static']
        if metadata.get('hazard', {}).get('dynamic') != dynamic:
            raise ValueError('Hazard state metadata disagrees with episode layout')
        shapes = {**KITCHEN_SHAPES, 'q': (13,), 'dq': (13,), 'robot_body_pose_w': (18, 7),
                  'robot_body_velocity': (18, 6), 'mustard': (13,), 'bowl': (13,)}
        if dynamic:
            shapes['hazard_state'] = (13,)
        state = {}
        for group in ('obs', 'next_obs'):
            for key, shape in shapes.items():
                path = f'{group}/{key}'
                if path not in handle or handle[path].shape != (count, *shape):
                    raise ValueError(f'Missing or malformed measured {path}; require --record-render-state')
                array = np.asarray(handle[path])
                if not np.isfinite(array).all():
                    raise ValueError(f'Nonfinite measured {path}')
                if key in ('robot_body_pose_w', 'mustard', 'bowl', 'hazard_state',
                           'kitchen_root_state', 'kitchen_link_pose'):
                    if np.any(np.linalg.norm(array[..., 3:7], axis=-1) < 1e-8):
                        raise ValueError(f'Invalid zero quaternion in {path}')
                if group == 'obs':
                    state[key] = array[indices]
        phases = handle['phase'].asstr()[:]
        times = np.asarray(handle['time'])
        if len(phases) != count or len(times) != count or not np.isfinite(times).all():
            raise ValueError('Action, phase and timestamp counts disagree')
        dt = float(handle.attrs['dt'])
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid source action interval')
        origin = np.asarray(handle.attrs.get('environment_origin_world_m', [0., 0., 0.]))
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('Invalid source environment origin')
    frames = [{'source_obs_index': index, 'phase': str(phases[index]),
               'source_time_seconds': float(times[index]),
               'state': {key: values[ordinal] for key, values in state.items()}}
              for ordinal, index in enumerate(indices)]
    # Optional real source RGB camera poses; never require RGB for a state-only source.
    image_path = source / 'images.h5'
    if image_path.is_file():
        with h5py.File(image_path, 'r') as images:
            for name in CAMERAS:
                if name not in images:
                    raise ValueError(f'Existing source RGB is missing {name}')
                group = images[name]
                if group['position_world'].shape != (count, 3) or group['quaternion_world_ros_xyzw'].shape != (count, 4):
                    raise ValueError(f'{name}: source camera count disagrees with actions')
                for frame in frames:
                    index = frame['source_obs_index']
                    frame.setdefault('recorded_cameras', {})[name] = {
                        'pose': np.r_[group['position_world'][index], group['quaternion_world_ros_xyzw'][index]],
                        'coordinate_frame': str(group.attrs.get('coordinate_frame', images.attrs.get('coordinate_frame', 'world'))),
                        'origin': np.asarray(group.attrs.get('environment_origin_world_m',
                                                            images.attrs.get('environment_origin_world_m', [0., 0., 0.]))) }
    return {'result': result, 'spec': spec, 'metadata': metadata, 'camera_config': cameras,
            'source_camera_metadata': _read_json(source / 'cameras.json') if (source / 'cameras.json').is_file() else None,
            'appearance': _read_json(source / 'appearance.json') if (source / 'appearance.json').is_file() else None,
            'frames': frames,
            'source_steps': count, 'indices': indices, 'partial': indices != list(range(count)),
            'dt': dt, 'joint_names': joint_names, 'body_names': body_names,
            'coordinate_frame': coordinate, 'source_origin': origin, 'dynamic_hazard': dynamic}


def translated_state(value, coordinate_frame, source_origin, replay_origin):
    import numpy as np
    result = np.asarray(value).copy()
    if coordinate_frame not in ('world', 'environment_local'):
        raise ValueError(f'Unsupported coordinate frame {coordinate_frame}')
    result[..., :3] += np.asarray(replay_origin) - (np.asarray(source_origin) if coordinate_frame == 'world' else 0)
    return result


def pose_errors(actual, expected):
    import numpy as np
    from scipy.spatial.transform import Rotation
    actual, expected = np.asarray(actual).reshape(-1, 7), np.asarray(expected).reshape(-1, 7)
    if actual.shape != expected.shape or not np.isfinite([actual, expected]).all():
        raise ValueError('Pose comparison needs matching finite xyz+xyzw arrays')
    angles = (Rotation.from_quat(expected[:, 3:]).inv() * Rotation.from_quat(actual[:, 3:])).magnitude()
    return {'position_max_error_m': float(np.linalg.norm(actual[:, :3] - expected[:, :3], axis=-1).max()),
            'rotation_max_error_rad': float(angles.max())}


def max_error(actual, expected):
    import numpy as np
    actual, expected = np.asarray(actual), np.asarray(expected)
    if actual.shape != expected.shape or not np.isfinite([actual, expected]).all():
        raise ValueError('State comparison needs matching finite arrays')
    return float(np.abs(actual - expected).max())


def fresh_render(sim, baseline_physics):
    """A new render generation is required even when physics time is frozen."""
    before = int(sim.render_generation)
    sim.render()
    if int(sim.get_physics_step_count()) != baseline_physics:
        raise RuntimeError('Physics advanced during offline rendering')
    if int(sim.render_generation) <= before:
        raise RuntimeError('Render generation did not advance')


def apply_camera_configuration(cfg, recorded):
    """Restore authored optics and mounting before camera construction."""
    from copy import deepcopy
    for key in ('prim_path', 'width', 'height', 'data_types', 'update_period'):
        if key in recorded:
            setattr(cfg, key, deepcopy(recorded[key]))
    offset = recorded['offset']
    cfg.offset.pos = tuple(offset['position'])
    cfg.offset.rot = tuple(offset['quaternion_xyzw'])
    cfg.offset.convention = offset['convention']
    for key, value in recorded['spawn'].items():
        if not hasattr(cfg.spawn, key):
            raise ValueError(f'Unsupported recorded camera spawn attribute {key}')
        if isinstance(getattr(cfg.spawn, key), tuple) and value is not None:
            value = tuple(value)
        setattr(cfg.spawn, key, deepcopy(value))
    cfg.update_latest_camera_pose = True
    return cfg


def refresh_native_render_products(scene, names=CAMERAS, *, events=None):
    """Replace only native RPs, retaining Camera/FrameView and output buffers.

    The preceding writer must have drained. This resets product-local history,
    not every RTX global cache; the original warm-up remains authoritative.
    """
    events = [] if events is None else events
    for name in names:
        camera = scene[name]
        old = camera._render_data
        if old is None or old.spec is None:
            raise ValueError(f'{name}: original native render product is unavailable')
        spec, outputs = old.spec, camera._data.output
        event = {'camera': name, 'old_render_product_path': str(old.render_product.path),
                 'new_render_product_path': None, 'passed': False}
        events.append(event)
        started = time.perf_counter()
        camera._render_data = None
        created = None
        try:
            # Native cleanup clears old.spec: retain the actual spec first.
            camera._renderer.cleanup(old)
            created = camera._renderer.create_render_data(spec)
            camera._renderer.set_outputs(created, outputs)
            camera._render_data = created
            event.update(new_render_product_path=str(created.render_product.path), passed=True)
        except BaseException as error:
            event['error'] = repr(error)
            if created is not None:
                try:
                    camera._renderer.cleanup(created)
                except Exception as cleanup_error:
                    event['cleanup_error'] = repr(cleanup_error)
            raise
        finally:
            event['elapsed_seconds'] = time.perf_counter() - started
    return events


def prepare_episode_visuals(args, scene, stage, spec, report, *, scene_cache=None,
                            apply_appearance_fn=None):
    """Create optional camera products before authoring this episode's appearance.

    Match a fresh scene's order: camera initialization precedes appearance edits.
    An old product must not be retired after these edits but before Kit has
    processed them. The original warm-up still runs after state restoration.
    """
    if apply_appearance_fn is None:
        from kitchen_safety.randomization import apply_appearance
        apply_appearance_fn = apply_appearance
    grouped_owner = grouped_before = None
    if getattr(args, 'grouped_camera', False):
        from kitchen_safety.grouped_camera import install_grouped_readout
        grouped_owner = scene_cache.group_owner if scene_cache is not None else None
        if grouped_owner is None:
            grouped_owner = install_grouped_readout(scene, CAMERAS)
            if scene_cache is not None:
                scene_cache.group_owner = grouped_owner
        grouped_before = grouped_owner.report()
        report['grouped_camera'] = {
            'before': grouped_before,
            'statistics_scope': 'Owner lifetime counters; per-episode differences are reported separately'}
    if getattr(args, 'refresh_render_products', False):
        refresh = {'performed': False, 'events': [],
                   'scope': 'Product-local history only; original warm-up unchanged',
                   'ordering': 'Refresh before applying current episode appearance'}
        report['render_product_refresh'] = refresh
        if report.get('scene_reuse', {}).get('scene_reused'):
            tick = time.perf_counter()
            if grouped_owner is not None:
                refresh['events'].append(grouped_owner.refresh_render_product())
            else:
                refresh_native_render_products(scene, events=refresh['events'])
            refresh.update(performed=True, elapsed_seconds=time.perf_counter() - tick)
        else:
            refresh['reason'] = 'New scene already has fresh render products'
    report['appearance'] = (apply_appearance_fn(stage, spec) if scene_cache is None else
                            scene_cache.apply_appearance(stage, spec))
    return grouped_owner, grouped_before


def render_episode(args, *, app_context=None, scene_cache=None):
    """Render one source, optionally borrowing a batch-owned Kit and scene.

    With no optional arguments the standalone lifecycle remains unchanged.
    The cache owns only scene construction and appearance reset; the same
    measured-state restoration, original warm-up and frame checks run here.
    """
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    started = time.perf_counter()
    args.source_episode, args.output = args.source_episode.resolve(), args.output.resolve()
    data = load_source(args.source_episode, args.indices)
    args.output.mkdir(parents=True, exist_ok=False)
    args.enable_cameras = True
    args.kit_args = ((args.kit_args or '') + f' --/log/file={args.output / "kit.log"}').strip()
    provenance = render_provenance(args, data['result'])
    report = {**provenance, 'completed': False, 'selected_frames_completed': False, 'partial': data['partial'],
              'complete_trajectory': not data['partial'],
              'source_episode': str(args.source_episode), 'source_steps': data['source_steps'],
              'source_obs_indices': data['indices'], 'frames': [], 'actions_executed': 0,
              'state_authority': 'trajectory.h5 obs[t] measured render state',
              'appearance_source': 'episode_spec',
              'production_quota_eligible': False, 'source_success_flag_copied': False,
              'pixel_identity_claimed': False, 'background_rgb': args.background_rgb,
              'grouped_camera_enabled': bool(getattr(args, 'grouped_camera', False)),
              'refresh_render_products_requested': bool(getattr(args, 'refresh_render_products', False)),
              'timings': {'source_load_s': time.perf_counter() - started, 'restore_s': 0.,
                          'forward_s': 0., 'render_dispatch_s': 0., 'capture_write_caller_s': 0.,
                          'state_comparison_s': 0., 'report_write_s': 0.},
              'limitations': ['Renderer temporal history is rebuilt; pixel identity is not claimed.',
                              'Articulation links are recomputed from measured root/q/dq; all recorded links and velocities are compared.']}

    def save():
        stamp = time.perf_counter()
        target = args.output / 'render_report.json'
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2))
        temporary.replace(target)
        report['timings']['report_write_s'] += time.perf_counter() - stamp

    (args.output / 'source.json').write_text(json.dumps({
        **provenance,
        'source_episode': str(args.source_episode), 'trajectory': str(args.source_episode / 'trajectory.h5'),
        'source_obs_indices': data['indices'], 'partial': data['partial'],
        'alignment': 'Output RGB[k] is measured source obs[source_obs_indices[k]], before its action.',
        'grouped_camera': bool(getattr(args, 'grouped_camera', False)),
        'actions_executed': 0}, indent=2))
    save()
    with (SimulationAppContext(args) if app_context is None else nullcontext(app_context)):
        import numpy as np
        import torch
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from kitchen_safety.background_camera import BackgroundCameraWriter
        from kitchen_safety.camera_device import with_camera_output_device
        from kitchen_safety.dataset import CameraWriter, camera_poses_from_body_states, numpy
        from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
        from kitchen_safety.render_state import KitchenPhysicsState

        env = writer = grouped_owner = None
        grouped_before = None
        try:
            if scene_cache is not None:
                report['scene_reuse'] = scene_cache.prepare_source(data)
            spec, source = data['spec'], data['result']
            seed = source.get('seed') or spec.get('seeds', {}).get('geometry', 0)
            configuration = data['metadata']['source_configuration']
            environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
                variant=source.get('variant', spec['layout']['obstacle']['category']), enable_cameras=True, collection_seed=seed,
                obstacle_id=spec['obstacle_id'], episode_spec=spec))
            builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
                device=args.device, seed=seed, presets='physx', num_envs=1,
                solve_relations=False, disable_fabric=False,
                recorder_dataset_export_dir_path=str(args.output / 'arena_recorder')))
            cfg, kwargs = builder.compose_manager_cfg()
            cfg.sim.dt = float(configuration['physics_dt'])
            cfg.decimation = int(round(data['dt'] / cfg.sim.dt))
            if cfg.decimation < 1 or abs(cfg.decimation * cfg.sim.dt - data['dt']) > 1e-9:
                raise ValueError('Source physical/action intervals are inconsistent')
            cfg.sim.render_interval = cfg.decimation
            cfg.sim.log_dir = str(args.output / 'isaaclab_logs')
            cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count = int(configuration['solver_position_iterations'])
            cfg.observations.camera_obs = None
            for name in CAMERAS:
                camera_cfg = apply_camera_configuration(getattr(cfg.scene, name), data['camera_config'][name])
                setattr(cfg.scene, name, with_camera_output_device(camera_cfg, args.camera_device))
            if scene_cache is None:
                env = builder.make_registered(cfg, kwargs)
                env.reset()
            else:
                env = scene_cache.get_or_create(builder, cfg, kwargs)
            e, scene = env.unwrapped, env.unwrapped.scene
            robot, sim = scene['robot'], e.sim
            if not sim.is_playing():
                raise RuntimeError('Initialized timeline must be playing for official FK refresh')
            joint_order = [data['joint_names'].index(name) for name in robot.joint_names]
            body_order = [data['body_names'].index(name) for name in robot.body_names]
            if len(joint_order) != len(data['joint_names']) or len(body_order) != len(data['body_names']):
                raise ValueError('Source and rebuilt robot joint/body sets differ')
            root_index = list(robot.body_names).index(data['metadata']['robot']['root_body_name'])
            origin = numpy(scene.env_origins)[0]
            if scene_cache is not None and scene_cache.kitchen is not None:
                kitchen = scene_cache.kitchen
            else:
                kitchen = KitchenPhysicsState(sim.stage, robot._physics_sim_view, scene.env_origins, e.device)
                if scene_cache is not None:
                    scene_cache.kitchen = kitchen
            if kitchen.metadata != data['metadata']['kitchen']:
                raise ValueError('Rebuilt kitchen ordering/configuration differs from recorded metadata')
            report.update(kitchen_metadata_matches=True, physics_device=str(e.device),
                          replay_origin_world_m=origin.tolist(), camera_devices={name: scene[name].device for name in CAMERAS},
                          physics_step_count_after_reset=int(sim.get_physics_step_count()))
            baseline_physics = int(sim.get_physics_step_count())
            report['physics_steps'] = {'before': baseline_physics, 'after': baseline_physics}
            grouped_owner, grouped_before = prepare_episode_visuals(
                args, scene, sim.stage, spec, report, scene_cache=scene_cache)
            report['appearance_authority'] = 'Recorded episode_spec appearance parameters applied with original single-environment global lights'
            report['source_rendered_appearance_available'] = data['appearance'] is not None
            report['appearance_report_equals_source'] = (report['appearance'] == data['appearance']
                                                         if data['appearance'] is not None else None)
            (args.output / 'appearance.json').write_text(json.dumps(report['appearance'], indent=2))
            writer = (BackgroundCameraWriter(args.output, scene, CAMERAS, data['dt']) if args.background_rgb else
                      CameraWriter(args.output, scene, CAMERAS, data['dt'], previews=False))
            report['timings']['initialization_s'] = time.perf_counter() - started - report['timings']['source_load_s']
            writer.file.attrs['source_episode'] = str(args.source_episode)
            writer.file.attrs['source_obs_indices'] = json.dumps(data['indices'])
            writer.file.attrs['partial'] = data['partial']

            def tensor(value):
                return torch.as_tensor(value, device=e.device, dtype=torch.float32).unsqueeze(0)

            observer_binding = _read_json(args.source_episode / 'safety_observer_binding.json') if (args.source_episode / 'safety_observer_binding.json').exists() else None
            for ordinal, frame in enumerate(data['frames']):
                tick = time.perf_counter()
                state = frame['state']
                expected_bodies = translated_state(state['robot_body_pose_w'][body_order], data['coordinate_frame'], data['source_origin'], origin)
                expected_velocities = state['robot_body_velocity'][body_order]
                robot.write_root_pose_to_sim_index(root_pose=tensor(expected_bodies[root_index]))
                robot.write_root_velocity_to_sim_index(root_velocity=tensor(expected_velocities[root_index]))
                robot.write_joint_state_to_sim_index(position=tensor(state['q'][joint_order]), velocity=tensor(state['dq'][joint_order]))
                expected_roots = {}
                for name, key in [('mustard_bottle', 'mustard'), ('bowl', 'bowl')]:
                    expected_roots[name] = translated_state(state[key], data['coordinate_frame'], data['source_origin'], origin)
                    scene[name].write_root_state_to_sim(tensor(expected_roots[name]))
                if data['dynamic_hazard']:
                    name = data['metadata']['hazard']['name']
                    expected_roots[name] = translated_state(state['hazard_state'], data['coordinate_frame'], data['source_origin'], origin)
                    scene[name].write_root_state_to_sim(tensor(expected_roots[name]))
                kitchen.restore({key: state[key][None] for key in KITCHEN_SHAPES})
                report['timings']['restore_s'] += time.perf_counter() - tick
                tick = time.perf_counter()
                sim.forward()
                scene.update(0.)
                sim.render_context.reset_scene_state_cadence()
                report['timings']['forward_s'] += time.perf_counter() - tick
                if ordinal == 0:
                    report['appearance_warmup'] = writer.warm_up(sim)
                    report['timings']['appearance_warmup_s'] = report['appearance_warmup']['elapsed_seconds']
                    save()
                    if not report['appearance_warmup']['passed']:
                        raise RuntimeError('Original single-episode appearance warm-up failed')
                tick = time.perf_counter()
                fresh_render(sim, baseline_physics)
                report['timings']['render_dispatch_s'] += time.perf_counter() - tick
                tick = time.perf_counter()
                capture_time_ns = time.time_ns()
                writer.write(frame['phase'])
                receive_time_ns = time.time_ns()
                if observer_binding:
                    from kitchen_safety.risk_camera import append_camera_provenance
                    append_camera_provenance(args.output, observer_binding, frame['source_obs_index'],
                        capture_time_ns, receive_time_ns, mode='offline_measured_state_render')
                report['timings']['capture_write_caller_s'] += time.perf_counter() - tick
                if int(sim.get_physics_step_count()) != baseline_physics:
                    raise RuntimeError('Physics advanced while recording restored states')
                tick = time.perf_counter()
                actual_bodies = np.concatenate((numpy(robot.data.body_pos_w)[0], numpy(robot.data.body_quat_w)[0]), axis=-1)
                actual_velocities = np.concatenate((numpy(robot.data.body_lin_vel_w)[0], numpy(robot.data.body_ang_vel_w)[0]), axis=-1)
                row = {key: frame[key] for key in ('source_obs_index', 'phase', 'source_time_seconds')}
                row.update(output_frame_index=ordinal, physics_step_count=int(sim.get_physics_step_count()),
                           render_generation=int(sim.render_generation),
                           robot={**pose_errors(actual_bodies, expected_bodies),
                                  'body_velocity_max_abs_error': max_error(actual_velocities, expected_velocities),
                                  'joint_position_max_abs_error': max_error(numpy(robot.data.joint_pos)[0], state['q'][joint_order]),
                                  'joint_velocity_max_abs_error': max_error(numpy(robot.data.joint_vel)[0], state['dq'][joint_order])},
                           roots={}, kitchen={}, cameras={})
                for name, expected in expected_roots.items():
                    actual = numpy(scene[name].data.root_state_w)[0]
                    row['roots'][name] = {**pose_errors(actual[:7], expected[:7]),
                                          'velocity_max_abs_error': max_error(actual[7:], expected[7:])}
                if scene_cache is not None and not data['dynamic_hazard']:
                    row['static_obstacle'] = scene_cache.static_pose_check(e, data)
                captured_kitchen = kitchen.capture()
                actual_kitchen = {key: np.asarray(value)[0] for key, value in captured_kitchen.items()}
                row['kitchen'] = {
                    'root': {**pose_errors(actual_kitchen['kitchen_root_state'][:, :7], state['kitchen_root_state'][:, :7]),
                             'velocity_max_abs_error': max_error(actual_kitchen['kitchen_root_state'][:, 7:], state['kitchen_root_state'][:, 7:])},
                    'links': {**pose_errors(actual_kitchen['kitchen_link_pose'], state['kitchen_link_pose']),
                              'velocity_max_abs_error': max_error(actual_kitchen['kitchen_link_velocity'], state['kitchen_link_velocity'])},
                    'joint_position_max_abs_error': max_error(actual_kitchen['kitchen_joint_pos'], state['kitchen_joint_pos']),
                    'joint_velocity_max_abs_error': max_error(actual_kitchen['kitchen_joint_vel'], state['kitchen_joint_vel'])}
                expected_cameras = camera_poses_from_body_states(expected_bodies[None], list(robot.body_names), data['camera_config'])
                for name in CAMERAS:
                    camera = scene[name]
                    actual = np.r_[numpy(camera.data.pos_w)[0], numpy(camera.data.quat_w_ros)[0]]
                    expected = np.r_[expected_cameras[name]['position_world'][0], expected_cameras[name]['quaternion_world_ros_xyzw'][0]]
                    item = {'versus_recorded_parent_body_pose': pose_errors(actual, expected),
                            'width': camera.cfg.width, 'height': camera.cfg.height,
                            'intrinsics': numpy(camera.data.intrinsic_matrices)[0].tolist()}
                    if name in frame.get('recorded_cameras', {}):
                        old = frame['recorded_cameras'][name]
                        item['versus_recorded_camera_pose'] = pose_errors(actual, translated_state(old['pose'], old['coordinate_frame'], old['origin'], origin))
                    if data['source_camera_metadata']:
                        item['intrinsics_max_abs_error_from_source'] = max_error(item['intrinsics'], data['source_camera_metadata'][name]['intrinsics'])
                    row['cameras'][name] = item
                report['timings']['state_comparison_s'] += time.perf_counter() - tick
                report['frames'].append(row)
                if ordinal % 30 == 0 or ordinal == len(data['frames']) - 1:
                    print(f'RENDERED {ordinal + 1}/{len(data["frames"])} source_obs={frame["source_obs_index"]}', flush=True)
                    save()
            report.update(selected_frames_completed=True, completed=not data['partial'],
                          rendered_frames=writer.frames, physics_step_count_after_render=int(sim.get_physics_step_count()),
                          physics_advanced=False)
            report['physics_steps']['after'] = int(sim.get_physics_step_count())
        except BaseException as error:
            report.update(completed=False, selected_frames_completed=False, error=repr(error), traceback=traceback.format_exc())
            raise
        finally:
            cleanup_errors = []
            try:
                if writer is not None:
                    writer.close()
                    report['camera_timings'] = writer.timings
                    report['rendered_frames'] = writer.frames
            except BaseException as error:
                cleanup_errors.append(f'camera close: {error!r}')
            finally:
                if grouped_owner is not None:
                    try:
                        if scene_cache is None:
                            grouped_owner.close()
                    except BaseException as error:
                        cleanup_errors.append(f'grouped camera close: {error!r}')
                    finally:
                        after = grouped_owner.report()
                        report.setdefault('grouped_camera', {})['after'] = after
                        if grouped_before is not None:
                            report['grouped_camera']['episode_read_count'] = after['grouped_read_count'] - grouped_before['grouped_read_count']
                            report['grouped_camera']['episode_requests'] = after['requests'] - grouped_before['requests']
                try:
                    if env is not None:
                        if 'physics_steps' in report:
                            report['physics_steps']['after'] = int(env.unwrapped.sim.get_physics_step_count())
                            report['physics_advanced'] = report['physics_steps']['after'] != report['physics_steps']['before']
                        if scene_cache is None:
                            env.close()
                except BaseException as error:
                    cleanup_errors.append(f'environment close: {error!r}')
                finally:
                    if cleanup_errors:
                        report.update(completed=False, selected_frames_completed=False, cleanup_errors=cleanup_errors)
                    if scene_cache is None:
                        report['timings']['total_until_environment_close_s'] = time.perf_counter() - started
                        report['timing_boundary'] = 'Includes source load, Kit initialization, warm-up, restoration, RGB and env.close; excludes SimulationApp shutdown'
                    else:
                        report['timings']['total_until_episode_close_s'] = time.perf_counter() - started
                        report['timing_boundary'] = 'Borrowed Kit: source load, scene preparation, original warm-up, restoration and RGB drain; shared Kit startup/final scene close are in batch report'
                    # SimulationAppContext.__exit__ may os._exit: persist before it.
                    save()
                if cleanup_errors:
                    raise RuntimeError('; '.join(cleanup_errors))
    return report


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--source-episode', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--collection-purpose', choices=['diagnostic', 'production'], default='diagnostic')
    parser.add_argument('--collection-source', help='Originating collection run; does not grant production acceptance')
    parser.add_argument('--indices', type=int, nargs='+', help='Increasing obs indices for a partial diagnostic')
    parser.add_argument('--camera-device', default=None)
    parser.add_argument('--background-rgb', action='store_true')
    parser.add_argument('--grouped-camera', action='store_true',
                        help='Opt-in diagnostic: group the original three camera readouts in one native render product')
    parser.add_argument('--refresh-render-products', action='store_true',
                        help='Refresh native render products between reused episodes; a standalone new scene already has fresh products')
    args = parser.parse_args()
    if args.num_envs != 1:
        parser.error('Offline rendering uses one environment')
    render_episode(args)


if __name__ == '__main__':
    main()
