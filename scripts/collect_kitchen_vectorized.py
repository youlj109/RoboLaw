"""Plan independent kitchen episodes and execute them in one batched Arena env.

This entry point is separate from the preserved single-environment collector.
All simulator updates happen here; episode controllers only plan and provide
actions. Geometry audits use recorded physical substeps after execution.
"""
import argparse
import json
import math
from pathlib import Path
import time
import traceback


def collection_inputs(specs, overrides_path=None, jobs_path=None):
    """Read ordinary job mapping and explicit collector overrides before Kit."""
    from kitchen_safety.collection_presets import resolve_collector_options
    overrides = {} if overrides_path is None else json.loads(Path(overrides_path).read_text())
    # Existing preset resolver rejects unknown/non-object overrides and retains
    # explicit False/zero/None rather than treating them as missing values.
    resolve_collector_options(cli_overrides=overrides, presets={})
    jobs = ([{'env_index': i} for i in range(len(specs))] if jobs_path is None else
            json.loads(Path(jobs_path).read_text()))
    if not isinstance(jobs, list) or len(jobs) != len(specs):
        raise ValueError('job-mapping must contain one object per episode spec')
    for i, job in enumerate(jobs):
        if not isinstance(job, dict) or type(job.get('env_index')) is not int or job['env_index'] != i:
            raise ValueError('job-mapping env_index must match its episode-spec list position')
    return overrides, jobs


def resolved_collector_options(obstacle_id, overrides):
    from kitchen_safety.collection_presets import resolve_collector_options
    return resolve_collector_options(obstacle_id, base_defaults={
        'grasp_tilt_deg': None, 'grasp_yaw_deg': None, 'grasp_offset_z_m': 0.,
        'grasp_offset_y_m': 0., 'transport_bottle_height_m': None,
        'supported_release': False, 'transport_tilt_cap_deg': None,
        'placement_strategy': 'pose', 'placement_tcp_height_m': None}, cli_overrides=overrides)


def render_camera_config(camera_rig):
    """Serialize the authored camera rig without constructing any sensor."""
    optical_fields = (
        'projection_type', 'focal_length', 'focus_distance', 'f_stop',
        'horizontal_aperture', 'vertical_aperture', 'horizontal_aperture_offset',
        'vertical_aperture_offset', 'clipping_range', 'lock_camera',
    )
    cameras = {}
    for name in camera_rig.camera_names():
        camera = getattr(camera_rig, name)
        cameras[name] = {
            'metadata_source': 'authored_camera_configuration_without_sensor_readback',
            'prim_path': camera.prim_path, 'width': camera.width, 'height': camera.height,
            'update_period': camera.update_period, 'data_types': list(camera.data_types),
            'offset': {'position': list(camera.offset.pos),
                       'quaternion_xyzw': list(camera.offset.rot),
                       'convention': camera.offset.convention},
            'spawn': {field: getattr(camera.spawn, field) for field in optical_fields
                      if hasattr(camera.spawn, field)},
        }
    return cameras


def capture_render_observation(kitchen_state, robot_data, hazard_data, origins, to_numpy):
    """Capture only instantaneous measured state; never render or step physics.

    KitchenPhysicsState already returns environment-local positions. Robot COM
    velocities retain world axes; only a dynamic hazard's position is shifted.
    """
    import numpy as np
    state = dict(kitchen_state.capture())
    state['robot_body_velocity'] = np.concatenate((to_numpy(robot_data.body_lin_vel_w),
                                                  to_numpy(robot_data.body_ang_vel_w)), axis=-1)
    if hazard_data is not None:
        state['hazard_state'] = to_numpy(hazard_data.root_state_w)
        state['hazard_state'][:, :3] -= origins
    return state


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--num-envs', dest='num_envs', type=int, default=argparse.SUPPRESS)
    parser.add_argument('--episode-specs', type=Path, required=True,
                        help='JSON list of independent episode specifications, one per environment')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--collector-overrides', type=Path,
                        help='JSON object of explicit grasp/placement overrides, applied after obstacle presets')
    parser.add_argument('--job-mapping', type=Path,
                        help='JSON list mapping each env_index to its originating batch attempt; output remains env_NNN')
    parser.add_argument('--collection-purpose', choices=['diagnostic', 'production'], default='diagnostic')
    parser.add_argument('--collection-source', help='Originating collection batch/run; metadata only, never acceptance')
    parser.add_argument('--observer-config', type=Path, help='Read-only SafeBench observer configuration')
    parser.add_argument('--run-id', help='Observer run identity; defaults to collection source/output')
    parser.add_argument('--record-rgb', action='store_true')
    parser.add_argument('--record-render-state', action='store_true',
                        help='Save action-rate full kitchen/robot state and original camera config for later rendering; does not enable cameras')
    parser.add_argument('--camera-device', default=None,
                        help='Optional CUDA camera buffers, including with --device cpu')
    parser.add_argument('--physics-threads', type=int, default=None,
                        help='PhysX worker count; 0 executes physics on the main thread')
    parser.add_argument('--torch-threads', type=int, default=1,
                        help='CPU PyTorch worker count; preserves the previous fixed default of one')
    parser.add_argument('--defer-clearance-diagnostics', action='store_true',
                        help='Replay report-only action clearance queries after execution; retain all samples')
    parser.add_argument('--background-rgb', action='store_true',
                        help='Encode and save captured CPU images on a bounded background writer')
    parser.add_argument('--reuse-phase-planner', action='store_true',
                        help='Reuse one cuRobo context between grasp and transport with refreshed collision state')
    parser.add_argument('--curobo-cuda-graph', action='store_true',
                        help='Warm up cuRobo CUDA graphs once per planner context')
    parser.add_argument('--fixed-appearance-benchmark', action='store_true',
                        help='Throughput diagnostic: keep authored USD appearance, render 128 preparation frames, and never count production quota')
    parser.add_argument('--warmup-mode', choices=['single_frame', 'temporal_mean', 'fixed_preparation'], default='single_frame',
                        help='fixed_preparation applies randomized appearance, renders 128 frames and retains convergence diagnostics; action RGB is never averaged')
    parser.add_argument('--warmup-window-frames', type=int, default=16,
                        help='Non-overlapping temporal_mean window; single_frame retains its original 16-frame cadence')
    parser.add_argument('--warmup-max-frames', type=int, default=1024,
                        help='Pure-render convergence budget; fixed_preparation always renders 128 frames without physics advancement')
    parser.add_argument('--render-spp', type=int, choices=[1, 2, 4, 8],
                        help='Optional RTX direct-light samples per pixel; camera geometry is unchanged')
    parser.add_argument('--render-aa-op', type=int, choices=range(5), default=None,
                        help='Optional post-initialization RTX antialiasing override; 3=DLSS, 4=DLAA')
    parser.add_argument('--lighting', choices=['shared', 'unchanged', 'linked_experimental'], default='shared',
                        help='shared requires equal light specs; unchanged/linked_experimental are explicit visual diagnostics')
    parser.add_argument('--max-steps', type=int, default=2400)
    parser.add_argument('--episode-length-s', type=float, default=120.0,
                        help='Arena task timeout in simulation seconds, including initialization; default 120')
    parser.add_argument('--validation-workers', type=int, default=1,
                        help='Independent CPU processes for full-world post-execution geometry checks')
    parser.add_argument('--replicate-physics', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if not math.isfinite(args.episode_length_s) or args.episode_length_s <= 0:
        parser.error('--episode-length-s must be finite and positive')
    if args.warmup_mode != 'temporal_mean' and args.warmup_window_frames != 16:
        parser.error('warmup-window-frames is configurable only with --warmup-mode temporal_mean')
    if args.warmup_window_frames < 1 or args.warmup_max_frames < max(128, 4 * args.warmup_window_frames):
        parser.error('Warm-up requires a positive window and at least max(128, 4 * window) render frames')
    observer_settings = None
    if args.observer_config:
        from kitchen_safety.risk_profile_loader import load_config
        observer_settings = load_config(args.observer_config)
    specs = json.loads(args.episode_specs.read_text())
    if not isinstance(specs, list) or len(specs) != args.num_envs:
        parser.error('episode-specs must contain exactly num-envs entries')
    if len({spec['obstacle_id'] for spec in specs}) != 1:
        parser.error('A vectorized batch must use the same obstacle asset topology')
    collector_overrides, collection_jobs = collection_inputs(specs, args.collector_overrides, args.job_mapping)
    if args.device != 'cpu' and not str(args.device).startswith('cuda'):
        parser.error('Simulation device must be cpu or cuda')
    if args.physics_threads is not None and args.physics_threads < 0:
        parser.error('physics-threads must be nonnegative')
    if args.torch_threads < 1:
        parser.error('torch-threads must be positive')
    if args.camera_device is not None and not str(args.camera_device).startswith('cuda'):
        parser.error('camera-device must be a CUDA device')
    if args.validation_workers < 1:
        parser.error('validation-workers must be positive')
    args.enable_cameras = args.record_rgb
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    args.kit_args = ((args.kit_args or '') + f' --/log/file={out / "kit.log"}').strip()
    started = time.perf_counter()
    report = {'status': 'initializing', 'num_envs': args.num_envs,
              'record_render_state': args.record_render_state,
              'production_quota_eligible': False, 'timings_seconds': {},
              'collection_purpose': args.collection_purpose, 'collection_source': args.collection_source,
              'acceptance_status': 'pending_batch_verification' if args.collection_purpose == 'production' else 'diagnostic',
              'collector_overrides': collector_overrides, 'jobs': collection_jobs,
              'episode_length_s': args.episode_length_s, 'max_steps': args.max_steps,
              'initialization_action_steps': 8,
              'appearance_mode': 'authored_fixed_benchmark' if args.fixed_appearance_benchmark else 'randomized_episode',
              'semantics': 'Independent cuRobo replanning, one Kit process, batched physics; final production acceptance belongs to verify_attempt'}
    (out / 'episode_specs.json').write_text(json.dumps(specs, indent=2))
    with SimulationAppContext(args):
        import h5py
        import numpy as np
        import torch
        from scipy.spatial.transform import Rotation
        from isaaclab.managers import SceneEntityCfg
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.tasks.predicates.spatial import object_on_destination
        from kitchen_safety.collection_presets import resolve_collector_options
        from kitchen_safety.collision_planner import CollisionWorld
        from kitchen_safety.deferred_safety import DeferredSafetyBuffer
        from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
        from kitchen_safety.grasp_contacts import BottleGraspContacts
        from kitchen_safety.planning import DroidKinematics
        from kitchen_safety.vectorized_episode import VectorizedEpisode
        from kitchen_safety.vectorized_scene import configure_vectorized_geometry

        torch.set_num_threads(args.torch_threads)
        if args.physics_threads is not None:
            import carb
            import omni.physx
            from omni.physx.bindings._physx import SETTING_NUM_THREADS
            carb.settings.get_settings().set_int(SETTING_NUM_THREADS, args.physics_threads)
            omni.physx.get_physx_interface().set_thread_count(args.physics_threads)
        report['timings_seconds']['app_startup_s'] = time.perf_counter() - started
        env = cameras = risk_runtime = None
        controllers, buffers, outputs = [], [], {}
        timings = report['timings_seconds']
        timings.update(planning_dispatch_s=0., physics_execution_s=0., camera_s=0.,
                       state_capture_s=0., deferred_validation_s=0., data_write_s=0.)
        if args.record_render_state:
            timings['render_state_capture_s'] = 0.
            report['render_state_timing_scope'] = (
                'Additional action-rate kitchen state, robot COM velocity and dynamic hazard state capture; '
                'included in state_capture_s, not an additional term in total wall time.')
        try:
            spec0 = specs[0]
            variant = spec0['layout']['obstacle']['category']
            environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
                variant=variant, obstacle_id=spec0['obstacle_id'], episode_spec=spec0,
                enable_cameras=args.enable_cameras, collection_seed=args.seed or 0,
                episode_length_s=args.episode_length_s))
            builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
                device=args.device, seed=args.seed or 0, presets='physx',
                num_envs=args.num_envs, env_spacing=args.env_spacing,
                solve_relations=False, disable_fabric=False,
                recorder_dataset_export_dir_path=str(out / 'arena_recorder')))
            cfg, kwargs = builder.compose_manager_cfg()
            cfg.sim.log_dir = str(out / 'isaaclab_logs')
            cfg.sim.render_interval = cfg.decimation
            cfg.scene.replicate_physics = args.replicate_physics
            cfg.scene.robot_contacts = ContactSensorCfg(
                prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
            if args.record_rgb:
                cfg.observations.camera_obs = None
                if args.camera_device is not None:
                    from kitchen_safety.camera_device import with_camera_output_device
                    for name in environment.embodiment.camera_config.camera_names():
                        setattr(cfg.scene, name, with_camera_output_device(
                            getattr(cfg.scene, name), args.camera_device))
                if args.render_spp is not None:
                    from isaaclab_physx.renderers.isaac_rtx_renderer_cfg import IsaacRtxRendererCfg
                    for name in environment.embodiment.camera_config.camera_names():
                        camera = getattr(cfg.scene, name)
                        if getattr(camera.renderer_cfg, 'renderer_type', None) == 'default':
                            camera.renderer_cfg = IsaacRtxRendererCfg()
                        camera.renderer_cfg.global_settings.samples_per_pixel = args.render_spp
            report['geometry_configuration'] = configure_vectorized_geometry(cfg, specs)
            env = builder.make_registered(cfg, kwargs)
            if observer_settings:
                from kitchen_safety.risk_binding import enable_contact_reports
                contact_report_status = enable_contact_reports(env.unwrapped.sim.stage)
            env.reset()
            e, scene = env.unwrapped, env.unwrapped.scene
            if not args.record_rgb:
                e.render_enabled = False
            robot = scene['robot']
            if args.record_rgb:
                import carb
                report['render_settings'] = {'direct_light_samples_per_pixel':
                    carb.settings.get_settings().get('/rtx/directLighting/sampledLighting/samplesPerPixel')}
            ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)
            body_index = robot.body_names.index('base_link')
            contact_names = scene['robot_contacts'].body_names
            nonfinger_ids = [i for i, name in enumerate(contact_names)
                            if name.startswith('panda_link') or name == 'base_link']
            dynamic = not spec0['layout']['obstacle']['static']

            def tensor(value):
                return value.torch if hasattr(value, 'torch') else value

            def cpu(value):
                return tensor(value).detach().cpu().numpy().copy()

            origins = cpu(scene.env_origins)
            kitchen_state = None
            if args.record_render_state:
                from kitchen_safety.render_state import KitchenPhysicsState
                kitchen_state = KitchenPhysicsState(
                    e.sim.stage, scene['robot_contacts']._physics_sim_view, origins, e.device)
                saved_camera_config = render_camera_config(environment.embodiment.camera_config)
            task = environment.task
            success_seen = np.zeros(args.num_envs, dtype=bool)
            termination_seen = {name: np.zeros(args.num_envs, dtype=bool)
                                for name in e.termination_manager.active_terms}

            def support():
                return cpu(object_on_destination(
                    e, object_cfg=SceneEntityCfg(task.pick_up_object.name),
                    destination_cfg=SceneEntityCfg(task.destination_location.name),
                    contact_sensor_cfg=task.contact_sensor_cfg,
                    force_threshold=task.force_threshold, velocity_threshold=task.velocity_threshold,
                    support_cone_half_angle_rad=task.support_cone_half_angle_rad)).astype(bool)

            def capture_reset(env_ids):
                # Record termination before the manager can reset an individual
                # environment. Release/retreat must still run after success.
                for name in termination_seen:
                    termination_seen[name] |= cpu(e.termination_manager.get_term(name)).astype(bool)

            e._reset_idx = capture_reset

            def snapshot_all():
                tick = time.perf_counter()
                values = dict(q=tensor(robot.data.joint_pos), dq=tensor(robot.data.joint_vel),
                              body_pos=tensor(robot.data.body_pos_w), body_quat=tensor(robot.data.body_quat_w),
                              mustard=tensor(scene['mustard_bottle'].data.root_state_w),
                              bowl=tensor(scene['bowl'].data.root_state_w),
                              robot_contact_forces=tensor(scene['robot_contacts'].data.net_forces_w))
                if dynamic:
                    values['hazard_pose'] = tensor(scene['glass'].data.root_state_w)[:, :7]
                shapes = {name: tuple(value.shape[1:]) for name, value in values.items()}
                packed = torch.cat([value.reshape(args.num_envs, -1) for value in values.values()], dim=1)
                packed = cpu(packed)
                arrays, offset = {}, 0
                for name, shape in shapes.items():
                    length = int(np.prod(shape))
                    arrays[name] = packed[:, offset:offset+length].reshape(args.num_envs, *shape)
                    offset += length
                supported = support()
                render_arrays = {}
                if kitchen_state is not None:
                    render_tick = time.perf_counter()
                    try:
                        render_arrays = capture_render_observation(
                            kitchen_state, robot.data, scene['glass'].data if dynamic else None,
                            origins, cpu)
                    finally:
                        timings['render_state_capture_s'] += time.perf_counter() - render_tick
                # Arena's success term includes settling/lift/continuous
                # support requirements. A transient support sample is not
                # sufficient to latch a successful task.
                success_seen[:] |= termination_seen.get('success', np.zeros(args.num_envs, dtype=bool))
                states = []
                for i in range(args.num_envs):
                    row = {name: value[i].copy() for name, value in arrays.items()}
                    row['body_pos'] -= origins[i]
                    for name in ('mustard', 'bowl', 'hazard_pose'):
                        if name in row:
                            row[name][:3] -= origins[i]
                    row['tcp_quat_xyzw'] = row['body_quat'][body_index].copy()
                    row['tcp'] = row['body_pos'][body_index] + Rotation.from_quat(row['tcp_quat_xyzw']).apply([.131, 0, 0])
                    row['robot_body_pose_w'] = np.concatenate((row.pop('body_pos'), row.pop('body_quat')), axis=-1)
                    row.update({name: values[i].copy() for name, values in render_arrays.items()})
                    row['arena_success'] = bool(success_seen[i])
                    row['final_on_destination'] = bool(supported[i])
                    states.append(row)
                timings['state_capture_s'] += time.perf_counter() - tick
                return states

            hold = torch.cat((tensor(robot.data.joint_pos)[:, ids],
                              torch.zeros((args.num_envs, 1), device=e.device)), dim=1)
            for _ in range(8):
                env.step(hold)
            if args.record_rgb:
                settings = carb.settings.get_settings()
                report['render_settings']['antialiasing_before_override'] = settings.get('/rtx/post/aa/op')
                if args.render_aa_op is not None:
                    settings.set_int('/rtx/post/aa/op', args.render_aa_op)
                report['render_settings']['antialiasing_requested'] = args.render_aa_op
                report['render_settings']['antialiasing_op'] = settings.get('/rtx/post/aa/op')
                report['render_settings']['render_mode'] = settings.get('/rtx/rendermode')
            states = snapshot_all()
            options = resolved_collector_options(spec0['obstacle_id'], collector_overrides)
            report['resolved_collector_options'] = dict(options)
            options.update(reuse_planner=True, max_steps=args.max_steps,
                           defer_clearance_diagnostics=args.defer_clearance_diagnostics,
                           reuse_phase_planner=args.reuse_phase_planner,
                           use_cuda_graph=args.curobo_cuda_graph)
            for i, spec in enumerate(specs):
                output = out / f'env_{i:03d}'
                output.mkdir()
                outputs[i] = output
                (output / 'episode_spec.json').write_text(json.dumps(spec, indent=2))
                provenance = {'collection_purpose': args.collection_purpose,
                    'collection_source': args.collection_source, 'job': collection_jobs[i],
                    'native_output': str(output), 'collector_overrides': collector_overrides,
                    'resolved_collector_options': report['resolved_collector_options'],
                    'episode_length_s': cfg.episode_length_s, 'max_steps': args.max_steps,
                    'initialization_action_steps': 8,
                    'acceptance_status': report['acceptance_status'], 'production_quota_eligible': False}
                (output / 'collection_provenance.json').write_text(json.dumps(provenance, indent=2))
                if kitchen_state is not None:
                    metadata = {
                        'version': 1, 'coordinate_frame': 'environment_local', 'quaternion_order': 'xyzw',
                        'environment_origin_world_m': origins[i].tolist(),
                        'recording': 'Measured obs[t] before action[t] and next_obs[t] after action[t]; action-rate, not physics-rate',
                        'kitchen': kitchen_state.metadata,
                        'robot': {'joint_names': list(robot.joint_names), 'body_names': list(robot.body_names),
                                  'root_body_name': 'panda_link0',
                                  'body_velocity_order': ['linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'],
                                  'body_velocity_semantics': 'Measured body_lin_vel_w and body_ang_vel_w: COM linear/angular velocity, world axes, no origin subtraction'},
                        'hazard': {'dynamic': dynamic,
                                   'name': {'property': 'glass', 'robot': 'barrier', 'human': 'person'}[variant],
                                   'state_field': 'hazard_state' if dynamic else None,
                                   'pose_field': 'hazard_pose' if dynamic else None},
                        'source_configuration': {
                            'collection_purpose': args.collection_purpose, 'collection_source': args.collection_source,
                            'job': collection_jobs[i], 'collector_overrides': collector_overrides,
                            'resolved_collector_options': report['resolved_collector_options'],
                            'simulation_device': str(e.device), 'num_envs': args.num_envs,
                            'torch_threads': args.torch_threads,
                            'physics_dt': cfg.sim.dt, 'action_dt': e.step_dt, 'record_rgb': args.record_rgb,
                            'solver_position_iterations': cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count},
                    }
                    (output / 'render_state_metadata.json').write_text(json.dumps(metadata, indent=2))
                    (output / 'camera_config.json').write_text(json.dumps(saved_camera_config, indent=2))
                layout, settings = spec['layout'], spec['layout'].get('planning', {})
                base = np.array(layout['robot_position'], dtype=float)
                base[2] += layout['stand_height']
                kin = DroidKinematics(base, layout['robot_yaw_deg'])
                world = CollisionWorld(e.sim.stage, kin,
                    clearances=settings.get('clearances_m', {}),
                    execution_clearances=settings.get('execution_clearances_m', {}),
                    default_clearance=settings.get('default_clearance_m', .002),
                    env_root=f'/World/envs/env_{i}', env_origin=origins[i])
                for name, key in [('mustard_bottle', 'mustard'), ('bowl', 'bowl')]:
                    world.set_obstacle_pose(name, states[i][key][:7])
                if dynamic:
                    world.set_obstacle_pose('glass', states[i]['hazard_pose'])
                contacts = BottleGraspContacts(scene['robot_contacts']._physics_sim_view, cfg.sim.dt,
                    env_root=f'/World/envs/env_{i}', env_origin=origins[i])
                controllers.append(VectorizedEpisode(
                    env_id=i, layout=layout, options=options, world=world, kin=kin,
                    grasp_contacts=contacts, step_dt=e.step_dt, episode_spec=spec, output=output,
                    arm_joint_ids=ids, joint_names=robot.joint_names, nonfinger_ids=nonfinger_ids))
                buffers.append(DeferredSafetyBuffer())

            if args.fixed_appearance_benchmark:
                report['appearance'] = {
                    'appearance_mode': 'authored_fixed_benchmark', 'production_quota_eligible': False,
                    'requested_spec_appearance_applied': False,
                    'material_and_lighting_source': 'Original authored USD; apply_vectorized_appearance was not called',
                    'geometry_semantics': 'All episode-spec geometry parameters, physical properties and camera configurations remain in use',
                    'convergence_checked': False}
                for i in outputs:
                    appearance = {**report['appearance'], 'env_index': i,
                                  'requested_spec_appearance': specs[i].get('appearance'),
                                  'requested_appearance_seed': specs[i].get('seeds', {}).get('appearance')}
                    (outputs[i] / 'appearance.json').write_text(json.dumps(appearance, indent=2))

            if args.record_rgb:
                from kitchen_safety.vectorized_dataset import VectorizedCameraWriter
                if not args.fixed_appearance_benchmark:
                    from kitchen_safety.vectorized_scene import apply_vectorized_appearance
                    report['appearance'] = apply_vectorized_appearance(e.sim.stage, specs, lighting=args.lighting)
                    for i in outputs:
                        appearance = {**report['appearance']['environments'][i],
                                      'lighting_mode': args.lighting,
                                      'lights': report['appearance']['lights'],
                                      'independent_lighting_render_verified': False}
                        (outputs[i] / 'appearance.json').write_text(json.dumps(appearance, indent=2))
                camera_options = {'background_write': True} if args.background_rgb else {}
                cameras = VectorizedCameraWriter(outputs, scene, dt=e.step_dt, env_origins=origins,
                                                 **camera_options)
                if args.fixed_appearance_benchmark:
                    from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
                    preparation_started = time.perf_counter()
                    physics_before = int(e.sim.get_physics_step_count())
                    preparation = {
                        'mode': 'authored_fixed_benchmark', 'passed': None, 'convergence_checked': False,
                        'render_frames': 0, 'requested_render_frames': 128, 'physics_advanced': False,
                        'physics_step_count_before': physics_before, 'physics_step_count_after': physics_before,
                        'render_generation_before': int(e.sim.render_generation),
                        'description': 'Fixed-count pure rendering for a throughput diagnostic; convergence was not tested.'}
                    report['appearance_warmup'] = preparation
                    try:
                        for _ in range(128):
                            generation = int(e.sim.render_generation)
                            e.sim.render()
                            ensure_isaac_rtx_render_update(force=True)
                            preparation['render_frames'] += 1
                            if int(e.sim.render_generation) <= generation:
                                raise RuntimeError('Fixed-appearance preparation did not advance render generation')
                            if int(e.sim.get_physics_step_count()) != physics_before:
                                raise RuntimeError('Physics advanced during fixed-appearance render preparation')
                    finally:
                        preparation['physics_step_count_after'] = int(e.sim.get_physics_step_count())
                        preparation['physics_advanced'] = preparation['physics_step_count_after'] != physics_before
                        preparation['render_generation_after'] = int(e.sim.render_generation)
                        preparation['elapsed_seconds'] = time.perf_counter() - preparation_started
                        cameras.timings['appearance_warmup_s'] += preparation['elapsed_seconds']
                        for writer in cameras.writers.values():
                            writer.file.attrs['appearance_mode'] = 'authored_fixed_benchmark'
                            writer.file.attrs['production_quota_eligible'] = False
                            writer.file.attrs['appearance_warmup'] = json.dumps(preparation)
                else:
                    report['appearance_warmup'] = cameras.warm_up(
                        e.sim, mode=args.warmup_mode, sample_every=args.warmup_window_frames,
                        max_frames=args.warmup_max_frames)
                    if args.warmup_mode == 'fixed_preparation':
                        if not report['appearance_warmup']['preparation_passed']:
                            raise RuntimeError('Batched camera pure-render preparation failed: ' + str(
                                report['appearance_warmup'].get('preparation_error')))
                    elif not report['appearance_warmup']['passed']:
                        raise RuntimeError('Batched camera appearance did not stabilize')

            if observer_settings:
                from kitchen_safety.risk_binding import ArenaRiskRuntime
                risk_runtime = ArenaRiskRuntime(e.sim.stage, robot, scene, specs, outputs, collection_jobs,
                    observer_settings, cfg.sim.dt, e.step_dt, origins, contact_report_status,
                    args.run_id or args.collection_source or str(out), environment.embodiment.camera_config.camera_names())
            active = {}
            original_update = scene.update
            original_sim_step = e.sim.step
            original_scene_write = scene.write_data_to_sim
            def observed_scene_write(*write_args, **write_kwargs):
                if risk_runtime and active:
                    risk_runtime.before_submission()
                return original_scene_write(*write_args, **write_kwargs)
            def observed_sim_step(*step_args, **step_kwargs):
                if risk_runtime and active:
                    risk_runtime.before_physics(e.action_manager.action)
                return original_sim_step(*step_args, **step_kwargs)
            if risk_runtime:
                e.sim.step = observed_sim_step
                scene.write_data_to_sim = observed_scene_write

            def checked_update(dt):
                original_update(dt)
                if risk_runtime and active:
                    risk_runtime.after_physics()
                if not active:
                    return
                values = [tensor(robot.data.joint_pos), tensor(scene['mustard_bottle'].data.root_state_w)[:, :7]]
                if dynamic:
                    values.append(tensor(scene['glass'].data.root_state_w)[:, :7])
                rows = cpu(torch.cat(values, dim=1))
                for i in active:
                    controller = controllers[i]
                    # Match the original collector: geometry checking begins
                    # after the initial 30 recorded settling actions.
                    if not controller.safety_enabled:
                        continue
                    bottle = rows[i, 13:20].copy()
                    bottle[:3] -= origins[i]
                    hazard = rows[i, 20:27].copy() if dynamic else None
                    if hazard is not None:
                        hazard[:3] -= origins[i]
                    buffers[i].append(q=rows[i, :13], mustard=bottle,
                        phase=controller.phase, action_index=len(controller.records),
                        attachment=controller.world.attachment, hazard_pose=hazard)

            scene.update = checked_update
            timings['initialization_s'] = time.perf_counter() - started
            report['status'] = 'executing'
            action_batch = cpu(hold)
            report['torch_memory_samples'] = []
            report['torch_memory_sample_scope'] = (
                'PyTorch CUDA allocator only, in bytes; excludes allocations owned directly by '
                'RTX/PhysX/Kit. Peak counters are process high-water marks, not reset here. '
                'Read after prepare when active phases change and on loop exit; no GPU synchronization.')

            def sample_torch_memory(step, phases, reason):
                sample = {'step': step, 'phases': dict(phases), 'reason': reason, 'unit': 'bytes'}
                try:
                    cuda_device = e.device if str(e.device).startswith('cuda') else 'cuda:0'
                    sample.update(device=str(cuda_device),
                                  allocated_bytes=int(torch.cuda.memory_allocated(cuda_device)),
                                  reserved_bytes=int(torch.cuda.memory_reserved(cuda_device)),
                                  max_allocated_bytes=int(torch.cuda.max_memory_allocated(cuda_device)),
                                  max_reserved_bytes=int(torch.cuda.max_memory_reserved(cuda_device)))
                except Exception as error:
                    # Diagnostic failure must not alter the collection algorithm.
                    sample['error'] = repr(error)
                report['torch_memory_samples'].append(sample)
                print('VECTORIZED_MEMORY', json.dumps(sample, separators=(',', ':')), flush=True)

            last_memory_phases = None
            memory_step = -1
            collection_loop_started = time.perf_counter()
            try:
                for step_index in range(args.max_steps):
                    memory_step = step_index
                    tick = time.perf_counter()
                    active = {}
                    for i, controller in enumerate(controllers):
                        action = controller.prepare(states[i])
                        if action is not None:
                            action_batch[i] = action
                            active[i] = controller.phase
                    timings['planning_dispatch_s'] += time.perf_counter() - tick
                    if active != last_memory_phases:
                        sample_torch_memory(step_index, active, 'phase_change_after_prepare')
                        last_memory_phases = dict(active)
                    if not active:
                        break
                    if cameras:
                        tick = time.perf_counter()
                        capture_time_ns = time.time_ns()
                        cameras.write(active, action_indices={i: len(controllers[i].records) for i in active})
                        receive_time_ns = time.time_ns()
                        if risk_runtime:
                            from kitchen_safety.risk_camera import append_camera_provenance
                            for i in active:
                                observer = risk_runtime.observers[i]
                                append_camera_provenance(outputs[i], dict(identity=observer.identity,
                                    physics_dt_s=observer.dt, action_dt_s=observer.action_dt, binding=observer.binding),
                                    len(controllers[i].records), capture_time_ns, receive_time_ns, mode='live_pre_action')
                        timings['camera_s'] += time.perf_counter() - tick
                    if risk_runtime:
                        risk_runtime.set_actions(active, controllers, action_batch)
                    tick = time.perf_counter()
                    _, _, terminated, truncated, _ = env.step(
                        torch.as_tensor(action_batch, dtype=torch.float32, device=e.device))
                    timings['physics_execution_s'] += time.perf_counter() - tick
                    states = snapshot_all()
                    terminated, truncated = cpu(terminated), cpu(truncated)
                    for i in active:
                        states[i].update(terminated=bool(terminated[i]), truncated=bool(truncated[i]))
                        controllers[i].observe(states[i])
                    if (step_index + 1) % 60 == 0:
                        print('VECTORIZED_COLLECT_STEP', step_index+1, active, flush=True)
                else:
                    report['step_budget_exhausted'] = True
                    for controller in controllers:
                        if not controller.done:
                            controller.fail('Batch action budget exhausted', kind='diagnostic_limit')
            except Exception as error:
                # Preserve partial physical records on renderer/backend failure.
                # A failed writer may already have a partial RGB frame, which
                # must never be reported as a successful episode.
                report['execution_error'] = traceback.format_exc()
                for controller in controllers:
                    if not controller.done:
                        controller.fail(error, kind='data_validation_failed')
            finally:
                timings['collection_loop_s'] = time.perf_counter() - collection_loop_started
                scene.update = original_update
                e.sim.step = original_sim_step
                scene.write_data_to_sim = original_scene_write
                sample_torch_memory(memory_step, active, 'loop_finally')

            camera_episode_timings = {}
            if cameras:
                try:
                    cameras.close()
                except Exception as error:
                    report['camera_close_error'] = repr(error)
                    for controller in controllers:
                        controller.fail(error, kind='data_validation_failed')
                report['camera_timings_seconds'] = cameras.timings
                camera_episode_timings = {i: writer.timings for i, writer in cameras.writers.items()}
                cameras = None

            # Export once for independent CPU validators. This is the complete
            # authored scene, including per-env static obstacles, not a proxy
            # containing only the target hazard.
            safety_results = None
            if args.validation_workers > 1 and len(controllers) > 1:
                from concurrent.futures import ProcessPoolExecutor
                import multiprocessing
                from kitchen_safety.vectorized_validation import validate_physics_archive
                tick = time.perf_counter()
                stage_path = out / 'validation_scene.usdc'
                e.sim.stage.Export(str(stage_path))
                jobs = []
                for i, controller in enumerate(controllers):
                    state_path = outputs[i] / 'physics_states.npz'
                    buffers[i].write_npz(state_path, arm_ids=ids, joint_names=robot.joint_names)
                    job = dict(stage_path=str(stage_path), physics_states_path=str(state_path),
                        episode_spec=specs[i], env_root=f'/World/envs/env_{i}', env_origin=origins[i].tolist(),
                        settled_poses=controller.result.get('settled_poses', {}), state_frame='env_local')
                    if args.defer_clearance_diagnostics:
                        diagnostic_path = outputs[i] / 'action_clearance_states.npz'
                        controller.write_clearance_archive(diagnostic_path)
                        job['action_clearance_states_path'] = str(diagnostic_path)
                    jobs.append(job)
                timings['validation_export_s'] = time.perf_counter() - tick
                tick = time.perf_counter()
                try:
                    with ProcessPoolExecutor(max_workers=min(args.validation_workers, len(controllers)),
                            mp_context=multiprocessing.get_context('spawn')) as executor:
                        safety_results = list(executor.map(validate_physics_archive, jobs))
                except Exception as error:
                    safety_results = [dict(passed=False, checks=0, error=repr(error)) for _ in controllers]
                timings['deferred_validation_s'] += time.perf_counter() - tick
                (out / 'physics_validation.json').write_text(json.dumps(safety_results, indent=2))

            for i, controller in enumerate(controllers):
                result = controller.result
                result['terminations'] = {name: bool(value[i]) for name, value in termination_seen.items()}
                if safety_results is None:
                    tick = time.perf_counter()
                    try:
                        buffers[i].write_npz(outputs[i] / 'physics_states.npz', arm_ids=ids, joint_names=robot.joint_names)
                        safety = buffers[i].validate(controller.world, ids, robot.joint_names)
                    except Exception as error:
                        safety = dict(passed=False, checks=0, error=repr(error))
                    timings['deferred_validation_s'] += time.perf_counter() - tick
                else:
                    safety = safety_results[i]
                if args.defer_clearance_diagnostics:
                    tick = time.perf_counter()
                    try:
                        if safety_results is None:
                            controller.write_clearance_archive(outputs[i] / 'action_clearance_states.npz')
                            controller.finalize_clearance_diagnostics()
                        else:
                            diagnostic = safety.get('action_clearance_diagnostics')
                            if diagnostic is None:
                                raise RuntimeError('Worker did not return action clearance diagnostics')
                            controller.finalize_clearance_diagnostics(diagnostic)
                    except Exception as error:
                        diagnostic_failure = result.setdefault('action_clearance_diagnostics', {})
                        diagnostic_failure.update(completed=False, error=repr(error))
                        result.update(success=False, failure_kind='data_validation_failed',
                                      error=f'Action clearance diagnostics failed: {error!r}')
                    timings['deferred_validation_s'] += time.perf_counter() - tick
                result['deferred_safety'] = safety
                result['physics_clearance_checks'] = safety['checks']
                if not safety['passed']:
                    result.update(success=False,
                        failure_kind='data_validation_failed' if safety.get('error') else 'physical_failed',
                        error='Deferred safety validation failed')
                tick = time.perf_counter()
                records = controller.records
                with h5py.File(outputs[i] / 'trajectory.h5', 'w') as f:
                    f.attrs.update(dt=e.step_dt, quaternion_order='xyzw',
                        joint_names=json.dumps(robot.joint_names), contact_body_names=json.dumps(contact_names),
                        robot_body_names=json.dumps(robot.body_names),
                        action_semantics='7 absolute arm joint targets in radians; binary gripper 0=open, 1=close',
                        coordinate_frame='environment_local', environment_origin_world_m=origins[i])
                    if args.record_render_state:
                        f.attrs['record_render_state'] = True
                    f.attrs['collection_purpose'] = args.collection_purpose
                    f.attrs['collection_source'] = args.collection_source or ''
                    f.attrs['collection_job'] = json.dumps(collection_jobs[i])
                    f.create_dataset('time', data=np.arange(len(records))*e.step_dt)
                    f.create_dataset('phase', data=np.array([r[0] for r in records], dtype=h5py.string_dtype()))
                    f.create_dataset('action', data=np.array([r[1] for r in records], dtype=np.float32).reshape(-1, 8))
                    for key in (records[0][2] if records else {}):
                        f.create_dataset('obs/'+key, data=np.array([r[2][key] for r in records]))
                        f.create_dataset('next_obs/'+key, data=np.array([r[3][key] for r in records]))
                result.update(steps=len(records), coordinate_frame='environment_local',
                              environment_origin_world_m=origins[i].tolist(),
                              record_render_state=args.record_render_state,
                              appearance_mode=report['appearance_mode'],
                              production_quota_eligible=False)
                result.update(collection_purpose=args.collection_purpose,
                              collection_source=args.collection_source, collection_job=collection_jobs[i],
                              collector_overrides=collector_overrides,
                              resolved_collector_options=report['resolved_collector_options'],
                              acceptance_status=report['acceptance_status'])
                result['execution_configuration'] = dict(
                    simulation_device=str(e.device), num_envs=args.num_envs,
                    camera_device=args.camera_device, physics_threads=args.physics_threads,
                    torch_threads=args.torch_threads,
                    record_render_state=args.record_render_state,
                    background_rgb=args.background_rgb,
                    reuse_phase_planner=args.reuse_phase_planner,
                    curobo_cuda_graph=args.curobo_cuda_graph,
                    render_settings=report.get('render_settings'),
                    physics_dt=cfg.sim.dt, action_dt=e.step_dt,
                    episode_length_s=cfg.episode_length_s, max_steps=args.max_steps,
                    initialization_action_steps=8,
                    execution_checks='deferred', camera_owner='batched_writer',
                    action_clearance_diagnostics='deferred' if args.defer_clearance_diagnostics else 'online',
                    replicate_physics=cfg.scene.replicate_physics,
                    solver_position_iterations=cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count)
                if args.record_render_state:
                    result['timings_seconds']['render_state_capture_shared_batch_s'] = timings['render_state_capture_s']
                    result['render_state_timing_scope'] = report['render_state_timing_scope']
                if args.record_rgb:
                    result['appearance_warmup'] = report['appearance_warmup']
                    result['appearance'] = json.loads((outputs[i] / 'appearance.json').read_text())
                    result['timings_seconds']['camera'] = camera_episode_timings.get(i, {})
                (outputs[i] / 'clearances.json').write_text(json.dumps(controller.clearance_records))
                (outputs[i] / 'grasp_contacts.json').write_text(json.dumps(controller.grasp_contacts.records))
                (outputs[i] / 'result.json').write_text(json.dumps(result, indent=2))
                timings['data_write_s'] += time.perf_counter() - tick
            if risk_runtime:
                risk_runtime.close([controller.result for controller in controllers])
            report.update(status='completed', successes=sum(bool(c.result.get('success')) for c in controllers),
                episodes=[{'env_id': i, 'output': str(outputs[i]), 'job': collection_jobs[i], 'success': bool(c.result.get('success')),
                           'steps': len(c.records), 'error': c.result.get('error')} for i, c in enumerate(controllers)])
        except Exception:
            report.update(status='error', error=traceback.format_exc())
            raise
        finally:
            if risk_runtime and not all(o.closed for o in risk_runtime.observers):
                risk_runtime.close([controller.result for controller in controllers])
            timings['total_before_close_s'] = time.perf_counter() - started
            (out / 'batch.json').write_text(json.dumps(report, indent=2))
            print('VECTORIZED_COLLECTION_RESULT', json.dumps(report), flush=True)
            if cameras:
                cameras.close()
            if env:
                env.close()


if __name__ == '__main__':
    main()
