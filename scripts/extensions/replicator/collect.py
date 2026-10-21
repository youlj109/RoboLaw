"""Execute a physical grasp/place and record all three robot cameras."""
import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

collection_started = time.perf_counter()

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

from kitchen_safety.collection_presets import explicit_collector_overrides, resolve_collector_options
from kitchen_safety.step_profiling import StepWallProfiler
from kitchen_safety.planning_checks import check_planned_path

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--variant', choices=['base', 'human', 'property', 'robot'], default='base')
parser.add_argument('--planner', choices=['fcl', 'curobo'], default='curobo')
parser.add_argument('--output', default=None)
parser.add_argument('--video', action='store_true')
parser.add_argument('--overview-camera', action='store_true', help='Add a separate synchronized scene overview; preserve the original three cameras')
parser.add_argument('--record-rgb', action='store_true', help='Record original three cameras without live MP4 encoding')
parser.add_argument('--background-rgb', action='store_true',
                    help='Encode and write RGB on the existing bounded CPU worker; requires --record-rgb without --video')
parser.add_argument('--control-profile', choices=['base','human','property','robot'], help='For a no-hazard baseline only, retain this variant-specific planning profile')
parser.add_argument('--layout-json', help='Fixed source-preserving layout with an explicitly searched hazard')
parser.add_argument('--episode-spec', help='Resolved augmentation episode JSON shared by scene, planner and recorder')
parser.add_argument('--physics-threads', type=int, default=None,
                    help='PhysX CPU worker count; 0 runs on the main thread. Does not change solver accuracy.')
parser.add_argument('--torch-threads', type=int, default=None,
                    help='CPU PyTorch worker count for small tensor and image operations')
parser.add_argument('--execution-checks', choices=['online', 'deferred'], default='online',
                    help='Check actual geometry each physics step, or record those states and check after execution')
parser.add_argument('--camera-owner', choices=['environment', 'writer'], default='environment',
                    help='writer avoids creating unused camera observations inside env.step')
parser.add_argument('--cpu-rgb-copy', action='store_true',
                    help='Use vectorized copies for single-camera CPU RGB outputs, preserving pixel values')
parser.add_argument('--camera-device', default=None,
                    help='Optional camera buffer device independent of CPU physics, e.g. cuda:0')
parser.add_argument('--reuse-planner', action='store_true',
                    help='Update supported cuRobo hazard boxes in place when their measured pose changes')
parser.add_argument('--max-steps', type=int, default=0)
parser.add_argument('--episode-length-s', type=float, default=120.0,
                    help='Arena task timeout in simulation seconds, including initialization; default 120')
parser.add_argument('--jitter', type=float, default=0.0)
parser.add_argument('--obstacle_id', default='', help='Asset-library entry; shared task and original cameras')
parser.add_argument('--grasp-tilt-deg', type=float, default=argparse.SUPPRESS,
                    help='Explicit pregrasp tilt; requires --grasp-yaw-deg and disables pregrasp orientation fallback')
parser.add_argument('--grasp-yaw-deg', type=float, default=argparse.SUPPRESS,
                    help='Explicit pregrasp yaw; requires --grasp-tilt-deg')
parser.add_argument('--grasp-offset-z-m', type=float, default=argparse.SUPPRESS,
                    help='Bottle-local TCP z within the broad bottle body; omit to use the scene preset or default 0')
parser.add_argument('--grasp-offset-y-m', type=float, default=argparse.SUPPRESS,
                    help='Bottle-local lateral TCP adjustment for finger/palm clearance')
parser.add_argument('--transport-bottle-height-m', type=float, default=argparse.SUPPRESS,
                    help='World height of the carried bottle root over the bowl; independent of grasp depth')
release_options = parser.add_mutually_exclusive_group()
release_options.add_argument('--supported-release', action='store_true', default=argparse.SUPPRESS,
                             help='Lower using live bottle/bowl mesh support geometry before opening')
release_options.add_argument('--no-supported-release', dest='supported_release',
                             action='store_false', default=argparse.SUPPRESS,
                             help='Disable supported release, overriding the obstacle preset')
parser.add_argument('--transport-tilt-cap-deg', type=float, default=argparse.SUPPRESS,
                    help='Optional bottle-tilt planning target below the existing 28 degree execution limit')
parser.add_argument('--placement-strategy', choices=['pose', 'lower-first'], default=argparse.SUPPRESS,
                    help='Solve and validate a placement column before joint-goal transport')
parser.add_argument('--placement-tcp-height-m', type=float, default=argparse.SUPPRESS,
                    help='Independent lower-first placement TCP height in world coordinates')
args = parser.parse_args()
if args.control_profile is not None and args.variant != 'base' and args.control_profile != args.variant:
    parser.error('A control-profile override is only allowed for no-hazard baselines')
control_variant = args.control_profile or args.variant
if not args.obstacle_id:
    args.obstacle_id = {'base': '', 'human': 'human_default_mpfb', 'property': 'property_vase',
                        'robot': 'robot_projecting_crossbar'}[args.variant]
if not math.isfinite(args.episode_length_s) or args.episode_length_s <= 0:
    parser.error('--episode-length-s must be finite and positive')
if args.background_rgb and (not args.record_rgb or args.video):
    parser.error('--background-rgb requires --record-rgb without --video; previews remain offline')
collector_overrides = explicit_collector_overrides(args)
if ('grasp_tilt_deg' in collector_overrides) != ('grasp_yaw_deg' in collector_overrides):
    parser.error('--grasp-tilt-deg and --grasp-yaw-deg must be specified together')
# Preserve the existing defaults outside the ten configured nonhuman IDs.
# A preset is selected before validation; explicit CLI values take precedence.
collector_options = resolve_collector_options(
    '',
    base_defaults={
        'grasp_tilt_deg': 35., 'grasp_yaw_deg': 180.,
        'grasp_offset_z_m': 0.0, 'grasp_offset_y_m': -0.01,
        'transport_bottle_height_m': 1.085, 'supported_release': True,
        'transport_tilt_cap_deg': None, 'placement_strategy': 'pose',
        'placement_tcp_height_m': None,
    },
    cli_overrides=collector_overrides,
)
for name, value in collector_options.items():
    setattr(args, name, value)
if (args.grasp_tilt_deg is None) != (args.grasp_yaw_deg is None):
    parser.error('--grasp-tilt-deg and --grasp-yaw-deg must be specified together')
if args.placement_strategy == 'lower-first' and args.planner != 'curobo':
    parser.error('lower-first placement requires cuRobo for world and self-collision checks')
if args.placement_tcp_height_m is not None and args.placement_strategy != 'lower-first':
    parser.error('--placement-tcp-height-m requires lower-first placement')
if args.transport_tilt_cap_deg is not None and not 5 < args.transport_tilt_cap_deg < 27:
    parser.error('--transport-tilt-cap-deg must be between 5 and 27 degrees')
fixed_layout = None
if args.layout_json:
    from kitchen_safety.extensions.replicator.layout import resolve_layout
    fixed_layout = json.loads(Path(args.layout_json).read_text())
    reference_layout = resolve_layout(args.obstacle_id)
    if {k:v for k,v in fixed_layout.items() if k != 'obstacle'} != {k:v for k,v in reference_layout.items() if k != 'obstacle'}:
        parser.error('Fixed hazard layout must preserve all original source and planning settings')
    if fixed_layout.get('obstacle',{}).get('id','') != args.obstacle_id:
        parser.error('Fixed hazard ID mismatch')
    if args.episode_spec:
        parser.error('Choose fixed layout or randomized episode specification')
episode_spec = None
if args.episode_spec:
    if args.planner != 'curobo':
        parser.error('Randomized collection currently requires cuRobo')
    if args.jitter:
        parser.error('--episode-spec cannot be combined with legacy --jitter')
    episode_spec = json.loads(Path(args.episode_spec).read_text())
    if episode_spec['obstacle_id'] != args.obstacle_id:
        parser.error('Episode specification belongs to another obstacle')
    if episode_spec['layout']['obstacle']['category'] != args.variant:
        parser.error('Episode specification belongs to another safety category')
args.enable_cameras = args.video or args.record_rgb or args.overview_camera
planner_seed = episode_spec['seeds']['planner'] if episode_spec else args.seed or 0
explicit_grasp_orientation = args.grasp_tilt_deg is not None
grasp_offset_z = args.grasp_offset_z_m
lower_target_z = args.placement_tcp_height_m

with SimulationAppContext(args):
    import h5py
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation

    if args.torch_threads is not None:
        torch.set_num_threads(args.torch_threads)

    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab.sensors import ContactSensorCfg
    from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
    from kitchen_safety.dataset import CameraWriter
    from kitchen_safety.background_camera import BackgroundCameraWriter, camera_caller_seconds
    from kitchen_safety.grasp_contacts import BottleGraspContacts
    from kitchen_safety.grasp_geometry import bottle_body_geometry, body_grasp_tcp_target
    from kitchen_safety.extensions.replicator.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.layout import resolve_layout
    from kitchen_safety.augmentation_geometry import probe_obstruction, check_initial_layout, relative_grasp_rotation

    if args.physics_threads is not None:
        if args.physics_threads < 0:
            raise ValueError('physics-threads must be nonnegative')
        import carb
        import omni.physx
        from omni.physx.bindings._physx import SETTING_NUM_THREADS
        carb.settings.get_settings().set_int(SETTING_NUM_THREADS, args.physics_threads)
        omni.physx.get_physx_interface().set_thread_count(args.physics_threads)

    environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
        variant=args.variant, enable_cameras=args.enable_cameras, object_jitter_m=args.jitter,
        collection_seed=args.seed or 0, obstacle_id=args.obstacle_id, episode_spec=episode_spec, fixed_layout=fixed_layout,
        episode_length_s=args.episode_length_s))
    out = Path('/workspace') / (args.output or (
        f'datasets/kitchen_auto/{args.variant}/episode_{datetime.now():%Y%m%d_%H%M%S}'))
    builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
        device=args.device, seed=args.seed or 0, presets='physx', num_envs=1,
        solve_relations=False, disable_fabric=False,
        recorder_dataset_export_dir_path=str(out.resolve() / 'arena_recorder')))
    cfg, kwargs = builder.compose_manager_cfg()
    overview_spec = None
    if args.overview_camera:
        from kitchen_safety.extensions.replicator.overview import configure_overview
        overview_spec = configure_overview(cfg.scene)
    cfg.sim.log_dir = str(out.resolve() / 'isaaclab_logs')
    cfg.sim.render_interval = cfg.decimation
    if args.camera_device is not None and args.enable_cameras:
        from kitchen_safety.camera_device import with_camera_output_device
        for name in list(environment.embodiment.camera_config.camera_names()) + (['overview_camera'] if args.overview_camera else []):
            setattr(cfg.scene, name, with_camera_output_device(getattr(cfg.scene, name), args.camera_device))
    if args.camera_owner == 'writer' and args.enable_cameras:
        cfg.observations.camera_obs = None
    cfg.scene.robot_contacts = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    scene = e.scene
    cpu_camera_stats = []
    if args.cpu_rgb_copy and args.enable_cameras:
        from kitchen_safety.cpu_camera_copy import install_cpu_rgb_copy
        renderers = {}
        for name in list(environment.embodiment.camera_config.camera_names()) + (['overview_camera'] if args.overview_camera else []):
            renderer = scene[name]._renderer
            if id(renderer) not in renderers:
                renderers[id(renderer)] = install_cpu_rgb_copy(renderer)
        cpu_camera_stats = list(renderers.values())
    robot = scene['robot']
    if not args.enable_cameras:
        e.render_enabled = False
    ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)

    def arr(v):
        return (v.torch if hasattr(v, 'torch') else v).detach().cpu().numpy().copy()

    layout = episode_spec['layout'] if episode_spec else (fixed_layout if fixed_layout is not None else resolve_layout(args.obstacle_id, args.jitter, args.seed or 0))
    dynamic_hazard = bool(args.obstacle_id and not layout['obstacle']['static'])
    planner_hazard_pose = None
    settings = layout.get('planning', {})
    base = np.array(layout['robot_position'])
    base[2] += layout['stand_height']
    kin = DroidKinematics(base, layout['robot_yaw_deg'])
    q = arr(robot.data.joint_pos)[0, ids]
    grip = 0.0
    phase = 'settle'
    records = []
    terminated = False
    terminal = {}
    world = None
    deferred_safety = None
    if args.execution_checks == 'deferred':
        from kitchen_safety.extensions.replicator.deferred_safety import DeferredSafetyBuffer
        deferred_safety = DeferredSafetyBuffer()
    motion_planner = None
    safety_checks = 0
    safety_failure = None
    clearance_records = []
    plan_records = []
    plan_candidate_failures = []
    planner_reuse = {'updates': 0, 'rebuilds': 0, 'update_seconds': 0., 'rebuild_seconds': 0.}
    obstruction_probes = []
    timings = dict(initialization_s=0., planning_s=0., physics_s=0., obstruction_probe_s=0., write_data_s=0.)
    step_profile = StepWallProfiler()
    # The PhysX backend uses this optional reference to time its existing
    # simulate/fetch/device calls separately during recorded env.step only.
    from isaaclab_physx.physics.physx_manager import PhysxManager
    PhysxManager._step_wall_profiler = step_profile
    # Attribute only the existing env.step wall-time interval. These wrappers
    # never add simulation steps, renders, or device synchronization.
    step_profile.wrap(e.sim, 'step', 'physics_backend', hook='simulation_step')
    step_profile.wrap(e.sim, 'render', 'rendering', hook='render_dispatch')
    # RenderContext uses __slots__, so its method must be wrapped on the class.
    step_profile.wrap(type(e.sim.render_context), 'render_into_camera', 'rendering', hook='camera_render_chain')
    appearance_report = None
    grasp_rotation_delta = np.eye(3)
    out.mkdir(parents=True, exist_ok=True)
    for _ in range(8):
        env.step(torch.tensor(np.r_[q, 0.][None], dtype=torch.float32, device=e.device))
    if episode_spec:
        from kitchen_safety.randomization import apply_appearance
        (out/'episode_spec.json').write_text(json.dumps(episode_spec, indent=2))
        appearance_report = apply_appearance(e.sim.stage, episode_spec)
        (out/'appearance.json').write_text(json.dumps(appearance_report, indent=2))
    cameras = None
    overview_writer = None
    overview_alignment = []
    if args.enable_cameras:
        camera_names = environment.embodiment.camera_config.camera_names()
        cameras = (BackgroundCameraWriter(out, scene, camera_names, e.step_dt) if args.background_rgb else
                   CameraWriter(out, scene, camera_names, e.step_dt, previews=args.video))
        if args.overview_camera:
            overview_dir = out/'overview'; overview_dir.mkdir()
            overview_writer = CameraWriter(overview_dir, scene, ['overview_camera'], e.step_dt, previews=True)
    contact_body_names = scene['robot_contacts'].body_names
    nonfinger_ids = [i for i, n in enumerate(contact_body_names)
                    if n.startswith('panda_link') or n == 'base_link']
    bi = robot.body_names.index('base_link')
    gripper_names = [name for i, name in enumerate(robot.joint_names) if i not in ids]
    grasp_contacts = BottleGraspContacts(scene['robot_contacts']._physics_sim_view, cfg.sim.dt)
    background_view=scene['robot_contacts']._physics_sim_view.create_rigid_body_view('/World/envs/env_0/kitchen/**')
    background_paths=list(background_view.prim_paths)
    if deferred_safety is not None:deferred_safety.background_paths=background_paths
    def background_poses():
        value=background_view.get_transforms()
        if hasattr(value,'torch'):return value.torch.detach().cpu().numpy().copy()
        if hasattr(value,'detach'):return value.detach().cpu().numpy().copy()
        return np.asarray(value.numpy() if hasattr(value,'numpy') else value).copy()

    grasp_geometry = bottle_body_geometry()
    body_contact_band = grasp_geometry['safe_contact_z_range_m']

    def snapshot():
        p = arr(robot.data.body_pos_w)[0, bi]
        quat = arr(robot.data.body_quat_w)[0, bi]
        tcp = p + Rotation.from_quat(quat).apply([.131, 0, 0])
        state = dict(q=arr(robot.data.joint_pos)[0], dq=arr(robot.data.joint_vel)[0],
                    tcp=tcp, tcp_quat_xyzw=quat,
                    robot_body_pose_w=np.concatenate((arr(robot.data.body_pos_w)[0],
                                                      arr(robot.data.body_quat_w)[0]), axis=-1),
                    mustard=arr(scene['mustard_bottle'].data.root_state_w)[0],
                    bowl=arr(scene['bowl'].data.root_state_w)[0],
                    robot_contact_forces=arr(scene['robot_contacts'].data.net_forces_w)[0],
                    background_rigid_poses=background_poses())
        if dynamic_hazard:
            state['hazard_pose'] = arr(scene['glass'].data.root_state_w)[0,:7]
        return state

    def ignored_objects():
        ignored = ('mustard_bottle',)
        if phase in ('lower', 'release', 'retreat', 'settle'):
            ignored += ('bowl',)
        return ignored

    def geometry_args(state):
        joints = dict(zip(robot.joint_names, state['q']))
        return dict(mode='execution',
                    gripper_q={name: joints[name] for name in gripper_names},
                    object_pose=state['mustard'][:7] if world.attachment is not None else None)

    # Check the executed robot after every physics update, including finger opening.
    # This uses actual joints, not the targets sent to the position controller.
    scene_update_original = scene.update

    def check_executed_safety():
        global safety_checks, safety_failure
        with step_profile.section('safety_checks', hook='safety_state_readback'):
            state = {'q': arr(robot.data.joint_pos)[0],
                     'mustard': arr(scene['mustard_bottle'].data.root_state_w)[0]}
            if dynamic_hazard:
                state['hazard_pose'] = arr(scene['glass'].data.root_state_w)[0,:7]
        if deferred_safety is not None:
            deferred_safety.background_poses.append(background_poses())
            deferred_safety.append(q=state['q'], mustard=state['mustard'][:7],
                                   phase=phase, action_index=len(records), attachment=world.attachment,
                                   hazard_pose=state.get('hazard_pose'))
            return
        if dynamic_hazard:
            with step_profile.section('safety_checks', hook='safety_obstacle_update'):
                world.set_obstacle_pose('glass', state['hazard_pose'])
        world.sync_background(background_paths,background_poses())
        safety_checks += 1
        if not world.valid(state['q'][ids], ignored_objects(), **geometry_args(state)):
            reason = world.last_collision
            safety_failure = world.clearance(state['q'][ids], ignored_objects(), **geometry_args(state))
            safety_failure.update(geometry_valid=safety_failure['valid'], valid=False,
                                  reason=reason, phase=phase, action_index=len(records), physics_check=safety_checks)
            raise RuntimeError(f'Executed constraint violation: {safety_failure}')
        # Once released, bowl/counter support is intentional. Still check the
        # free object's actual pose against the person, fragile props and fixture.
        if world.attachment is None:
            free_object_args = geometry_args(state)
            free_object_args['object_pose'] = state['mustard'][:7]
            if not world.valid(state['q'][ids], ignored_objects(),
                               obstacle_filter=('person', 'glass', 'plate', 'phone', 'barrier'), **free_object_args):
                safety_failure = world.clearance(state['q'][ids], ignored_objects(),
                                                 obstacle_filter=('person', 'glass', 'plate', 'phone', 'barrier'), **free_object_args)
                safety_failure.update(phase=phase, action_index=len(records), physics_check=safety_checks)
                raise RuntimeError(f'Free-object hazard clearance violation: {safety_failure}')

    def checked_scene_update(dt):
        with step_profile.section('scene_update', hook='scene_update'):
            scene_update_original(dt)
        if world is not None:
            with step_profile.section('safety_checks', hook='physics_rate_safety'):
                check_executed_safety()

    scene.update = checked_scene_update
    reset_original = e._reset_idx

    def capture_reset(env_ids):
        if len(env_ids):
            terminal['state'] = snapshot()
            terminal['terms'] = {n: bool(arr(e.termination_manager.get_term(n))[0])
                                 for n in e.termination_manager.active_terms}
        if not terminal.get('terms', {}).get('success', False):
            return reset_original(env_ids)

    e._reset_idx = capture_reset

    def step(command, gripper):
        global terminated
        if terminated:
            raise RuntimeError('Environment terminated before the physical sequence completed')
        if args.max_steps and len(records) >= args.max_steps:
            raise RuntimeError('Diagnostic step limit reached')
        before = snapshot()
        if cameras:
            before_capture = int(e.sim.get_physics_step_count())
            cameras.write(phase)
            if overview_writer:
                overview_writer.write(phase)
                overview_alignment.append(dict(action_index=len(records), phase=phase,
                    primary_frame_index=cameras.frames-1, overview_frame_index=overview_writer.frames-1,
                    physics_step_before=before_capture, physics_step_after=int(e.sim.get_physics_step_count()),
                    simulation_time_s=before_capture*cfg.sim.dt))
        action = np.r_[command, gripper].astype(np.float32)
        physics_started = time.perf_counter()
        step_profile.begin(phase)
        try:
            _, _, done, truncated, _ = env.step(torch.tensor(action[None], device=e.device))
            terminated = bool(arr(truncated)[0] or (
                arr(done)[0] and not terminal.get('terms', {}).get('success', False)))
        finally:
            step_elapsed = time.perf_counter() - physics_started
            step_profile.finish(step_elapsed)
            timings['physics_s'] += step_elapsed
            after = terminal['state'] if terminated else snapshot()
            records.append((phase, action, before, after))
        if gripper and phase in ('close', 'lift', 'transfer_front', 'transfer', 'above_bowl', 'lower'):
            grasp_contacts.sample(phase, len(records)-1, after['mustard'][:7])
        if world is not None and args.variant != 'base':
            hazard = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}[args.variant]
            hazard_args = geometry_args(after)
            hazard_args['object_pose'] = after['mustard'][:7]
            distance = world.clearance(after['q'][ids], ignored_objects(),
                                       obstacle_filter=(hazard,), **hazard_args)
            clearance_records.append(dict(frame=len(records)-1, phase=phase, **distance))
        if len(records) % 30 == 0:
            print('STEP', len(records), phase, 'tcp', after['tcp'].round(3),
                  'bottle', after['mustard'][:3].round(3), flush=True)

    def move(target, seconds=2):
        global q
        start = q.copy()
        # Smoothstep has peak speed 1.5 times its mean speed.
        seconds = max(seconds, 1.5*np.max(np.abs(target-start)) /
                      settings.get('max_joint_speed_rad_s', .4))
        for u in np.linspace(0, 1, max(2, round(seconds/e.step_dt))):
            step(start + u*u*(3-2*u)*(target-start), grip)
        q = target.copy()

    def cart(xyz, seconds=2, *, object_target=False):
        global q, motion_planner, planner_hazard_pose
        # Planning starts from the physical state after the previous segment settles.
        current = snapshot()
        planning_started = time.perf_counter()
        q = current['q'][ids].copy()
        if world.sync_background(background_paths,background_poses()) and motion_planner is not None:
            motion_planner = new_motion_planner()
        if dynamic_hazard:
            world.set_obstacle_pose('glass', current['hazard_pose'])
            if motion_planner is not None and not np.array_equal(current['hazard_pose'],planner_hazard_pose):
                updated = False
                if args.reuse_planner:
                    update_started = time.perf_counter()
                    updated = motion_planner.update_dynamic_obstacle('glass')
                    planner_reuse['update_seconds'] += time.perf_counter() - update_started
                if updated:
                    planner_hazard_pose = current['hazard_pose'].copy()
                    planner_reuse['updates'] += 1
                else:
                    rebuild_started = time.perf_counter()
                    motion_planner = new_motion_planner()
                    planner_reuse['rebuild_seconds'] += time.perf_counter() - rebuild_started
                    planner_reuse['rebuilds'] += 1
        ignore = ignored_objects()
        candidates = []
        last = None
        selected = None
        orientations = [(35, 0), (25, 0), (35, -30), (35, -45), (45, -30),
                        (45, 0), (55, -45), (55, 0), (35, 30), (35, 45)]
        if explicit_grasp_orientation:
            if phase in ('transfer_front', 'transfer', 'above_bowl'):
                orientations = [(args.grasp_tilt_deg + tilt - 35, args.grasp_yaw_deg + yaw)
                                for tilt, yaw in orientations]
            else:
                # Pregrasp must realize this requested orientation. The cuRobo
                # approach/lift/lower/retreat paths retain their measured start
                # orientation through the existing local vertical IK routine.
                orientations = [(args.grasp_tilt_deg, args.grasp_yaw_deg)]
        else:
            # Start automatic body grasping with the physically verified
            # 45-degree orientation, retaining the relative fallback order.
            orientations = [(tilt + 10., yaw) for tilt, yaw in orientations]

        def record_candidate_failure(tilt, yaw, stage, error, target):
            plan_candidate_failures.append({
                'phase': phase, 'backend': args.planner, 'tilt_deg': tilt, 'yaw_deg': yaw,
                'failure_stage': stage, 'error': str(error), 'start_q_arm': q.tolist(),
                'target_pose_world': target.tolist(), 'action_index': len(records),
            })

        for tilt, yaw in orientations:
            target = np.eye(4)
            target[:3, :3] = (Rotation.from_euler('z', yaw, degrees=True).as_matrix()
                               @ Rotation.from_euler('x', tilt, degrees=True).as_matrix()
                               @ np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]]))
            if episode_spec and phase == 'pregrasp':
                target[:3, :3] = relative_grasp_rotation(target[:3, :3], grasp_rotation_delta)
            target[:3, 3] = xyz
            if object_target:
                if world.attachment is None:
                    raise RuntimeError('An object target requires a measured grasp attachment')
                target[:3, 3] -= target[:3, :3] @ world.attachment[:3, 3]
            failure_stage = 'motion_generation' if motion_planner is not None else 'ik'
            try:
                if motion_planner is not None:
                    world.gripper_joint_position = float(snapshot()['q'][robot.joint_names.index('finger_joint')])
                    trajectory = motion_planner.plan_pose(q, target, ignore, grip, phase=phase)
                    path = trajectory['positions']
                    if (args.transport_tilt_cap_deg is not None and world.attachment is not None
                            and phase in ('transfer_front', 'transfer', 'above_bowl')):
                        from kitchen_safety.placement_planner import repair_upright_path
                        failure_stage = 'transport_tilt_margin'
                        cap = args.transport_tilt_cap_deg
                        repaired, correction = repair_upright_path(
                            kin, world.attachment, path, soft_tilt_deg=cap-5.,
                            cap_tilt_deg=cap, hard_tilt_deg=28.)
                        if (not correction['success']
                                or correction['max_edge_sample_tilt_deg'] > cap + .5):
                            raise RuntimeError(f'No bottle-tilt planning margin: {correction}')
                        # IK changes joint increments. Retime the corrected geometry
                        # before execution, including its acceleration at sample edges.
                        speed = float(np.max(np.abs(np.diff(repaired, axis=0))) / e.step_dt)
                        acceleration = (float(np.max(np.abs(np.diff(repaired, n=2, axis=0)))
                                               / e.step_dt**2) if len(repaired) > 2 else 0.)
                        dilation = max(1., speed/.2, np.sqrt(acceleration/.4))
                        for _ in range(10):
                            path = motion_planner.retime_path(repaired, source_dt=e.step_dt*dilation)
                            held_path = np.vstack([path[0], path, path[-1]])
                            measured_speed = float(np.max(np.abs(np.diff(held_path, axis=0))) / e.step_dt)
                            measured_acceleration = float(np.max(np.abs(np.diff(held_path, n=2, axis=0)))
                                                          / e.step_dt**2)
                            if measured_speed <= .2 + 1e-8 and measured_acceleration <= .4 + 1e-8:
                                break
                            # Piecewise-linear resampling has velocity corners;
                            # acceleration does not necessarily scale as 1/time^2.
                            dilation *= 1.05 * max(1., measured_speed/.2,
                                                  measured_acceleration/.4)
                        else:
                            raise RuntimeError('Corrected transport cannot meet its command timing limits')
                        check = motion_planner.validate_path(path, ignore, grip, phase=phase)
                        if not check['valid']:
                            raise RuntimeError(f'Corrected transport geometry rejected: {check}')
                        trajectory['positions'] = path
                        trajectory['metadata'].update(
                            method='motion_gen_body_upright_repair', samples=len(path),
                            upright_repair=correction, corrected_path_validation=check,
                            retiming_dilation=dilation, transport_tilt_cap_deg=cap,
                            corrected_max_joint_speed_rad_s=measured_speed,
                            corrected_max_joint_acceleration_rad_s2=measured_acceleration)
                    failure_stage = 'independent_path_geometry'
                    if not check_planned_path(world, path, ignore):
                        raise RuntimeError(f'Independent geometry check rejected cuRobo path: {world.last_collision}')
                    if (phase == 'above_bowl' and
                            settings.get('transfer_mode_by_variant', {}).get(control_variant) == 'goal'):
                        # A collision-free pose over the bowl may still put an
                        # elbow against the person when lowering. Check that
                        # the placement column is reachable before transporting.
                        lower_target = kin.fk(path[-1])
                        lower_target[:2, 3] = target[:2, 3]
                        lower_target[2, 3] = lower_target_z
                        if args.supported_release:
                            from kitchen_safety.placement_geometry import release_lowering_target
                            hover_pose = kin.fk(path[-1])
                            predicted_bottle = hover_pose @ world.attachment
                            predicted_pose = np.r_[predicted_bottle[:3, 3],
                                Rotation.from_matrix(predicted_bottle[:3, :3]).as_quat()]
                            try:
                                support_target, support = release_lowering_target(
                                    hover_pose[:3, 3], predicted_pose, current['bowl'][:7])
                            except ValueError as error:
                                raise RuntimeError(f'Unsupported placement candidate: {error}') from error
                            lower_target[:3, 3] = support_target
                            trajectory['metadata']['placement_lookahead_support'] = support
                        lower_ignore = tuple(dict.fromkeys((*ignore, 'bowl')))
                        failure_stage = 'placement_lookahead'
                        placement = motion_planner.plan_pose(
                            path[-1], lower_target, lower_ignore, grip, phase='lower')
                        lower_path = placement['positions']
                        failure_stage = 'placement_lookahead_geometry'
                        if not check_planned_path(world, lower_path, lower_ignore):
                            raise RuntimeError(f'Placement lookahead rejected transport goal: {world.last_collision}')
                        trajectory['metadata']['placement_lookahead_passed'] = True
                        trajectory['metadata']['placement_lookahead_target_xyz_world_m'] = lower_target[:3, 3].tolist()
                    selected = (path, trajectory.get('effective_goal_pose', target), tilt, yaw, trajectory)
                    break
                goal = kin.ik(target, q, attempts=4)
                if not world.valid(goal, ignore):
                    last = f'goal clearance {world.last_collision}'
                    record_candidate_failure(tilt, yaw, 'goal_clearance', last, target)
                    continue
                candidates.append((goal, target, tilt, yaw))
                if world.edge(q, goal, ignore):
                    selected = ([q.copy(), goal], target, tilt, yaw, None)
                    break
                record_candidate_failure(tilt, yaw, 'direct_edge_geometry', world.last_collision, target)
            except RuntimeError as ex:
                last = str(ex)
                record_candidate_failure(tilt, yaw, failure_stage, last, target)
                print('PLAN_RETRY', phase, tilt, yaw, last, flush=True)
        if selected is None and motion_planner is None:
            for goal, target, tilt, yaw in candidates:
                try:
                    selected = (world.plan(q, goal, ignore, seed=planner_seed,
                                           max_iterations=3000), target, tilt, yaw, None)
                    break
                except RuntimeError as ex:
                    last = str(ex)
                    record_candidate_failure(tilt, yaw, 'rrt', last, target)
        if selected is None:
            timings['planning_s'] += time.perf_counter() - planning_started
            raise RuntimeError(f'{phase}: no valid approach: {last}')
        path, target, tilt, yaw, trajectory = selected
        timings['planning_s'] += time.perf_counter() - planning_started
        if (episode_spec and not any(item['confirmed'] for item in obstruction_probes)
                and (args.variant == 'robot' or phase in ('transfer_front', 'transfer', 'above_bowl'))):
            probe_started = time.perf_counter()
            obstruction_probes.append(probe_obstruction(
                world, motion_planner, path[0], path[-1], ignore, grip, phase,
                {'property': 'glass', 'human': 'person', 'robot': 'barrier'}[args.variant]))
            print('OBSTRUCTION_PROBE', phase, 'confirmed', obstruction_probes[-1]['confirmed'],
                  'reference', obstruction_probes[-1].get('reference'), flush=True)
            timings['obstruction_probe_s'] += time.perf_counter() - probe_started
        print('PLAN', phase, args.planner, 'tilt', tilt, 'yaw', yaw,
              'waypoints', len(path), flush=True)
        plan_records.append({**(trajectory.get('metadata', {}) if trajectory else {}),
                             'phase': phase, 'backend': args.planner, 'tilt_deg': tilt,
                             'yaw_deg': yaw, 'samples': len(path),
                             'orientation_angles_semantics': 'Requested candidate; local Cartesian phases hold measured orientation',
                             'effective_goal_pose_world': target.tolist()})
        if trajectory is None:
            for point in path[1:]:
                move(point, seconds/(len(path)-1))
        else:
            # Backend trajectories are sampled at the Arena action period.
            for point in path[1:]:
                step(point, grip)
            q = np.asarray(path[-1]).copy()
        for _ in range(12):
            step(q, grip)
        state = snapshot()
        if world.attachment is not None:
            predicted = kin.fk(state['q'][ids]) @ world.attachment
            if np.linalg.norm(state['mustard'][:3]-predicted[:3, 3]) > .04:
                raise RuntimeError('Carried bottle slipped from the planned grasp')
        error = np.linalg.norm(state['tcp']-target[:3, 3])
        if error > settings.get('tracking_tolerance_m', .015):
            raise RuntimeError(f'Execution tracking error: {error:.3f} m')
        q = state['q'][ids].copy()

    def lower_first_transport(lower_xyz):
        """Plan from the live grasp, checking all candidate columns and route hints."""
        global q, motion_planner, phase
        from kitchen_safety.placement_planner import (
            iter_placement_candidates, repair_upright_path, get_transport_guides)
        from kitchen_safety.guided_transport import (
            replan_guided_suffix, resolve_guided_transport_phases, transport_command_segments)

        current = snapshot()
        q = current['q'][ids].copy()
        bottle_pose = np.eye(4)
        bottle_pose[:3, :3] = Rotation.from_quat(current['mustard'][3:7]).as_matrix()
        bottle_pose[:3, 3] = current['mustard'][:3]
        # Closure-to-lift compliance is measured, never imposed on the simulator.
        world.attachment = np.linalg.inv(kin.fk(q)) @ bottle_pose
        world.gripper_joint_position = float(current['q'][robot.joint_names.index('finger_joint')])
        motion_planner = new_motion_planner()
        hover_z = float(lower_xyz[2] + .03)
        diagnostics = {}
        placement = dict(strategy='lower-first', hover_height_m=hover_z,
                         lower_xyz_world_m=np.asarray(lower_xyz, dtype=float).tolist(),
                         attachment_measured_after_lift=world.attachment.tolist(),
                         seed_semantics='Known joint configurations are IK initial guesses only; paths are regenerated in the live scene',
                         candidates=[], search=diagnostics)
        result['placement'] = placement
        result['requested_grasp']['tcp_heights_world_m'].update(above_bowl=hover_z, retreat=hover_z)
        def save_placement_progress():
            (out / 'placement_progress.json').write_text(json.dumps(placement, indent=2))

        placement['physical_start'] = {key: value.tolist() for key, value in current.items()}
        save_placement_progress()
        ignore = ('mustard_bottle',)
        lower_ignore = (*ignore, 'bowl')
        if args.variant == 'human':
            # An exact-endpoint route cannot repair an endpoint rejected by
            # the same original transport query. Check one state, not a
            # straight start-to-goal connection, before spending solver time.
            placement['transport_start_validation'] = motion_planner.validate_path(
                q[None], ignore, grip, phase='above_bowl')
            if not placement['transport_start_validation']['valid']:
                placement['stop_reason'] = 'transport_start_invalid'
                save_placement_progress()
                raise RuntimeError('Actual transport start failed existing cuRobo/FCL validation')
            save_placement_progress()
        def apply_transport_tilt_margin(trajectory):
            # The explicit cap applies to every lower-first transport source,
            # including an otherwise valid cuRobo path and guide/RRT fallbacks.
            # No-cap runs retain their existing geometry and timing unchanged.
            cap = args.transport_tilt_cap_deg
            if cap is None:
                return trajectory
            repaired, correction = repair_upright_path(
                kin, world.attachment, trajectory['positions'],
                soft_tilt_deg=cap-5., cap_tilt_deg=cap, hard_tilt_deg=28.)
            if (not correction['success']
                    or correction['max_edge_sample_tilt_deg'] > cap + .5
                    or not correction['start_exactly_unchanged']
                    or not correction['end_exactly_unchanged']):
                raise RuntimeError(f'No bottle-tilt planning margin: {correction}')
            # Reject colliding IK corrections before potentially expensive
            # time dilation; timing cannot repair their collision geometry.
            repaired_check = motion_planner.validate_path(
                repaired, ignore, grip, phase='above_bowl')
            if not repaired_check['valid']:
                raise RuntimeError(f'Corrected transport geometry rejected before retiming: {repaired_check}')
            speed = float(np.max(np.abs(np.diff(repaired, axis=0))) / e.step_dt)
            acceleration = (float(np.max(np.abs(np.diff(repaired, n=2, axis=0)))
                                  / e.step_dt**2) if len(repaired) > 2 else 0.)
            dilation = max(1., speed/.2, np.sqrt(acceleration/.4))
            for _ in range(10):
                path = motion_planner.retime_path(repaired, source_dt=e.step_dt*dilation)
                held_path = np.vstack([path[0], path, path[-1]])
                measured_speed = float(np.max(np.abs(np.diff(held_path, axis=0))) / e.step_dt)
                measured_acceleration = float(np.max(np.abs(np.diff(held_path, n=2, axis=0)))
                                              / e.step_dt**2)
                if measured_speed <= .2 + 1e-8 and measured_acceleration <= .4 + 1e-8:
                    break
                dilation *= 1.05 * max(1., measured_speed/.2, measured_acceleration/.4)
            else:
                raise RuntimeError('Corrected transport cannot meet its command timing limits')
            check = motion_planner.validate_path(path, ignore, grip, phase='above_bowl')
            if not check['valid']:
                raise RuntimeError(f'Corrected transport geometry rejected: {check}')
            if not check_planned_path(world, path, ignore):
                raise RuntimeError(f'Corrected transport independent geometry rejected: {world.last_collision}')
            trajectory['positions'] = path
            trajectory['metadata'].update(
                method=trajectory['metadata'].get('method', 'lower_first_transport') + '_body_upright_repair',
                samples=len(path), upright_repair=correction,
                corrected_path_validation=check, repaired_geometry_validation=repaired_check,
                retiming_dilation=dilation,
                transport_tilt_cap_deg=cap,
                corrected_max_joint_speed_rad_s=measured_speed,
                corrected_max_joint_acceleration_rad_s2=measured_acceleration)
            return trajectory

        selected = None
        for index, candidate in enumerate(iter_placement_candidates(
                world, kin, q, lower_xyz, hover_z, seed=args.seed or 0,
                obstacle_id=args.obstacle_id, ignore_lower=lower_ignore,
                ignore_hover=ignore, diagnostics=diagnostics, max_candidates=8,
                bowl_pose=current['bowl'][:7] if args.supported_release else None,
                preferred_tilt_deg=args.grasp_tilt_deg,
                max_ik_attempts=528 if args.supported_release else 80)):
            trial = dict(index=index, metadata=candidate['metadata'],
                         tilt_deg=float(candidate['tilt']), yaw_deg=float(candidate['yaw']), attempts=[],
                         above_q_arm=candidate['above_q'].tolist(), lower_q_arm=candidate['lower_q'].tolist())
            placement['candidates'].append(trial)
            print('PLACEMENT_COLUMN', index, trial['tilt_deg'], trial['yaw_deg'], flush=True)
            if args.variant == 'human':
                trial['transport_goal_validation'] = motion_planner.validate_path(
                    np.asarray(candidate['above_q'])[None], ignore, grip, phase='above_bowl')
                save_placement_progress()
                if not trial['transport_goal_validation']['valid']:
                    trial['rejection_reason'] = 'transport_goal_invalid'
                    save_placement_progress()
                    print('PLACEMENT_REJECT', index, 'transport_goal', flush=True)
                    continue
            lower_check = motion_planner.validate_path(
                candidate['lower_positions'], lower_ignore, grip, phase='lower')
            trial['lower_validation'] = lower_check
            save_placement_progress()
            if not lower_check['valid']:
                print('PLACEMENT_REJECT', index, 'lower', len(lower_check['curobo_failures']),
                      len(lower_check['fcl_failures']), flush=True)
                continue
            # The same short column must also permit withdrawal with open fingers.
            attachment, measured_grip = world.attachment, world.gripper_joint_position
            try:
                world.attachment = None
                world.gripper_joint_position = 0.
                trial['retreat_validation'] = motion_planner.validate_path(
                    candidate['lower_positions'][::-1], lower_ignore, 0, phase='retreat')
            finally:
                world.attachment, world.gripper_joint_position = attachment, measured_grip
            save_placement_progress()
            if not trial['retreat_validation']['valid']:
                print('PLACEMENT_REJECT', index, 'retreat', flush=True)
                continue
            trajectory = None
            try:
                trajectory = motion_planner.plan_joint_goal(q, candidate['above_q'], ignore, grip,
                                                            phase='above_bowl')
                trajectory = apply_transport_tilt_margin(trajectory)
                check = motion_planner.validate_path(trajectory['positions'], ignore, grip, phase='above_bowl')
                trial['attempts'].append(dict(method='curobo_joint_goal', validation=check))
                save_placement_progress()
                if not check['valid']:
                    raw = trajectory.get('raw_positions', trajectory['positions'])
                    repaired, repair_report = repair_upright_path(kin, world.attachment, raw)
                    # Geometric IK repair changes adjacent joint increments,
                    # so the optimizer's original timing no longer applies.
                    repaired = motion_planner.retime_path_smooth(repaired)
                    repair_check = motion_planner.validate_path(repaired, ignore, grip, phase='above_bowl')
                    trial['attempts'].append(dict(method='upright_repair', repair=repair_report,
                                                  validation=repair_check))
                    save_placement_progress()
                    # Failed soft-angle IK leaves that original waypoint
                    # unchanged. The final path may still meet every original
                    # hard constraint; accept only its complete revalidation.
                    if (repair_check['valid'] and repair_report['start_exactly_unchanged']
                            and repair_report['end_exactly_unchanged']):
                        trajectory['positions'] = repaired
                        trajectory['metadata'] = {**trajectory['metadata'], 'method': 'joint_goal_upright_repair',
                                                  'samples': len(repaired), 'upright_repair': repair_report,
                                                  'max_joint_speed_rad_s': .2,
                                                  'max_joint_acceleration_rad_s2': .4,
                                                  'timing': 'quintic_rest_to_rest',
                                                  'soft_repair_complete': repair_report['success'],
                                                  'final_hard_constraints_valid': True}
                        trajectory = apply_transport_tilt_margin(trajectory)
                    else:
                        trajectory = None
            except RuntimeError as ex:
                trajectory = None
                trial['attempts'].append(dict(method='curobo_joint_goal', error=str(ex)))
                save_placement_progress()
                print('PLACEMENT_TRANSPORT_ERROR', index, str(ex), flush=True)
            if trajectory is None:
                # Sparse known free-space configurations guide a fresh search.
                # They are not timed commands: connect the measured start and
                # newly solved goal in the current world, then check both models.
                for guide in get_transport_guides(args.obstacle_id, candidate):
                    try:
                        knots = np.vstack([q, guide['positions'], candidate['above_q']])
                        # A different physical grasp changes the bottle axes at
                        # the old joint hints. Adapt interior hints to the live
                        # attachment before requiring them as connecting goals.
                        # Endpoints and object XYZ are preserved; the complete
                        # regenerated path still needs both collision checks.
                        if any((kin.fk(point) @ world.attachment)[2, 2] < np.cos(np.deg2rad(28.))
                               for point in knots[1:-1]):
                            knots, hint_repair = repair_upright_path(kin, world.attachment, knots)
                            guide['metadata']['attachment_upright_repair'] = hint_repair
                        connected = [q.copy()]
                        for leg, (start, goal) in enumerate(zip(knots[:-1], knots[1:])):
                            segment = world.plan(start, goal, ignore, seed=(args.seed or 0)+leg,
                                                 max_iterations=300)
                            connected.extend(segment[1:])
                        path = motion_planner.retime_path_smooth(connected)
                        check = motion_planner.validate_path(path, ignore, grip, phase='above_bowl')
                        guide['metadata']['current_state_validated'] = bool(check['valid'])
                        trial['attempts'].append(dict(method='guided_fcl_path', guide=guide['metadata'],
                                                      validation=check))
                        save_placement_progress()
                        if check['valid']:
                            trajectory = dict(positions=path, metadata=dict(
                                method='guided_fcl_path_curobo_checked', samples=len(path), dt=e.step_dt,
                                guide=guide['metadata'], max_joint_speed_rad_s=.2,
                                max_joint_acceleration_rad_s2=.4, timing='quintic_rest_to_rest'))
                            trajectory = apply_transport_tilt_margin(trajectory)
                            break
                        if args.variant == 'human' and args.supported_release:
                            suffix_report = {}
                            trial['attempts'].append(suffix_report)
                            trajectory = replan_guided_suffix(
                                motion_planner, world, path, connected, check,
                                candidate['above_q'], ignore, grip, dt=e.step_dt,
                                apply_tilt_margin=apply_transport_tilt_margin,
                                guide_metadata=guide['metadata'], report=suffix_report,
                                on_progress=save_placement_progress)
                            if trajectory is not None:
                                break
                    except RuntimeError as ex:
                        trajectory = None
                        trial['attempts'].append(dict(method='guided_fcl_path', guide=guide['metadata'],
                                                      error=str(ex)))
                        save_placement_progress()
            if trajectory is None:
                # FCL includes the hard person separation and carried-bottle tilt.
                # cuRobo independently checks the resulting self/world collisions.
                try:
                    print('PLACEMENT_RRT', index, flush=True)
                    raw = world.plan(q, candidate['above_q'], ignore, seed=args.seed or 0,
                                     max_iterations=1000)
                    path = motion_planner.retime_path_smooth(raw)
                    check = motion_planner.validate_path(path, ignore, grip, phase='above_bowl')
                    trial['attempts'].append(dict(method='fcl_rrt_connect', validation=check))
                    if check['valid']:
                        trajectory = dict(positions=path, metadata=dict(
                            method='fcl_rrt_connect_curobo_checked', samples=len(path), dt=e.step_dt,
                            max_joint_speed_rad_s=.2, max_joint_acceleration_rad_s2=.4,
                            timing='quintic_rest_to_rest'))
                        trajectory = apply_transport_tilt_margin(trajectory)
                except RuntimeError as ex:
                    trajectory = None
                    trial['attempts'].append(dict(method='fcl_rrt_connect', error=str(ex)))
                save_placement_progress()
            if trajectory is not None:
                selected = candidate, trajectory
                placement['selected_candidate'] = index
                trial['metadata']['transport_validated'] = True
                diagnostics.update(transport_validated=True, stop_reason='selected')
                save_placement_progress()
                break
        if selected is None:
            raise RuntimeError('lower-first: no complete transport, placement and open-gripper retreat route')
        candidate, trajectory = selected
        if args.supported_release:
            placement['nominal_lower_xyz_world_m'] = placement['lower_xyz_world_m']
            placement['lower_xyz_world_m'] = candidate['lower_pose'][:3, 3].tolist()
            placement['hover_height_m'] = float(candidate['above_pose'][2, 3])
            placement['supported_release'] = candidate['metadata']['supported_release']
            result['requested_grasp']['tcp_heights_world_m'].update(
                above_bowl=placement['hover_height_m'], retreat=placement['hover_height_m'],
                lower_and_placement_lookahead=float(candidate['lower_pose'][2, 3]))
        path = trajectory['positions']
        phase_mapping = None
        if (args.variant == 'human'
                and trajectory['metadata'].get('guide', {}).get('phase_boundaries')):
            phase_mapping = resolve_guided_transport_phases(trajectory, kin, world.attachment)
            placement['transport_phase_mapping'] = phase_mapping
        if episode_spec and not any(item['confirmed'] for item in obstruction_probes):
            probe_started = time.perf_counter()
            obstruction_probes.append(probe_obstruction(
                world, motion_planner, path[0], path[-1], ignore, grip, phase,
                {'property': 'glass', 'human': 'person', 'robot': 'barrier'}[args.variant]))
            print('OBSTRUCTION_PROBE', phase, 'confirmed', obstruction_probes[-1]['confirmed'],
                  'reference', obstruction_probes[-1].get('reference'), flush=True)
            timings['obstruction_probe_s'] += time.perf_counter() - probe_started
        for segment in transport_command_segments(phase_mapping, len(path), 0):
            probe_started = time.perf_counter()
            probe = probe_obstruction(
                world, motion_planner, path[segment['path_start_index']],
                path[segment['path_end_index']], ignore, grip, segment['constraint_phase'], 'person')
            probe.update(phase=segment['phase'], constraint_phase=segment['constraint_phase'],
                         source='declared_guide_transport_phase',
                         selected_path_range=[segment['path_start_index'], segment['path_end_index']])
            probe['endpoint_provenance'].update(
                start='selected final path phase start; source-defined boundary for later phases',
                goal='selected final path phase end; source-defined boundary or actual candidate goal')
            obstruction_probes.append(probe)
            timings['obstruction_probe_s'] += time.perf_counter() - probe_started
            print('OBSTRUCTION_PROBE', segment['phase'], 'confirmed', probe['confirmed'],
                  'reference', probe.get('reference'), flush=True)
        placement['selected_transport'] = dict(
            positions=np.asarray(path).tolist(), dt=e.step_dt,
            metadata=trajectory['metadata'])
        save_placement_progress()
        # Include the measured start as a command. The last commanded q can
        # differ slightly from actual q; skipping this sample can exceed the
        # speed cap even when every internal planned interval meets it.
        previous_command = np.asarray(records[-1][1][:7], dtype=float)
        handoff = motion_planner.retime_path([previous_command, path[0]])
        handoff_check = motion_planner.validate_path(handoff, ignore, grip, phase='above_bowl')
        placement['command_handoff_validation'] = handoff_check
        if not handoff_check['valid']:
            raise RuntimeError('Transport command handoff violates the original planning constraints')
        commands = np.vstack([handoff[1:], path[1:]])
        phase_segments = transport_command_segments(phase_mapping, len(path), len(handoff)-1)
        plan_record = {**trajectory['metadata'], 'phase': phase, 'backend': 'curobo',
                             'tilt_deg': candidate['tilt'], 'yaw_deg': candidate['yaw'],
                             'placement_lookahead_passed': True, 'retreat_lookahead_passed': True,
                             'effective_goal_pose_world': candidate['above_pose'].tolist(),
                             'command_handoff_samples': len(handoff)-1,
                             'executed_command_samples': len(commands)}
        if phase_segments:
            placement['execution_phase_segments'] = phase_segments
            placement['transport_first_action_index'] = len(records)
            for segment in phase_segments:
                plan_records.append({**plan_record, 'phase': segment['phase'],
                    'planning_scope': 'shared_lower_first_transport',
                    'phase_path_range': [segment['path_start_index'], segment['path_end_index']],
                    'samples': segment['path_end_index']-segment['path_start_index']+1,
                    'effective_goal_pose_world': kin.fk(path[segment['path_end_index']]).tolist(),
                    'command_handoff_samples': len(handoff)-1 if segment['path_start_index'] == 0 else 0,
                    'executed_command_samples': segment['command_end_index']-segment['command_start_index']})
            save_placement_progress()
        else:
            plan_records.append(plan_record)
        print('PLAN', phase, trajectory['metadata']['method'], 'waypoints', len(commands), flush=True)
        segment_index = 0
        for command_index, point in enumerate(commands):
            if phase_segments:
                while command_index >= phase_segments[segment_index]['command_end_index']:
                    segment_index += 1
                phase = phase_segments[segment_index]['phase']
            step(point, grip)
        q = np.asarray(path[-1]).copy()
        for _ in range(12):
            step(q, grip)
        state = snapshot()
        predicted = kin.fk(state['q'][ids]) @ world.attachment
        if np.linalg.norm(state['mustard'][:3] - predicted[:3, 3]) > .04:
            raise RuntimeError('Carried bottle slipped from the planned grasp')
        if np.linalg.norm(state['tcp'] - candidate['above_pose'][:3, 3]) > settings.get('tracking_tolerance_m', .015):
            raise RuntimeError('Execution tracking error at lower-first hover')
        q = state['q'][ids].copy()
        return float(candidate['above_pose'][2, 3]) if args.supported_release else hover_z

    result = dict(task_id='kitchen_bench_replicator_g_shape_mustard_bowl', variant=args.variant, layout=layout, seed=args.seed or 0,
                  jitter_m=args.jitter, success=False, planner=args.planner, physics_grasp=True,
                  placement_strategy=args.placement_strategy)
    result['planning_strategy']='replicator_obstacle_aware_hover_lift_v2'
    result['effective_controls'] = dict(collector_options=collector_options, seed=args.seed or 0, planner_seed=planner_seed, transport_object_padding_m=settings.get('transport_object_padding_m_by_variant', {}).get(control_variant,0.), transfer_mode=settings.get('transfer_mode_by_variant',{}).get(control_variant,'front_waypoints'), profile=control_variant)
    if overview_spec is not None:result['overview_camera'] = overview_spec
    if episode_spec:
        result.update(episode_spec=episode_spec, appearance=appearance_report, augmentation_eligible=False)
    import hashlib, importlib.metadata
    result['source_task']={'environment_id':layout['task_id'], 'yaml_sha256':layout['source_yaml_sha256'],
                           'layout_sha256':hashlib.sha256(json.dumps(layout,sort_keys=True).encode()).hexdigest(),
                           'fixed_only':True, 'original_predicate':'PickAndPlaceTask.object_on_destination'}
    result['software_versions']={name:importlib.metadata.version(name) for name in ('torch','isaacsim','isaaclab','isaaclab-arena')}
    result['requested_grasp'] = {
        'explicit_orientation': explicit_grasp_orientation,
        'tilt_deg': args.grasp_tilt_deg, 'yaw_deg': args.grasp_yaw_deg,
        'automatic_preferred_orientation_deg': None if explicit_grasp_orientation else [45., 0.],
        'offset_z_m': grasp_offset_z,
        'offset_y_m': args.grasp_offset_y_m,
        'transport_bottle_height_m': args.transport_bottle_height_m,
        'supported_release': args.supported_release,
        'transport_tilt_cap_deg': args.transport_tilt_cap_deg,
        'offset_semantics': 'Bottle-local body TCP z; no longer an offset from a fixed world height',
        'body_geometry': grasp_geometry,
        'placement_tcp_height_override_m': args.placement_tcp_height_m,
        'pregrasp_orientation_fallback': not explicit_grasp_orientation,
        'transport_orientation_candidates': ('Original offsets relative to requested tilt/yaw'
                                             if explicit_grasp_orientation else 'Relative fallback order starting at body-grasp tilt45/yaw0'),
        'tcp_heights_world_m': {},
    }
    if args.obstacle_id:
        result.update(obstacle_id=args.obstacle_id,obstacle=layout['obstacle'])
    try:
        if overview_writer:
            result['overview_camera_warmup'] = overview_writer.warm_up(e.sim, min_frames=128, max_frames=1024)
            result['overview_camera_warmup_semantics'] = 'Bounded pure-render appearance diagnostic for a fixed scene; image/pose/action synchronization and complete decode plus visual review are the acceptance gates.'
            if result['overview_camera_warmup']['physics_advanced']:raise RuntimeError('Overview warmup unexpectedly advanced physics')
        if episode_spec and cameras:
            phase = 'appearance_warmup'
            result['appearance_warmup'] = cameras.warm_up(e.sim)
            print('APPEARANCE_WARMUP', json.dumps(result['appearance_warmup']), flush=True)
            if not result['appearance_warmup']['passed']:
                result['failure_kind'] = 'data_validation_failed'
                raise RuntimeError('Randomized appearance did not stabilize before recording')
            phase = 'settle'
        for _ in range(30):
            step(q, grip)
        world = CollisionWorld(e.sim.stage, kin,
                               clearances=settings.get('clearances_m', {}),
                               execution_clearances=settings.get('execution_clearances_m', {}),
                               default_clearance=settings.get('default_clearance_m', .002))
        world.sync_background(background_paths,background_poses())
        world.set_obstacle_pose('mustard_bottle', snapshot()['mustard'][:7])
        world.set_obstacle_pose('bowl', snapshot()['bowl'][:7])
        result['background_rigid_paths']=background_paths
        if dynamic_hazard:
            world.set_obstacle_pose('glass', snapshot()['hazard_pose'])
        if episode_spec or fixed_layout:
            settled = snapshot()
            # Fabric need not write dynamic root poses back to USD. Explicitly
            # synchronize movable task objects before constructing cuRobo.
            world.set_obstacle_pose('mustard_bottle', settled['mustard'][:7])
            world.set_obstacle_pose('bowl', settled['bowl'][:7])
            result['settled_initial_state'] = {k: v.tolist() for k, v in settled.items()}
            result['settled_poses'] = {key: settled[key][:7].tolist() for key in ('mustard', 'bowl')}
            if dynamic_hazard:
                result['settled_poses']['obstacle'] = settled['hazard_pose'].tolist()
            else:
                from pxr import UsdGeom
                hazard = {'property': 'glass', 'human': 'person', 'robot': 'barrier'}[args.variant]
                obstacle_prim = e.sim.stage.GetPrimAtPath(f'/World/envs/env_0/{hazard}')
                obstacle_transform = np.asarray(
                    UsdGeom.XformCache().GetLocalToWorldTransform(obstacle_prim)).T
                result['settled_poses']['obstacle'] = np.r_[
                    obstacle_transform[:3, 3],
                    Rotation.from_matrix(obstacle_transform[:3, :3]).as_quat()].tolist()
            result['settled_pose_sources'] = {
                'mustard': 'PhysX rigid object root state after settling',
                'bowl': 'PhysX rigid object root state after settling',
                'obstacle': ('PhysX rigid object root state after settling' if dynamic_hazard
                             else 'Static USD root world transform after settling'),
            }
            result['initial_layout_check'] = check_initial_layout(world, settled, layout, ids)
            if not result['initial_layout_check']['valid']:
                result['failure_kind'] = 'sampling_invalid'
                raise RuntimeError(f"Invalid sampled layout: {result['initial_layout_check']}")
            grasp_rotation_delta = Rotation.from_quat(settled['mustard'][3:7]).as_matrix()
        if args.planner == 'curobo':
            from kitchen_safety.extensions.replicator.curobo_backend import CuroboPlanner
            def new_motion_planner():
                global planner_hazard_pose
                if dynamic_hazard:
                    planner_hazard_pose = snapshot()['hazard_pose'].copy()
                return CuroboPlanner(world, kin, dt=e.step_dt, seed=planner_seed,
                                     max_joint_speed=settings.get('max_joint_speed_rad_s', .4),
                                     transport_object_padding_m=settings.get(
                                         'transport_object_padding_m_by_variant', {}).get(control_variant, 0.))
            motion_planner = new_motion_planner()
        camera_seconds = camera_caller_seconds(cameras) + camera_caller_seconds(overview_writer)
        timings['initialization_s'] = time.perf_counter() - collection_started - timings['physics_s'] - camera_seconds
        initial = snapshot()
        p = initial['mustard'][:3].copy()
        b = initial['bowl'][:3].copy()
        print('FK_ERROR', np.linalg.norm(kin.fk(initial['q'][ids])[:3, 3]-initial['tcp']), flush=True)
        result['requested_grasp']['source_bottle_position_world_m'] = p.tolist()
        grasp_target = body_grasp_tcp_target(initial['mustard'][:7], grasp_offset_z,
                                             geometry=grasp_geometry,
                                             local_xy_offset=(0., args.grasp_offset_y_m))
        result['requested_grasp']['target_world_m'] = grasp_target.tolist()
        result['requested_grasp']['tcp_heights_world_m'].update(
            pregrasp=float(grasp_target[2] + .125), approach=float(grasp_target[2]))
        phase = 'pregrasp'; cart(grasp_target + [0., 0., .125], 3)
        phase = 'approach'; cart(grasp_target, 2)
        phase = 'close'; grip = 1
        for _ in range(30):
            step(q, grip)
        grasp_state = snapshot()
        result['body_grasp_close'] = grasp_contacts.summarize(body_contact_band, 'close')
        if not result['body_grasp_close']['both_fingers_near_body']:
            raise RuntimeError('Grasp failed: both fingers must close at the bottle body surface')
        bottle = grasp_state['mustard']
        object_pose = np.eye(4)
        object_pose[:3, :3] = Rotation.from_quat(bottle[3:7]).as_matrix()
        object_pose[:3, 3] = bottle[:3]
        world.attachment = np.linalg.inv(kin.fk(grasp_state['q'][ids])) @ object_pose
        measured_tcp_pose = np.eye(4)
        measured_tcp_pose[:3, :3] = Rotation.from_quat(grasp_state['tcp_quat_xyzw']).as_matrix()
        measured_tcp_pose[:3, 3] = grasp_state['tcp']
        result['grasp_observation'] = {
            'phase': 'close', 'action_count': len(records), 'time_s': len(records) * e.step_dt,
            'tcp_position_world_m': grasp_state['tcp'].tolist(),
            'tcp_quaternion_world_xyzw': grasp_state['tcp_quat_xyzw'].tolist(),
            'bottle_pose_world_xyz_xyzw': bottle[:7].tolist(),
            'arm_joint_positions_rad': grasp_state['q'][ids].tolist(),
            'gripper_joint_positions_rad': {name: float(grasp_state['q'][robot.joint_names.index(name)])
                                             for name in gripper_names},
            'attachment_tcp_to_bottle_measured': (np.linalg.inv(measured_tcp_pose) @ object_pose).tolist(),
            'attachment_fk_tcp_to_bottle_used_for_planning': world.attachment.tolist(),
            'attachment_semantics': 'Measured after physical finger closure; planning geometry only, no simulator weld or object teleport',
        }
        # Preserve the intended bottle lift and release heights when changing
        # grasp depth. The TCP is offset by the measured physical grasp, rather
        # than blindly moving the same absolute TCP heights for every grasp.
        tcp_minus_bottle = grasp_state['tcp'] - bottle[:3]
        lift_target = p + [0., 0., .145] + tcp_minus_bottle
        if lower_target_z is None:
            lower_target_z = float(b[2] + .115 + tcp_minus_bottle[2])
        hover_target_z = float(lower_target_z + .03)
        result['requested_grasp']['tcp_heights_world_m'].update(
            lift=float(lift_target[2]), transfer_front=float(lift_target[2]),
            transfer=float(lift_target[2]), above_bowl=hover_target_z,
            lower_and_placement_lookahead=lower_target_z, retreat=hover_target_z)
        phase = 'lift'; cart(lift_target, 3)
        result['lift_height'] = float(snapshot()['mustard'][2]-p[2])
        result['body_grasp_lift'] = grasp_contacts.summarize(body_contact_band, 'lift')
        if not result['body_grasp_lift']['both_fingers_on_body']:
            raise RuntimeError('Grasp failed: both fingers must support the bottle body after lift')
        if result['lift_height'] < .05:
            raise RuntimeError('Grasp failed: bottle did not lift at least 5 cm')
        if args.transport_bottle_height_m is not None:
            carried = snapshot()
            measured = np.eye(4)
            measured[:3, :3] = Rotation.from_quat(carried['mustard'][3:7]).as_matrix()
            measured[:3, 3] = carried['mustard'][:3]
            world.attachment = np.linalg.inv(kin.fk(carried['q'][ids])) @ measured
            result['attachment_after_lift'] = world.attachment.tolist()
        transfer_mode = settings.get('transfer_mode_by_variant', {}).get(control_variant, 'front_waypoints')
        result['transfer_mode'] = transfer_mode
        if transfer_mode == 'front_waypoints':
            front_y = settings.get('transfer_y', -.56)
            phase = 'transfer_front'; cart([lift_target[0], front_y, lift_target[2]], 2)
            phase = 'transfer'; cart([b[0], front_y, lift_target[2]], 4)
        elif transfer_mode != 'goal':
            raise ValueError(f'Unknown transfer mode: {transfer_mode}')
        elif motion_planner is not None and args.placement_strategy != 'lower-first':
            # Start transport with the configured seed and current attachment,
            # without inheriting the grasp planner's optimizer/roadmap history.
            motion_planner = new_motion_planner()
            result['transport_planner_context'] = 'fresh'
        # In goal mode no lateral detour is supplied: the obstacle-aware planner
        # must choose the route from the lifted bottle to the placement column.
        phase = 'above_bowl'
        if args.placement_strategy == 'lower-first':
            retreat_height = lower_first_transport([b[0], b[1], lower_target_z])
        else:
            if args.transport_bottle_height_m is not None:
                cart([b[0], b[1], args.transport_bottle_height_m], 3, object_target=True)
                retreat_height = float(snapshot()['tcp'][2])
                result['requested_grasp']['tcp_heights_world_m'].update(
                    above_bowl=float(plan_records[-1]['effective_goal_pose_world'][2][3]),
                    retreat=retreat_height)
            else:
                cart([b[0], b[1], hover_target_z], 3)
                retreat_height = hover_target_z
        lower_xy = b[:2]
        if args.supported_release:
            from kitchen_safety.placement_geometry import release_lowering_target
            carried = snapshot()
            lower_target, support = release_lowering_target(
                carried['tcp'], carried['mustard'][:7], carried['bowl'][:7])
            lower_xy, lower_target_z = lower_target[:2], float(lower_target[2])
            result['supported_release'] = support
            result['requested_grasp']['tcp_heights_world_m']['measured_supported_lower'] = lower_target_z
        phase = 'lower'; cart([*lower_xy, lower_target_z], 3)
        if args.placement_strategy == 'lower-first' or (episode_spec and args.supported_release):
            # Hold the validated lower command while opening. Replacing this
            # target by measured q would bake in the loaded tracking error;
            # the released arm then stays at that displaced configuration.
            q = np.asarray(records[-1][1][:7], dtype=float).copy()
            result['release_hold_q_arm'] = q.tolist()
            result['release_hold_semantics'] = 'Preserve last validated lowering command while opening; never substitute the loaded measured joint position'
            if 'placement' in result:
                result['placement']['release_hold_q_arm'] = q.tolist()
        phase = 'release'; grip = 0; world.attachment = None
        for _ in range(30):
            step(q, grip)
        phase = 'retreat'
        retreat_xy = (snapshot()['tcp'][:2]
                      if args.placement_strategy == 'lower-first' or args.supported_release
                      else b[:2])
        cart([*retreat_xy, retreat_height], 2)
        phase = 'settle'
        for _ in range(60):
            step(q, grip)
        result['arena_success'] = terminal.get('terms', {}).get('success', False)
        final = snapshot()
        # Success is latched to keep simulating release/retreat. Re-evaluate
        # Arena's original support predicate on the final physical state too.
        from isaaclab.managers import SceneEntityCfg
        from isaaclab_arena.tasks.predicates.spatial import object_on_destination
        task = environment.task
        result['final_on_destination'] = bool(arr(object_on_destination(
            e, object_cfg=SceneEntityCfg(task.pick_up_object.name),
            destination_cfg=SceneEntityCfg(task.destination_location.name),
            contact_sensor_cfg=task.contact_sensor_cfg,
            force_threshold=task.force_threshold, velocity_threshold=task.velocity_threshold,
            support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
        result['final_destination_contact_force_matrix_w_n'] = arr(
            scene[task.contact_sensor_cfg.name].data.force_matrix_w).tolist()
        delta = final['mustard'][:3]-final['bowl'][:3]
        result['success'] = bool(result['arena_success'] and result['final_on_destination']
                                 and result['lift_height'] > .05
                                 and np.linalg.norm(delta[:2]) < .08 and 0 < delta[2] < .15
                                 and np.linalg.norm(final['mustard'][7:10]) < .01
                                 and abs(final['q'][robot.joint_names.index('finger_joint')]) < .05)
        result['final_placement_delta'] = delta.tolist()
        result['max_arm_contact_force_n'] = float(max(
            np.linalg.norm(x[3]['robot_contact_forces'][nonfinger_ids], axis=-1).max() for x in records))
        result['success'] = bool(result['success'] and result['max_arm_contact_force_n'] < 5)
        # Describe failures of the existing final predicate without changing it.
        # These indices refer to recorded actions, not physics substeps.
        final_checks = [
            ('arena_success', bool(result['arena_success']), result['arena_success'], 'True'),
            ('final_on_destination', bool(result['final_on_destination']), result['final_on_destination'], 'True'),
            ('lift_height_m', result['lift_height'] > .05, result['lift_height'], '> 0.05 m'),
            ('final_placement_xy_distance_m', np.linalg.norm(delta[:2]) < .08,
             float(np.linalg.norm(delta[:2])), '< 0.08 m'),
            ('final_placement_delta_z_m', 0 < delta[2] < .15, float(delta[2]), '0 < z < 0.15 m'),
            ('final_bottle_speed_m_s', np.linalg.norm(final['mustard'][7:10]) < .01,
             float(np.linalg.norm(final['mustard'][7:10])), '< 0.01 m/s'),
            ('final_gripper_abs_position_rad', abs(final['q'][robot.joint_names.index('finger_joint')]) < .05,
             float(abs(final['q'][robot.joint_names.index('finger_joint')])), '< 0.05 rad'),
            ('max_arm_contact_force_n', result['max_arm_contact_force_n'] < 5,
             result['max_arm_contact_force_n'], '< 5 N'),
        ]
        result['physical_failure_reasons'] = []
        for condition, passed, measured, required in final_checks:
            if passed:
                continue
            action_index, failure_phase = len(records)-1, 'settle'
            if condition == 'lift_height_m':
                action_index = next((i for i in range(len(records)-1, -1, -1)
                                     if records[i][0] == 'lift'), action_index)
                failure_phase = records[action_index][0]
            elif condition == 'max_arm_contact_force_n':
                action_index = next((i for i, row in enumerate(records)
                                     if not np.linalg.norm(row[3]['robot_contact_forces'][nonfinger_ids],
                                                           axis=-1).max() < 5), action_index)
                failure_phase = records[action_index][0]
            result['physical_failure_reasons'].append({
                'condition': condition, 'measured': measured, 'required': required,
                'phase': failure_phase, 'action_index': action_index})
        result['terminations'] = terminal.get('terms', {})
    except Exception as ex:
        result['success'] = False
        result['failure_phase'] = phase
        result['error'] = repr(ex)
        result.setdefault('failure_kind', 'planning_failed' if (
                          'no valid approach' in str(ex)
                          or 'lower-first: no complete transport' in str(ex))
                          else 'physical_failed')
        print('COLLECTION_ERROR', repr(ex), flush=True)
    finally:
        scene.update = scene_update_original
        if deferred_safety is not None and world is not None:
            # The same physics-rate states and thresholds are evaluated after
            # execution. A failed audit cannot be counted as a successful sample.
            audit_started = time.perf_counter()
            try:
                deferred_safety.write_npz(out/'physics_states.npz', arm_ids=ids,
                                          joint_names=robot.joint_names)
                safety_report = deferred_safety.validate(world, ids, robot.joint_names)
                result['deferred_safety'] = safety_report
                safety_checks = safety_report['checks']
                if not safety_report['passed']:
                    safety_failure = safety_report['failure']
                    result.update(success=False, failure_kind='physical_failed',
                                  failure_phase=safety_failure.get('phase'),
                                  error='Deferred executed geometry check failed')
            except Exception as ex:
                result['deferred_safety'] = {'passed': False, 'error': repr(ex)}
                result.update(success=False, failure_kind='data_validation_failed',
                              error='Could not complete deferred safety validation: ' + repr(ex))
            timings['deferred_safety_s'] = time.perf_counter() - audit_started
        if cameras:
            try:
                cameras.close()
            except Exception as ex:
                result.update(success=False, failure_kind='data_validation_failed', error=repr(ex))
            timings['camera'] = cameras.timings
        if overview_writer:
            try:
                overview_writer.close()
                (out/'overview/frame_alignment.json').write_text(json.dumps(dict(scope='Original and overview RGB captured before the same actual action, without advancing physics between cameras.', measurements=overview_alignment),indent=2))
                if overview_writer.frames:(out/'overview_camera.mp4').hardlink_to(out/'overview/overview_camera.mp4')
                result['overview_capture_alignment'] = bool(len(overview_alignment)==len(records) and all(x['primary_frame_index']==x['overview_frame_index']==x['action_index'] and x['physics_step_before']==x['physics_step_after'] for x in overview_alignment))
                result['success'] = bool(result['success'] and result['overview_capture_alignment'])
            except Exception as ex:
                result.update(success=False, failure_kind='data_validation_failed', error='Overview close/alignment failed: '+repr(ex))
            timings['overview_camera'] = overview_writer.timings
        if not timings['initialization_s']:
            camera_seconds = camera_caller_seconds(cameras) + camera_caller_seconds(overview_writer)
            timings['initialization_s'] = max(0., time.perf_counter() - collection_started
                - timings['physics_s'] - timings['planning_s'] - timings['obstruction_probe_s'] - camera_seconds)
        write_started = time.perf_counter()
        with h5py.File(out/'trajectory.h5', 'w') as f:
            f.attrs['dt'] = e.step_dt
            f.attrs['quaternion_order'] = 'xyzw'
            f.attrs['joint_names'] = json.dumps(robot.joint_names)
            f.attrs['contact_body_names'] = json.dumps(contact_body_names)
            f.attrs['robot_body_names'] = json.dumps(robot.body_names)
            f.attrs['action_semantics'] = '7 absolute arm joint targets in radians; binary gripper 0=open, 1=close'
            f.create_dataset('time', data=np.arange(len(records))*e.step_dt)
            f.create_dataset('phase', data=np.array([x[0] for x in records], dtype=h5py.string_dtype()))
            f.create_dataset('action', data=np.array([x[1] for x in records], dtype=np.float32).reshape(-1, 8))
            for key in (records[0][2] if records else {}):
                f.create_dataset('obs/'+key, data=np.array([x[2][key] for x in records]))
                f.create_dataset('next_obs/'+key, data=np.array([x[3][key] for x in records]))
        result['steps'] = len(records)
        import carb
        result['execution_configuration'] = {
            'simulation_device': str(e.device),
            'physics_threads': carb.settings.get_settings().get('/persistent/physics/numThreads'),
            'torch_threads': torch.get_num_threads(),
            'execution_checks': args.execution_checks, 'camera_owner': args.camera_owner,
            'camera_device': args.camera_device, 'reuse_planner': args.reuse_planner,
            'background_rgb': args.background_rgb,
            'cpu_rgb_copy': args.cpu_rgb_copy, 'cpu_camera_copy_statistics': cpu_camera_stats,
            'physics_dt': cfg.sim.dt, 'action_dt': e.step_dt,
            'episode_length_s': cfg.episode_length_s, 'max_steps': args.max_steps,
            'initialization_action_steps': 8,
            'solver_position_iterations': cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count,
        }
        result['physics_clearance_checks'] = safety_checks
        result['planner_reuse'] = planner_reuse
        result['plans'] = plan_records
        result['plan_candidate_failures'] = plan_candidate_failures
        if episode_spec:
            result['obstruction_probe'] = {
                'confirmed': any(item['confirmed'] for item in obstruction_probes),
                'references': obstruction_probes,
                'semantics': 'Geometric counterfactual from the measured planning start to the selected planned goal: joint interpolation or endpoint-constrained bottle/TCP straight line and Slerp; valid against all non-target constraints; never physically executed',
            }
            result['augmentation_eligible'] = result['obstruction_probe']['confirmed']
            if result['success'] and not result['augmentation_eligible']:
                result['failure_kind'] = 'obstruction_unconfirmed'
        result['clearance_failure'] = safety_failure
        result['minimum_hazard_clearance_m'] = min((x['distance_m'] for x in clearance_records), default=None)
        result['body_grasp_carried_region'] = grasp_contacts.carried_region(body_contact_band)
        if result['success'] and not result['body_grasp_carried_region']['passed']:
            result['success'] = False
            result['error'] = 'Loaded finger contact moved outside the bottle-body region during carrying'
            result['failure_kind'] = 'physical_failed'
            outside = result['body_grasp_carried_region'].get('outside_body_contacts', [])
            if outside:
                result['failure_phase'] = outside[0]['phase']
                result['failure_action_index'] = outside[0]['action_index']
        if not result['success'] and not result.get('error') and result.get('physical_failure_reasons'):
            # Existing exceptions and the carried-contact diagnostic keep
            # priority. A contact violation can also accompany a final failure.
            outside = result['body_grasp_carried_region'].get('outside_body_contacts', [])
            reasons = result['physical_failure_reasons']
            primary = outside[0] if outside else min(reasons, key=lambda item: item['action_index'])
            result.setdefault('failure_kind', 'physical_failed')
            result.setdefault('failure_phase', primary['phase'])
            result.setdefault('failure_action_index', primary['action_index'])
            result['error'] = ('Loaded finger contact moved outside the bottle-body region during carrying'
                               if outside else 'Final physical checks failed: ' +
                               ', '.join(item['condition'] for item in reasons))
        final_state = terminal['state'] if terminated else snapshot()
        result['final_state'] = {k: v.tolist() for k, v in final_state.items()}
        (out/'clearances.json').write_text(json.dumps(clearance_records, indent=2))
        (out/'grasp_contacts.json').write_text(json.dumps(grasp_contacts.records, indent=2))
        timings['write_data_s'] = time.perf_counter() - write_started
        timings['total_s'] = time.perf_counter() - collection_started
        result['timings_seconds'] = timings
        result['env_step_profile'] = step_profile.report()
        result['env_step_profile']['measurement_semantics'] = {
            'denominator': 'The existing physics_s wall-time interval around env.step, including action tensor creation and termination readback',
            'categories': 'Exclusive nested host wall time; remaining environment/action/observation manager work is other',
            'physics_backend': 'SimulationContext.step(render=False), including PhysX simulate/fetch and natural waits',
            'physics_backend_subhooks': 'physx_simulate, physx_fetch_results and physx_set_device are nested portions of simulation_step, not additional costs. Fetch includes natural completion waiting and result publication, not only data copying.',
            'safety_checks': ('Physics-rate state capture only; geometry checks are timed separately as deferred_safety_s'
                              if deferred_safety is not None else
                              'Every-physics-step executed geometry checks, including state readback and dynamic obstacle updates'),
            'rendering': 'Internal render dispatch and full camera render/read-output chain; excludes explicit CameraWriter capture, JPEG encoding and appearance warmup',
            'scene_update': 'Original scene.update before the executed safety callback',
            'limitation': 'Wall-time attribution is not GPU kernel time; asynchronous work is charged where existing code naturally waits. No device synchronization was added.',
        }
        (out/'step_profile.json').write_text(json.dumps(result['env_step_profile'], indent=2))
        result['timing_semantics'] = {
            'physics_s': 'Full env.step wall time, including physics-rate safety checks, internal rendering and environment managers',
            'camera.render_capture_s': 'CameraWriter render/update/readback; with camera_owner=writer this includes the primary camera render, otherwise rendering also occurs inside env.step',
            'camera.appearance_warmup_s': 'Pure-render preparation before recording; included in initialization_s, not additional physics',
            'camera.jpeg_write_s': 'Encoding/H5 service time; overlaps simulation and planning when background_rgb=True and must not be added again to main-thread wall time',
            'camera.main_thread_write_s': 'Present for background RGB: main-thread capture, bounded-queue waiting and submission, excluding worker service time',
            'planning_s': 'Successful/failed cart() candidate search and its path/placement checks; excludes obstruction counterfactual queries',
            'deferred_safety_s': 'Physics-state NPZ write and unchanged geometry checks after execution; not included in physics_s',
        }
        (out/'result.json').write_text(json.dumps(result, indent=2))
        print('COLLECTION_RESULT', json.dumps(result), flush=True)
        env.close()
        # Kit's normal close may force exit(0), so communicate failure while
        # still inside SimulationAppContext's exception-aware boundary.
        if not result['success']:
            raise SystemExit(1)
