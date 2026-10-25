"""Observe new Replicator source tasks without robot manipulation or state writes.

Original camera streams remain unchanged. Task objects keep their real names;
the sink is observed as its source static ObjectReference, never a fake body.
Every post-reset physics substep is recorded, including the initial state.
"""
import json
import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

TASK_CAMERAS = {
    'kitchen_bench_replicator_g_shape_mustard_bowl': ((1., -2.3, 1.5), (2.4, -1.4, .85)),
    'kitchen_bench_replicator_g_shape_banana_bowl': ((-.8, .3, 2.2), (-2.05, 2.1, .95)),
    'kitchen_bench_replicator_g_shape_bowl_sink': ((.6, -1.4, 2.2), (2.6, .05, .9)),
}
MUSTARD_TASK = 'kitchen_bench_replicator_g_shape_mustard_bowl'
ROOT = Path(__file__).resolve().parents[3]


def environment_route(task_id):
    if task_id not in TASK_CAMERAS:
        raise ValueError('Unsupported neutral source task: ' + task_id)
    return 'KitchenSafetyEnvironment' if task_id == MUSTARD_TASK else 'AssignedTaskEnvironment'


def mustard_evidence_layout(layout):
    """Normalize evidence names only; pass the untouched layout to its factory."""
    if layout['task_id'] != MUSTARD_TASK or layout.get('obstacle'):
        raise ValueError('Require the original no-hazard G-mustard base layout')
    result = copy.deepcopy(layout)
    result['objects'] = {
        'mustard_bottle': dict(usd_path='assets/robolab/objects/hope/mustard_bottle.usd',
            position=layout['mustard_position'], yaw_deg=layout.get('mustard_yaw_deg', 0.)),
        'bowl': dict(usd_path='assets/robolab/objects/ycb/bowl.usd',
            position=layout['bowl_position'], yaw_deg=layout.get('bowl_yaw_deg', 0.))}
    return result


def build_environment(args):
    route = environment_route(args.task_id)
    if route == 'KitchenSafetyEnvironment':
        from kitchen_safety.extensions.replicator.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
        from scripts.extensions.replicator.audit_task_source_relation import check_layout, source
        layout = json.loads((ROOT/'configs/extensions/replicator/g_shape_mustard.json').read_text())
        normalized = mustard_evidence_layout(layout)
        audit = check_layout(normalized)
        if not audit['initial_source_relations_passed']:
            raise ValueError('Original G-mustard source relations failed')
        _, original = source(args.task_id)
        environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(variant='base', enable_cameras=True,
            collection_seed=42, fixed_layout=layout, episode_length_s=max(args.episode_length_seconds, args.duration_seconds+30.)))
        environment.resolved_task_layout = normalized
        environment.original_collector_layout = layout
        environment.source_task_spec, environment.source_relation_audit = original, audit
        return environment
    from kitchen_safety.extensions.replicator.task_environment import AssignedTaskCfg, AssignedTaskEnvironment
    return AssignedTaskEnvironment().build(AssignedTaskCfg(task_id=args.task_id, variant='base', enable_cameras=True,
        episode_length_s=max(args.episode_length_seconds, args.duration_seconds+30.)))


def initial_pose_audit(body_poses, body_names, object_states, object_names, layout):
    expected = {'panda_link0': (np.asarray(layout['robot_position'])+[0, 0, layout['stand_height']], layout['robot_yaw_deg'])}
    expected.update({name: (item['position'], item['yaw_deg']) for name, item in layout['objects'].items()})
    rows = {}
    for name, (position, yaw) in expected.items():
        pose = body_poses[body_names.index(name)] if name == 'panda_link0' else object_states[object_names.index(name)]
        translation = float(np.linalg.norm(pose[:3]-position))
        rotation = float((Rotation.from_euler('z', yaw, degrees=True).inv()*Rotation.from_quat(pose[3:7])).magnitude())
        rows[name] = dict(position_error_m=translation, rotation_error_rad=rotation,
                          passed=translation <= 1e-5 and rotation <= 1e-5)
    return dict(passed=all(row['passed'] for row in rows.values()), objects=rows,
                scope='Actual post-reset robot base and named task object root poses compared with the unchanged authored source layout')


def motion(values, start=0):
    poses = np.asarray(values)[start:]
    if not len(poses):
        raise ValueError('Missing predeclared stability interval')
    angles = np.rad2deg((Rotation.from_quat(poses[:, 3:7]) * Rotation.from_quat(poses[0, 3:7]).inv()).magnitude())
    return dict(max_translation_m=float(np.linalg.norm(poses[:, :3]-poses[0, :3], axis=1).max()),
                max_rotation_deg=float(angles.max()))


def audit(data, info):
    """CPU-only numeric gates; semantic visibility still requires image review."""
    steps = np.asarray(data['physics_step'])
    n = len(steps)
    if n < 2 or not np.isfinite(info['physics_dt']) or info['physics_dt'] <= 0:
        raise ValueError('Missing initial state/substeps or invalid physics dt')
    objects, backgrounds = info['task_object_names'], info['background_paths']
    if not objects or len(objects) != len(set(objects)) or not backgrounds:
        raise ValueError('Missing or duplicate task/body names')
    shapes = dict(q=(n, len(info['joint_names'])), dq=(n, len(info['joint_names'])),
        robot_body_pose_w=(n, len(info['robot_body_names']), 7),
        task_object_states=(n, len(objects), 13), background=(n, len(backgrounds), 7),
        background_velocity=(n, len(backgrounds), 6),
        static_anchor_transforms=(n, len(info['static_anchor_paths']), 4, 4),
        task_reference_transforms=(n, len(info['task_reference_paths']), 4, 4))
    for key, shape in shapes.items():
        if np.asarray(data[key]).shape != shape or not np.isfinite(data[key]).all():
            raise ValueError('Missing/nonfinite/misaligned native evidence: ' + key)
    elapsed = (steps - steps[0]) * info['physics_dt']
    task_indices = np.flatnonzero(elapsed >= info['task_settling_seconds']-1e-8)
    free_indices = np.flatnonzero(elapsed >= info['background_settling_seconds']-1e-8)
    if not len(task_indices) or not len(free_indices):
        raise ValueError('Recording does not reach predeclared settling windows')
    task_rows = {name: motion(data['task_object_states'][:, i], int(task_indices[0])) for i, name in enumerate(objects)}
    linked = set(info['background_topology']['joint_connected_native_paths'])
    body_rows = []
    for i, path in enumerate(backgrounds):
        start = 0 if path in linked else int(free_indices[0])
        change = motion(data['background'][:, i], start)
        body_rows.append(dict(path=path, joint_connected=path in linked, reference_time_s=float(elapsed[start]), **change,
            passed=change['max_translation_m'] <= .002 and change['max_rotation_deg'] <= 1.))
    static_change = float(np.abs(data['static_anchor_transforms']-data['static_anchor_transforms'][0]).max()) if len(info['static_anchor_paths']) else 0.
    reference_change = float(np.abs(data['task_reference_transforms']-data['task_reference_transforms'][0]).max()) if len(info['task_reference_paths']) else 0.
    camera_steps = np.asarray(data['capture_physics_steps'])
    expected_count = info['frames'] * info['decimation'] + 1
    camera_ok = camera_steps.shape == (info['frames'], 2) and np.all(camera_steps[:, 0] == camera_steps[:, 1]) and \
        np.array_equal(camera_steps[:, 0], info['reset_physics_step'] + np.arange(info['frames']) * info['decimation'])
    checks = dict(
        full_post_reset_physics_coverage=n == expected_count and int(steps[0]) == info['reset_physics_step'] and
            np.all(np.diff(steps) == 1) and int(steps[-1]) == info['reset_physics_step'] + expected_count - 1,
        requested_observation_duration=elapsed[-1] >= info['requested_duration_seconds']-1e-8,
        no_resets_or_callback_failures=not info.get('runtime_error') and not info['callback_errors'],
        original_source_relations=info['source_relation_audit']['initial_source_relations_passed'] is True and
            len(info['source_relation_audit']['checks']) == info['expected_source_relations'] and
            all(row['passed'] is True for row in info['source_relation_audit']['checks']),
        recorded_initial_source_poses=info['recorded_initial_source_poses']['passed'] is True,
        robot_neutral_hold=float(np.abs(data['q']-data['q'][0]).max()) <= .001,
        task_objects_stable=all(row['max_translation_m'] <= .001 and row['max_rotation_deg'] <= 1. for row in task_rows.values()),
        background_joint_coverage=bool(linked) and linked <= set(backgrounds) and not info['background_topology']['unobserved_joint_connected_bodies'],
        all_background_stable=all(row['passed'] for row in body_rows),
        legal_static_anchors=all(row['source_initial_max_matrix_difference'] <= 1e-6 and not row['animated_source_ancestors']
            and not row['animated_runtime_ancestors'] and row['no_rigid_body_ancestor'] for row in info['static_anchors'].values()),
        static_anchors_stable=static_change <= 1e-6,
        legal_task_references=all(row['source_initial_max_matrix_difference'] <= 1e-6 and not row['animated_source_ancestors']
            and not row['animated_runtime_ancestors'] and row['no_rigid_body_ancestor'] for row in info['task_references'].values()),
        task_references_stable=reference_change <= 1e-6,
        camera_capture_alignment=bool(camera_ok))
    checks = {key: bool(value) for key, value in checks.items()}
    return dict(passed=all(checks.values()), checks=checks, observed_physics_samples=n, expected_physics_samples=expected_count,
        observed_duration_seconds=float(elapsed[-1]), task_objects=task_rows, background_results=body_rows,
        robot_max_joint_change_rad=float(np.abs(data['q']-data['q'][0]).max()),
        static_anchor_max_matrix_difference=static_change, task_reference_max_matrix_difference=reference_change)


def run(args):
    import torch
    from pxr import Usd, UsdGeom, UsdPhysics
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.dataset import CameraWriter, decode_video
    from kitchen_safety.extensions.replicator.overview import configure_overview
    from kitchen_safety.extensions.replicator.startup_recorder import array, background_topology, sha

    if args.torch_threads is not None:
        torch.set_num_threads(args.torch_threads)
    if args.physics_threads is not None:
        import carb
        import omni.physx
        from omni.physx.bindings._physx import SETTING_NUM_THREADS
        carb.settings.get_settings().set_int(SETTING_NUM_THREADS, args.physics_threads)
        omni.physx.get_physx_interface().set_thread_count(args.physics_threads)
    envdef = build_environment(args)
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42, device=args.device,
        presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    eye, target = TASK_CAMERAS[args.task_id]
    overview = configure_overview(cfg.scene, eye=args.overview_eye or eye, target=args.overview_target or target)
    env = builder.make_registered(cfg, kwargs)
    writers, original_update = [], None
    rows = {key: [] for key in ['physics_step', 'q', 'dq', 'robot_body_pose_w', 'task_object_states', 'background', 'background_velocity',
            'robot_contact_force', 'static_anchor_transforms', 'task_reference_transforms', 'capture_physics_steps']}
    info = dict(schema_version=2, task_id=args.task_id, passed=False, callback_errors=[], overview=overview,
        requested_duration_seconds=args.duration_seconds, background_settling_seconds=args.background_settling_seconds,
        environment_factory=environment_route(args.task_id), seed=42,
        collector_source_layout=getattr(envdef, 'original_collector_layout', envdef.resolved_task_layout),
        task_settling_seconds=2., source_relation_audit=envdef.source_relation_audit,
        expected_source_relations=len(envdef.source_task_spec['relations']),
        background_physics=envdef.background_physics_provenance, layout=envdef.resolved_task_layout,
        control='Hold initial arm command with open gripper; no runtime object/joint state setters.',
        predeclared_tolerances=dict(robot_joint_rad=.001, task_translation_m=.001, task_rotation_deg=1.,
            background_translation_m=.002, background_rotation_deg=1., static_matrix=1e-6),
        scope='All physics substeps after explicit reset through the requested neutral observation. Task identities are preserved. '
              'Joint-connected background bodies from reset; free background after 5s; task objects after 2s. '
              'Original cameras plus a candidate overview; visibility requires separate image inspection. No task success claim.')
    try:
        env.reset()
        e, robot = env.unwrapped, env.unwrapped.scene['robot']
        reset_step = int(e.sim.get_physics_step_count())
        if args.cpu_rgb_copy:
            from kitchen_safety.cpu_camera_copy import install_cpu_rgb_copy
            renderers = {}
            for name in list(envdef.embodiment.camera_config.camera_names())+['overview_camera']:
                renderer = e.scene[name]._renderer
                if id(renderer) not in renderers:
                    renderers[id(renderer)] = install_cpu_rgb_copy(renderer)
            info['cpu_camera_copy_statistics'] = list(renderers.values())
        import carb
        info['execution_configuration'] = dict(simulation_device=str(e.device), physics_threads=carb.settings.get_settings().get('/persistent/physics/numThreads'),
            torch_threads=torch.get_num_threads(), camera_owner='writer', camera_device=None, cpu_rgb_copy=args.cpu_rgb_copy,
            physics_dt=float(e.physics_dt), action_dt=float(e.step_dt), episode_length_s=max(args.episode_length_seconds, args.duration_seconds+30.))
        ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
        command = torch.tensor(np.r_[array(robot.data.joint_pos)[0, ids], 0.][None], dtype=torch.float32, device=e.device)
        layout, original = envdef.resolved_task_layout, envdef.source_task_spec
        source_path = Path(layout['kitchen_usd']).resolve()
        source = Usd.Stage.Open(str(source_path))
        source_root = source.GetDefaultPrim()
        if not source_root:
            raise RuntimeError('Source kitchen has no default prim')
        live_root = '/World/envs/env_0/kitchen'
        view = e.scene['robot_contacts']._physics_sim_view.create_rigid_body_view(live_root + '/**')
        topology = background_topology(e.sim.stage, view)

        def animated(prim):
            paths = []
            while prim and not prim.IsPseudoRoot():
                xf = UsdGeom.Xformable(prim)
                if xf and xf.TransformMightBeTimeVarying():
                    paths.append(str(prim.GetPath()))
                prim = prim.GetParent()
            return paths

        def transform(prim):
            return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T

        def static_evidence(live_path):
            prim = e.sim.stage.GetPrimAtPath(live_path)
            if not prim:
                raise ValueError('Missing source static reference/anchor: ' + live_path)
            source_prim_path = str(source_root.GetPath()) + live_path[len(live_root):]
            source_prim = source.GetPrimAtPath(source_prim_path)
            if not source_prim:
                raise ValueError('Static reference/anchor has no source counterpart: ' + source_prim_path)
            ancestor, has_rigid = prim, False
            while ancestor and not ancestor.IsPseudoRoot():
                has_rigid |= ancestor.HasAPI(UsdPhysics.RigidBodyAPI)
                ancestor = ancestor.GetParent()
            source_matrix, live_matrix = transform(source_prim), transform(prim)
            return dict(source_path=source_prim_path, initial_world_transform=live_matrix.tolist(),
                source_world_transform=source_matrix.tolist(), no_rigid_body_ancestor=not has_rigid,
                source_initial_max_matrix_difference=float(np.abs(source_matrix-live_matrix).max()),
                animated_source_ancestors=animated(source_prim), animated_runtime_ancestors=animated(prim))

        anchors = {path: static_evidence(path) for path in topology['static_anchor_paths']}
        references = {item['id']: live_root + '/' + item['prim_path'].lstrip('/') for item in original['object_references']}
        reference_evidence = {name: dict(path=path, **static_evidence(path)) for name, path in references.items()}
        objects = list(layout['objects'])
        physics_dt, action_dt = float(e.physics_dt), float(e.step_dt)
        decimation = round(action_dt / physics_dt)
        if abs(decimation * physics_dt-action_dt) > 1e-9:
            raise ValueError('Nonintegral action/physics sampling ratio')
        info.update(reset_physics_step=reset_step, dt=action_dt, physics_dt=physics_dt, decimation=decimation,
            frames=int(np.ceil(args.duration_seconds/action_dt)), joint_names=list(robot.joint_names), robot_body_names=list(robot.body_names),
            task_object_names=objects, task_reference_names=list(references), task_reference_paths=list(references.values()),
            task_references=reference_evidence, background_paths=list(view.prim_paths), background_topology=topology,
            static_anchor_paths=list(anchors), static_anchors=anchors,
            source_kitchen=dict(path=str(source_path), sha256=sha(source_path)),
            object_sources={name: dict(path=item['usd_path'], sha256=sha(item['usd_path'])) for name, item in layout['objects'].items()})

        def capture():
            cache = UsdGeom.XformCache()
            rows['physics_step'].append(int(e.sim.get_physics_step_count()))
            rows['q'].append(array(robot.data.joint_pos)[0])
            rows['dq'].append(array(robot.data.joint_vel)[0])
            rows['robot_body_pose_w'].append(np.concatenate([array(robot.data.body_pos_w)[0], array(robot.data.body_quat_w)[0]], axis=-1))
            rows['task_object_states'].append(np.stack([array(e.scene[name].data.root_state_w)[0] for name in objects]))
            rows['background'].append(array(view.get_transforms()))
            rows['background_velocity'].append(array(view.get_velocities()))
            rows['robot_contact_force'].append(array(e.scene['robot_contacts'].data.net_forces_w)[0])
            for key, paths in [('static_anchor_transforms', list(anchors)), ('task_reference_transforms', list(references.values()))]:
                rows[key].append(np.asarray([np.asarray(cache.GetLocalToWorldTransform(e.sim.stage.GetPrimAtPath(path))).T
                    for path in paths]).reshape(-1, 4, 4))

        capture()
        info['recorded_initial_source_poses'] = initial_pose_audit(rows['robot_body_pose_w'][0], info['robot_body_names'],
            rows['task_object_states'][0], objects, layout)
        if rows['physics_step'][0] != reset_step:
            raise RuntimeError('Observation setup advanced physics after reset')
        original_update = e.scene.update

        def update(dt, *positional, **keywords):
            value = original_update(dt, *positional, **keywords)
            if int(e.sim.get_physics_step_count()) != rows['physics_step'][-1]:
                try:
                    capture()
                except Exception as error:
                    info['callback_errors'].append(repr(error))
                    raise
            return value

        e.scene.update = update
        original_cameras = envdef.embodiment.camera_config.camera_names()
        info['original_camera_names'] = list(original_cameras)
        writers.append(CameraWriter(args.output, e.scene, original_cameras, action_dt, previews=True))
        (args.output / 'overview').mkdir()
        writers.append(CameraWriter(args.output/'overview', e.scene, ['overview_camera'], action_dt, previews=True))
        for _ in range(32):
            e.sim.render()
        if int(e.sim.get_physics_step_count()) != reset_step:
            raise RuntimeError('Pure-render camera initialization advanced physics')
        for index in range(info['frames']):
            before = int(e.sim.get_physics_step_count())
            for writer in writers:
                writer.write('neutral_hold')
            after = int(e.sim.get_physics_step_count())
            rows['capture_physics_steps'].append([before, after])
            if before != after:
                raise RuntimeError('Camera capture advanced physics')
            _, _, done, truncated, _ = env.step(command)
            if bool(array(done)[0]) or bool(array(truncated)[0]):
                raise RuntimeError('Neutral observation terminated or reset; reject this diagnostic')
    except Exception as error:
        info['runtime_error'] = f'{type(error).__name__}: {error}'
    finally:
        for writer in writers:
            try:
                writer.close()
            except Exception as error:
                info['runtime_error'] = f'Video close failed: {error}'
        if original_update is not None:
            env.unwrapped.scene.update = original_update
        try:
            env.close()
        except Exception as error:
            info['runtime_error'] = f'Environment close failed: {error}'
    arrays = {key: np.asarray(value) for key, value in rows.items()}
    np.savez_compressed(args.output/'neutral_states.npz', **arrays)
    try:
        info.update(audit(arrays, info))
        decoded = {}
        for relative, size in [(name + '.mp4', (1280, 720)) for name in info['original_camera_names']] + [
                ('all_cameras.mp4', (1920, 360)), ('overview/overview_camera.mp4', (1280, 720)), ('overview/all_cameras.mp4', (640, 360))]:
            decoded[relative] = dict(frames=decode_video(args.output/relative, info['frames'], expected_size=size, expected_dt=info['dt']),
                                    sha256=sha(args.output/relative))
        info['video_validation'] = decoded
        info['checks']['all_six_videos_fully_decoded'] = len(decoded) == 6
        info['passed'] = all(info['checks'].values())
    except Exception as error:
        info.update(passed=False, audit_error=f'{type(error).__name__}: {error}')
    info['input_sha256'] = {'neutral_states.npz': sha(args.output/'neutral_states.npz'),
        str(Path(__file__).resolve()): sha(__file__)}
    info['visual_review'] = 'Pending original-camera and candidate-overview inspection; numeric/video decode pass alone does not certify visibility.'
    (args.output/'neutral_probe.json').write_text(json.dumps(info, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: value for key, value in info.items() if key in ['task_id', 'passed', 'checks', 'runtime_error', 'audit_error',
        'observed_physics_samples', 'observed_duration_seconds', 'task_objects']}, indent=2))
    return 0 if info['passed'] else 2


def require_successful_report(output, return_code):
    """Raise *inside* Kit context: its normal shutdown can force exit status 0."""
    report = json.loads((Path(output)/'neutral_probe.json').read_text())
    if return_code != 0 or report.get('passed') is not True:
        raise RuntimeError('Neutral task probe failed; retained evidence: ' + str(Path(output)/'neutral_probe.json'))


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--task-id', choices=sorted(TASK_CAMERAS), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-seconds', type=float, default=45.)
    parser.add_argument('--episode-length-seconds', type=float, default=180.)
    parser.add_argument('--physics-threads', type=int, default=None)
    parser.add_argument('--torch-threads', type=int, default=None)
    parser.add_argument('--cpu-rgb-copy', action='store_true')
    parser.add_argument('--background-settling-seconds', type=float, default=5.)
    parser.add_argument('--overview-eye', type=float, nargs=3)
    parser.add_argument('--overview-target', type=float, nargs=3)
    args = parser.parse_args()
    if not np.isfinite(args.episode_length_seconds) or args.episode_length_seconds < args.duration_seconds or \
            (args.physics_threads is not None and args.physics_threads < 0) or (args.torch_threads is not None and args.torch_threads < 1):
        parser.error('Require a valid task timeout, nonnegative PhysX worker count and positive torch worker count')
    if not np.isfinite([args.duration_seconds, args.background_settling_seconds]).all() or not args.background_settling_seconds == 5. < args.duration_seconds:
        parser.error('Require finite duration > 5 seconds and preserve the original 5-second background settling window')
    args.enable_cameras = True
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        with SimulationAppContext(args):
            try:
                code = run(args)
                # SimulationAppContext may os._exit(0) on normal exit. Raising here
                # invokes its error branch (nonzero), including failed numeric gates.
                require_successful_report(args.output, code)
            except Exception as error:
                report = args.output/'neutral_probe.json'
                if not report.exists():
                    report.write_text(json.dumps(dict(passed=False, task_id=args.task_id,
                        error=f'{type(error).__name__}: {error}', scope='Environment/probe failed before complete observation.'), indent=2)+'\n')
                raise
        return 0
    except Exception as error:
        report = args.output/'neutral_probe.json'
        if not report.exists():
            report.write_text(json.dumps(dict(passed=False, task_id=args.task_id,
                error=f'{type(error).__name__}: {error}', scope='Environment/probe failed; no successful observation claim.'), indent=2)+'\n')
        raise


if __name__ == '__main__':
    raise SystemExit(main())
