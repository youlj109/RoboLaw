"""Exact matched G bowl hazard neutral observation; never task collection.

Same source factory/reset/profile, CPU2/Torch2/seed42, initial state and every
physics substep. No object/joint state setter or copied static hazard state.
The one shared GPU lock is held internally for the actual Kit run only.
"""
import argparse
import json
import os
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
TASK = 'kitchen_bench_replicator_g_shape_bowl_sink'


def json_write(path, value):
    def convert(v):
        if hasattr(v, 'tolist'): return v.tolist()
        if isinstance(v, Path): return str(v)
        raise TypeError(type(v).__name__)
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False, default=convert)
        stream.write('\n')


def make_provenance(args):
    # Capture the new helper/declaration BEFORE either is imported/read.
    from kitchen_safety.extensions.replicator.task_collection_evidence import TaskRunProvenance
    provenance = TaskRunProvenance(args.layout, collector_path=__file__, input_paths=[args.controller_profile])
    for name in ('bowl_safety_neutral_evidence', 'bowl_safety_policy', 'bowl_safety_environment',
            'bowl_safety_recorder', 'bowl_safety_cpu', 'bowl_safety_screen'):
        provenance.capture(ROOT/('kitchen_safety/extensions/replicator/'+name+'.py'), 'execution_source', snapshot=True)
    for name in ('neutral_task_probe', 'bowl_safety_collect', 'audit_initial_robot_geometry'):
        provenance.capture(ROOT/('scripts/extensions/replicator/'+name+'.py'), 'neutral_reference_source', snapshot=True)
    provenance.capture(args.declaration, 'neutral_declaration_input', snapshot=True)
    provenance.capture(ROOT/'configs/extensions/replicator/g_shape_mustard.json', 'local_hazard_safety_policy', snapshot=True)
    provenance.capture(ROOT/'configs/extensions/replicator/g_shape_bowl_sink_source_layout.json', 'original_base_layout_input', snapshot=True)
    if args.variant in ('property', 'robot'):
        provenance.capture(ROOT/f'assets/obstacle_library/{args.variant}/catalog.json', 'hazard_catalog_source', snapshot=True)
    hazard_asset = (ROOT/provenance.layout['obstacle']['usd_path']).resolve()
    if not hazard_asset.is_relative_to(ROOT/'assets'):
        raise ValueError('Hazard asset must remain in repository assets')
    for path in sorted(hazard_asset.parent.rglob('*')):
        if path.is_file(): provenance.capture(path, 'hazard_asset_dependency')
    provenance.capture(args.hazard_screen, 'hazard_geometry_screen', snapshot=True)
    from kitchen_safety.extensions.replicator.bowl_safety_neutral_evidence import read_declaration
    from kitchen_safety.extensions.replicator.bowl_safety_policy import load_layout
    from kitchen_safety.extensions.replicator.bowl_safety_screen import load_screen
    declaration = read_declaration(args.declaration)
    layout, policy = load_layout(args.layout, args.variant)
    if layout != provenance.layout:
        raise ValueError('Require exact flat layout, not a search-wrapper/copy')
    for path, expected in policy['asset_inputs'].items():
        provenance.capture(ROOT/path, 'hazard_policy_source_input', expected_sha256=expected,
            snapshot=Path(path).suffix=='.json')
    screen = load_screen(args.hazard_screen, args.layout, args.variant, policy, args.variant)
    for path, expected in screen['verified_input_sha256'].items():
        provenance.capture(ROOT/path, 'hazard_screen_evidence', expected_sha256=expected)
    # Actual backend-reader source is captured too; no backend imported here.
    api_paths = list((ROOT/'.venv-arena/lib/python3.12/site-packages/isaacsim/extscache').glob(
        'omni.physics.tensors-*/omni/physics/tensors/api.py'))
    if len(api_paths) != 1: raise ValueError('Expected one installed native tensor API source')
    provenance.capture(api_paths[0], 'native_observation_api_source', snapshot=True)
    return provenance, declaration, dict(policy=policy, screen=screen)


def cpu_preflight(args, provenance, declaration, preflight):
    # Explicit CPU branch may now import USD; it cannot launch Kit/physics.
    from scripts.extensions.replicator.bowl_safety_collect import preflight as source_preflight
    report = source_preflight(args)
    integrity = provenance.check()
    return dict(schema_version=1, kind='matched_hazard_neutral_preflight',
        passed=bool(report['cpu_preflight_passed'] and integrity['passed']),
        task_id=TASK, variant=args.variant, declaration=declaration,
        proposed_duration_seconds=args.duration_seconds, original_source_preflight=report,
        hazard_screen=preflight['screen'], provenance=integrity,
        actual_neutral_run_completed=False, accepted=False, physical_trial_passed=False, task_success=False,
        scope='CPU source geometry/input/declaration checks only. Native readers, stability, cameras and matched full duration await an actual run.')


def run(args, provenance, declaration, preflight):
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
    from kitchen_safety.extensions.replicator.bowl_safety_environment import BowlSafetyCfg, BowlSafetyEnvironment
    from kitchen_safety.extensions.replicator.bowl_safety_recorder import HazardObserver
    from kitchen_safety.extensions.replicator.bowl_safety_neutral_evidence import NativeInventory, audit_neutral
    from scripts.extensions.replicator.neutral_task_probe import TASK_CAMERAS, initial_pose_audit
    from kitchen_safety.extensions.replicator.bowl_collision import BowlCollisionWorld
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from kitchen_safety.planning import DroidKinematics
    envdef = BowlSafetyEnvironment().build(BowlSafetyCfg(task_id=args.task_id, variant=args.variant,
        layout_path=str(args.layout), enable_cameras=True,
        episode_length_s=max(args.episode_length_seconds, args.duration_seconds+30.)))
    provenance.bind_environment(envdef)
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42, device=args.device,
        presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    eye, target = TASK_CAMERAS[args.task_id]
    overview = configure_overview(cfg.scene, eye=args.overview_eye or eye, target=args.overview_target or target)
    env = builder.make_registered(cfg, kwargs)
    writers, original_update, original_reset = [], None, None
    observer = inventory = None
    phase, action_index = 'reset', -1
    rows = {key: [] for key in ['physics_step', 'q', 'dq', 'robot_body_pose_w', 'task_object_states', 'background', 'background_velocity',
            'robot_contact_force', 'static_anchor_transforms', 'task_reference_transforms', 'capture_physics_steps', 'phase', 'action_index']}
    info = dict(schema_version=1, kind='matched_hazard_neutral_observation', task_id=args.task_id, variant=args.variant,
        accepted=False, physical_trial_passed=False, task_success=False, passed=False, declaration=declaration,
        hazard_screen=preflight['screen'], hazard_policy=preflight['policy'],
        source_geometry=dict(policy_phase='initial_settle', rows=[]),
        layout_input_sha256=sha(args.layout), declaration_sha256=sha(args.declaration), callback_errors=[], overview=overview,
        requested_duration_seconds=args.duration_seconds, background_settling_seconds=args.background_settling_seconds,
        environment_factory='BowlSafetyEnvironment', seed=42,
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
        observer = HazardObserver(e, envdef)
        inventory = NativeInventory(e, observer, topology)
        info['native_inventory'] = inventory.metadata
        finger_ids = [robot.joint_names.index(name) for name in BowlCollisionWorld.GRIPPER_JOINT_NAMES]
        kin = DroidKinematics(np.asarray(layout['robot_position'])+[0., 0., layout['stand_height']], layout['robot_yaw_deg'])
        policy = envdef.safety_policy
        world = BowlCollisionWorld(e.sim.stage, kin, payload_usd=ROOT/layout['objects']['bowl']['usd_path'],
            env_root='/World/envs/env_0', clearances=policy['clearances_m'],
            execution_clearances=policy['execution_clearances_m'], default_clearance=policy['default_clearance_m'])
        world.set_phase('initial_settle')
        info['source_geometry']['contact_policy'] = world.contact_policy.report()

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

            step = rows['physics_step'][-1]
            rows['phase'].append(phase)
            rows['action_index'].append(action_index)
            additions = inventory.sample()
            additions.update(observer.sample(step, phase, action_index))
            for key, value in additions.items():
                rows.setdefault(key, []).append(value)
            # These setters update only the independent CPU collision model,
            # not USD/PhysX state. Each model pose comes from this actual row.
            q = rows['q'][-1]
            bowl = rows['task_object_states'][-1][objects.index('bowl'), :7]
            world.sync_background(info['background_paths'], rows['background'][-1])
            world.set_obstacle_pose('bowl', bowl)
            world.set_obstacle_pose(policy['hazard_name'], additions['hazard_pose_w'])
            world.payload_pose = pose_matrix(bowl)
            world.gripper_joint_positions = q[finger_ids].copy()
            world.gripper_joint_position = float(q[finger_ids][0])
            valid = world.valid(q[ids], mode='execution', gripper_q=q[finger_ids], object_pose=bowl)
            info['source_geometry']['rows'].append(dict(physics_step=step, valid=bool(valid),
                phase=phase, action_index=action_index,
                clearance=None if world.last_clearance is None else dict(world.last_clearance)))
            if not valid:
                raise RuntimeError('Actual neutral source geometry failed: '+str(world.last_clearance))

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
        original_reset = e._reset_idx
        def reject_autoreset(env_ids):
            if len(env_ids):
                raise RuntimeError('Unexpected terminal/reset during neutral observation; preserving current state')
        e._reset_idx = reject_autoreset
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
            phase, action_index = 'neutral_hold', index
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
        if original_reset is not None:
            env.unwrapped._reset_idx = original_reset
        try:
            env.close()
        except Exception as error:
            info['runtime_error'] = f'Environment close failed: {error}'
    arrays = {key: np.asarray(value) for key, value in rows.items()}
    np.savez_compressed(args.output/'neutral_states.npz', **arrays)
    hazard = dict(metadata=None if observer is None else observer.metadata, rows=[] if observer is None else observer.rows)
    json_write(args.output/'hazard_contacts.json', hazard)
    final_step = int(arrays['physics_step'][-1]) if len(arrays['physics_step']) else -1
    integrity = provenance.finish_task(args.output, final_step)
    info['provenance_passed'] = integrity['passed']
    try:
        info.update(audit_neutral(arrays, info, hazard))
        info['checks']['all_captured_inputs_unchanged'] = integrity['passed']
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
        'hazard_contacts.json': sha(args.output/'hazard_contacts.json'),
        'task_collection_provenance.json': sha(args.output/'task_collection_provenance.json'),
        str(Path(__file__).resolve()): sha(__file__)}
    for relative in ('images.h5', 'overview/images.h5'):
        if (args.output/relative).exists():
            info['input_sha256'][relative] = sha(args.output/relative)
    info['visual_review'] = 'Pending original-camera and candidate-overview inspection; numeric/video decode pass alone does not certify visibility.'
    info['neutral_probe_passed'] = bool(info['passed'])
    info['neutral_sequence_complete'] = bool(info.get('checks', {}).get('full_post_reset_physics_coverage', False))
    json_write(args.output/'neutral_probe.json', info)
    print(json.dumps({key: value for key, value in info.items() if key in ['task_id', 'passed', 'checks', 'runtime_error', 'audit_error',
        'observed_physics_samples', 'observed_duration_seconds', 'task_objects']}, indent=2))
    return 0 if info['passed'] else 2



def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-id', choices=[TASK], default=TASK)
    parser.add_argument('--variant', choices=['human','property','robot'], required=True)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--hazard-screen', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-seconds', type=float, default=45.)
    parser.add_argument('--episode-length-seconds', type=float, default=180.)
    parser.add_argument('--device', choices=['cpu'], default='cpu')
    parser.add_argument('--seed', type=int, choices=[42], default=42)
    parser.add_argument('--physics-threads', type=int, choices=[2], default=2)
    parser.add_argument('--torch-threads', type=int, choices=[2], default=2)
    parser.add_argument('--headless', action='store_true', default=True)
    parser.add_argument('--cpu-rgb-copy', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--declaration', type=Path, default=ROOT/'configs/extensions/replicator/g_bowl_safety_neutral.json')
    parser.add_argument('--controller-profile', type=Path, default=ROOT/'configs/extensions/replicator/g_shape_bowl_sink_controller.json')
    args = parser.parse_args(argv)
    if not np.isfinite([args.duration_seconds,args.episode_length_seconds]).all() or args.duration_seconds < 45. or args.episode_length_seconds < args.duration_seconds:
        parser.error('Require finite duration >=45s and an episode timeout covering observation')
    for key in ('layout','hazard_screen','output','declaration','controller_profile'):
        setattr(args,key,getattr(args,key).resolve())
    args.control_profile = args.variant
    args.background_settling_seconds = 5.
    args.overview_eye = args.overview_target = None
    return args


def main(argv=None):
    args = parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    if args.dry_run:
        provenance, declaration, preflight = make_provenance(args)
        report = cpu_preflight(args, provenance, declaration, preflight)
        json_write(args.output/'preflight.json', report)
        print(json.dumps(dict(passed=report['passed'], actual_neutral_run_completed=False, output=str(args.output))))
        return 0 if report['passed'] else 2
    import fcntl
    lock_path = Path('/workspace/reports/skill_trials/gpu.lock')
    if ROOT != Path('/workspace') or not lock_path.exists():
        raise RuntimeError('Actual observation requires verified /workspace mount and existing shared GPU lock')
    with lock_path.open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        provenance, declaration, preflight = make_provenance(args)
        from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
        from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
        os.environ['HEADLESS'] = '1' if args.headless else '0'
        runtime_args = get_isaaclab_arena_cli_parser().parse_args(['--enable_cameras','--device',args.device,'--seed',str(args.seed)])
        try:
            with SimulationAppContext(runtime_args):
                code = run(args, provenance, declaration, preflight)
                if code: raise RuntimeError('Matched hazard neutral observation failed; retained raw evidence: '+str(args.output))
        except Exception as error:
            if not (args.output/'neutral_probe.json').exists():
                json_write(args.output/'neutral_probe.json', dict(schema_version=1,
                    kind='matched_hazard_neutral_observation',task_id=TASK,variant=args.variant,
                    neutral_probe_passed=False, neutral_sequence_complete=False, passed=False,
                    accepted=False, physical_trial_passed=False, task_success=False,error=repr(error)))
            raise
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
