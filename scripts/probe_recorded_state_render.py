"""Render three measured obs states without executing their recorded actions.

This is an incomplete-scene diagnostic, never a production episode. The source
does not contain the nested kitchen articulation states or renderer history.
Run through scripts/arena_python.sh; SimulationApp imports remain deferred.
"""
import io
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DEFAULT_SOURCE = ROOT / 'reports/collection_speed_20261018/fast_actual10/property_glass_bottle'


def select_indices(phases, requested=None):
    """Select obs[0], the first post-close obs, and mid-transport obs."""
    phases = list(phases)
    if requested is None:
        lift = [i for i, phase in enumerate(phases) if phase == 'lift']
        carry = [i for i, phase in enumerate(phases) if phase == 'above_bowl']
        if not lift or not carry:
            raise ValueError('Automatic selection needs lift and above_bowl phases; supply --indices')
        requested = [0, lift[0], carry[len(carry) // 2]]
    values = list(requested)
    if len(values) != 3 or len(set(values)) != 3 or any(i < 0 or i >= len(phases) for i in values):
        raise ValueError('Exactly three distinct in-range obs indices are required')
    return values


def translated_pose(value, *, coordinate_frame, source_origin, replay_origin):
    """Map source world or environment-local positions into replay world."""
    import numpy as np
    result = np.asarray(value).copy()
    if coordinate_frame not in ('world', 'environment_local'):
        raise ValueError(f'Unsupported source coordinate frame: {coordinate_frame}')
    offset = np.asarray(replay_origin) - (np.asarray(source_origin) if coordinate_frame == 'world' else 0)
    result[..., :3] += offset
    return result


def pose_error(actual, expected):
    import numpy as np
    from scipy.spatial.transform import Rotation
    actual, expected = np.asarray(actual), np.asarray(expected)
    if actual.shape != (7,) or expected.shape != (7,) or not np.isfinite([actual, expected]).all():
        raise ValueError('Pose comparison requires two finite xyz+xyzw vectors')
    angle = (Rotation.from_quat(expected[3:7]).inv() * Rotation.from_quat(actual[3:7])).magnitude()
    return {'position_error_m': float(np.linalg.norm(actual[:3] - expected[:3])),
            'rotation_error_rad': float(angle), 'actual_xyz_xyzw': actual.tolist(),
            'expected_xyz_xyzw': expected.tolist()}


def pure_render(sim, ensure_update, frames):
    """Pump RTX at fixed measured state, without a physics step or action."""
    before = int(sim.get_physics_step_count())
    generation = int(sim.render_generation)
    started = time.perf_counter()
    for _ in range(frames):
        previous = int(sim.render_generation)
        sim.render()
        ensure_update(force=True)
        if int(sim.get_physics_step_count()) != before:
            raise RuntimeError('Physics advanced during recorded-state rendering')
        if int(sim.render_generation) <= previous:
            raise RuntimeError('Render generation did not advance')
    return {'render_frames': frames, 'physics_step_count_before': before,
            'physics_step_count_after': int(sim.get_physics_step_count()),
            'physics_advanced': False, 'render_generation_before': generation,
            'render_generation_after': int(sim.render_generation),
            'elapsed_seconds': time.perf_counter() - started,
            'convergence_checked': False}


def load_source(source, requested=None):
    import h5py
    import numpy as np
    source = Path(source)
    result = json.loads((source / 'result.json').read_text())
    spec = json.loads((source / 'episode_spec.json').read_text())
    metadata = json.loads((source / 'cameras.json').read_text())
    if result.get('success') is not True:
        raise ValueError('This diagnostic expects a successful measured source episode')
    with h5py.File(source / 'trajectory.h5', 'r') as handle:
        phases = handle['phase'].asstr()[:]
        indices = select_indices(phases, requested)
        frames = []
        for index in indices:
            state = {key: np.asarray(value[index]) for key, value in handle['obs'].items()}
            for key in ('q', 'dq', 'robot_body_pose_w', 'mustard', 'bowl'):
                if key not in state or not np.isfinite(state[key]).all():
                    raise ValueError(f'Missing or nonfinite measured obs/{key}')
            frames.append({'source_obs_index': index, 'phase': str(phases[index]),
                           'source_time_seconds': float(handle['time'][index]), 'state': state})
        data = {'result': result, 'spec': spec, 'camera_metadata': metadata,
                'frames': frames, 'source_steps': len(phases), 'dt': float(handle.attrs['dt']),
                'joint_names': json.loads(handle.attrs['joint_names']),
                'body_names': json.loads(handle.attrs['robot_body_names']),
                'coordinate_frame': str(handle.attrs.get('coordinate_frame', 'world')),
                'source_origin': np.asarray(handle.attrs.get('environment_origin_world_m', [0., 0., 0.]))}
    with h5py.File(source / 'images.h5', 'r') as images:
        for frame in frames:
            index = frame['source_obs_index']
            frame['recorded_cameras'] = {}
            for name in metadata:
                group = images[name]
                frame['recorded_cameras'][name] = {
                    'pose': np.r_[group['position_world'][index], group['quaternion_world_ros_xyzw'][index]],
                    'coordinate_frame': str(group.attrs.get('coordinate_frame', images.attrs.get('coordinate_frame', 'world'))),
                    'source_origin': np.asarray(group.attrs.get('environment_origin_world_m', images.attrs.get('environment_origin_world_m', [0., 0., 0.]))),
                    'jpeg': bytes(group['jpeg'][index]),
                }
    return data


def kitchen_coverage(stage):
    """Inventory composed kitchen physics, without claiming it stayed still."""
    from pxr import Usd, UsdPhysics
    roots = [prim for prim in stage.Traverse() if prim.GetName() == 'kitchen']
    bodies, articulations = [], []
    for root in roots:
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid = UsdPhysics.RigidBodyAPI(prim)
                bodies.append({'path': str(prim.GetPath()),
                               'enabled': bool(rigid.GetRigidBodyEnabledAttr().Get()),
                               'kinematic': bool(rigid.GetKinematicEnabledAttr().Get())})
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulations.append(str(prim.GetPath()))
    return {'complete_scene_state_restored': False, 'kitchen_roots': [str(p.GetPath()) for p in roots],
            'rigid_bodies': bodies, 'articulation_roots': articulations,
            'enabled_nonkinematic_body_count': sum(row['enabled'] and not row['kinematic'] for row in bodies),
            'source_has_nested_kitchen_states': False,
            'limitation': 'Nested kitchen dynamics were not recorded. Their reset poses here are not evidence '
                          'of their measured poses at the selected source times.'}


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--source-episode', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--indices', type=int, nargs=3, default=None)
    parser.add_argument('--camera-device', default=None)
    parser.add_argument('--preparation-frames', type=int, default=128)
    parser.add_argument('--render-frames-per-state', type=int, default=16)
    parser.add_argument('--inventory-kitchen-physics', action='store_true',
                        help='Read current nested kitchen backend state without advancing physics')
    args = parser.parse_args()
    if args.num_envs != 1 or args.preparation_frames < 1 or args.render_frames_per_state < 1:
        parser.error('Use one environment and positive pure-render frame counts')
    args.source_episode, args.output = args.source_episode.resolve(), args.output.resolve()
    args.enable_cameras = True
    data = load_source(args.source_episode, args.indices)
    args.output.mkdir(parents=True, exist_ok=False)
    args.kit_args = ((args.kit_args or '') + f' --/log/file={args.output / "kit.log"}').strip()
    report = {'diagnostic_only': True, 'production_quota_eligible': False, 'completed': False,
              'source_episode': str(args.source_episode), 'state_authority': 'trajectory.h5 obs[t] measured states',
              'actions_executed': 0, 'future_states_used': False,
              'source_coordinate_frame': data['coordinate_frame'], 'source_origin_world_m': data['source_origin'].tolist(),
              'indices': [row['source_obs_index'] for row in data['frames']], 'frames': [],
              'limitations': ['Nested kitchen dynamic state is unavailable and is not reconstructed.',
                              'Dynamic hazard velocities are unrecorded and set to zero; its measured pose is restored.',
                              'Fixed-base root velocity is set to zero; its measured panda_link0 pose is restored.',
                              'RTX temporal history and original per-link velocities are unavailable; pixel identity is not claimed.',
                              'Robot link poses are recomputed by official FK, then compared with all recorded links; no link error is hidden.']}

    def save():
        (args.output / 'report.json').write_text(json.dumps(report, indent=2))

    save()
    try:
        with SimulationAppContext(args):
            import numpy as np
            import torch
            from PIL import Image
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
            from kitchen_safety.camera_device import with_camera_output_device
            from kitchen_safety.dataset import CAMERA_NAMES, camera_poses_from_body_states, numpy
            from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
            from kitchen_safety.randomization import apply_appearance
            from kitchen_safety.vectorized_dataset import VectorizedCameraWriter

            env = writer = None
            try:
                spec, source_result = data['spec'], data['result']
                seed = source_result.get('seed', args.seed)
                environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
                    variant=source_result['variant'], enable_cameras=True, collection_seed=seed,
                    obstacle_id=spec['obstacle_id'], episode_spec=spec))
                builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
                    device=args.device, seed=seed, presets='physx', num_envs=1,
                    solve_relations=False, disable_fabric=False,
                    recorder_dataset_export_dir_path=str(args.output / 'arena_recorder')))
                cfg, kwargs = builder.compose_manager_cfg()
                cfg.sim.dt, cfg.decimation = 1 / 120, 8
                cfg.sim.render_interval = cfg.decimation
                cfg.sim.log_dir = str(args.output / 'isaaclab_logs')
                cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count = 64
                cfg.observations.camera_obs = None
                for name in CAMERA_NAMES:
                    camera_cfg = with_camera_output_device(getattr(cfg.scene, name), args.camera_device)
                    camera_cfg.update_latest_camera_pose = True
                    setattr(cfg.scene, name, camera_cfg)
                env = builder.make_registered(cfg, kwargs)
                env.reset()
                e = env.unwrapped
                scene, robot, sim = e.scene, e.scene['robot'], e.sim
                if not sim.is_playing():
                    raise RuntimeError('Official PhysX FK refresh requires the initialized timeline to be playing')
                joint_order = [data['joint_names'].index(name) for name in robot.joint_names]
                body_order = [data['body_names'].index(name) for name in robot.body_names]
                if len(joint_order) != len(data['joint_names']) or len(body_order) != len(data['body_names']):
                    raise ValueError('Source and rebuilt robot joint/body sets differ')
                origin = numpy(scene.env_origins)[0]
                report.update(replay_origin_world_m=origin.tolist(), physics_device=str(e.device),
                              camera_devices={name: scene[name].device for name in CAMERA_NAMES},
                              physics_step_count_after_reset=int(sim.get_physics_step_count()),
                              kitchen_coverage=kitchen_coverage(sim.stage))
                if args.inventory_kitchen_physics:
                    from kitchen_safety.render_state import read_kitchen_physics_inventory
                    started = time.perf_counter()
                    before_inventory = int(sim.get_physics_step_count())
                    inventory = read_kitchen_physics_inventory(sim.stage, robot._physics_sim_view)
                    inventory.update(elapsed_seconds=time.perf_counter() - started,
                                     physics_step_count_before=before_inventory,
                                     physics_step_count_after=int(sim.get_physics_step_count()))
                    report['kitchen_backend_inventory'] = inventory
                    save()
                report['appearance'] = apply_appearance(sim.stage, spec)
                original_appearance = json.loads((args.source_episode / 'appearance.json').read_text())
                report['appearance_report_equals_source'] = report['appearance'] == original_appearance
                (args.output / 'appearance.json').write_text(json.dumps(report['appearance'], indent=2))
                # Reuse batched capture with a zero-frame scratch sink. Only
                # the explicitly indexed PNGs below are diagnostic observations.
                writer = VectorizedCameraWriter({0: args.output / 'writer_state'}, scene,
                                                env_origins=origin[None, :], dt=data['dt'])
                dynamic = not spec['layout']['obstacle']['static']
                baseline_steps = int(sim.get_physics_step_count())

                def tensor(value):
                    return torch.as_tensor(value, device=e.device, dtype=torch.float32).unsqueeze(0)

                for ordinal, source_frame in enumerate(data['frames']):
                    source_index, state = source_frame['source_obs_index'], source_frame['state']
                    expected_bodies = translated_pose(
                        state['robot_body_pose_w'][body_order], coordinate_frame=data['coordinate_frame'],
                        source_origin=data['source_origin'], replay_origin=origin)
                    root_pose = expected_bodies[list(robot.body_names).index('panda_link0')]
                    robot.write_root_pose_to_sim_index(root_pose=tensor(root_pose))
                    robot.write_root_velocity_to_sim_index(root_velocity=tensor(np.zeros(6)))
                    robot.write_joint_state_to_sim_index(position=tensor(state['q'][joint_order]),
                                                        velocity=tensor(state['dq'][joint_order]))
                    expected_roots = {}
                    for name, key in [('mustard_bottle', 'mustard'), ('bowl', 'bowl')]:
                        root = translated_pose(state[key], coordinate_frame=data['coordinate_frame'],
                                               source_origin=data['source_origin'], replay_origin=origin)
                        expected_roots[name] = root
                        scene[name].write_root_state_to_sim(tensor(root))
                    if dynamic:
                        pose = translated_pose(state['hazard_pose'], coordinate_frame=data['coordinate_frame'],
                                               source_origin=data['source_origin'], replay_origin=origin)
                        expected_roots['glass'] = np.r_[pose, np.zeros(6)]
                        scene['glass'].write_root_state_to_sim(tensor(expected_roots['glass']))
                    scene.write_data_to_sim()
                    sim.forward()
                    scene.update(0.)
                    sim.render_context.reset_scene_state_cadence()
                    preparation = pure_render(sim, ensure_isaac_rtx_render_update,
                                              args.preparation_frames if ordinal == 0 else args.render_frames_per_state)
                    capture = writer._capture()
                    if int(sim.get_physics_step_count()) != baseline_steps:
                        raise RuntimeError('Physics advanced after measured-state restoration')
                    actual_bodies = np.concatenate((numpy(robot.data.body_pos_w)[0], numpy(robot.data.body_quat_w)[0]), axis=-1)
                    row = {key: source_frame[key] for key in ('source_obs_index', 'phase', 'source_time_seconds')}
                    row.update(output_frame_index=ordinal, preparation=preparation,
                               physics_step_count_after_capture=int(sim.get_physics_step_count()),
                               render_generation_after_capture=int(sim.render_generation),
                               q_max_abs_error_rad=float(np.max(np.abs(numpy(robot.data.joint_pos)[0] - state['q'][joint_order]))),
                               dq_max_abs_error_rad_per_s=float(np.max(np.abs(numpy(robot.data.joint_vel)[0] - state['dq'][joint_order]))),
                               body_errors={name: pose_error(actual_bodies[i], expected_bodies[i])
                                            for i, name in enumerate(robot.body_names)}, root_errors={}, cameras={})
                    for name, expected in expected_roots.items():
                        actual = numpy(scene[name].data.root_state_w)[0]
                        row['root_errors'][name] = {**pose_error(actual[:7], expected[:7]),
                            'velocity_max_abs_error': float(np.max(np.abs(actual[7:] - expected[7:]))),
                            'velocity_source': 'unrecorded_zero_for_diagnostic' if name == 'glass' else 'measured_obs'}
                    expected_camera = camera_poses_from_body_states(
                        expected_bodies[None, ...], list(robot.body_names), data['camera_metadata'])
                    frame_dir = args.output / f'obs_{source_index:06d}_{row["phase"]}'
                    frame_dir.mkdir()
                    for name in CAMERA_NAMES:
                        item, original = capture[name], source_frame['recorded_cameras'][name]
                        actual_pose = np.r_[item['position'][0], item['quaternion'][0]]
                        recorded_pose = translated_pose(original['pose'], coordinate_frame=original['coordinate_frame'],
                                                        source_origin=original['source_origin'], replay_origin=origin)
                        derived = np.r_[expected_camera[name]['position_world'][0],
                                        expected_camera[name]['quaternion_world_ros_xyzw'][0]]
                        rgb = item['rgb'][0]
                        source_rgb = np.asarray(Image.open(io.BytesIO(original['jpeg'])).convert('RGB'))
                        if source_rgb.shape != rgb.shape:
                            raise ValueError(f'{name}: source/replay dimensions differ')
                        Image.fromarray(rgb).save(frame_dir / f'{name}.png')
                        Image.fromarray(source_rgb).save(frame_dir / f'{name}_source.png')
                        cam_cfg, old_cfg = scene[name].cfg, data['camera_metadata'][name]
                        row['cameras'][name] = {
                            'versus_recorded_camera_pose': pose_error(actual_pose, recorded_pose),
                            'versus_recorded_parent_body_pose': pose_error(actual_pose, derived),
                            'intrinsics_max_abs_error': float(np.max(np.abs(item['intrinsics'][0] - old_cfg['intrinsics']))),
                            'width': cam_cfg.width, 'height': cam_cfg.height,
                            'mount_position_max_abs_error_m': float(np.max(np.abs(np.asarray(cam_cfg.offset.pos) - old_cfg['offset']['position']))),
                            'mount_quaternion_max_abs_error': float(np.max(np.abs(np.asarray(cam_cfg.offset.rot) - old_cfg['offset']['quaternion_xyzw']))),
                            'mount_convention_matches': cam_cfg.offset.convention == old_cfg['offset']['convention'],
                            'parent_prim_matches': cam_cfg.prim_path.rsplit('/', 1)[0] == old_cfg['prim_path'].rsplit('/', 1)[0],
                            'png': str((frame_dir / f'{name}.png').relative_to(args.output)),
                            'source_png': str((frame_dir / f'{name}_source.png').relative_to(args.output)),
                            'mean_absolute_rgb_difference_from_source': float(np.abs(rgb.astype(np.float32) - source_rgb).mean()),
                            'pixel_difference_is_acceptance_test': False}
                    report['frames'].append(row)
                    save()
                report.update(completed=True, physics_step_count_after_probe=int(sim.get_physics_step_count()),
                              physics_advanced_during_probe=False,
                              body_position_max_error_m=max(v['position_error_m'] for f in report['frames'] for v in f['body_errors'].values()),
                              body_rotation_max_error_rad=max(v['rotation_error_rad'] for f in report['frames'] for v in f['body_errors'].values()))
            except BaseException as error:
                report.update(completed=False, error=repr(error), traceback=traceback.format_exc())
                raise
            finally:
                try:
                    if writer is not None:
                        writer.close()
                    if env is not None:
                        env.close()
                except BaseException as error:
                    report.update(completed=False, error=repr(error), traceback=traceback.format_exc())
                    raise
                finally:
                    # Kit shutdown may exit the process. Persist the final
                    # measurements before SimulationAppContext.__exit__.
                    save()
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
        raise
    finally:
        save()


if __name__ == '__main__':
    main()
