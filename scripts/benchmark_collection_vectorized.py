"""Single Kit / multiple Arena environments: GPU physics action-replay benchmark.

Example (run with scripts/arena_python.sh):
  scripts/benchmark_collection_vectorized.py --device cuda:0 \
    --num-envs 4 --source-episode reports/collection_speed_20261018/gpu_cameras3/property_vase \
    --output reports/collection_speed_20261018/vectorized4

Every environment replays the same existing absolute-joint actions. This is a
parallel-physics feasibility measurement, not replanning, data augmentation,
or a production safety/visual validation. Optional --record-rgb saves the three
original 720p views before every action; it does not run geometric audits.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def fixed_render_warmup(sim, ensure_render_update, frames=128):
    """Exercise fixed materials without physics or an appearance quality claim."""
    started = time.perf_counter()
    before = int(sim.get_physics_step_count())
    generation_before = int(sim.render_generation)
    for _ in range(frames):
        generation = int(sim.render_generation)
        sim.render()
        if int(sim.render_generation) <= generation:
            raise RuntimeError('Warm-up render did not advance render_generation')
        ensure_render_update(force=True)
        if int(sim.get_physics_step_count()) != before:
            raise RuntimeError('Physics advanced during fixed pure-render warm-up')
    return dict(render_frames=frames, elapsed_seconds=time.perf_counter() - started,
                physics_step_count_before=before, physics_step_count_after=int(sim.get_physics_step_count()),
                physics_advanced=False, render_generation_before=generation_before,
                render_generation_after=int(sim.render_generation),
                appearance_convergence_tested=False, randomization_applied=False,
                semantics='Fixed render count for throughput diagnosis; no appearance convergence certification')


def write_environment_trajectory(output, arrays, env_index, origin, actions, phases,
                                 dt, joint_names, body_names, contact_names):
    """Split GPU-batched state readback into the existing obs/action/next_obs format."""
    import h5py
    import numpy as np
    from scipy.spatial.transform import Rotation
    origin = np.asarray(origin)
    local = {}
    for name, values in arrays.items():
        value = values[:, env_index].copy()
        if name in ('mustard', 'bowl', 'hazard'):
            value[:, :3] -= origin
        elif name == 'robot_body_pose_w':
            value[:, :, :3] -= origin
        if name == 'hazard':
            name, value = 'hazard_pose', value[:, :7]
        local[name] = value
    tcp_parent = local['robot_body_pose_w'][:, body_names.index('base_link')]
    local['tcp_quat_xyzw'] = tcp_parent[:, 3:7].copy()
    local['tcp'] = tcp_parent[:, :3] + Rotation.from_quat(tcp_parent[:, 3:7]).apply([.131, 0, 0])
    count = len(actions)
    if any(len(value) != count + 1 for value in local.values()):
        raise ValueError('Batched state chain must contain T+1 states for T actions')
    with h5py.File(Path(output) / 'trajectory.h5', 'w') as f:
        f.attrs.update(dt=dt, quaternion_order='xyzw', joint_names=json.dumps(list(joint_names)),
                       robot_body_names=json.dumps(list(body_names)),
                       contact_body_names=json.dumps(list(contact_names)),
                       action_semantics='7 absolute arm joint targets in radians; binary gripper 0=open, 1=close',
                       coordinate_frame='environment_local', environment_origin_world_m=origin)
        f.create_dataset('time', data=np.arange(count) * dt)
        f.create_dataset('phase', data=np.asarray(phases, dtype=h5py.string_dtype()))
        f.create_dataset('action', data=actions)
        for name, values in local.items():
            f.create_dataset('obs/' + name, data=values[:-1])
            f.create_dataset('next_obs/' + name, data=values[1:])


def main():
    started = time.perf_counter()
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--num-envs', dest='num_envs', type=int, default=argparse.SUPPRESS)
    parser.add_argument('--source-episode', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New benchmark directory')
    parser.add_argument('--steps', type=int, default=0, help='Control actions to replay; 0 means the complete source')
    parser.add_argument('--replicate-physics', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--record-rgb', action='store_true', help='Save all three original 720p views per environment, no MP4')
    parser.add_argument('--render-spp', type=int, default=None,
                        help='Override Isaac RTX direct-lighting samples per pixel; other renderer settings stay unchanged')
    args = parser.parse_args()
    if not str(args.device).startswith('cuda'):
        parser.error('This GPU physics benchmark requires --device cuda:0 (or another CUDA device)')
    if args.num_envs < 1 or args.steps < 0:
        parser.error('--num-envs must be positive and --steps nonnegative')
    if args.render_spp is not None and (args.render_spp < 1 or not args.record_rgb):
        parser.error('--render-spp requires --record-rgb and a positive integer')
    args.enable_cameras = args.record_rgb
    args.source_episode = args.source_episode.resolve()
    args.output = args.output.resolve()
    source_result = json.loads((args.source_episode / 'result.json').read_text())
    if source_result.get('success') is not True:
        parser.error('The source episode must have result.success=true')
    args.output.mkdir(parents=True, exist_ok=False)
    args.kit_args = ((args.kit_args or '') + f' --/log/file={args.output / "kit.log"}').strip()
    spec = source_result.get('episode_spec') or {
        'obstacle_id': source_result['obstacle_id'], 'layout': source_result['layout']}
    report = {
        'semantics': 'Same recorded absolute-joint actions replayed in N GPU environments in one Kit app; optional fixed-material RGB; no replanning or FCL/body-grasp safety audit; not production data',
        'production_quota_eligible': False, 'completed': False,
        'source_episode': str(args.source_episode), 'source_success': source_result['success'],
        'num_envs': args.num_envs, 'device': args.device, 'replicate_physics': args.replicate_physics,
        'record_rgb': args.record_rgb, 'render_spp_requested': args.render_spp,
        'timings_seconds': {},
    }
    app_start = time.perf_counter()
    with SimulationAppContext(args):
        report['timings_seconds']['app_startup_s'] = time.perf_counter() - app_start
        import h5py
        import numpy as np
        import torch
        from isaaclab.managers import SceneEntityCfg
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.tasks.predicates.spatial import object_on_destination
        from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment

        env = cameras = None
        sim_methods = {}
        try:
            tick = time.perf_counter()
            with h5py.File(args.source_episode / 'trajectory.h5', 'r') as source:
                total = len(source['action'])
                count = min(args.steps or total, total)
                actions_np = source['action'][:count]
                phases = source['phase'].asstr()[:count]
                initial = {name: value[0] for name, value in source['obs'].items()}
                source_final = {name: value[count - 1] for name, value in source['next_obs'].items()}
                source_joint_names = json.loads(source.attrs['joint_names'])
                source_dt = float(source.attrs['dt'])
            if count == 0 or actions_np.shape != (count, 8) or not np.isfinite(actions_np).all():
                raise ValueError('Source must contain finite nonempty 8-dimensional actions')
            if not np.isclose(source_dt, 8 / 120, rtol=0, atol=1e-10):
                raise ValueError('Source action period must equal 8/120 seconds')
            report.update(source_steps=total, requested_steps=count, full_source_replay=count == total,
                          source_initialization='Source obs[0] q/dq and bottle/bowl state, translated by each environment origin; dynamic hazard pose restored with zero unrecorded velocity')
            report['timings_seconds']['source_read_s'] = time.perf_counter() - tick

            tick = time.perf_counter()
            environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
                variant=source_result['variant'], enable_cameras=args.record_rgb,
                collection_seed=source_result.get('seed', args.seed),
                obstacle_id=source_result['obstacle_id'], episode_spec=spec))
            builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
                device=args.device, seed=source_result.get('seed', args.seed), presets='physx',
                num_envs=args.num_envs, env_spacing=args.env_spacing,
                solve_relations=False, disable_fabric=False,
                recorder_dataset_export_dir_path=str(args.output / 'arena_recorder')))
            cfg, kwargs = builder.compose_manager_cfg()
            cfg.sim.dt, cfg.decimation = 1 / 120, 8
            cfg.sim.render_interval = cfg.decimation
            cfg.sim.log_dir = str(args.output / 'isaaclab_logs')
            cfg.scene.replicate_physics = args.replicate_physics
            cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count = 64
            cfg.scene.robot_contacts = ContactSensorCfg(
                prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
            if args.record_rgb:
                from kitchen_safety.dataset import CAMERA_NAMES
                # The writer owns the camera readback, not the observation manager.
                cfg.observations.camera_obs = None
                if args.render_spp is not None:
                    for name in CAMERA_NAMES:
                        getattr(cfg.scene, name).renderer_cfg.global_settings.samples_per_pixel = args.render_spp
            env = builder.make_registered(cfg, kwargs)
            env.reset()
            e = env.unwrapped
            if not args.record_rgb:
                e.render_enabled = False
            scene, robot = e.scene, e.scene['robot']
            if list(robot.joint_names) != source_joint_names:
                raise ValueError('Source joint order differs from this robot')
            report['timings_seconds']['environment_build_reset_s'] = time.perf_counter() - tick
            report.update(physics_dt=cfg.sim.dt, decimation=cfg.decimation, action_dt=e.step_dt,
                          solver_position_iterations=64, actual_device=str(e.device))

            def tensor(value):
                return value.torch if hasattr(value, 'torch') else value

            def cpu(value):
                return tensor(value).detach().cpu().numpy().copy()

            def repeated(value):
                return torch.as_tensor(value, device=e.device, dtype=torch.float32).repeat(args.num_envs, 1)

            origins = tensor(scene.env_origins).clone()
            ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)
            initial_hold = torch.cat((tensor(robot.data.joint_pos)[:, ids],
                                      torch.zeros((args.num_envs, 1), device=e.device)), dim=1)
            tick = time.perf_counter()
            for _ in range(8):
                env.step(initial_hold)
            torch.cuda.synchronize(e.device)
            report['timings_seconds']['eight_step_initial_warmup_s'] = time.perf_counter() - tick
            tick = time.perf_counter()
            robot.write_joint_state_to_sim_index(position=repeated(initial['q']), velocity=repeated(initial['dq']))
            for name, key in [('mustard_bottle', 'mustard'), ('bowl', 'bowl')]:
                state = repeated(initial[key])
                state[:, :3] += origins
                scene[name].write_root_state_to_sim(state)
            dynamic_hazard = bool(spec['layout'].get('obstacle') and not spec['layout']['obstacle']['static'])
            if dynamic_hazard and 'hazard_pose' in initial:
                hazard_state = repeated(np.r_[initial['hazard_pose'], np.zeros(6)])
                hazard_state[:, :3] += origins
                scene['glass'].write_root_state_to_sim(hazard_state)
            scene.write_data_to_sim()
            e.sim.forward()
            scene.update(0.)
            report['timings_seconds']['source_state_restore_s'] = time.perf_counter() - tick
            outputs = {}
            if args.record_rgb:
                import carb
                from kitchen_safety.vectorized_dataset import VectorizedCameraWriter
                from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
                outputs = {i: args.output / f'env_{i:03d}' for i in range(args.num_envs)}
                cameras = VectorizedCameraWriter(outputs, scene, dt=e.step_dt, env_origins=cpu(origins))
                report['fixed_render_warmup'] = fixed_render_warmup(e.sim, ensure_isaac_rtx_render_update)
                report['timings_seconds']['fixed_render_warmup_s'] = report['fixed_render_warmup']['elapsed_seconds']
                settings = carb.settings.get_settings()
                report['renderer_settings'] = {key: settings.get(key) for key in (
                    '/rtx/rendermode', '/rtx/directLighting/sampledLighting/samplesPerPixel',
                    '/rtx/pathtracing/maxSamplesPerLaunch', '/rtx/post/aa/op', '/rtx/post/dlss/execMode')}
                report['render_spp_semantics'] = 'IsaacRtxRendererGlobalSettingsCfg.samples_per_pixel: direct lighting, not total path-tracing samples'
                for sink in cameras.writers.values():
                    sink.file.attrs['fixed_render_warmup'] = json.dumps(report['fixed_render_warmup'])
                # These nested wall timers do not synchronize every physics step.
                # GPU work can finish in rendering/readback and be charged there.
                for name, metric in (('step', 'physics_sim_step_call_s'), ('render', 'internal_sim_render_s')):
                    sim_methods[name] = getattr(e.sim, name)
                    report['timings_seconds'][metric] = 0.
                    def measured(*values, _original=sim_methods[name], _metric=metric, **options):
                        tick = time.perf_counter()
                        try:
                            return _original(*values, **options)
                        finally:
                            report['timings_seconds'][_metric] += time.perf_counter() - tick
                    setattr(e.sim, name, measured)

            # Keep recording the live termination predicates without resetting
            # environments mid-replay. Failures are reported, never hidden by a reset.
            termination_seen = {name: torch.zeros(args.num_envs, dtype=torch.bool, device=e.device)
                                for name in e.termination_manager.active_terms}

            def capture_reset(env_ids):
                for name in termination_seen:
                    termination_seen[name] |= tensor(e.termination_manager.get_term(name)).bool()

            e._reset_idx = capture_reset
            actions = torch.as_tensor(actions_np, dtype=torch.float32, device=e.device)[:, None, :].repeat(1, args.num_envs, 1)
            states = {name: [] for name in ('q', 'dq', 'mustard', 'bowl')}
            if dynamic_hazard:
                states['hazard'] = []
            if args.record_rgb:
                states.update(robot_body_pose_w=[], robot_contact_forces=[])

            def capture():
                states['q'].append(tensor(robot.data.joint_pos).clone())
                states['dq'].append(tensor(robot.data.joint_vel).clone())
                states['mustard'].append(tensor(scene['mustard_bottle'].data.root_state_w).clone())
                states['bowl'].append(tensor(scene['bowl'].data.root_state_w).clone())
                if dynamic_hazard:
                    states['hazard'].append(tensor(scene['glass'].data.root_state_w).clone())
                if args.record_rgb:
                    states['robot_body_pose_w'].append(torch.cat((tensor(robot.data.body_pos_w),
                                                                 tensor(robot.data.body_quat_w)), dim=-1).clone())
                    states['robot_contact_forces'].append(tensor(scene['robot_contacts'].data.net_forces_w).clone())

            contact_names = scene['robot_contacts'].body_names
            arm_ids = [i for i, name in enumerate(contact_names) if name.startswith('panda_link') or name == 'base_link']
            max_arm_force = torch.zeros(args.num_envs, device=e.device)
            capture()
            torch.cuda.synchronize(e.device)
            torch.cuda.reset_peak_memory_stats(e.device)
            report['timings_seconds']['initialization_s'] = time.perf_counter() - started
            report['timings_seconds'].update(physics_step_call_s=0., state_capture_s=0.)
            tick = time.perf_counter()
            for index, action in enumerate(actions):
                if cameras:
                    cameras.write({i: str(phases[index]) for i in outputs},
                                  action_indices={i: index for i in outputs})
                step_started = time.perf_counter()
                env.step(action)
                report['timings_seconds']['physics_step_call_s'] += time.perf_counter() - step_started
                state_started = time.perf_counter()
                capture()
                forces = tensor(scene['robot_contacts'].data.net_forces_w)[:, arm_ids]
                max_arm_force = torch.maximum(max_arm_force, torch.linalg.vector_norm(forces, dim=-1).amax(dim=1))
                report['timings_seconds']['state_capture_s'] += time.perf_counter() - state_started
                if (index + 1) % 60 == 0:
                    print(f'VECTORIZED_STEP {index + 1}/{count} envs={args.num_envs} phase={phases[index]}', flush=True)
            torch.cuda.synchronize(e.device)
            elapsed = time.perf_counter() - tick
            report['timings_seconds']['replay_loop_s'] = elapsed
            report['gpu_memory'] = {
                'scope': 'PyTorch allocator peak since replay start; excludes Kit/RTX/PhysX allocations; use external NVML for total VRAM',
                'peak_allocated_bytes': torch.cuda.max_memory_allocated(e.device),
                'peak_reserved_bytes': torch.cuda.max_memory_reserved(e.device)}
            if cameras:
                report['camera_frames_by_env'] = {str(i): frames for i, frames in cameras.frames.items()}
                cameras.close()
                report['camera_timings_seconds'] = dict(cameras.timings)
                report['timings_seconds']['render_capture_s'] = cameras.timings['render_capture_s']
                report['timings_seconds']['jpeg_write_s'] = cameras.timings['jpeg_write_s']
                report['timings_seconds']['camera_close_s'] = cameras.timings['close_s']
                report['rgb_alignment'] = 'RGB[t] and obs[t] precede action[t]; no skipped or padded image frames'
                cameras = None
            report.update(executed_control_steps=count, physics_steps_per_env=count * cfg.decimation,
                          aggregate_physics_steps=count * cfg.decimation * args.num_envs,
                          simulated_seconds_per_env=count * e.step_dt,
                          aggregate_control_steps_per_second=count * args.num_envs / elapsed,
                          aggregate_physics_steps_per_second=count * cfg.decimation * args.num_envs / elapsed,
                          aggregate_simulated_seconds_per_wall_second=count * e.step_dt * args.num_envs / elapsed,
                          replay_equivalents_per_hour=args.num_envs * 3600 / elapsed,
                          full_source_equivalents_per_hour=args.num_envs * count / total * 3600 / elapsed)

            tick = time.perf_counter()
            task = environment.task
            supported = cpu(object_on_destination(
                e, object_cfg=SceneEntityCfg(task.pick_up_object.name),
                destination_cfg=SceneEntityCfg(task.destination_location.name),
                contact_sensor_cfg=task.contact_sensor_cfg, force_threshold=task.force_threshold,
                velocity_threshold=task.velocity_threshold, support_cone_half_angle_rad=task.support_cone_half_angle_rad))
            arrays = {name: cpu(torch.stack(rows)) for name, rows in states.items()}
            terminations = {name: cpu(value) for name, value in termination_seen.items()}
            final_q, bottle, bowl = arrays['q'][-1], arrays['mustard'][-1], arrays['bowl'][-1]
            delta = bottle[:, :3] - bowl[:, :3]
            lift = arrays['mustard'][:, :, 2].max(axis=0) - arrays['mustard'][0, :, 2]
            forces = cpu(max_arm_force)
            origin_np = cpu(origins)
            gripper = np.abs(final_q[:, robot.joint_names.index('finger_joint')])
            finite = all(np.isfinite(array).all() for array in arrays.values())
            report['environments'] = []
            for index in range(args.num_envs):
                checks = {
                    'full_source_replay': count == total,
                    'arena_success_seen': bool(terminations.get('success', np.zeros(args.num_envs))[index]),
                    'final_on_destination': bool(supported[index]), 'lift_above_5cm': bool(lift[index] > .05),
                    'placement_xy_under_8cm': bool(np.linalg.norm(delta[index, :2]) < .08),
                    'placement_z_0_to_15cm': bool(0 < delta[index, 2] < .15),
                    'bottle_speed_under_1cm_s': bool(np.linalg.norm(bottle[index, 7:10]) < .01),
                    'gripper_open': bool(gripper[index] < .05), 'arm_force_under_5N': bool(forces[index] < 5),
                    'no_other_termination': not any(bool(value[index]) for name, value in terminations.items() if name != 'success'),
                    'finite_states': finite,
                }
                report['environments'].append({
                    'env_index': index, 'origin_world': origin_np[index].tolist(),
                    'diagnostic_pick_place_success': all(checks.values()), 'checks': checks,
                    'lift_height_m': float(lift[index]), 'max_arm_contact_force_n': float(forces[index]),
                    'final_bottle_state_world': bottle[index].tolist(), 'final_bowl_state_world': bowl[index].tolist(),
                    'final_bottle_position_local': (bottle[index, :3] - origin_np[index]).tolist(),
                    'final_placement_delta_m': delta[index].tolist(), 'final_joint_positions': final_q[index].tolist(),
                    'source_final_bottle_position_difference_m': float(np.linalg.norm(
                        bottle[index, :3] - origin_np[index] - source_final['mustard'][:3])),
                })
            successes = sum(row['diagnostic_pick_place_success'] for row in report['environments'])
            report.update(completed=True, finite_states=finite, diagnostic_pick_place_success_count=successes,
                          diagnostic_successful_replays_per_hour=successes * 3600 / elapsed)
            report['timings_seconds']['final_checks_readback_s'] = time.perf_counter() - tick
            tick = time.perf_counter()
            np.savez(args.output / 'states.npz', **arrays, actions=actions_np, phase=np.asarray(phases, dtype=str),
                     environment_origins=origin_np, dt=e.step_dt, joint_names=np.asarray(robot.joint_names, dtype=str))
            if args.record_rgb:
                for index, output in outputs.items():
                    write_environment_trajectory(output, arrays, index, origin_np[index], actions_np, phases,
                                                 e.step_dt, robot.joint_names, robot.body_names, contact_names)
                    diagnostic = report['environments'][index]
                    diagnostic.update(output=str(output), rgb_frames=report['camera_frames_by_env'][str(index)])
                    result = dict(success=diagnostic['diagnostic_pick_place_success'], steps=count,
                                  diagnostic_only=True, production_quota_eligible=False,
                                  semantics=report['semantics'], source_episode=str(args.source_episode),
                                  source_steps=total, full_source_replay=count == total,
                                  variant=source_result['variant'], obstacle_id=source_result['obstacle_id'],
                                  layout=spec['layout'], physical_diagnostic=diagnostic,
                                  coordinate_frame='environment_local', environment_origin_world_m=origin_np[index].tolist(),
                                  fixed_render_warmup=report['fixed_render_warmup'],
                                  renderer_settings=report['renderer_settings'])
                    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            report['timings_seconds']['state_write_s'] = time.perf_counter() - tick
            report['state_semantics'] = 'states[t, env] precedes shared action[t]; final state is states[T]; xyz are world coordinates including environment origins; quaternion xyzw'
        except Exception:
            report['completed'] = False
            report['error'] = traceback.format_exc()
            raise
        finally:
            if cameras:
                try:
                    cameras.close()
                    report['camera_timings_seconds'] = dict(cameras.timings)
                    report['camera_frames_by_env'] = {str(i): frames for i, frames in cameras.frames.items()}
                except Exception:
                    report['camera_close_error'] = traceback.format_exc()
            for name, method in sim_methods.items():
                setattr(env.unwrapped.sim, name, method)
            report['timings_seconds']['total_before_close_s'] = time.perf_counter() - started
            report['timing_semantics'] = (
                'Replay loop includes env.step, GPU state clones/contact-force reduction and, when record_rgb, all three views and JPEG/H5 writes. '
                'Only RGB uses per-frame host image readback; state readback is deferred. Initialization includes startup and warm-up; '
                'replay excludes initialization, final checks, state write and camera close. physics_sim_step_call_s and internal_sim_render_s '
                'are nested within physics_step_call_s (env.step), not additive. Timers are host wall times; asynchronous GPU work can be charged '
                'to render_capture_s/readback. CUDA synchronizes at loop boundaries, not individual substeps. No planning or FCL validation. '
                'full_source_equivalents_per_hour normalizes short prefixes by their fraction of source length; it is not measured successful episodes/hour.')
            if report.get('completed'):
                report['end_to_end_diagnostic_successes_per_hour'] = report['diagnostic_pick_place_success_count'] * 3600 / report['timings_seconds']['total_before_close_s']
            (args.output / 'benchmark.json').write_text(json.dumps(report, indent=2) + '\n')
            print('VECTORIZED_RESULT', json.dumps({key: report.get(key) for key in (
                'completed', 'num_envs', 'executed_control_steps', 'diagnostic_pick_place_success_count',
                'aggregate_physics_steps_per_second', 'timings_seconds', 'error')}), flush=True)
            if env is not None:
                env.close()


if __name__ == '__main__':
    main()
