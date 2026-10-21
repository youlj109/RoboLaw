"""Measure the actual kitchen physics without planning or FCL checks."""
import argparse
import json
import time
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--output', required=True)
parser.add_argument('--obstacle-id', default='property_vase')
parser.add_argument('--steps', type=int, default=160)
parser.add_argument('--solver-iterations', type=int, default=0)
parser.add_argument('--physics-threads', type=int, default=None)
parser.add_argument('--num-envs', dest='num_envs', type=int, default=argparse.SUPPRESS)
parser.add_argument('--baseline-only', action='store_true',
                    help='Measure two baseline windows without changing Fabric settings')
args = parser.parse_args()
args.enable_cameras = False

with SimulationAppContext(args):
    import carb
    import numpy as np
    import torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab.sensors import ContactSensorCfg
    from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from kitchen_safety.step_profiling import StepWallProfiler
    from isaaclab_physx.physics.physx_manager import PhysxManager

    if args.physics_threads is not None:
        import omni.physx
        from omni.physx.bindings._physx import SETTING_NUM_THREADS
        carb.settings.get_settings().set_int(SETTING_NUM_THREADS, args.physics_threads)
        omni.physx.get_physx_interface().set_thread_count(args.physics_threads)

    environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
        variant='property', enable_cameras=False, obstacle_id=args.obstacle_id))
    builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
        device=args.device, seed=42, presets='physx', num_envs=args.num_envs,
        env_spacing=args.env_spacing,
        solve_relations=False, disable_fabric=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    if args.solver_iterations:
        cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count = args.solver_iterations
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    e.render_enabled = False
    robot = e.scene['robot']
    ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)
    positions = robot.data.joint_pos
    q = (positions.torch if hasattr(positions, 'torch') else positions)[:, ids]
    action = torch.cat((q, q.new_zeros((args.num_envs, 1))), dim=1)
    for _ in range(12):
        env.step(action)

    settings = carb.settings.get_settings()
    keys = ['fabricUpdateTransformations', 'fabricUpdateVelocities', 'fabricUseGPUInterop',
            'updateToUsd', 'updateVelocitiesToUsd', 'suppressReadback', 'numThreads', 'physxDispatcher']
    original = {key: settings.get('/physics/' + key) for key in keys}
    report = {'device': args.device, 'num_envs': args.num_envs,
              'dt': cfg.sim.dt, 'decimation': cfg.decimation,
              'original_settings': original, 'solver_iterations': args.solver_iterations or 64,
              'physics_threads': settings.get('/persistent/physics/numThreads'),
              'cases': [], 'semantics': 'Static hold; no planning, FCL or image capture. Diagnostic only.'}
    print('PHYSICS_SETTINGS', json.dumps(report), flush=True)

    cases = [('baseline', {}), ('gpu_interop', {'fabricUseGPUInterop': True}),
             ('no_fabric_outputs', {'fabricUpdateTransformations': False, 'fabricUpdateVelocities': False}),
             ('baseline_repeat', {})]
    if args.baseline_only:
        cases = [('baseline', {}), ('baseline_repeat', {})]
    for name, changes in cases:
        for key, value in original.items():
            if value is not None:
                settings.set('/physics/' + key, value)
        for key, value in changes.items():
            settings.set('/physics/' + key, value)
        # Exercise the identical update/command loop without any recording callbacks.
        for _ in range(4):
            env.step(action)
        profiler = StepWallProfiler()
        PhysxManager._step_wall_profiler = profiler
        started = time.perf_counter()
        profiler.begin(name)
        for _ in range(max(1, args.steps // cfg.decimation)):
            env.step(action)
        elapsed = time.perf_counter() - started
        profiler.finish(elapsed)
        result = {'name': name, 'settings': changes, 'seconds': elapsed,
                  'physics_steps': (args.steps // cfg.decimation) * cfg.decimation,
                  'profile': profiler.report()}
        report['cases'].append(result)
        print('PHYSICS_CASE', json.dumps({k: v for k, v in result.items() if k != 'profile'}), flush=True)
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(report, indent=2))
    env.close()
