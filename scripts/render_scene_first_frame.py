"""Render the original three cameras at reset, without advancing physics."""
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--variant', choices=['human', 'property', 'robot'], default='human')
parser.add_argument('--obstacle-id', default='')
parser.add_argument('--output', required=True)
args = parser.parse_args()
args.enable_cameras = True
output = Path(args.output)
output.mkdir(parents=True, exist_ok=False)

with SimulationAppContext(args):
    import numpy as np
    import torch
    from PIL import Image
    from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg

    def cpu(value):
        return (value.torch if hasattr(value, 'torch') else value).detach().cpu().numpy()

    environment = KitchenSafetyEnvironment().build(
        KitchenSafetyCfg(variant=args.variant, obstacle_id=args.obstacle_id, collection_seed=42, enable_cameras=True))
    builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
        device=args.device, seed=42, presets='physx', num_envs=1, solve_relations=False,
        disable_fabric=False))
    env = builder.make_registered()
    try:
        env.reset()
        scene = env.unwrapped.scene
        initial_step = env.unwrapped.sim.get_physics_step_count()
        for _ in range(32):
            env.unwrapped.sim.render()
        cameras = {}
        for name in environment.embodiment.camera_config.camera_names():
            camera = scene[name]
            camera.update(0., force_recompute=True)
            rgb = cpu(camera.data.output['rgb'])[0, :, :, :3]
            if rgb.shape != (720, 1280, 3) or not np.isfinite(rgb).all() or rgb.std() <= 1:
                raise RuntimeError(f'{name}: invalid or blank RGB image, shape={rgb.shape}')
            Image.fromarray(rgb.astype(np.uint8)).save(output / f'{name}.png')
            cameras[name] = {
                'width': 1280, 'height': 720,
                'intrinsics': cpu(camera.data.intrinsic_matrices)[0].tolist(),
                'offset': {'position': list(camera.cfg.offset.pos),
                           'quaternion_xyzw': list(camera.cfg.offset.rot),
                           'convention': camera.cfg.offset.convention},
                'position_world': cpu(camera.data.pos_w)[0].tolist(),
                'quaternion_world_ros_xyzw': cpu(camera.data.quat_w_ros)[0].tolist(),
                'rgb_std': float(rgb.std()),
            }
        if len(cameras) != 3:
            raise RuntimeError(f'Expected the original three DROID cameras, got {list(cameras)}')
        state = {name: cpu(obj.data.root_state_w)[0].tolist()
                 for name, obj in scene.rigid_objects.items()}
        report = {'passed': True, 'scope': 'First frame after reset; 32 render-only warmup updates; no actions or physics steps after reset.',
                  'variant': args.variant, 'obstacle_id': args.obstacle_id, 'seed': 42,
                  'reset_physics_step': initial_step, 'capture_physics_step': env.unwrapped.sim.get_physics_step_count(),
                  'dt': float(env.unwrapped.step_dt), 'cameras': cameras, 'rigid_states': state}
        assert env.unwrapped.sim.get_physics_step_count() == initial_step
        (output / 'metadata.json').write_text(json.dumps(report, indent=2) + '\n')
        print('REPRODUCTION_SCENE_READY', args.variant, output, flush=True)
    finally:
        env.close()
