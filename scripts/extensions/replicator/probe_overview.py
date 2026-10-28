"""Render one actual initialized G-shape human scene before recording a rollout."""
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--output', required=True)
parser.add_argument('--eye', nargs=3, type=float)
parser.add_argument('--target', nargs=3, type=float)
args = parser.parse_args()
args.enable_cameras = True
output = Path(args.output)
output.mkdir(parents=True, exist_ok=False)

with SimulationAppContext(args):
    import numpy as np
    import torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.replicator.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from kitchen_safety.extensions.replicator.overview import configure_overview, OVERVIEW_SIZE
    from kitchen_safety.extensions.replicator.layout import resolve_layout
    from kitchen_safety.dataset import CameraWriter, numpy as arr, decode_video

    obstacle_id = 'human_default_mpfb'
    seed = args.seed if args.seed is not None else 42
    environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
        variant='human', enable_cameras=True, collection_seed=seed,
        obstacle_id=obstacle_id, episode_length_s=180.))
    builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
        num_envs=1, seed=seed, device=args.device, presets='physx',
        solve_relations=False, disable_fabric=False))
    config, kwargs = builder.compose_manager_cfg()
    original_camera_names = list(environment.embodiment.camera_config.camera_names())
    camera = configure_overview(config.scene, eye=args.eye, target=args.target)
    config.observations.camera_obs = None
    config.sim.log_dir = str(output / 'logs')
    config.sim.render_interval = config.decimation
    env = builder.make_registered(config, kwargs)
    env.reset()
    actual_env = env.unwrapped
    robot = actual_env.scene['robot']
    joint_ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)
    q = arr(robot.data.joint_pos)[0, joint_ids]
    for _ in range(8):
        env.step(torch.tensor(np.r_[q, 0.][None], device=actual_env.device, dtype=torch.float32))
    writer = CameraWriter(output, actual_env.scene, ['overview_camera'], actual_env.step_dt, previews=True)
    warmup = writer.warm_up(actual_env.sim, min_frames=64, max_frames=256)
    writer.write('overview_probe')
    writer.close()
    decoded = decode_video(output / 'overview_camera.mp4', 1,
                           expected_size=OVERVIEW_SIZE, expected_dt=actual_env.step_dt)
    (output / 'probe.json').write_text(json.dumps({
        'camera': camera, 'original_camera_names': original_camera_names,
        'layout': resolve_layout(obstacle_id, 0., seed),
        'warmup': warmup, 'frames': decoded,
        'scope': 'One actual initialized scene render, not task success and not a collection sample.',
    }, indent=2))
    env.close()
