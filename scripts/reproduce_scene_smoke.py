"""Load a default scene, hold the robot, and save the original three RGB views.

Worker for reproduce_project.py. This is an installation/rendering smoke test,
not a grasp/place episode or a benchmark success.
"""
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--variant', choices=['human', 'property', 'robot'], default='human')
parser.add_argument('--steps', type=int, default=30)
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
        KitchenSafetyCfg(variant=args.variant, enable_cameras=True))
    builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
        device=args.device, presets='physx', num_envs=1, solve_relations=False,
        disable_fabric=False))
    env = builder.make_registered()
    try:
        env.reset()
        scene = env.unwrapped.scene
        robot = scene['robot']
        ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)
        q = cpu(robot.data.joint_pos)[0, ids]
        action = torch.tensor([list(q) + [0.]], device=env.unwrapped.device, dtype=torch.float32)
        for _ in range(args.steps):
            with torch.inference_mode():
                env.step(action)
        cameras = {}
        for name in environment.embodiment.camera_config.camera_names():
            camera = scene[name]
            rgb = cpu(camera.data.output['rgb'])[0, :, :, :3]
            if rgb.shape != (720, 1280, 3) or not np.isfinite(rgb).all() or rgb.std() <= 1:
                raise RuntimeError(f'{name}: invalid or blank RGB image, shape={rgb.shape}')
            Image.fromarray(rgb.astype(np.uint8)).save(output / f'{name}.jpg')
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
        report = {'passed': True, 'scope': 'Scene/physics/RGB smoke; robot held at initial pose, no pick/place test.',
                  'variant': args.variant, 'steps': args.steps,
                  'dt': float(env.unwrapped.step_dt), 'cameras': cameras, 'rigid_states': state}
        (output / 'smoke.json').write_text(json.dumps(report, indent=2) + '\n')
        print('REPRODUCTION_SCENE_READY', args.variant, output, flush=True)
    finally:
        env.close()
