"""Build/reset a kitchen variant and hold the robot's initial joint targets."""
from pathlib import Path
import json

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser=get_isaaclab_arena_cli_parser()
parser.add_argument('--variant',choices=['base','human','property','robot'],default='human')
parser.add_argument('--scene_config',type=str,default=None)
parser.add_argument('--steps',type=int,default=120)
parser.add_argument('--obstacle_id',default='')
args=parser.parse_args()
if args.scene_config:
    scene_config=json.loads(Path(args.scene_config).read_text())
    args.variant=scene_config['variant']
    args.obstacle_id=scene_config.get('obstacle_id',args.obstacle_id)

with SimulationAppContext(args):
    import torch
    from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    environment=KitchenSafetyEnvironment().build(KitchenSafetyCfg(variant=args.variant,enable_cameras=False,
                                                                 obstacle_id=args.obstacle_id))
    builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device=args.device,presets='physx',
                            num_envs=1,solve_relations=False,disable_fabric=True))
    env=builder.make_registered()
    env.reset()
    robot=env.unwrapped.scene['robot']
    joint_ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    action=torch.zeros((1,8),device=env.unwrapped.device)
    joint_pos=robot.data.joint_pos
    if hasattr(joint_pos,'torch'):joint_pos=joint_pos.torch
    action[:,:7]=joint_pos[:,joint_ids]
    action[:,7]=0.0
    for _ in range(args.steps):
        with torch.inference_mode():env.step(action)
    print('ARENA_SCENE_READY',args.variant,flush=True)
    state={}
    for name in ['mustard_bottle','bowl','glass','plate','phone']:
        if name in env.unwrapped.scene.rigid_objects:
            pos=env.unwrapped.scene[name].data.root_pos_w
            if hasattr(pos,'torch'):pos=pos.torch
            state[name]=pos.detach().cpu().tolist()
    state_name=args.obstacle_id or args.variant
    (Path('/workspace/reports')/f'arena_{state_name}_state.json').write_text(json.dumps(state,indent=2))
    env.close()
