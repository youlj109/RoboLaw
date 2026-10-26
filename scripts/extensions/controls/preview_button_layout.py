"""Static human placement preview, without planning or commanded robot motion."""
import argparse
import hashlib
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--controls-task', choices=['left', 'right'], required=True)
parser.add_argument('--config', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.enable_cameras = True
args.output.mkdir(parents=True, exist_ok=False)
config = json.loads(args.config.read_text())

with SimulationAppContext(args):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.controls.environment import build_environment
    from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
    from kitchen_safety.dataset import CameraWriter, numpy as arr
    from kitchen_safety.planning import DroidKinematics
    from search_avoidance import fixed_shapes, minimum

    envdef, _ = build_environment(variant='human', fixed_pose=config['robot_pose_xyzw'],
        trial_config=config, task=args.controls_task)
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42,
        device=args.device, presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot = e.scene['robot']
    before = arr(robot.data.joint_pos)[0].copy()
    writer = CameraWriter(args.output, e.scene,
        envdef.embodiment.camera_config.camera_names(), e.step_dt, previews=True)
    for _ in range(48):
        e.sim.render()
    writer.write('layout')
    writer.close()
    after = arr(robot.data.joint_pos)[0]
    pose = config['robot_pose_xyzw']
    kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:]).as_euler('xyz', degrees=True)[2])
    world = ControlsCollisionWorld(e.sim.stage, kin, env_root='/World/envs/env_0')
    human = [row for row in world.objects if '/person/' in row[0]]
    kitchen = [row for row in world.objects if '/kitchen/' in row[0]]
    q = before[[robot.joint_names.index('panda_joint'+str(i)) for i in range(1, 8)]]
    shapes = world.shapes(q, dict(zip(robot.joint_names, before)))
    spec = config['variants']['human']
    arm = json.loads((Path(spec['asset']).parent/'pose.json').read_text())
    rotation = Rotation.from_euler('z', spec['yaw_deg'], degrees=True)
    hand = rotation.apply(np.array(arm['after']['hand_r']))+np.array(spec['position'])
    report = {
        'scope': 'Static layout draft only. No planning, actions or trajectory validation. '
                 'Reset initializes the scene; subsequent capture uses pure rendering.',
        'accepted_for_collection': False,
        'config': str(args.config),
        'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
        'human': spec, 'hand_bone_endpoints_world_m': hand.tolist(),
        'robot_q_changed_during_render': not np.array_equal(before, after),
        'initial_robot_human': minimum(shapes, human),
        'fixed_base_human': minimum(fixed_shapes(pose), human),
        'kitchen_human': minimum(kitchen, human),
    }
    (args.output/'layout_review.json').write_text(json.dumps(report, indent=2)+'\n')
    print('STATIC_LAYOUT', json.dumps(report), flush=True)
    env.close()
