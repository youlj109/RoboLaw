"""Bounded bring-up diagnostic of the original lime placement task."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
parser = get_isaaclab_arena_cli_parser()
args = parser.parse_args()
args.enable_cameras = True
with SimulationAppContext(args):
    import numpy as np
    from pxr import UsdGeom, UsdPhysics
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
    from kitchen_safety.dataset import CameraWriter, numpy as arr
    envdef, assets = build_environment()
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42,
        device="cpu", presets='physx', solve_relations=True, resolve_on_reset=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot = e.scene['robot']
    out = Path('/workspace/reports/skill_trials/lightwheel_pick_place/inspection')
    out.mkdir(parents=True, exist_ok=True)
    data = {'robot_pose': arr(robot.data.root_state_w)[0].tolist(),
            'robot_joints': dict(zip(robot.joint_names, arr(robot.data.joint_pos)[0].tolist())),
            'objects': {name: arr(e.scene[name].data.root_state_w)[0].tolist() for name in ('lime','wooden_bowl','broccoli','sweet_potato','red_bell_pepper')}}
    cache=UsdGeom.BBoxCache(0,['default','render','proxy','guide'],False,True)
    data['bounds']={}
    for prim in e.sim.stage.Traverse():
        if prim.GetName() in ('lime','wooden_bowl','counter_top'):
            box=cache.ComputeWorldBound(prim).ComputeAlignedRange()
            data['bounds'][str(prim.GetPath())]=[list(box.GetMin()),list(box.GetMax())]
    (out/'state.json').write_text(json.dumps(data,indent=2))
    writer=CameraWriter(out,e.scene,envdef.embodiment.camera_config.camera_names(),e.step_dt,previews=True)
    for _ in range(32):e.sim.render()
    writer.write('initial');writer.close()
    e.sim.stage.Export(str(out/'stage.usda'))
    print('LIME_INSPECTION_COMPLETE',json.dumps(data),flush=True)
    env.close()
