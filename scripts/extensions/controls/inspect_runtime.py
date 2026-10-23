"""Bounded bring-up diagnostic of the original left-cancel-button task."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--controls-task', choices=['left','right','temperature'], default='left')
parser.add_argument('--output', default='reports/skill_trials/controls/inspection')
parser.add_argument('--repair-oven-racks',action='store_true')
parser.add_argument('--balance-temperature-knob',action='store_true')
args = parser.parse_args()
args.enable_cameras = True
with SimulationAppContext(args):
    import numpy as np
    from pxr import UsdGeom, UsdPhysics
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.controls.environment import build_environment
    from kitchen_safety.dataset import CameraWriter, numpy as arr
    from kitchen_safety.extensions.controls.task_contract import task_contract
    contract=task_contract(args.controls_task)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    envdef, assets = build_environment(task=args.controls_task,repair_oven_racks=args.repair_oven_racks,
        balance_temperature_knob=args.balance_temperature_knob)
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42,
        device=args.device, presets='physx', solve_relations=True, resolve_on_reset=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot, toaster = e.scene['robot'], e.scene[contract['asset']]
    data = {'robot_pose': arr(robot.data.root_state_w)[0].tolist(),
            'robot_joints': dict(zip(robot.joint_names, arr(robot.data.joint_pos)[0].tolist())),
            'robot_bodies': robot.body_names,
            'toaster_joints': dict(zip(toaster.joint_names,arr(toaster.data.joint_pos)[0].tolist())),
            'toaster_joint_limits': arr(toaster.data.joint_pos_limits)[0].tolist(),
            'toaster_bodies': toaster.body_names,
            'toaster_body_positions': arr(toaster.data.body_pos_w)[0].tolist(),
            'task_id':contract['task_id'], 'target_joint':contract['joint'],
            'stage_button': [], 'joint_metadata': []}
    cache=UsdGeom.XformCache()
    for prim in e.sim.stage.Traverse():
        if any(token in str(prim.GetPath()) for token in ('cancel','knob_temp')):
            data['stage_button'].append({'path':str(prim.GetPath()), 'type':prim.GetTypeName(),
                'transform':np.array(cache.GetLocalToWorldTransform(prim)).T.tolist()})
        if prim.IsA(UsdPhysics.Joint) and contract['joint'] in str(prim.GetPath()):
            data['joint_metadata'].append({'path':str(prim.GetPath()),
                'attributes':{str(a.GetName()):str(a.Get()) for a in prim.GetAttributes()},
                'relationships':{str(r.GetName()):[str(t) for t in r.GetTargets()] for r in prim.GetRelationships()}})
    (out/'state.json').write_text(json.dumps(data,indent=2))
    writer=CameraWriter(out,e.scene,envdef.embodiment.camera_config.camera_names(),e.step_dt,previews=True)
    for _ in range(32):e.sim.render()
    writer.write('initial');writer.close()
    e.sim.stage.Export(str(out/'stage.usda'))
    print('CONTROLS_INSPECTION_COMPLETE',json.dumps(data),flush=True)
    env.close()
