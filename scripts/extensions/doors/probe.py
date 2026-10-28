import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser(); p.add_argument('--variant', default='base');p.add_argument('--output',default='reports/skill_trials/doors/probe');a=p.parse_args();a.enable_cameras=True
with SimulationAppContext(a):
    import numpy as np,torch
    import warp as wp
    from types import SimpleNamespace
    if not hasattr(wp,"torch"):wp.torch=SimpleNamespace(device_from_torch=wp.device_from_torch)
    from scipy.spatial.transform import Rotation
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.doors.environment import build
    from kitchen_safety.dataset import CameraWriter,numpy as arr
    from pxr import UsdGeom,UsdPhysics,Usd
    environment,graph=build(a.variant,True)
    builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device='cpu',seed=42,presets='physx',num_envs=1,solve_relations=True))
    cfg,kw=builder.compose_manager_cfg();cfg.observations.camera_obs=None
    env=builder.make_registered(cfg,kw);env.reset();e=env.unwrapped
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    robot=e.scene['robot'];ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    q=arr(robot.data.joint_pos)[0,ids]
    for _ in range(10):env.step(torch.tensor(np.r_[q,0.][None],dtype=torch.float32,device=e.device))
    cam=CameraWriter(out,e.scene,environment.embodiment.camera_config.camera_names(),e.step_dt)
    for _ in range(3):cam.write('probe');env.step(torch.tensor(np.r_[q,0.][None],dtype=torch.float32,device=e.device))
    cam.close()
    cache=UsdGeom.XformCache();rows=[]
    for prim in e.sim.stage.Traverse():
        if 'fridge' in str(prim.GetPath()).lower() and (prim.IsA(UsdPhysics.Joint) or 'handle_main' in prim.GetName()):
            rows.append({'path':str(prim.GetPath()),'transform':np.array(cache.GetLocalToWorldTransform(prim)).T.tolist()})
    report={'robot_position':arr(robot.data.root_pos_w).tolist(),'robot_quaternion':arr(robot.data.root_quat_w).tolist(),'robot_body_names':robot.body_names,'robot_joint_names':robot.joint_names,'q':q.tolist(),'fridge_joint_names':e.scene['fridge'].joint_names,'fridge_joint_q':arr(e.scene['fridge'].data.joint_pos).tolist(),'fridge_limits':arr(e.scene['fridge'].data.joint_pos_limits).tolist(),'prims':rows,'dt':e.step_dt}
    (out/'probe.json').write_text(json.dumps(report,indent=2));print('DOOR_PROBE',json.dumps(report),flush=True)
    env.close()
