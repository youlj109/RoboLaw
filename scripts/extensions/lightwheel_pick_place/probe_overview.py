"""Render a single actual scene overview before collecting the human rollout."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--output',required=True);a=p.parse_args();a.enable_cameras=True
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
with SimulationAppContext(a):
    import numpy as np,torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
    from kitchen_safety.extensions.lightwheel_pick_place.overview import configure_overview
    from kitchen_safety.dataset import CameraWriter,numpy as arr,decode_video
    d,_=build_environment(enable_cameras=True,variant='human');b=ArenaEnvBuilder(d,ArenaEnvBuilderCfg(num_envs=1,seed=42,device='cpu',presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=42));cfg,kw=b.compose_manager_cfg();cfg.observations.camera_obs=None
    camera=configure_overview(cfg.scene);cfg.sim.log_dir=str(out/'logs');cfg.sim.render_interval=cfg.decimation
    env=b.make_registered(cfg,kw);env.reset();e=env.unwrapped;robot=e.scene['robot'];ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True);q=arr(robot.data.joint_pos)[0,ids]
    for _ in range(20):env.step(torch.tensor(np.r_[q,0.][None],device=e.device,dtype=torch.float32))
    w=CameraWriter(out,e.scene,['overview_camera'],e.step_dt,previews=True);warm=w.warm_up(e.sim,min_frames=64,max_frames=256);w.write('overview_probe');w.close()
    decoded=decode_video(out/'overview_camera.mp4',1,expected_size=(1280,720),expected_dt=e.step_dt)
    (out/'probe.json').write_text(json.dumps({'camera':camera,'warmup':warm,'frames':decoded,'scope':'One actual initialized scene render, not task success and not a collection sample.'},indent=2));env.close()
