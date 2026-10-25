"""Bounded physical layout preview with cameras and neutral arm; no planning."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--task',required=True);p.add_argument('--variant',required=True);p.add_argument('--variant-config');p.add_argument('--fixed-layout');p.add_argument('--output',required=True)
a=p.parse_args();a.enable_cameras=True
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
with SimulationAppContext(a):
    import cv2,numpy as np,torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
    from kitchen_safety.extensions.lightwheel_pick_place.overview import configure_overview
    from kitchen_safety.dataset import CameraWriter,numpy as arr
    d,assets=build_environment(enable_cameras=True,variant=a.variant,task_id=a.task,fixed_layout_path=a.fixed_layout,variant_config_path=a.variant_config)
    b=ArenaEnvBuilder(d,ArenaEnvBuilderCfg(num_envs=1,seed=43,device=a.device,presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=42))
    cfg,kw=b.compose_manager_cfg();cfg.observations.camera_obs=None;configure_overview(cfg.scene);cfg.sim.log_dir=str(out/'logs');cfg.sim.render_interval=cfg.decimation
    env=b.make_registered(cfg,kw);env.reset();e=env.unwrapped;robot=e.scene['robot'];ids=[robot.joint_names.index('panda_joint'+str(i)) for i in range(1,8)];q=arr(robot.data.joint_pos)[0,ids];command=torch.tensor(np.r_[q,0.][None],device=e.device,dtype=torch.float32)
    names=list(d.embodiment.camera_config.camera_names())+['overview_camera'];w=CameraWriter(out,e.scene,names,e.step_dt,previews=True);w.warm_up(e.sim,min_frames=64,max_frames=256)
    from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task
    spec=resolve_task(a.task)
    original={'original_asset_initial_poses':{n:assets[n]._get_initial_pose_as_pose().to_dict() for n in ('droid',)+spec.rigid_objects}}
    (out/'initial_layout.json').write_text(json.dumps(original,indent=2))
    states=[]
    for i in range(61):
        env.step(command)
        if i in (0,30,60):
            for name in names:
                rgb=arr(e.scene[name].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
                if not cv2.imwrite(str(out/(name+'_'+str(i)+'.jpg')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):raise RuntimeError('Image write failed')
            from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task
            row={'frame':i,'objects':{n:arr(e.scene[n].data.root_state_w)[0,:13].tolist() for n in resolve_task(a.task).rigid_objects}}
            if a.variant=='property':row['glass']=arr(e.scene['glass'].data.root_state_w)[0,:7].tolist()
            states.append(row)
            w.write('neutral_layout')
    w.close();(out/'layout.json').write_text(json.dumps({'variant':a.variant,'config':json.load(open(a.variant_config)).get(a.variant,{}) if a.variant_config else {},'hazard_states':states,'scope':'Neutral physical layout preview only; no MotionGen and no success claim'},indent=2));env.close()
