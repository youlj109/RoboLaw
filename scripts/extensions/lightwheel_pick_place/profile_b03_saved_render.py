"""Static layout editing preview only: no trajectory, physics stepping or success claim."""
import json,hashlib
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--task',required=True);p.add_argument('--variant',required=True);p.add_argument('--variant-config',required=True);p.add_argument('--fixed-layout',required=True);p.add_argument('--display-layout',required=True);p.add_argument('--export-scene',action='store_true');p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.enable_cameras=True;a.output.mkdir(parents=True,exist_ok=False)
with SimulationAppContext(a):
 import cv2,numpy as np,torch,copy
 from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
 from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
 from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
 from kitchen_safety.extensions.lightwheel_pick_place.overview import configure_overview
 from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task
 from kitchen_safety.dataset import CameraWriter,numpy as arr
 d,_=build_environment(enable_cameras=True,variant=a.variant,task_id=a.task,fixed_layout_path=a.fixed_layout,variant_config_path=a.variant_config)
 b=ArenaEnvBuilder(d,ArenaEnvBuilderCfg(num_envs=1,seed=43,device=a.device,presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=42));cfg,kw=b.compose_manager_cfg();cfg.observations.camera_obs=None
 spec_for_view=resolve_task(a.task);view_poses=json.load(open(a.display_layout))['original_asset_initial_poses'];view_target=np.mean([view_poses[n]['position_xyz'] for n in (spec_for_view.pick_object,spec_for_view.destination_object)],axis=0);is_banana=len(spec_for_view.pick_objects)>1
 camera=configure_overview(cfg.scene,eye=[.6,-1.3,2.3] if is_banana else (view_target+np.array([.8,-1.5,1.3])).tolist(),target=[2.15,-2.65,.9] if is_banana else view_target.tolist());cfg.sim.log_dir=str(a.output/'logs');cfg.sim.render_interval=cfg.decimation
 if a.variant=='human' and is_banana:
  main_camera=cfg.scene.overview_camera
  configure_overview(cfg.scene,eye=[.8,-4.6,2.1],target=[1.85,-2.8,.8])
  cfg.scene.person_review_camera=copy.deepcopy(cfg.scene.overview_camera);cfg.scene.person_review_camera.prim_path='{ENV_REGEX_NS}/PersonReviewCamera';cfg.scene.overview_camera=main_camera
 env=b.make_registered(cfg,kw);env.reset();e=env.unwrapped;spec=resolve_task(a.task);poses=json.load(open(a.display_layout))['original_asset_initial_poses']
 # This is scene editing for visual review, never a task demonstration or collection.
 for name in spec.rigid_objects:
  row=poses[name];pose=torch.tensor([row['position_xyz']+row['rotation_xyzw']],device=e.device,dtype=torch.float32)
  e.scene[name].write_root_pose_to_sim(pose);e.scene[name].write_root_velocity_to_sim(torch.zeros((1,6),device=e.device))
 robot=e.scene['robot'];positions=arr(robot.data.joint_pos).copy()
 for i,angle in enumerate([0.,-60.,0.,-150.,0.,90.,0.],1):positions[:,robot.joint_names.index('panda_joint'+str(i))]=np.deg2rad(angle)
 robot.write_joint_state_to_sim(torch.tensor(positions,device=e.device,dtype=torch.float32),torch.zeros(positions.shape,device=e.device,dtype=torch.float32))
 names=list(d.embodiment.camera_config.camera_names())+['overview_camera']+(['person_review_camera'] if a.variant=='human' and is_banana else []);w=CameraWriter(a.output,e.scene,names,e.step_dt,previews=True);start=int(e.sim.get_physics_step_count());w.warm_up(e.sim,min_frames=2,max_frames=4,sample_every=1,stable_windows=2)
 import time
 from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
 timings=[]
 for refresh in (True,False,True):
  for name in names:e.scene[name].cfg.update_latest_camera_pose=refresh
  row={'update_latest_camera_pose':refresh,'cameras':{}}
  tick=time.perf_counter();e.sim.render();row['sim_render_s']=time.perf_counter()-tick
  tick=time.perf_counter();ensure_isaac_rtx_render_update(force=True);row['ensure_render_s']=time.perf_counter()-tick
  for name in names:
   tick=time.perf_counter();e.scene[name].update(0.,force_recompute=True);update_s=time.perf_counter()-tick
   tick=time.perf_counter();rgb=arr(e.scene[name].data.output['rgb']);row['cameras'][name]={'update_s':update_s,'rgb_read_s':time.perf_counter()-tick,'shape':list(rgb.shape)}
  row['physics_steps']=int(e.sim.get_physics_step_count())-start
  if row['physics_steps']!=0:raise RuntimeError('Rendering probe unexpectedly advanced physics')
  timings.append(row);print('STATIC_RENDER_TIMING',json.dumps(row),flush=True)
 (a.output/'render_profile.json').write_text(json.dumps({'scope':'Rendering timing only, no Physics steps or acceptance data','measurements':timings},indent=2))
 for name in names:
  rgb=arr(e.scene[name].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
  if not cv2.imwrite(str(a.output/(name+'.jpg')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):raise RuntimeError('Image write failed')
 w.close();report={'variant':a.variant,'scope':'Static scene placement review only. Original task poses were set for display; physics was not advanced. No trajectory, physical stability or success claim. This entry is never used for data collection.','physics_steps_during_render':int(e.sim.get_physics_step_count())-start,'display_layout':a.display_layout,'variant_config':a.variant_config,'variant_config_sha256':hashlib.sha256(Path(a.variant_config).read_bytes()).hexdigest(),'variant_spec':json.load(open(a.variant_config)).get(a.variant,{}),'overview':camera}
 if a.export_scene:e.sim.stage.Export(str(a.output/'scene.usda'))
 (a.output/'layout.json').write_text(json.dumps(report,indent=2));print('LAYOUT_IMAGES_READY',a.output,flush=True);env.close()
