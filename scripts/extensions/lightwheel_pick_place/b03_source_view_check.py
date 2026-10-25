"""Bounded native-contact view timing diagnostic, no planning or task-success claim."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.enable_cameras=False;a.output.mkdir(parents=True,exist_ok=False)
with SimulationAppContext(a):
 import numpy as np,torch
 from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
 from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
 from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
 from kitchen_safety.extensions.lightwheel_pick_place.task_config import BANANA_TASK,resolve_task
 from kitchen_safety.extensions.lightwheel_pick_place.contacts import LimeSourceContacts
 from kitchen_safety.dataset import numpy as arr
 from isaaclab.sensors import ContactSensorCfg
 spec=resolve_task(BANANA_TASK);d,_=build_environment(enable_cameras=False,task_id=BANANA_TASK,variant='human',fixed_layout_path='configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json',variant_config_path='configs/extensions/lightwheel_pick_place/variants_bananas_review_layout_v2.json');b=ArenaEnvBuilder(d,ArenaEnvBuilderCfg(num_envs=1,seed=43,device=a.device,presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=42));cfg,kw=b.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1);cfg.sim.log_dir=str(a.output/'logs');env=b.make_registered(cfg,kw);env.reset();e=env.unwrapped
 poses=json.load(open('configs/extensions/lightwheel_pick_place/fixed_layout_bananas_settled_review.json'))['original_asset_initial_poses']
 for name in spec.rigid_objects:
  row=poses[name];e.scene[name].write_root_pose_to_sim(torch.tensor([row['position_xyz']+row['rotation_xyzw']],device=e.device,dtype=torch.float32));e.scene[name].write_root_velocity_to_sim(torch.zeros((1,6),device=e.device))
 view=e.scene['robot_contacts']._physics_sim_view
 old={n:LimeSourceContacts(view,cfg.sim.dt,pick_object=n,source_surface=spec.source_surface_abs) for n in spec.pick_objects}
 robot=e.scene['robot'];ids=[robot.joint_names.index('panda_joint'+str(i)) for i in range(1,8)];q=arr(robot.data.joint_pos)[0,ids];command=torch.tensor(np.r_[q,0.][None],device=e.device,dtype=torch.float32);records=[]
 for i in range(601):
  env.step(command)
  if i in (19,59,299,599,600):
   record=dict(action=i,contacts={n:v.sample() for n,v in old.items()},poses={n:arr(e.scene[n].data.root_state_w)[0,:13].tolist() for n in spec.pick_objects});records.append(record);print('SOURCE_VIEWS',i,{n:r['count'] for n,r in record['contacts'].items()},flush=True)
  if i==599:
   fresh=LimeSourceContacts(view,cfg.sim.dt,pick_object='banana_1',source_surface=spec.source_surface_abs);print('FRESH_VIEW_IMMEDIATE',fresh.sample()['count'],flush=True)
 report={'scope':'Native source view initialization/rest timing only; no planning, task execution or success claim','records':records,'fresh_after_step':fresh.sample()};(a.output/'source_view_timing.json').write_text(json.dumps(report,indent=2));env.close()
