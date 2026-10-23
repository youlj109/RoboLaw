"""Recorded source-reset neutral trial for C02/C03; no target-state writes."""
import json,hashlib,traceback
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--task-id',required=True);p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=40.);p.add_argument('--mobility',action='store_true');p.add_argument('--variant-config',default='configs/extensions/doors/c02_c03_initial_lwx.json')
a=p.parse_args();a.enable_cameras=True
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
with SimulationAppContext(a):
 import numpy as np,torch,warp as wp
 from types import SimpleNamespace
 if not hasattr(wp,'torch'):wp.torch=SimpleNamespace(device_from_torch=wp.device_from_torch)
 torch.set_num_threads(4)
 from scipy.spatial.transform import Rotation
 from pxr import Usd,UsdGeom,UsdPhysics
 from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
 from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
 from isaaclab.sensors import ContactSensorCfg
 from kitchen_safety.extensions.doors.task_environment import build,ROOT
 from kitchen_safety.extensions.doors.video import DoorCameraWriter
 from kitchen_safety.extensions.doors.task_observation import MechanismObserver
 from kitchen_safety.dataset import numpy as arr,decode_video
 from kitchen_safety.grasp_contacts import _numpy as contact_arr
 from kitchen_safety.planning import DroidKinematics
 from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
 environment,graph=build(a.task_id,variant_config=a.variant_config)
 builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device='cpu',seed=42,presets='physx',num_envs=1,solve_relations=True,resolve_on_reset=False,recorder_dataset_export_dir_path=str(out/'arena')))
 cfg,kw=builder.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.sim.render_interval=cfg.decimation;cfg.terminations.success=None
 cfg.scene.robot.spawn.activate_contact_sensors=True
 cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
 env=builder.make_registered(cfg,kw);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot']
 mechanisms=MechanismObserver(e.sim.stage,scene['robot_contacts']._physics_sim_view,contact_arr)
 ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True);q=arr(robot.data.joint_pos)[0,ids]
 base=arr(robot.data.root_pos_w)[0];yaw=Rotation.from_quat(arr(robot.data.root_quat_w)[0]).as_euler('xyz',degrees=True)[2]
 world=DoorCollisionWorld(e.sim.stage,DroidKinematics(base,yaw),env_root='/World/envs/env_0')
 from kitchen_safety.extensions.doors.task_model import resolve
 geometry=resolve(e.sim.stage,a.task_id,world)
 metadata={'door_geometry':geometry,'task':a.task_id,'robot_base':base.tolist(),'robot_yaw_deg':float(yaw),'robot_asset_pose':{'position':list((environment.embodiment.get_initial_pose().poses[0] if hasattr(environment.embodiment.get_initial_pose(),'poses') else environment.embodiment.get_initial_pose()).position_xyz),'rotation':list((environment.embodiment.get_initial_pose().poses[0] if hasattr(environment.embodiment.get_initial_pose(),'poses') else environment.embodiment.get_initial_pose()).rotation_xyzw)},'robot_usd':environment.embodiment.scene_config.robot.spawn.usd_path,'mechanism_inventory':mechanisms.inventory,'mechanism_mapping_issues':mechanisms.issues,'physics_dt':cfg.sim.dt,'dt':e.step_dt,'robot_joint_names':robot.joint_names,'initial_q':arr(robot.data.joint_pos)[0].tolist(),'source_task':graph.model_dump(mode='json') if hasattr(graph,'model_dump') else str(graph)}
 (out/'metadata.json').write_text(json.dumps(metadata,indent=2,default=str))
 samples=[];checks=[];action_index=0;failure=None
 def record():
  step=int(e.sim.get_physics_step_count())
  if samples and samples[-1]['physics_step']==step:return
  samples.append({'physics_step':step,'action_index':action_index,'mechanisms':mechanisms.read(),'q':arr(robot.data.joint_pos)[0].tolist()})
 original=scene.update
 def update(dt):original(dt);record()
 scene.update=update
 def apply_efforts():
  if a.mobility:
    for path,view in mechanisms.views.items():
     names=next(v['joint_names'] for v in mechanisms.inventory if v['path']==path)
     forces=np.zeros_like(contact_arr(view.get_dof_positions()),dtype=np.float32)
     for index,name in enumerate(names):
      if name in ('handle_temp_joint','handle_joint') or 'knob' in name:
       forces[0,index]=1.0 if 1.0 <= action_index*e.step_dt < 1.5 else 0.0
     view.set_dof_actuation_forces(wp.array(forces,dtype=wp.float32,device=view.get_dof_positions().device),wp.array([0],dtype=wp.uint32,device=view.get_dof_positions().device))
 original_sim_step=e.sim.step
 def sim_step(*args,**kwargs):
  apply_efforts()
  return original_sim_step(*args,**kwargs)
 e.sim.step=sim_step
 cam=DoorCameraWriter(out,scene,environment.embodiment.camera_config.camera_names(),e.step_dt)
 try:
  record()
  for action_index in range(int(np.ceil(a.seconds/e.step_dt))):
   cam.write('finite_torque_mobility' if a.mobility else 'neutral_hold');env.step(torch.tensor(np.r_[q,0.][None],dtype=torch.float32,device=e.device))
   angles=dict(zip(robot.joint_names,arr(robot.data.joint_pos)[0]));ok=world.valid(arr(robot.data.joint_pos)[0,ids],mode='execution',gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES})
   checks.append({'action_index':action_index,'valid':ok,'failure':world.last_clearance})
   if action_index%100==0:print('NEUTRAL',action_index,'safe',ok,flush=True)
 except Exception as exc:failure=str(exc);traceback.print_exc()
 finally:
  cam.close();(out/'physics.json').write_text(json.dumps(samples));(out/'audit.json').write_text(json.dumps(checks))
  result={'mode':'finite_torque_mobility' if a.mobility else 'neutral','external_pulse_nm':1.0 if a.mobility else 0.0,'completed':failure is None,'mechanism_mapping_passed':not mechanisms.issues,'failure':failure,'frames':len(checks),'physics_samples':len(samples),'all_robot_clearances_valid':all(x['valid'] for x in checks)}
  (out/'result.json').write_text(json.dumps(result,indent=2));print('RESULT',result,flush=True);env.close()
