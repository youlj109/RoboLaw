"""Diagnostic initialization at saved resting poses; never counts as task collection."""
import json,copy
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--solver-position',type=int);p.add_argument('--solver-velocity',type=int);p.add_argument('--seconds',type=float,default=20.)
a=p.parse_args();a.enable_cameras=False
if a.output.exists():raise FileExistsError(a.output)
with SimulationAppContext(a):
 import numpy as np,torch,h5py
 from pxr import PhysxSchema,Usd,UsdPhysics
 from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
 from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
 from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
 from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec,picks
 from kitchen_safety.dataset import numpy as arr
 r=json.load(open(a.episode/'result.json'));spec=episode_spec(r);d,_=build_environment(enable_cameras=False,variant='base',task_id=spec.task_id,fixed_layout_path=r['fixed_layout_source']['path'])
 builder=ArenaEnvBuilder(d,ArenaEnvBuilderCfg(num_envs=1,seed=43,device='cpu',presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=42));cfg,kw=builder.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.episode_length_s=max(cfg.episode_length_s,a.seconds+60.)
 for key,value in list(vars(cfg.scene).items()):
  if value is not None and 'CameraCfg' in type(value).__name__:setattr(cfg.scene,key,None)
 before={}
 for n in (*picks(spec),spec.destination_object):
  actor=getattr(cfg.scene,n);props=actor.spawn.rigid_props;print('SPAWN_PROPS',n,type(props).__name__,str(props),flush=True);before[n]=str(props)
  if a.solver_position is not None or a.solver_velocity is not None:
   from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg
   props=copy.deepcopy(props) if props is not None else PhysxRigidBodyPropertiesCfg()
   if a.solver_position is not None:props.solver_position_iteration_count=a.solver_position
   if a.solver_velocity is not None:props.solver_velocity_iteration_count=a.solver_velocity
   actor.spawn.rigid_props=props
 env=builder.make_registered(cfg,kw);env.reset();e=env.unwrapped;e._reset_idx=lambda ids:None;robot=e.scene['robot'];torch.set_num_threads(2)
 with h5py.File(a.episode/'trajectory.h5') as h:
  for n in spec.rigid_objects:
   pose=h['next_obs/'+n][-1,:7];e.scene[n].write_root_pose_to_sim(torch.tensor(pose[None],device=e.device));e.scene[n].write_root_velocity_to_sim(torch.zeros((1,6),device=e.device))
 ids=[robot.joint_names.index('panda_joint'+str(i)) for i in range(1,8)];q=arr(robot.data.joint_pos)[0,ids];action=torch.tensor(np.r_[q,0.][None],device=e.device,dtype=torch.float32);rows={n:[] for n in (*picks(spec),spec.destination_object)};attrs={}
 for n in rows:
  prims=[p for p in Usd.PrimRange(e.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n),Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  attrs[n]=[{x:p.GetAttribute(x).Get() for x in ['physxRigidBody:solverPositionIterationCount','physxRigidBody:solverVelocityIterationCount','physxRigidBody:sleepThreshold']} for p in prims]
 for i in range(int(a.seconds/e.step_dt)):
  env.step(action)
  for n in rows:rows[n].append(arr(e.scene[n].data.root_state_w)[0].copy())
 summary={}
 for n,states in rows.items():
  x=np.array(states);v=np.linalg.norm(x[-60:,7:10],axis=1);summary[n]={'last4s_speed_min_median_max_m_s':np.quantile(v,[0,.5,1]).tolist(),'last4s_max_translation_m':float(np.linalg.norm(x[-60:,:3]-x[-60,:3],axis=1).max()),'final_state':x[-1].tolist()}
 report={'scope':'DIAGNOSTIC ONLY: initialized saved final poses and zero velocities before controls. No grasp/lift, no task success claim, cannot replace any actual collection.','source_episode':str(a.episode),'seconds':a.seconds,'solver_position':a.solver_position,'solver_velocity':a.solver_velocity,'original_spawn_properties':before,'runtime_usd_solver_properties':attrs,'summary':summary};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2));print('REST_SOLVER_DIAGNOSTIC',json.dumps(report),flush=True);env.close()
