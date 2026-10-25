"""Task-scoped neutral hold: actual task rigid states, robot q/dq and original cameras."""
import hashlib,json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--task',required=True);p.add_argument('--fixed-layout',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--duration-seconds',type=float,default=120.)
a=p.parse_args();a.enable_cameras=True
if a.duration_seconds<2:raise ValueError('Hold duration must exceed 2 seconds')
a.output.mkdir(parents=True,exist_ok=False)
with SimulationAppContext(a):
    import numpy as np,torch
    from scipy.spatial.transform import Rotation
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
    from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task,picks
    from kitchen_safety.dataset import CameraWriter,numpy as arr
    spec=resolve_task(a.task);d,_=build_environment(enable_cameras=True,variant='base',task_id=a.task,fixed_layout_path=a.fixed_layout)
    b=ArenaEnvBuilder(d,ArenaEnvBuilderCfg(num_envs=1,seed=43,device=a.device,presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=42));cfg,kw=b.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.sim.log_dir=str(a.output/'logs');cfg.sim.render_interval=cfg.decimation
    env=b.make_registered(cfg,kw);env.reset();e=env.unwrapped;robot=e.scene['robot'];ids=[robot.joint_names.index('panda_joint'+str(i)) for i in range(1,8)];q0=arr(robot.data.joint_pos)[0].copy();command=torch.tensor(np.r_[q0[ids],0.][None],device=e.device,dtype=torch.float32)
    def reject_reset(env_ids):
        if len(env_ids):raise RuntimeError('Unexpected terminal reset during task neutral hold')
    e._reset_idx=reject_reset
    for _ in range(60):env.step(command)
    writer=CameraWriter(a.output,e.scene,d.embodiment.camera_config.camera_names(),e.step_dt,previews=True);writer.warm_up(e.sim,min_frames=64,max_frames=256)
    rows={n:[] for n in spec.rigid_objects};rows.update(q=[],dq=[],physics_step=[],camera_aligned=[])
    frames=int(np.ceil(a.duration_seconds/e.step_dt))
    for index in range(frames):
        step_before=int(e.sim.get_physics_step_count())
        for n in spec.rigid_objects:rows[n].append(arr(e.scene[n].data.root_state_w)[0].copy())
        rows['q'].append(arr(robot.data.joint_pos)[0].copy());rows['dq'].append(arr(robot.data.joint_vel)[0].copy());writer.write('task_neutral');rows['physics_step'].append(step_before);rows['camera_aligned'].append(step_before==int(e.sim.get_physics_step_count()));env.step(command)
        if index%300==0:print('TASK_NEUTRAL',index,frames,flush=True)
    writer.close();data={n:np.asarray(v) for n,v in rows.items()};np.savez_compressed(a.output/'task_neutral_states.npz',**data)
    tol={'task_translation_m':.001,'task_rotation_deg':1.,'distractor_translation_m':.002,'distractor_rotation_deg':1.,'robot_joint_rad':.001};task_names=(*picks(spec),spec.destination_object);objects={}
    for n in spec.rigid_objects:
        x=data[n];translation=float(np.linalg.norm(x[:,:3]-x[0,:3],axis=1).max());angle=float(np.rad2deg((Rotation.from_quat(x[:,3:7])*Rotation.from_quat(x[0,3:7]).inv()).magnitude()).max());objects[n]={'max_translation_m':translation,'max_rotation_deg':angle}
    robot_delta=float(np.max(np.abs(data['q']-data['q'][0])));task_pass=all(objects[n]['max_translation_m']<=.001 and objects[n]['max_rotation_deg']<=1 for n in task_names);other_pass=all(v['max_translation_m']<=.002 and v['max_rotation_deg']<=1 for n,v in objects.items() if n not in task_names);aligned=bool(data['camera_aligned'].all());finite=all(np.isfinite(v).all() for v in data.values())
    report={'passed':bool(task_pass and other_pass and robot_delta<=.001 and aligned and finite),'acceptance_scope':'user-requested task scope','task':a.task,'duration_seconds':frames*e.step_dt,'frames':frames,'dt':e.step_dt,'initial_settling_seconds':60*e.step_dt,'fixed_layout_sha256':hashlib.sha256(Path(a.fixed_layout).read_bytes()).hexdigest(),'source_yaml_sha256':hashlib.sha256(Path('third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench',a.task+'.yaml').read_bytes()).hexdigest(),'predeclared_tolerances':tol,'task_objects_passed':bool(task_pass),'distractors_passed':bool(other_pass),'robot_hold_passed':bool(robot_delta<=.001),'robot_max_joint_change_rad':robot_delta,'camera_capture_alignment':aligned,'finite':bool(finite),'objects':objects,'unoperated_joint_state_passed':None,'scope_exclusions':['Unrelated global kitchen-fixture joint neutral gate excluded by explicit user instruction; not inspected or claimed passed.']}
    with (a.output/'neutral_hold.json').open('x') as f:json.dump(report,f,indent=2)
    print('TASK_NEUTRAL_RESULT',json.dumps(report),flush=True);env.close()
