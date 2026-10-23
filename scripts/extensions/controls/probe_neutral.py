"""Recorded source-task neutral hold, including all kitchen articulation DOFs."""
import hashlib
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser=get_isaaclab_arena_cli_parser()
parser.add_argument('--controls-task',choices=['left','right','temperature'],required=True)
parser.add_argument('--inspection',required=True)
parser.add_argument('--output',required=True)
parser.add_argument('--seconds',type=float,default=50.)
parser.add_argument('--repair-oven-racks',action='store_true')
parser.add_argument('--rack-mobility',action='store_true',help='Append an external-force diagnostic, never a robot demonstration')
parser.add_argument('--balance-temperature-knob',action='store_true')
args=parser.parse_args()
if not 30<=args.seconds<=120:
    parser.error('Neutral duration must be between 30 and 120 seconds')
args.enable_cameras=True
out=Path(args.output)
out.mkdir(parents=True,exist_ok=False)

with SimulationAppContext(args):
    import numpy as np
    import torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.controls.environment import build_environment
    from kitchen_safety.extensions.controls.task_contract import task_contract
    from kitchen_safety.extensions.controls.mechanism_monitor import MechanismMonitor
    from kitchen_safety.dataset import CameraWriter, numpy as arr, decode_video

    contract=task_contract(args.controls_task)
    inspection_path=Path(args.inspection)/'state.json'
    inspection=json.loads(inspection_path.read_text())
    if inspection['task_id']!=contract['task_id']:
        raise ValueError('Neutral pose belongs to another task')
    if args.rack_mobility and (args.controls_task!='temperature' or not args.repair_oven_racks):
        raise ValueError('Rack mobility requires the explicit temperature rack model')
    pose=inspection['robot_pose'][:7]
    definition,assets=build_environment(task=args.controls_task,fixed_pose=pose,repair_oven_racks=args.repair_oven_racks,
        balance_temperature_knob=args.balance_temperature_knob)
    definition.task.episode_length_s=180.
    builder=ArenaEnvBuilder(definition,ArenaEnvBuilderCfg(num_envs=1,seed=42,
        device=args.device,presets='physx',solve_relations=False))
    cfg,kwargs=builder.compose_manager_cfg()
    cfg.observations.camera_obs=None
    cfg.sim.render_interval=cfg.decimation
    env=builder.make_registered(cfg,kwargs)
    env.reset()
    e=env.unwrapped
    robot,target=e.scene['robot'],e.scene[contract['asset']]
    monitor=MechanismMonitor(e.sim.stage)
    rows=[]
    original_update=e.scene.update
    def monitored_update(dt):
        original_update(dt)
        monitor.sample('neutral',len(rows))
    e.scene.update=monitored_update
    ids=[robot.joint_names.index(f'panda_joint{i}') for i in range(1,8)]
    q=arr(robot.data.joint_pos)[0,ids]
    command=torch.tensor(np.r_[q,0][None],dtype=torch.float32,device=e.device)
    def state():
        return {'robot_q':arr(robot.data.joint_pos)[0].tolist(),
            'target_q':arr(target.data.joint_pos)[0].tolist(),
            'target_dq':arr(target.data.joint_vel)[0].tolist()}
    initial=state()
    writer=CameraWriter(out,e.scene,definition.embodiment.camera_config.camera_names(),e.step_dt,previews=True)
    for _ in range(32):e.sim.render()
    failure=None
    steps=round(args.seconds/e.step_dt)
    pulse_steps=round(2/e.step_dt) if args.rack_mobility else 0
    rest_steps=round(3/e.step_dt) if args.rack_mobility else 0
    total_steps=steps+pulse_steps+rest_steps
    neutral_audit=None
    if args.rack_mobility:
        rack_bodies=[target.body_names.index(f'Oven038_rack{i}') for i in (0,1)]
        rack_joints=[target.joint_names.index(f'rack{i}_joint') for i in (0,1)]
        zero=torch.zeros((1,2,3),device=e.device)
        force=zero.clone()
        force[:,:,1]=-.05
    try:
        for i in range(total_steps):
            phase='neutral' if i<steps else ('external_force_mobility' if i<steps+pulse_steps else 'passive_damping')
            if i==steps:
                neutral_audit=monitor.report()
                e.scene.update=original_update
            if args.rack_mobility:
                target.set_external_force_and_torque(force if phase=='external_force_mobility' else zero,zero,body_ids=rack_bodies)
            before=state()
            writer.write(phase)
            env.step(command)
            rows.append({'index':i,'phase':phase,'before':before,'after':state()})
            if i%150==0:print('NEUTRAL_STEP',i,total_steps,phase,flush=True)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        failure=f'{type(exc).__name__}: {exc}'
    finally:
        writer.close()
        audit=neutral_audit or monitor.report()
        (out/'background_mechanism_audit.json').write_text(json.dumps(audit,indent=2))
        robot_excursion=max((float(np.max(np.abs(np.array(r['after']['robot_q'])-initial['robot_q']))) for r in rows),default=float('inf'))
        videos={}
        for name in (*definition.embodiment.camera_config.camera_names(),'all_cameras'):
            try:videos[name]=decode_video(out/(name+'.mp4'),len(rows),expected_dt=e.step_dt)
            except Exception as exc:
                videos[name]={'error':str(exc)}
                failure=failure or 'Video decode failed'
        mobility=None
        if args.rack_mobility and len(rows)==total_steps:
            before=np.asarray(rows[steps]['before']['target_q'])[rack_joints]
            after=np.asarray(rows[-1]['after']['target_q'])[rack_joints]
            delta=after-before
            parameters={}
            for name in ('get_dof_stiffnesses','get_dof_dampings'):
                value=getattr(target.root_physx_view,name)()
                parameters[name]=np.asarray(value.numpy() if hasattr(value,'numpy') else value).tolist()
            stiffness=np.asarray(parameters['get_dof_stiffnesses'])[0,rack_joints]
            damping=np.asarray(parameters['get_dof_dampings'])[0,rack_joints]
            mobility={'passed':bool(np.all(delta>.005) and np.all(delta<.2) and np.all(stiffness==0) and np.allclose(damping,1.)),
                'delta_m':delta.tolist(),'force_body_local_n':[0.,-.05,0.],'pulse_seconds':2.,
                'runtime_parameters':parameters,'scope':'External force diagnostic only, not robot task success. No joint or object state setters.'}
        report={'accepted':failure is None and audit['passed'] and len(rows)==total_steps and robot_excursion<.0001 and (mobility is None or mobility['passed']),
            'task_id':contract['task_id'],'source_sha256':hashlib.sha256(contract['source'].read_bytes()).hexdigest(),
            'inspection_sha256':hashlib.sha256(inspection_path.read_bytes()).hexdigest(),
            'initial_robot_pose':pose,'initial_state':initial,'target_joint_names':target.joint_names,
            'neutral_seconds':min(len(rows),steps)*e.step_dt,'dt':e.step_dt,'physics_dt':cfg.sim.dt,
            'rack_mobility':mobility,
            'robot_excursion_max_rad':robot_excursion,'robot_excursion_limit_rad':.0001,
            'failure':failure,'videos':videos,'states':rows,
            'toaster_physics_provenance':definition.toaster_physics_provenance,
            'faucet_physics_provenance':definition.faucet_physics_provenance,
            'oven_physics_provenance':definition.oven_physics_provenance,
            'temperature_knob_physics_provenance':definition.temperature_knob_physics_provenance,
            'object_state_setters_after_reset':False,
            'scope':'All kitchen q/dq at every neutral physics update; three original cameras at action rate. No torque or robot task execution; does not establish avoidance or mechanism mobility.'}
        (out/'validation.json').write_text(json.dumps(report,indent=2))
        print('NEUTRAL_RESULT',report['accepted'],len(rows),robot_excursion,failure,flush=True)
        env.close()
    if not report['accepted']:raise SystemExit(2)
