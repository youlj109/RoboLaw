"""Physically press the original toaster button with cuRobo free-space planning."""
import hashlib
import json
import time
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser=get_isaaclab_arena_cli_parser()
parser.add_argument('--variant',choices=['base','human','property','robot'],default='base')
parser.add_argument('--output',required=True)
parser.add_argument('--config',default='configs/extensions/controls/left_button_trial.json')
parser.add_argument('--max-steps',type=int,default=300)
parser.add_argument('--roll-deg',type=float,default=0)
parser.add_argument('--tcp-z-offset',type=float,default=-.006)
parser.add_argument('--tcp-x-offset',type=float,default=0)
parser.add_argument('--probe-only',action='store_true')
args=parser.parse_args()
args.enable_cameras=True
out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
started=time.monotonic()
with SimulationAppContext(args):
    import h5py
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.controls.environment import build_environment, TASK_ID, SOURCE
    from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.dataset import CameraWriter, numpy as arr, decode_video
    from kitchen_safety.curobo_backend import CuroboPlanner
    from kitchen_safety.grasp_contacts import _numpy as contact_array
    torch.set_num_threads(4)
    trial_config=json.loads(Path(args.config).read_text())
    original_pose=trial_config['robot_pose_xyzw']
    envdef,assets=build_environment(variant=args.variant,fixed_pose=original_pose,trial_config=trial_config)
    envdef.task.episode_length_s=120.
    builder=ArenaEnvBuilder(envdef,ArenaEnvBuilderCfg(num_envs=1,seed=args.seed or 42,
        device=args.device,presets='physx',solve_relations=False))
    cfg,kwargs=builder.compose_manager_cfg()
    cfg.observations.camera_obs=None
    cfg.sim.render_interval=cfg.decimation
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kwargs);env.reset();e=env.unwrapped
    robot=e.scene['robot'];toaster=e.scene['toaster']
    arm_ids=[robot.joint_names.index(f'panda_joint{i}') for i in range(1,8)]
    button_id=toaster.joint_names.index('sideL_button_cancel_joint')
    kin=DroidKinematics(original_pose[:3],Rotation.from_quat(original_pose[3:]).as_euler('xyz',degrees=True)[2])
    q=arr(robot.data.joint_pos)[0,arm_ids]
    for _ in range(8):env.step(torch.tensor(np.r_[q,0][None],dtype=torch.float32,device=e.device))
    world=ControlsCollisionWorld(e.sim.stage,kin,env_root='/World/envs/env_0')
    # Preserve the benchmark's 60 mm planning / 50 mm execution glass margin.
    writer=CameraWriter(out,e.scene,envdef.embodiment.camera_config.camera_names(),e.step_dt,previews=True)
    for _ in range(32):e.sim.render()
    robot_contacts=e.scene['robot_contacts']
    button_path='/World/envs/env_0/kitchen/toaster_main_group/Toaster015_sideL_button_cancel'
    views={name:robot_contacts._physics_sim_view.create_rigid_contact_view(
        [f'/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/{name}'],
        filter_patterns=[[button_path]],max_contact_data_count=1024)
        for name in ('left_inner_finger','right_inner_finger')}
    records=[];plans=[];audits=[];physics_audits=[];contacts=[];failure=None;phase='close';success_step=None
    toaster_body_keys={i:f'/World/envs/env_0/kitchen/toaster_main_group/{name}'
        for i,name in enumerate(toaster.body_names)
        if any(p.startswith(f'/World/envs/env_0/kitchen/toaster_main_group/{name}/') for p,*_ in world.objects)}
    baked_root_scales={}
    for key in toaster_body_keys.values():
        # USD authoring scales are already baked into the world collider
        # vertices/primitive bounds. PhysX body poses contain rigid rotation
        # only, so remove the static scale from the *reference pose*, never
        # from collision geometry, before applying measured rigid deltas.
        reference=world._root_reference_transforms[key].copy()
        u,s,vh=np.linalg.svd(reference[:3,:3]);rotation=u@vh
        if np.linalg.det(rotation)<0:raise ValueError('Reflected articulation root is unsupported')
        baked_root_scales[key]=s.tolist();reference[:3,:3]=rotation
        world._root_reference_transforms[key]=reference
    def refresh_world(state):
        for i,key in toaster_body_keys.items():world.set_obstacle_pose(key,state['toaster_body_pose'][i])
        if args.variant=='property':world.set_obstacle_pose('glass',state['hazard_pose'])
    def snapshot():
        joints=arr(robot.data.joint_pos)[0]
        return dict(q=joints.tolist(),dq=arr(robot.data.joint_vel)[0].tolist(),
            tcp=kin.fk(joints[arm_ids]).tolist(),toaster_q=arr(toaster.data.joint_pos)[0].tolist(),
            robot_body_pose=np.c_[arr(robot.data.body_pos_w)[0],arr(robot.data.body_quat_w)[0]].tolist(),
            toaster_body_pose=np.c_[arr(toaster.data.body_pos_w)[0],arr(toaster.data.body_quat_w)[0]].tolist(),
            contact_forces=arr(robot_contacts.data.net_forces_w)[0].tolist(),
            predicate=bool(assets['toaster'].is_pressed(e)[0]),
            hazard_pose=arr(e.scene['glass'].data.root_state_w)[0,:7].tolist() if args.variant=='property' else None)
    terminal={}
    original_reset=e._reset_idx
    def preserve_success(ids):
        if len(ids) and bool(assets['toaster'].is_pressed(e)[0]):
            terminal['success_state']=snapshot()
            return
        return original_reset(ids)
    e._reset_idx=preserve_success
    original_scene_update=e.scene.update
    def checked_scene_update(dt):
        original_scene_update(dt)
        state=snapshot();refresh_world(state)
        joints=dict(zip(robot.joint_names,state['q']))
        world.allow_button_contact=(phase in ('press','hold'))
        valid=world.valid(np.array(state['q'])[arm_ids],mode='execution',
            gripper_q={k:joints[k] for k in world.GRIPPER_JOINT_NAMES})
        entry={'action_index':len(records),'phase':phase,'valid':bool(valid)}
        if not valid:entry['violation']=world.last_clearance
        physics_audits.append(entry)
        if not valid:raise RuntimeError('Physics-step clearance violation: '+json.dumps(entry))
    e.scene.update=checked_scene_update
    initial_state=snapshot()
    def step(command,grip=1):
        global success_step
        if len(records)>=args.max_steps:raise RuntimeError('Bounded diagnostic step budget reached')
        before=snapshot();writer.write(phase)
        action=np.r_[command,grip].astype('f4')
        try:env.step(torch.tensor(action[None],device=e.device))
        finally:
            after=snapshot()
            records.append(dict(phase=phase,action=action.tolist(),before=before,after=after))
        joints=dict(zip(robot.joint_names,after['q']))
        refresh_world(after)
        world.allow_button_contact=(phase in ('press','hold'))
        measurement=world.clearance(np.array(after['q'])[arm_ids],mode='execution',
            gripper_q={k:joints[k] for k in world.GRIPPER_JOINT_NAMES})
        audits.append(dict(action_index=len(records)-1,phase=phase,**measurement))
        contact={'action_index':len(records)-1,'phase':phase,'fingers':{}}
        for name,view in views.items():
            force,points,normals,separation,counts,starts=map(contact_array,view.get_contact_data(dt=cfg.sim.dt))
            n=int(counts.reshape(-1)[0]);a=int(starts.reshape(-1)[0])
            contact['fingers'][name]={'count':n,'force':force[a:a+n].reshape(-1).tolist(),
                'normals':normals[a:a+n].reshape(-1,3).tolist(),
                'points':points[a:a+n].reshape(-1,3).tolist(),'separation':separation[a:a+n].reshape(-1).tolist()}
        contacts.append(contact)
        if after['predicate'] and success_step is None:success_step=len(records)-1
        if len(records)%15==0 or after['predicate']:
            print('CONTROL_STEP',len(records),phase,'joint',after['toaster_q'][button_id],
                'tcp',np.array(after['tcp'])[:3,3].round(4).tolist(),'clearance',measurement,flush=True)
        if not measurement['valid']:
            raise RuntimeError('Executed clearance violation: '+json.dumps(measurement))
        return after
    try:
        for _ in range(15):step(q)
        q=np.array(snapshot()['q'])[arm_ids]
        # This task closes an empty gripper. The shared bottle collector's
        # default 0.38 rad carry width does not describe this 0.785 rad pose.
        world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
        planner=CuroboPlanner(world,kin,dt=e.step_dt,seed=args.seed or 42,max_joint_speed=.4)
        target=np.eye(4)
        target[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()@Rotation.from_euler('x',args.roll_deg,degrees=True).as_matrix()
        target[:3,3]=[2.08087763+args.tcp_x_offset,-.44,1.03313965+args.tcp_z_offset]
        phase='prepress'
        plan=planner.plan_pose(q,target,grip_binary=1,phase='prepress')
        validation=planner.validate_path(plan['positions'],grip_binary=1,phase='prepress')
        plan['metadata']['dense_path_validation']=validation
        plan['metadata']['requested_tcp_pose']=target.tolist()
        plans.append(plan['metadata'])
        if not validation['valid']:raise RuntimeError('Prepress path failed complete cuRobo/FCL validation: '+json.dumps(validation))
        for command in plan['positions']:step(command)
        for _ in range(8):step(plan['positions'][-1])
        if not args.probe_only:
            phase='press'
            for y in np.arange(-.439,-.363,.0005):
                target[1,3]=y
                q=np.array(snapshot()['q'])[arm_ids]
                command=kin.ik(target,q,attempts=4)
                for _ in range(2):
                    current=step(command)
                    if current['predicate']:break
                if current['predicate']:
                    phase='hold'
                    for _ in range(5):step(command)
                    break
            if success_step is None:raise RuntimeError('Button did not reach original pressed predicate')
    except Exception as exc:
        import traceback
        failure=f'{type(exc).__name__}: {exc}';traceback.print_exc()
    finally:
        writer.close()
        (out/'records.json').write_text(json.dumps(records))
        (out/'contacts.json').write_text(json.dumps(contacts,indent=2))
        (out/'plans.json').write_text(json.dumps(plans,indent=2))
        if records:
            np.savez_compressed(out/'trajectory.npz',
                actions=np.array([r['action'] for r in records],dtype='f4'),
                q=np.array([r['before']['q'] for r in records],dtype='f4'),
                next_q=np.array([r['after']['q'] for r in records],dtype='f4'),
                toaster_q=np.array([r['before']['toaster_q'] for r in records],dtype='f4'),
                next_toaster_q=np.array([r['after']['toaster_q'] for r in records],dtype='f4'),
                phase=np.array([r['phase'] for r in records]))
        (out/'audit.json').write_text(json.dumps({'passed':bool(audits and all(r['valid'] for r in audits) and all(r['valid'] for r in physics_audits)),
            'scope':'Actual measured robot geometry at every physics update and recorded action; only inner-finger/button contact is allowed during press and hold',
            'world_collision_shapes':len(world.objects),'all_kitchen_colliders_included':True,
            'moving_toaster_colliders_updated_from_measured_body_poses':True,
            'usd_static_scales_baked_into_collision_geometry':baked_root_scales,
            'physics_updates':physics_audits,'measurements':audits},indent=2))
        validation={'frames':writer.frames,'record_count':len(records),'videos':{},'passed':False}
        for name in (*envdef.embodiment.camera_config.camera_names(),'all_cameras'):
            validation['videos'][name]=decode_video(out/(name+'.mp4'),len(records),expected_dt=e.step_dt)
        validation['passed']=writer.frames==len(records) and len(records)>0
        (out/'validation.json').write_text(json.dumps(validation,indent=2))
        contact_count=sum(v['count'] for c in contacts for v in c['fingers'].values())
        # This PhysX 6.1 contact view returns signed normal reaction scalars.
        # Keep the raw signed values; load is the magnitude, independently
        # corroborated by button displacement and robot net contact forces.
        loaded_contact_count=sum(sum(abs(f)>.01 for f in v['force']) for c in contacts for v in c['fingers'].values())
        result={'task_id':TASK_ID,'variant':args.variant,'seed':args.seed or 42,
            'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'success':success_step is not None,'success_action_index':success_step,
            'accepted':bool(success_step is not None and failure is None and loaded_contact_count>0 and validation['passed']),
            'button_contact_count':contact_count,'button_loaded_contact_count':loaded_contact_count,
            'failure':failure,'seconds':time.monotonic()-started,
            'button_joint_limits_m':[0.,.0030166050419211388],
            'original_predicate':'Pressable.is_pressed(normalized_joint_position > 0.5)',
            'variant_spec':trial_config['variants'].get(args.variant),
            'config_sha256':hashlib.sha256(Path(args.config).read_bytes()).hexdigest(),
            'initial_state':initial_state,
            'control_parameters':{'planner_seed':args.seed or 42,'max_joint_speed_rad_s':.4,
                'roll_deg':args.roll_deg,'tcp_z_offset_m':args.tcp_z_offset,'tcp_x_offset_m':args.tcp_x_offset,
                'press_y_increment_m':.0005,'physics_actions_per_press_increment':2},
            'final':snapshot(),'initial_robot_pose':original_pose,
            'physical_object_setters_after_reset':False,'fixed_only':True,'dt':e.step_dt,
            'robot_body_names':robot.body_names,'robot_joint_names':robot.joint_names,
            'robot_contact_body_names':robot_contacts.body_names,
            'action_semantics':'7 absolute arm joint targets in rad + binary gripper (0=open,1=closed)'}
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print('CONTROLS_RESULT',json.dumps(result),flush=True)
        env.close()
    # Arena can terminate the process during normal context teardown. Raise
    # failures while still inside it; successful teardown must have no exception.
    if not result['accepted']:raise SystemExit(2)
