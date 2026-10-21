"""One genuine lime-in-wooden-bowl rollout with cuRobo and original Arena success."""
import hashlib,json,time,traceback
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
parser=get_isaaclab_arena_cli_parser()
parser.add_argument('--variant',choices=['base','human','property','robot'],default='base')
parser.add_argument('--output',required=True)
parser.add_argument('--overview-camera',action='store_true',help='Supplementary fixed camera; preserve all original policy cameras')
parser.add_argument('--grasp-yaw',type=float,default=0)
parser.add_argument('--grasp-tilt',type=float,default=35)
parser.add_argument('--grasp-height-offset',type=float,default=.015,
    help='TCP height above measured lime geometry center; metres')
args=parser.parse_args();args.enable_cameras=True
if args.num_envs!=1:raise ValueError('This verified entry runs one environment; parallel collection is not implemented')
seed=42 if args.seed is None else args.seed
placement_seed=seed if args.placement_seed is None else args.placement_seed
out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
started=time.perf_counter()
with SimulationAppContext(args):
    import h5py,numpy as np,torch
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment,TASK_ID,SOURCE
    from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
    from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
    from kitchen_safety.extensions.lightwheel_pick_place.contacts import LimeGraspContacts
    from kitchen_safety.planning import DroidKinematics,transform
    from kitchen_safety.dataset import CameraWriter,numpy as arr,decode_video
    torch.set_num_threads(2)
    envdef,assets=build_environment(enable_cameras=True,variant=args.variant)
    builder=ArenaEnvBuilder(envdef,ArenaEnvBuilderCfg(num_envs=1,seed=seed,device='cpu',
        presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=placement_seed))
    cfg,kwargs=builder.compose_manager_cfg();cfg.observations.camera_obs=None
    overview_spec=None
    if args.overview_camera:
        from kitchen_safety.extensions.lightwheel_pick_place.overview import configure_overview
        overview_spec=configure_overview(cfg.scene)
    cfg.sim.log_dir=str(out/'logs');cfg.sim.render_interval=cfg.decimation
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kwargs);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot']
    ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    q=arr(robot.data.joint_pos)[0,ids];grip=0.;phase='settle';records=[];clearances=[];plans=[];physics_audits=[]
    terminal={};world=None;planner=None;writer=None;overview_writer=None;overview_alignment=[];error=None
    finger_names=[n for i,n in enumerate(robot.joint_names) if i not in ids]
    bi=robot.body_names.index('base_link')
    def snapshot():
        quat=arr(robot.data.body_quat_w)[0,bi]
        return {'q':arr(robot.data.joint_pos)[0], 'dq':arr(robot.data.joint_vel)[0],
          'tcp':arr(robot.data.body_pos_w)[0,bi]+Rotation.from_quat(quat).apply([.131,0,0]),
          'tcp_quat_xyzw':quat,'lime':arr(scene['lime'].data.root_state_w)[0],
          'wooden_bowl':arr(scene['wooden_bowl'].data.root_state_w)[0],
          'robot_body_pose_w':np.concatenate([arr(robot.data.body_pos_w)[0],arr(robot.data.body_quat_w)[0]],axis=-1),
          'robot_contact_forces':arr(scene['robot_contacts'].data.net_forces_w)[0],
          **{name:arr(scene[name].data.root_state_w)[0] for name in ('broccoli','sweet_potato','red_bell_pepper')},
          **({'hazard_pose':arr(scene['glass'].data.root_state_w)[0,:7]} if args.variant=='property' else {})}
    def capture_reset(env_ids):
        if len(env_ids):
            terminal['state']=snapshot()
            terminal['terms']={n:bool(arr(e.termination_manager.get_term(n))[0]) for n in e.termination_manager.active_terms}
    e._reset_idx=capture_reset
    def ignores():
        return ('lime',)+(('wooden_bowl',) if phase in ('lower','release','retreat','settle_final') else ())
    def geometry(state):
        return dict(mode='execution',gripper_q={n:state['q'][robot.joint_names.index(n)] for n in finger_names})
    physics_checks=0
    original_update=scene.update
    def checked_update(dt):
        global physics_checks
        original_update(dt)
        if world is None:return
        s=snapshot();physics_checks+=1
        world.allowed_phase=phase
        if args.variant=='property':world.set_obstacle_pose('glass',s['hazard_pose'])
        for name in ('wooden_bowl','broccoli','sweet_potato','red_bell_pepper'):
            world.set_obstacle_pose(name,s[name][:7])
        # Full environment robot check; intended grasp/support masks are phase-specific.
        full=world.clearance(s['q'][ids],ignores(),object_pose=s['lime'][:7],**geometry(s))
        physics_audits.append({'action':len(records),'phase':phase,'q_arm':s['q'][ids].tolist(),
            'gripper_q':{k:float(v) for k,v in geometry(s)['gripper_q'].items()},'lime_pose':s['lime'][:7].tolist(),
            'source_lift_floor_z_m':getattr(world,'source_lift_floor_z',None),**full})
        if not full['valid']:raise RuntimeError(f'Executed collision/clearance: {full}')
        if phase=='lift' and hasattr(world,'source_lift_floor_z'):
            actual_lime_points=world.lime_vertices@Rotation.from_quat(s['lime'][3:7]).as_matrix().T+s['lime'][:3]
            world.source_lift_floor_z=max(world.source_lift_floor_z,float(actual_lime_points[:,2].min())-.0005)
        if args.variant!='base':
            hazard={'human':'person','property':'glass','robot':'barrier'}[args.variant]
            c=world.clearance(s['q'][ids],('lime',),object_pose=s['lime'][:7],obstacle_filter=(hazard,),**geometry(s))
            clearances.append({'action':len(records),'phase':phase,**c})
            if not c['valid']:raise RuntimeError(f'Unsafe lime/robot clearance: {c}')
    scene.update=checked_update
    contacts=LimeGraspContacts(scene['robot_contacts']._physics_sim_view,cfg.sim.dt)
    def step(command,gripper):
        before=snapshot()
        if writer:
            before_capture=int(e.sim.get_physics_step_count())
            writer.write(phase)
            if overview_writer:
                overview_writer.write(phase)
                overview_alignment.append({'action_index':len(records),'phase':phase,
                    'primary_frame_index':writer.frames-1,'overview_frame_index':overview_writer.frames-1,
                    'physics_step_before':before_capture,'physics_step_after':int(e.sim.get_physics_step_count()),
                    'simulation_time_s':before_capture*cfg.sim.dt})
        action=np.r_[command,gripper].astype(np.float32)
        try:env.step(torch.tensor(action[None],device=e.device))
        finally:records.append((phase,action,before,snapshot()))
        if terminal and not terminal.get('terms',{}).get('success',False):raise RuntimeError(f'Arena termination: {terminal["terms"]}')
        if phase in ('close','lift','transfer','lower'):contacts.sample(phase,len(records)-1,snapshot()['lime'][:7])
        if len(records)%30==0:print('STEP',len(records),phase,snapshot()['lime'][:3].round(4),flush=True)
    result={'task':TASK_ID,'variant':args.variant,'success':False,'planner':'curobo','physical_grasp':True,
      'seed':seed,'placement_seed':placement_seed,'fixed_only':True,
      'source_predicate':'Original PickAndPlaceTask: settled, lifted, destination support and low speed',
      'requested_grasp_tilt_deg':args.grasp_tilt,'requested_grasp_yaw_deg':args.grasp_yaw,
      'requested_grasp_height_offset_m':args.grasp_height_offset,
      'source_yaml_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
      'original_asset_initial_poses':{name:assets[name]._get_initial_pose_as_pose().to_dict() for name in ('lime','wooden_bowl','broccoli','sweet_potato','red_bell_pepper','droid') if name in assets and assets[name].get_initial_pose() is not None},
      'simulation_attachment':False,'robot_joint_names':robot.joint_names,'robot_body_names':robot.body_names,'dt':e.step_dt}
    try:
        for _ in range(20):step(q,0.)
        initial=snapshot();result['initial_object_states']={name:arr(scene[name].data.root_state_w)[0].tolist() for name in ('lime','wooden_bowl','broccoli','sweet_potato','red_bell_pepper')};result['initial_lime_pose']=initial['lime'].tolist();result['initial_bowl_pose']=initial['wooden_bowl'].tolist()
        base=arr(robot.data.root_state_w)[0,:7];kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2])
        result['fk_error_m']=float(np.linalg.norm(kin.fk(initial['q'][ids])[:3,3]-initial['tcp']))
        if result['fk_error_m']>.005:raise RuntimeError('Robot FK disagrees with PhysX TCP')
        result['robot_base_pose']=base.tolist()
        world=CollisionWorld(e.sim.stage,kin)
        initial_lime_points=world.lime_vertices@Rotation.from_quat(initial['lime'][3:7]).as_matrix().T+initial['lime'][:3]
        world.source_resting_min_z=float(initial_lime_points[:,2].min())
        world.source_resting_xy=initial['lime'][:2].copy()
        result['source_support_contact_model']={'source_resting_min_z_m':world.source_resting_min_z,
            'source_resting_xy_m':world.source_resting_xy.tolist(),'source_resting_extra_tolerance_m':.0005,'source_resting_max_xy_displacement_m':.05,
            'lift_support_policy':'Measured legal closed-grasp hull bottom minus 0.5 mm, then monotonically tightened during lift','surface':'/World/envs/env_0/kitchen/counter_main_main_group/top_geometry_right',
            'reason':'Original PhysX convexDecomposition resting pose differs from raw source-mesh surface. Preserve that measured initial support only; forbid further lowering or lateral dragging. Lift uses its own measured closed-grasp lower bound, tightened as the actual hull rises.'}
        e.sim.stage.Export(str(out/'scene.usda'))
        (out/'initial_setup.json').write_text(json.dumps({
            'original_asset_initial_poses':result['original_asset_initial_poses'],
            'initial_object_states':result['initial_object_states'],
            'robot_base_pose':result['robot_base_pose'],
            'robot_joint_names':robot.joint_names,'robot_joint_positions':initial['q'].tolist(),
            'source_yaml_sha256':result['source_yaml_sha256']},indent=2))
        result['start_fcl']=world.clearance(initial['q'][ids],('lime',),**geometry(initial))
        planner=LimePlanner(world,kin,dt=e.step_dt,seed=seed,max_joint_speed=.4)
        writer=CameraWriter(out,scene,envdef.embodiment.camera_config.camera_names(),e.step_dt,previews=True)
        if args.overview_camera:
            overview_dir=out/'overview';overview_dir.mkdir()
            overview_writer=CameraWriter(overview_dir,scene,['overview_camera'],e.step_dt,previews=True)
            result['overview_camera']=overview_spec
        # Unrecorded settling records are omitted so RGB/trajectory alignment is exact.
        records.clear();clearances.clear()
        result['camera_warmup']=writer.warm_up(e.sim,min_frames=64,max_frames=256)
        if overview_writer:result['overview_camera_warmup']=overview_writer.warm_up(e.sim,min_frames=64,max_frames=256)
        def cart(position,*,rotation=None):
            global q
            world.allowed_phase=phase
            s=snapshot();q=s['q'][ids].copy()
            world.gripper_joint_position=float(s['q'][robot.joint_names.index('finger_joint')])
            if args.variant=='property':
                world.set_obstacle_pose('glass',s['hazard_pose']);planner.update_dynamic_obstacle('glass')
            target=np.eye(4);target[:3,3]=position
            target[:3,:3]=(kin.fk(q)[:3,:3] if rotation is None else rotation)
            try:trajectory=planner.plan_pose(q,target,ignores(),grip,phase=phase)
            except RuntimeError:
                checker=planner.motion_gen.world_coll_checker
                names=[n for n in planner.obstacle_names if not planner._ignored(n,ignores())]
                def valid_subset(subset):
                    selected=set(subset)
                    for n in planner.obstacle_names:checker.enable_obstacle(n,enable=n in selected)
                    return bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(q[None])).all())
                bad=[]
                def bisect(group):
                    if not group or valid_subset(group):return
                    if len(group)==1:bad.extend(group);return
                    m=len(group)//2;bisect(group[:m]);bisect(group[m:])
                if valid_subset([]):bisect(names)
                result['start_invalid_colliders']=bad
                result['start_self_collision_only']=not valid_subset([])
                for name in planner.obstacle_names:checker.enable_obstacle(name,enable=name in names)
                raise
            validation=planner.validate_path(trajectory['positions'],ignores(),grip,phase=phase)
            plans.append({'phase':phase,'target':target.tolist(),'metadata':trajectory['metadata'],'validation':validation,'executed':False})
            if not validation['valid']:raise RuntimeError(f'Path validation failed in {phase}: {validation}')
            if phase=='pregrasp':
                contact_target=target.copy();contact_target[2,3]-=.125
                lookahead=planner.plan_pose(trajectory['positions'][-1],contact_target,ignores(),grip,phase='approach')
                checked=planner.validate_path(lookahead['positions'],ignores(),grip,phase='approach')
                plans[-1]['approach_lookahead']=checked
                if not checked['valid']:raise RuntimeError('Approach lookahead is not collision-free')
            plans[-1]['executed']=True
            for command in trajectory['positions']:step(command,grip)
            q=trajectory['positions'][-1].copy()
            for _ in range(12):step(q,grip)
        p=initial['lime'][:3].copy()
        target_center=transform(initial['lime'][:3],initial['lime'][3:7])@np.r_[world.object_center,1]
        target_center[2]+=args.grasp_height_offset
        R=Rotation.from_euler('z',args.grasp_yaw,degrees=True).as_matrix()@Rotation.from_euler('x',args.grasp_tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
        phase='pregrasp'
        candidates=list(dict.fromkeys([(args.grasp_tilt,args.grasp_yaw),(25,0),(20,0)]))
        result['pregrasp_candidates']=[]
        for tilt,yaw in candidates:
            R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
            try:
                cart(target_center[:3]+[0,0,.125],rotation=R)
                result['selected_grasp']={'tilt':tilt,'yaw':yaw};break
            except RuntimeError as exc:
                result['pregrasp_candidates'].append({'tilt':tilt,'yaw':yaw,'error':str(exc)})
                if records:raise
        else:raise RuntimeError('All pregrasp pose candidates failed')
        phase='approach';cart(target_center[:3])
        phase='close';grip=1.
        for _ in range(30):step(q,grip)
        held=snapshot();result['close_contacts']=contacts.summarize([-.001,.061],'close',last_actions=5)
        if not result['close_contacts']['both_fingers_near_body']:raise RuntimeError('No bilateral physical lime grasp')
        world.attachment=np.linalg.inv(kin.fk(held['q'][ids]))@transform(held['lime'][:3],held['lime'][3:7])
        result['planning_attachment_tcp_to_lime']=world.attachment.tolist()
        held_points=world.lime_vertices@Rotation.from_quat(held['lime'][3:7]).as_matrix().T+held['lime'][:3]
        held_min_z=float(held_points[:,2].min())
        if held_min_z<world.source_resting_min_z-.0005:raise RuntimeError('Closed grasp went below the original calibrated source support')
        world.source_lift_floor_z=held_min_z-.0005
        result['lift_support_contact_model']={'measured_closed_grasp_min_z_m':held_min_z,
            'initial_lower_bound_m':world.source_lift_floor_z,'jitter_tolerance_m':.0005,
            'xy_anchor_m':world.source_resting_xy.tolist(),'max_xy_displacement_m':.05,
            'policy':'Only exact source countertop: physics-step lower bound increases with the highest measured hull bottom, allowing 0.5 mm jitter; no lowering/dragging through source or other objects.'}
        phase='lift';cart(held['tcp']+[0,0,.16])
        lifted=snapshot();result['lift_m']=float(lifted['lime'][2]-p[2]);result['lift_contacts']=contacts.summarize([-.001,.061],'lift',last_actions=5)
        if result['lift_m']<.08 or not result['lift_contacts']['both_fingers_on_body']:raise RuntimeError('Lime failed physical lift/contact check')
        bowl=snapshot()['wooden_bowl'];offset=lifted['tcp']-lifted['lime'][:3]
        # Wooden bowl is 13cm tall; release above the rim then let gravity settle the fruit.
        desired_root=bowl[:3]+np.array([0,0,.18])
        phase='transfer';cart(desired_root+offset)
        phase='lower';cart(bowl[:3]+np.array([0,0,.135])+offset)
        phase='release';grip=0.;world.attachment=None
        for _ in range(35):step(q,grip)
        phase='retreat';before_retreat=len(records)
        try:cart(snapshot()['tcp']+[0,0,.14])
        except RuntimeError as exc:
            # Withdrawal is not part of the source pick-and-place predicate.
            # If the optional path is rejected before any motion, keep the
            # already checked open-gripper pose and let the fruit settle.
            # Never swallow any failure after a retreat action was executed.
            if len(records)!=before_retreat:raise
            result['optional_retreat_skipped']={'reason':str(exc),
                'executed_actions':0,'policy':'Hold the already safe open-gripper pose; original final support/speed predicate remains mandatory.'}
        phase='settle_final'
        for _ in range(120):
            step(q,grip)
            if terminal.get('terms',{}).get('success',False) and np.linalg.norm(snapshot()['lime'][7:10])<.003:break
        final=snapshot();result['final_lime']=final['lime'].tolist();result['terminal_terms']=terminal.get('terms',{})
        result['final_live_success']=bool(arr(e.termination_manager.get_term('success'))[0])
        # Re-evaluate the original support/velocity predicate at the saved final
        # physical state: a previously completed predicate sequence can latch.
        from isaaclab.managers import SceneEntityCfg
        from isaaclab_arena.tasks.predicates.spatial import object_on_destination
        task=envdef.task
        result['final_on_destination']=bool(arr(object_on_destination(e,
            object_cfg=SceneEntityCfg(task.pick_up_object.name),
            destination_cfg=SceneEntityCfg(task.destination_location.name),
            contact_sensor_cfg=task.contact_sensor_cfg,force_threshold=task.force_threshold,
            velocity_threshold=task.velocity_threshold,
            support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
        result['final_destination_contact_force_matrix_w_n']=arr(scene[task.contact_sensor_cfg.name].data.force_matrix_w).tolist()
        result['success']=bool(result['terminal_terms'].get('success',False) and result['final_live_success'] and result['final_on_destination'] and abs(final['q'][robot.joint_names.index('finger_joint')])<.05)
        if args.variant=='property':
            result['property_displacement_m']=float(np.linalg.norm(final['hazard_pose'][:3]-initial['hazard_pose'][:3]))
            result['success'] &= result['property_displacement_m']<.01
    except Exception as exc:
        result['error']=repr(exc);result['traceback']=traceback.format_exc();print(result['traceback'],flush=True)
    finally:
        scene.update=original_update
        if writer:writer.close()
        if overview_writer:
            overview_writer.close()
            (out/'overview/frame_alignment.json').write_text(json.dumps({
                'scope':'Primary and supplementary RGB captured before the same actual action, without advancing physics between cameras.',
                'measurements':overview_alignment},indent=2))
            if overview_writer.frames:
                (out/'overview_camera.mp4').hardlink_to(out/'overview/overview_camera.mp4')
                result['overview_video_validation']=decode_video(out/'overview_camera.mp4',len(records),expected_size=(1280,720),expected_dt=e.step_dt)
            result['overview_capture_alignment']=bool(len(overview_alignment)==len(records) and all(
                x['primary_frame_index']==x['overview_frame_index']==x['action_index'] and
                x['physics_step_before']==x['physics_step_after'] for x in overview_alignment))
            result['success'] &= result['overview_capture_alignment']
        result['frames']=len(records);result['physics_safety_checks']=physics_checks;result['wall_s']=time.perf_counter()-started
        result['plans']=plans;result['clearance_checks']=len(clearances)
        result['cartesian_diagnostics']=getattr(planner,'cartesian_diagnostics',[])
        result['full_world_audit_passed']=bool(physics_audits and all(x['valid'] for x in physics_audits))
        (out/'full_world_audit.json').write_text(json.dumps({'passed':result['full_world_audit_passed'],
            'scope':'Every physics update: measured arm, articulated gripper and actual lime pose against all source colliders. Original round-fruit convex hull; measured initial resting support with 0.5 mm extra tolerance and 50 mm xy bound during pregrasp/approach/close; measured closed-grasp support with 0.5 mm jitter and monotonically tightening height bound during lift; fruit/bowl support allowed only during placement.',
            'minimum':min(physics_audits,key=lambda x:x['slack_m']) if physics_audits else None,
            'measurements':physics_audits},indent=2))
        if clearances:result['minimum_clearance']=min(clearances,key=lambda x:x['slack_m'])
        (out/'result.json').write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
        (out/'clearances.json').write_text(json.dumps(clearances,indent=2))
        (out/'contacts.json').write_text(json.dumps(contacts.records,indent=2))
        with h5py.File(out/'trajectory.h5','w') as f:
            f.attrs.update(task=TASK_ID,variant=args.variant,success=result['success'],dt=e.step_dt)
            f.create_dataset('time',data=np.arange(len(records))*e.step_dt)
            f.create_dataset('phase',data=np.array([r[0] for r in records],dtype=h5py.string_dtype()))
            f.create_dataset('action',data=np.array([r[1] for r in records]))
            if records:
                for group,index in [('obs',2),('next_obs',3)]:
                    for key in records[0][index]:f.create_dataset(group+'/'+key,data=np.array([r[index][key] for r in records]))
        if writer and writer.frames:
            result['video_validation']={name:decode_video(out/(name+'.mp4'),len(records),expected_dt=e.step_dt) for name in (*writer.names,'all_cameras')}
            (out/'result.json').write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
        print('LIME_RESULT',json.dumps({k:v for k,v in result.items() if k not in ('plans','traceback','close_contacts','lift_contacts')}),flush=True)
        env.close()
    if not result['success']:raise SystemExit(2)
