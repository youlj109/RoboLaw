"""One genuine Lightwheel pick-and-place rollout with cuRobo and original Arena success."""
import hashlib,json,time,traceback
from pathlib import Path
COLLECTOR_SOURCE_BYTES=Path(__file__).read_bytes()
COLLECTOR_SOURCE_SHA256=hashlib.sha256(COLLECTOR_SOURCE_BYTES).hexdigest()
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
parser=get_isaaclab_arena_cli_parser()
parser.add_argument('--task',default='kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian',
    help='Arena env id; default B01 lime. Use lemon shelf→plate id for B02.')
parser.add_argument('--pick-order',nargs='+',help='Execution order only for original parallel multi-pick task; all original picks remain required')
parser.add_argument('--variant',choices=['base','human','property','robot'],default='base')
parser.add_argument('--output',required=True)
parser.add_argument('--fixed-layout',help='Use the same saved original reset poses, including for the obstacle-free baseline')
parser.add_argument('--initial-state-layout',type=Path,help='Reset-only measured rigid poses from the approved scene; never written during execution')
parser.add_argument('--variant-config',help='Alternative safety-only hazard specifications; source task poses are unchanged')
parser.add_argument('--smooth-lift',action='store_true',help='Twice-duration smoothstep lift retiming; use the identical setting for baseline and all variants')
parser.add_argument('--allow-grasp-fallback',action='store_true',help='Legacy diagnostic fallback to 25/20 degrees; leave disabled for controlled avoidance comparisons')
parser.add_argument('--overview-resolution',type=int,nargs=2,default=(1280,720),help='Supplementary overview only; original policy camera sizes are unchanged')
parser.add_argument('--overview-camera',action='store_true',help='Supplementary fixed camera; preserve all original policy cameras')
parser.add_argument('--grasp-yaw',type=float,default=0)
parser.add_argument('--grasp-tilt',type=float,default=35)
parser.add_argument('--grasp-height-offset',type=float,default=.015,
    help='TCP height above measured pick-object geometry center; metres')
parser.add_argument('--grasp-lateral-x',type=float,default=0.0,
    help='World-frame +X TCP bias at pick center for finger centerline; metres')
parser.add_argument('--grasp-local-approach',action='store_true',
    help='Shelf: build pregrasp/standoff along -TCP_Z from selected wrist R (not fixed world offset)')
parser.add_argument('--defer-added-hazard-audit',action='store_true',help='With deferred full-world audit: retain every120Hz actual state, check added hazards15Hz live, require complete120Hz CPU recheck before validation')
parser.add_argument('--defer-full-world-audit',action='store_true',help='Record every 120 Hz state; check added hazards at 120 Hz, full world at 15 Hz live, and require complete 120 Hz offline full-world audit before acceptance')
parser.add_argument('--transfer-route-search',choices=('direct','upper'),default='direct',help='Try bounded upper waypoints at fixed hazards if direct transfer fails')
parser.add_argument('--grasp-body-point',nargs=3,type=float,default=None,help='Measured object-local solid grasp point; recorded as a control')
parser.add_argument('--grasp-search',choices=('fixed','b03-family','b03-topdown'),default='fixed')
parser.add_argument('--pregrasp-planner',choices=('auto','motion_gen','cartesian_line'),default='auto')
parser.add_argument('--pregrasp-standoff',type=float,default=None)
parser.add_argument('--grasp-finger-bias',type=float,default=0.0,
    help='Offset along grasp-frame finger-open axis (TCP Y col); metres')
parser.add_argument('--shelf-retract-y',type=float,default=-0.12,
    help='Shelf extract: final TCP delta Y (usually negative toward robot); metres')
parser.add_argument('--shelf-rise-z',type=float,default=0.12,
    help='Shelf extract: rise after retract clear of board_1; metres')
parser.add_argument('--shelf-retract-steps',type=int,default=3,
    help='Shelf extract: number of equal retract waypoints before rise')
parser.add_argument('--shelf-retract-clear-z',type=float,default=0.02,
    help='Shelf extract: small Z clearance held during level retract (deck clear, below board_1)')
parser.add_argument('--shelf-preclear-z',type=float,default=0.03,
    help='Shelf extract: pure +Z micro-rise first so attached fruit clears board_0 before -Y retract')
parser.add_argument('--shelf-lift-template',type=Path,default=None,
    help='Optional JSON with waypoints_delta_m from scan_shelf_extract; overrides CLI retract grid')
parser.add_argument('--place-lower-mode',choices=('cartesian_line','transfer','skip','steps'),default='cartesian_line',
    help='Shelf/plate lower: local Z line (default), MotionGen, skip-to-release, or multi-step Z')
parser.add_argument('--place-lower-z',type=float,default=None,
    help='Override TASK_SPEC.lower_z_m (metres above destination root)')
parser.add_argument('--place-lower-steps',type=int,default=3,
    help='When place-lower-mode=steps: equal Z drops from transfer height to lower height')
parser.add_argument('--place-clear-rise-z',type=float,default=0.0,
    help='After transfer, pure +Z rise before lower/release (metres); helps clear plate rim')
args=parser.parse_args();args.enable_cameras=True
if args.defer_added_hazard_audit and not args.defer_full_world_audit:raise ValueError('Deferred added hazards require complete deferred world audit')
if args.num_envs!=1:raise ValueError('This verified entry runs one environment; parallel collection is not implemented')
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task, picks as task_picks
TASK_SPEC=resolve_task(args.task)
if args.pick_order:
    from dataclasses import replace
    if not TASK_SPEC.pick_objects or len(args.pick_order)!=len(TASK_SPEC.pick_objects) or set(args.pick_order)!=set(TASK_SPEC.pick_objects):raise ValueError('Pick order must contain every original parallel pick exactly once')
    TASK_SPEC=replace(TASK_SPEC,pick_object=args.pick_order[0],pick_objects=tuple(args.pick_order))
TASK_ID=TASK_SPEC.task_id
ROOT=Path(__file__).resolve().parents[3]
SOURCE=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench'/(TASK_ID+'.yaml')
PICK=TASK_SPEC.pick_object
DEST=TASK_SPEC.destination_object
RIGID=TASK_SPEC.rigid_objects
SOURCE_SURFACE_PATH=(TASK_SPEC.source_surface_abs or ('/World/envs/env_0/kitchen'+TASK_SPEC.source_surface_suffix))
seed=42 if args.seed is None else args.seed
placement_seed=seed if args.placement_seed is None else args.placement_seed
out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
(out/'collector_source_snapshot.py').write_bytes(COLLECTOR_SOURCE_BYTES)
started=time.perf_counter()
with SimulationAppContext(args):
    import h5py,numpy as np,torch
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
    from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
    from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
    from kitchen_safety.extensions.lightwheel_pick_place.contacts import LimeGraspContacts,LimeSourceContacts
    from kitchen_safety.extensions.lightwheel_pick_place.source_support import JITTER_M,MAX_XY_M,DETACH_CLEARANCE_M,update_detached,native_contact_is_shallow
    from kitchen_safety.extensions.lightwheel_pick_place.lemon_kinematics import LemonKinematics
    from kitchen_safety.planning import DroidKinematics,transform
    from kitchen_safety.dataset import CameraWriter,numpy as arr,decode_video
    torch.set_num_threads(2)
    envdef,assets=build_environment(enable_cameras=True,variant=args.variant,task_id=TASK_ID,
        fixed_layout_path=args.fixed_layout,variant_config_path=args.variant_config)
    builder=ArenaEnvBuilder(envdef,ArenaEnvBuilderCfg(num_envs=1,seed=seed,device='cpu',
        presets='physx',solve_relations=True,resolve_on_reset=False,placement_seed=placement_seed))
    cfg,kwargs=builder.compose_manager_cfg();cfg.observations.camera_obs=None
    # PhysX raw source-contact views require reporting enabled before spawn.
    if PICK=='pepsi_can':cfg.scene.pepsi_can.spawn.activate_contact_sensors=True
    overview_spec=None
    if args.overview_camera:
        from kitchen_safety.extensions.lightwheel_pick_place.overview import configure_overview
        view={}
        if len(task_picks(TASK_SPEC))>1:view={'eye':[.6,-1.3,2.3],'target':[2.15,-2.65,.9]}
        elif PICK in ('green_ladle','pepsi_can'):
            layout=json.loads(Path(args.fixed_layout or TASK_SPEC.default_fixed_layout).read_text())['original_asset_initial_poses'];target=np.mean([layout[n]['position_xyz'] for n in (PICK,DEST)],axis=0)
            view={'eye':(target+np.array([.8,-1.5,1.3])).tolist(),'target':target.tolist()}
        overview_spec=configure_overview(cfg.scene,**view)
        width,height=args.overview_resolution
        if min(width,height)<64 or max(width,height)>4096:raise ValueError('Invalid supplementary resolution')
        cfg.scene.overview_camera.width=width;cfg.scene.overview_camera.height=height;overview_spec['resolution']=[width,height]
    cfg.sim.log_dir=str(out/'logs');cfg.sim.render_interval=cfg.decimation
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kwargs);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot']
    reset_layout_source=None
    if args.initial_state_layout:
        reset_layout=json.loads(args.initial_state_layout.read_text())['original_asset_initial_poses']
        before_reset_writes=int(e.sim.get_physics_step_count())
        for name in RIGID:
            pose=reset_layout[name]
            scene[name].write_root_pose_to_sim(torch.tensor([pose['position_xyz']+pose['rotation_xyzw']],device=e.device,dtype=torch.float32))
            scene[name].write_root_velocity_to_sim(torch.zeros((1,6),device=e.device))
        if int(e.sim.get_physics_step_count())!=before_reset_writes:raise RuntimeError('Initialization advanced physics')
        reset_layout_source={'path':str(args.initial_state_layout),'sha256':hashlib.sha256(args.initial_state_layout.read_bytes()).hexdigest(),'scope':'Once during reset, before controls, observations and task execution; no execution-time object pose writes.'}
    ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    q=arr(robot.data.joint_pos)[0,ids];grip=0.;phase='settle';records=[];clearances=[];plans=[];physics_audits=[]
    terminal={};world=None;planner=None;writer=None;overview_writer=None;overview_alignment=[];error=None
    finger_names=[n for i,n in enumerate(robot.joint_names) if i not in ids]
    bi=robot.body_names.index('base_link')
    def snapshot():
        quat=arr(robot.data.body_quat_w)[0,bi]
        row={'q':arr(robot.data.joint_pos)[0], 'dq':arr(robot.data.joint_vel)[0],
          'tcp':arr(robot.data.body_pos_w)[0,bi]+Rotation.from_quat(quat).apply([.131,0,0]),
          'tcp_quat_xyzw':quat,
          'robot_body_pose_w':np.concatenate([arr(robot.data.body_pos_w)[0],arr(robot.data.body_quat_w)[0]],axis=-1),
          'robot_contact_forces':arr(scene['robot_contacts'].data.net_forces_w)[0],
          **{name:arr(scene[name].data.root_state_w)[0] for name in RIGID},
          **({'hazard_pose':arr(scene['glass'].data.root_state_w)[0,:7]} if args.variant=='property' else {})}
        # Keep legacy lime/wooden_bowl keys as aliases for pick/destination.
        row['lime']=row[PICK]; row['wooden_bowl']=row[DEST]
        return row
    def physics_snapshot():
        # Physics audit never consumed TCP/body poses/dq/net robot forces.
        # Keep precisely the same q, finger, source/task and hazard inputs;
        # complete native snapshots remain at every15Hz action boundary.
        row={'q':arr(robot.data.joint_pos)[0],**{name:arr(scene[name].data.root_state_w)[0] for name in RIGID}}
        if args.variant=='property':row['hazard_pose']=arr(scene['glass'].data.root_state_w)[0,:7]
        row['lime']=row[PICK]
        return row

    def capture_reset(env_ids):
        if len(env_ids):
            terminal['state']=snapshot()
            terminal['terms']={n:bool(arr(e.termination_manager.get_term(n))[0]) for n in e.termination_manager.active_terms}
    e._reset_idx=capture_reset
    def ignores():
        return (PICK,)+((DEST,) if phase in ('lower','release','retreat','settle_final') else ())
    def geometry(state):
        return dict(mode='execution',gripper_q={n:state['q'][robot.joint_names.index(n)] for n in finger_names})
    physics_checks=0
    original_update=scene.update
    def checked_update(dt):
        global physics_checks
        original_update(dt)
        if world is None:return
        s=physics_snapshot();physics_checks+=1
        if physics_checks==1:
            reference=snapshot()
            if any(not np.array_equal(s[k],reference[k]) for k in ('q','lime',*RIGID,*(['hazard_pose'] if args.variant=='property' else []))):raise RuntimeError('Physics snapshot audit-input parity failed')
            result['physics_snapshot_input_parity_verified']=True
        world.allowed_phase=phase
        if args.variant=='property':world.set_obstacle_pose('glass',s['hazard_pose'])
        for name in RIGID:
            if name == PICK: continue
            world.set_obstacle_pose(name,s[name][:7])
        source_contact=source_contacts.sample()
        actual_lime_points=world.lime_vertices@Rotation.from_quat(s['lime'][3:7]).as_matrix().T+s['lime'][:3]
        actual_min_z=float(actual_lime_points[:,2].min());detached_before=world.source_detached
        native_calibration=bool(world.source_support_mode=='native_contact_calibrated_until_detached' and not detached_before and phase in ('pregrasp','approach','close','lift') and native_contact_is_shallow(source_contact))
        if native_calibration:
            world.source_resting_min_z=actual_min_z
            world.source_lift_floor_z=actual_min_z-JITTER_M

        # Full environment robot check; intended grasp/support masks are phase-specific.
        full=({'valid':None,'audit_pending':True} if args.defer_full_world_audit and physics_checks%8 else world.clearance(s['q'][ids],ignores(),object_pose=s['lime'][:7],**geometry(s)))
        physics_audits.append({'action':len(records),'phase':phase,'q_arm':s['q'][ids].tolist(),
            'gripper_q':{k:float(v) for k,v in geometry(s)['gripper_q'].items()},'lime_pose':s['lime'][:7].tolist(),'pick_object':PICK,
            **({'rigid_poses':{n:s[n][:7].tolist() for n in RIGID}} if args.defer_full_world_audit else {}),
            **({'hazard_pose':s['hazard_pose'].tolist()} if args.variant=='property' else {}),
            'source_contact':source_contact,'source_hull_min_z_m':actual_min_z,
            'source_detached_before':detached_before,'source_calibration_native_contact':native_calibration,
            'source_support_calibrated_min_z_m':float(world.source_resting_min_z),
            'source_lift_floor_z_m':getattr(world,'source_lift_floor_z',None),**full})
        if full['valid'] is False:raise RuntimeError(f'Executed collision/clearance: {full}')
        world.source_detached=update_detached(detached=world.source_detached,hull_min_z=actual_min_z,
            surface_top=world.source_surface_top,source_contact_count=source_contact['count'])
        physics_audits[-1]['source_detached_after']=world.source_detached
        if args.variant!='base':
            hazard={'human':'person','property':'glass','robot':'barrier'}[args.variant]
            c=({'valid':None,'audit_pending':True} if args.defer_added_hazard_audit and physics_checks%8 else world.clearance(s['q'][ids],(PICK,),object_pose=s['lime'][:7],obstacle_filter=(hazard,),**geometry(s)))
            if c['valid'] is not None:physics_audits[-1]['live_added_hazard_clearance']=c
            clearances.append({'action':len(records),'phase':phase,**c})
            if c['valid'] is False:raise RuntimeError(f'Unsafe lime/robot clearance: {c}')
    scene.update=checked_update
    from pxr import Usd,UsdPhysics
    pick_rigid_paths={}
    for name in task_picks(TASK_SPEC):
        candidates=[str(prim.GetPath()) for prim in Usd.PrimRange(e.sim.stage.GetPrimAtPath('/World/envs/env_0/'+name)) if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(candidates)!=1:raise RuntimeError('Expected exactly one original rigid body for '+name+': '+str(candidates))
        pick_rigid_paths[name]=candidates[0]
    contacts=LimeGraspContacts(scene['robot_contacts']._physics_sim_view,cfg.sim.dt,pick_object=PICK,rigid_body_path=pick_rigid_paths[PICK])
    contact_trackers={PICK:contacts}
    pick_action_ranges={PICK:{'start':0}}
    source_contact_trackers={name:LimeSourceContacts(scene['robot_contacts']._physics_sim_view,cfg.sim.dt,pick_object=name,source_surface=SOURCE_SURFACE_PATH,rigid_body_path=pick_rigid_paths[name]) for name in task_picks(TASK_SPEC)}
    source_contacts=source_contact_trackers[PICK]
    step_wall_profile={'before_snapshot_s':0.,'camera_s':0.,'env_step_s':0.,'after_snapshot_s':0.,'contact_s':0.,'steps':0}
    def step(command,gripper):
        tick=time.perf_counter();before=snapshot();step_wall_profile['before_snapshot_s']+=time.perf_counter()-tick
        tick=time.perf_counter()
        if writer:
            before_capture=int(e.sim.get_physics_step_count())
            writer.write(phase)
            if overview_writer:
                overview_writer.write(phase)
                overview_alignment.append({'action_index':len(records),'phase':phase,
                    'primary_frame_index':writer.frames-1,'overview_frame_index':overview_writer.frames-1,
                    'physics_step_before':before_capture,'physics_step_after':int(e.sim.get_physics_step_count()),
                    'simulation_time_s':before_capture*cfg.sim.dt})
        step_wall_profile['camera_s']+=time.perf_counter()-tick
        action=np.r_[command,gripper].astype(np.float32)
        tick=time.perf_counter()
        try:env.step(torch.tensor(action[None],device=e.device))
        finally:
            step_wall_profile['env_step_s']+=time.perf_counter()-tick;tick=time.perf_counter()
            records.append((phase,action,before,snapshot()))
            step_wall_profile['after_snapshot_s']+=time.perf_counter()-tick;step_wall_profile['steps']+=1
        if terminal and not terminal.get('terms',{}).get('success',False):raise RuntimeError(f'Arena termination: {terminal["terms"]}')
        tick=time.perf_counter()
        if phase in ('close','lift','transfer','lower'):contacts.sample(phase,len(records)-1,snapshot()['lime'][:7])
        step_wall_profile['contact_s']+=time.perf_counter()-tick
        if len(records)%30==0:print('STEP',len(records),phase,snapshot()['lime'][:3].round(4),flush=True);print('STEP_WALL_PROFILE',json.dumps(step_wall_profile),flush=True)
    result={'step_wall_profile_seconds':step_wall_profile,'collector_source_sha256':COLLECTOR_SOURCE_SHA256,'task':TASK_ID,'variant':args.variant,'pregrasp_planner_mode':args.pregrasp_planner,'success':False,'planner':'curobo','physical_grasp':True,
      'seed':seed,'placement_seed':placement_seed,'fixed_only':True,
      'source_predicate':'Original PickAndPlaceTask: settled, lifted, destination support and low speed',
      'requested_grasp_tilt_deg':args.grasp_tilt,'requested_grasp_yaw_deg':args.grasp_yaw,
      'requested_grasp_height_offset_m':args.grasp_height_offset,
      'smooth_lift':args.smooth_lift,
      'allowed_grasp_fallback':args.allow_grasp_fallback,
      'added_hazard_audit_schedule':'120Hz recorded/offline with15Hz live checkpoints' if args.defer_added_hazard_audit else '120Hz live',
      'defer_added_hazard_audit':bool(args.defer_added_hazard_audit),
      'full_world_audit_schedule':(('120Hz recorded/offline; full world 15Hz live; added hazards '+('120Hz offline/15Hz live' if args.defer_added_hazard_audit else '120Hz live')) if args.defer_full_world_audit else '120Hz live'),
      'source_yaml_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
      'original_asset_initial_poses':{name:assets[name]._get_initial_pose_as_pose().to_dict() for name in ('droid',)+RIGID if name in assets and assets[name].get_initial_pose() is not None},
      'simulation_attachment':False,'robot_joint_names':robot.joint_names,'robot_body_names':robot.body_names,'dt':e.step_dt}
    if reset_layout_source:result['initial_state_layout_source']=reset_layout_source
    if args.fixed_layout:
        result['fixed_layout_source']={'path':args.fixed_layout,'sha256':hashlib.sha256(Path(args.fixed_layout).read_bytes()).hexdigest()}
    if args.variant!='base':
        config_path=Path(args.variant_config or TASK_SPEC.default_variants)
        result['variant_spec']=json.loads(config_path.read_text())[args.variant]
        result['variant_config_source']={'path':str(config_path),'sha256':hashlib.sha256(config_path.read_bytes()).hexdigest()}
    import signal
    def preserve_on_external_stop(signum,frame):
        result['success']=False;result['interruption_signal']=int(signum)
        raise TimeoutError('External stop requested; preserve partial observations, never claim native success')
    previous_signal_handlers={sig:signal.signal(sig,preserve_on_external_stop) for sig in (signal.SIGTERM,signal.SIGINT)}
    try:
        for _ in range(20):step(q,0.)
        initial=snapshot();result['initial_object_states']={name:arr(scene[name].data.root_state_w)[0].tolist() for name in RIGID};result['initial_lime_pose']=initial['lime'].tolist();result['initial_bowl_pose']=initial['wooden_bowl'].tolist();result['pick_object']=PICK;result['destination_object']=DEST;result['pick_objects']=list(task_picks(TASK_SPEC))
        base=arr(robot.data.root_state_w)[0,:7];kin=(LemonKinematics if PICK=='lemon' else DroidKinematics)(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2])
        result['fk_error_m']=float(np.linalg.norm(kin.fk(initial['q'][ids])[:3,3]-initial['tcp']))
        if result['fk_error_m']>.005:raise RuntimeError('Robot FK disagrees with PhysX TCP')
        result['robot_base_pose']=base.tolist()
        movable=tuple(n for n in RIGID if n!=PICK)+('glass',)
        world=CollisionWorld(e.sim.stage,kin,pick_object=PICK,destination_object=DEST,
            source_surface_suffix=TASK_SPEC.source_surface_suffix,movable_objects=movable)
        initial_lime_points=world.lime_vertices@Rotation.from_quat(initial['lime'][3:7]).as_matrix().T+initial['lime'][:3]
        world.source_resting_min_z=float(initial_lime_points[:,2].min())
        world.source_resting_xy=initial['lime'][:2].copy()
        world.source_support_mode=('native_contact_calibrated_until_detached' if len(task_picks(TASK_SPEC))>1 else 'native_rest_until_detached');world.source_detached=False
        world.source_resting_root=initial['lime'][:3].copy()
        world.source_surface_top=max((v[3][2] for v in world.objects if v[0]==SOURCE_SURFACE_PATH or v[0].startswith(SOURCE_SURFACE_PATH.rstrip('/')+'/') or v[0].endswith(TASK_SPEC.source_surface_suffix)), default=float(initial['lime'][2]))
        result['per_pick_initial_source_contacts']={name:view.sample() for name,view in source_contact_trackers.items()}
        initial_source_contacts=source_contacts.sample()
        if not initial_source_contacts['count']:raise RuntimeError('No measured original pick-object/source-surface support contacts')
        result['source_support_contact_model']={'source_resting_min_z_m':world.source_resting_min_z,
            'source_resting_xy_m':world.source_resting_xy.tolist(),'source_resting_extra_tolerance_m':.0005,'source_resting_max_xy_displacement_m':.05,
            'mode':world.source_support_mode,'native_contact_max_penetration_m':JITTER_M,'root_z_gate':False,'source_resting_root_m':world.source_resting_root.tolist(),
            'surface':SOURCE_SURFACE_PATH,'surface_top_m':float(world.source_surface_top),'detach_clearance_m':DETACH_CLEARANCE_M,
            'initial_source_contacts':initial_source_contacts,
            'lift_support_policy':'Native legal rest collision-hull lower bound until geometric separation and zero exact-source contacts; detach latches permanently.',
            'reason':'Raw source hull differs from native PhysX convexDecomposition rest. Preserve only that measured source-contact region during small fruit rolling at liftoff; all external margins are unchanged. Closed-grasp hull-bottom monotonicity was an over-strong post-added condition, not native contact semantics.'}
        e.sim.stage.Export(str(out/'scene.usda'))
        (out/'initial_setup.json').write_text(json.dumps({
            'original_asset_initial_poses':result['original_asset_initial_poses'],
            'initial_object_states':result['initial_object_states'],
            'robot_base_pose':result['robot_base_pose'],
            'robot_joint_names':robot.joint_names,'robot_joint_positions':initial['q'].tolist(),
            'source_yaml_sha256':result['source_yaml_sha256']},indent=2))
        result['start_fcl']=world.clearance(initial['q'][ids],(PICK,),**geometry(initial))
        planner=LimePlanner(world,kin,dt=e.step_dt,seed=seed,max_joint_speed=.4);planner.pose_ik_joint_fallback=bool(args.grasp_search=='b03-topdown' or args.transfer_route_search=='upper')
        use_pregrasp_phase=bool(abs(TASK_SPEC.pregrasp_offset_m[2])>=max(abs(TASK_SPEC.pregrasp_offset_m[0]),abs(TASK_SPEC.pregrasp_offset_m[1])))
        writer=CameraWriter(out,scene,envdef.embodiment.camera_config.camera_names(),e.step_dt,previews=True)
        if args.overview_camera:
            overview_dir=out/'overview';overview_dir.mkdir()
            overview_writer=CameraWriter(overview_dir,scene,['overview_camera'],e.step_dt,previews=True)
            result['overview_camera']=overview_spec
        # Unrecorded settling records are omitted so RGB/trajectory alignment is exact.
        records.clear();clearances.clear()
        result['camera_warmup']=writer.warm_up(e.sim,min_frames=64,max_frames=256)
        if overview_writer:
            result['overview_camera_warmup']=overview_writer.warm_up(e.sim,min_frames=64,max_frames=256)
            import cv2
            overview_initial=arr(scene['overview_camera'].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
            if not cv2.imwrite(str(out/'overview_initial.png'),cv2.cvtColor(overview_initial,cv2.COLOR_RGB2BGR)):
                raise RuntimeError('Could not save actual initial overview for visibility review')
        def close_with_feedback(window):
            global phase,grip
            from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import close_recenter_delta
            for attempt in range(3):
                phase='close';grip=1.
                for _ in range(30):step(q,grip)
                measured=contacts.summarize(list(TASK_SPEC.grasp_body_z_range),'close',last_actions=window)
                note={'pick':PICK,'attempt':attempt,'bilateral_near':measured['both_fingers_near_body'],'bilateral_load':measured['both_fingers_on_body']}
                result.setdefault('close_feedback',[]).append(note)
                if measured['both_fingers_on_body']:return measured
                if attempt==2:break
                state=snapshot();solution=result['grasp_solutions'][PICK];delta,detail=close_recenter_delta(world,kin,state,ids,robot.joint_names,contacts.records[-1],solution);note.update(detail)
                print('GRASP_RECENTER',PICK,attempt,detail,flush=True)
                grip=0.
                for _ in range(12):step(q,grip)
                phase='approach';cart(snapshot()['tcp']+delta,force_phase='cartesian_line')
            raise RuntimeError('Bilateral loaded grasp not established after bounded feedback recentering')

        def all_current_native_predicates():
            from isaaclab.managers import SceneEntityCfg
            from isaaclab_arena.tasks.predicates.spatial import object_on_destination
            for task in getattr(envdef.task,'subtasks',[envdef.task]):
                if not bool(arr(object_on_destination(e,object_cfg=SceneEntityCfg(task.pick_up_object.name),destination_cfg=SceneEntityCfg(task.destination_location.name),contact_sensor_cfg=task.contact_sensor_cfg,force_threshold=task.force_threshold,velocity_threshold=task.velocity_threshold,support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0]):return False
            return True

        def capture_placement_checkpoint(name):
            from isaaclab.managers import SceneEntityCfg
            from isaaclab_arena.tasks.predicates.spatial import object_on_destination
            task=next(t for t in getattr(envdef.task,'subtasks',[envdef.task]) if t.pick_up_object.name==name)
            passed=bool(arr(object_on_destination(e,object_cfg=SceneEntityCfg(name),destination_cfg=SceneEntityCfg(task.destination_location.name),contact_sensor_cfg=task.contact_sensor_cfg,force_threshold=task.force_threshold,velocity_threshold=task.velocity_threshold,support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
            row={'action_index':len(records)-1,'passed':passed,'force_matrix_w_n':arr(scene[task.contact_sensor_cfg.name].data.force_matrix_w).tolist(),'object_state':snapshot()[name].tolist()}
            result.setdefault('placement_checkpoints',{})[name]=row;print('PLACEMENT_CHECKPOINT',name,passed,row['force_matrix_w_n'],flush=True)

        def cart(position,*,rotation=None,force_phase=None):
            global q,planner
            world.allowed_phase=phase
            s=snapshot();q=s['q'][ids].copy()
            if world.attachment is not None and phase in ('lift','transfer','lower'):
                # Collision model feedback only; never edits a PhysX body/constraint.
                measured_attachment=np.linalg.inv(kin.fk(q))@transform(s['lime'][:3],s['lime'][3:7])
                previous=np.asarray(world.attachment)
                result.setdefault('planning_attachment_updates',[]).append({
                    'pick_object':PICK,'phase':phase,'action_index':len(records),
                    'tcp_to_pick':measured_attachment.tolist(),
                    'translation_correction_m':float(np.linalg.norm(measured_attachment[:3,3]-previous[:3,3])),
                    'rotation_correction_deg':float(np.rad2deg(Rotation.from_matrix(measured_attachment[:3,:3]@previous[:3,:3].T).magnitude())),
                    'source':'Measured rigid pose and arm FK at segment start; collision model only.'})
                world.attachment=measured_attachment
            world.gripper_joint_position=float(s['q'][robot.joint_names.index('finger_joint')])
            if args.variant=='property':
                world.set_obstacle_pose('glass',s['hazard_pose'])
                if len(task_picks(TASK_SPEC))==1:planner.update_dynamic_obstacle('glass')
            if len(task_picks(TASK_SPEC))>1 or PICK in ('green_ladle','pepsi_can'):
                for name in RIGID:
                    if name!=PICK:
                        world.set_obstacle_pose(name,s[name][:7])
                # The backend only refreshes hazard boxes. Rebuild for moved task meshes.
                import gc
                planner=None;gc.collect()
                planner=LimePlanner(world,kin,dt=e.step_dt,seed=seed,max_joint_speed=.4);planner.pose_ik_joint_fallback=bool(args.grasp_search=='b03-topdown' or args.transfer_route_search=='upper')
            if phase=='pregrasp':planner.pregrasp_contact_offset_m=np.asarray(pregrasp_offset).copy()
            target=np.eye(4);target[:3,3]=position
            target[:3,:3]=(kin.fk(q)[:3,:3] if rotation is None else rotation)
            # Shelf/lateral approach cannot use cuRobo vertical-column phases.
            # Pure +Z extract segments keep phase=lift (local cartesian); laterals use transfer.
            # Shelf lower is ~1 cm under transfer; vertical-column IK residual fails — use MotionGen.
            planning_phase=phase
            if force_phase is not None:
                planning_phase=force_phase
            elif (not use_pregrasp_phase) and phase in ('pregrasp','approach','lift','lower'):
                planning_phase='transfer'
            try:
                if planning_phase=='cartesian_line':
                    trajectory=planner.plan_cartesian_line(q,target,ignores(),grip,phase='lift')
                else:
                    if (args.grasp_search in ('b03-family','b03-topdown') or args.transfer_route_search=='upper') and planning_phase=='transfer':
                        from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import plan_transfer
                        trajectory=plan_transfer(planner,q,target,ignores(),grip)
                    else:trajectory=planner.plan_pose(q,target,ignores(),grip,phase=planning_phase)
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
            if phase=='lift' and args.smooth_lift:
                raw=np.asarray(trajectory['positions']);count=2*(len(raw)-1)+1
                progress=np.linspace(0.,1.,count);progress=3*progress**2-2*progress**3
                trajectory['positions']=np.column_stack([np.interp(progress,np.linspace(0.,1.,len(raw)),raw[:,j]) for j in range(7)])
                trajectory['metadata']={**trajectory['metadata'],'execution_retiming':{'method':'twice-duration smoothstep','original_samples':len(raw),'samples':count,'same_geometric_joint_path':True}}
            validation=planner.validate_path(trajectory['positions'],ignores(),grip,phase=phase)
            plans.append({'phase':phase,'target':target.tolist(),'metadata':trajectory['metadata'],'validation':validation,'executed':False})
            if not validation['valid']:raise RuntimeError(f'Path validation failed in {phase}: {validation}')
            if phase=='pregrasp' and use_pregrasp_phase:
                contact_target=target.copy();contact_target[:3,3]-=pregrasp_offset
                lookahead=planner.plan_pose(trajectory['positions'][-1],contact_target,ignores(),grip,phase='approach')
                checked=planner.validate_path(lookahead['positions'],ignores(),grip,phase='approach')
                plans[-1]['approach_lookahead']=checked
                if not checked['valid']:raise RuntimeError('Approach lookahead is not collision-free')
            plans[-1]['executed']=True
            for command in trajectory['positions']:step(command,grip)
            q=trajectory['positions'][-1].copy()
            for _ in range(12):step(q,grip)
        p=initial['lime'][:3].copy()
        grasp_body_point=np.asarray(args.grasp_body_point if args.grasp_body_point is not None else world.object_center,float)
        result['grasp_point_body_m']=grasp_body_point.tolist()
        result['transfer_route_policy']={'mode':args.transfer_route_search,'pose_ik_joint_goal_fallback':bool(args.grasp_search=='b03-topdown' or args.transfer_route_search=='upper'),'upper_rises_m':[.08,.16,.24] if args.transfer_route_search=='upper' else []}
        target_center=transform(initial['lime'][:3],initial['lime'][3:7])@np.r_[grasp_body_point,1]
        target_center[2]+=args.grasp_height_offset
        target_center[0]+=args.grasp_lateral_x
        result['grasp_lateral_x_m']=float(args.grasp_lateral_x)
        result['grasp_finger_bias_m']=float(args.grasp_finger_bias)
        result['grasp_local_approach']=bool(args.grasp_local_approach)
        topdown_solution=None
        if args.grasp_search=='b03-topdown':
            from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import search_topdown,TOPDOWN_POLICY
            for name in RIGID:
                if name!=PICK:world.set_obstacle_pose(name,initial[name][:7])
            topdown_solution,attempts=search_topdown(world,kin,q,initial['lime'][:7],bias_x_m=args.grasp_lateral_x,finger_bias_m=args.grasp_finger_bias,dt=e.step_dt,seed=seed)
            result['grasp_search_policy']=TOPDOWN_POLICY;result['grasp_solutions']={PICK:{k:v for k,v in topdown_solution.items() if k not in ('rotation','center','offset')}};result['grasp_search_attempts']={PICK:attempts}
            target_center[:3]=topdown_solution['center']-args.grasp_finger_bias*topdown_solution['rotation'][:,1];use_pregrasp_phase=True
        # use_pregrasp_phase computed earlier from TASK_SPEC for cart() planning-phase selection.
        candidates=list(dict.fromkeys([(args.grasp_tilt,args.grasp_yaw)]+(
            [(25,0),(20,0),(35,90),(35,-90),(10,90),(10,-90),(0,90),(0,-90)] if args.allow_grasp_fallback else [])))
        if args.grasp_search=='b03-family':
            candidates=list(dict.fromkeys([(args.grasp_tilt,args.grasp_yaw),(-35,180),(-55,180),(-75,180),(-90,180),(-35,0),(-55,0),(-75,0),(-90,0)]))
            result['grasp_search_policy']={'name':'b03-family-v1','candidates':candidates,'contact_standoff_m':.02,'lift_m':.12,'closed_grip_preflight_rad':.383,'upper_route_rises_m':[.08,.16,.24]}
        if topdown_solution:candidates=[(topdown_solution['tilt'],topdown_solution['yaw'])]
        result['pregrasp_candidates']=[]
        selected_R=None
        for tilt,yaw in candidates:
            R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]],dtype=float)
            # Optional finger-open bias in grasp frame (TCP Y = column 1).
            center=target_center[:3]+float(args.grasp_finger_bias)*R[:,1]
            if args.grasp_local_approach and (not use_pregrasp_phase):
                # TCP Z (col 2) should point into the object; hover back along -TCP_Z.
                approach=R[:,2]; approach=approach/max(float(np.linalg.norm(approach)),1e-6)
                pregrasp_offset=-(args.pregrasp_standoff if args.pregrasp_standoff is not None else (.17 if PICK=='lemon' else .18))*approach
            else:
                pregrasp_offset=(np.array([0.,0.,.08]) if args.grasp_search=='b03-topdown' else np.asarray(TASK_SPEC.pregrasp_offset_m,dtype=float))
            try:
                phase='pregrasp' if use_pregrasp_phase else 'approach'
                # Use an oriented hover before the short local contact approach.
                if args.grasp_search=='b03-family':
                    from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import preflight_grasp
                    for name in RIGID:
                        if name!=PICK:world.set_obstacle_pose(name,initial[name][:7])
                    planner=LimePlanner(world,kin,dt=e.step_dt,seed=seed,max_joint_speed=.4);planner.pose_ik_joint_fallback=bool(args.grasp_search=='b03-topdown' or args.transfer_route_search=='upper')
                    preflight=preflight_grasp(planner,q,R,center,pregrasp_offset,initial['lime'][:7],PICK)
                    result.setdefault('grasp_preflight_passes',[]).append({'tilt':tilt,'yaw':yaw,**preflight})
                cart(center+pregrasp_offset,rotation=R, force_phase=('transfer' if args.pregrasp_planner=='motion_gen' else ('cartesian_line' if args.pregrasp_planner=='cartesian_line' or (args.pregrasp_planner=='auto' and args.grasp_local_approach and not use_pregrasp_phase and PICK!='lemon') else None)))
                result['selected_grasp']={'tilt':tilt,'yaw':yaw,'phase':phase,'use_pregrasp_phase':use_pregrasp_phase,
                    'tcp_z_world':approach.tolist() if (args.grasp_local_approach and not use_pregrasp_phase) else None}
                if topdown_solution:result['selected_grasp']['height_offset_m']=topdown_solution['height_offset_m']
                if topdown_solution:
                    for key in ('pinch_z_offset_m','waist_y_shift_m'):
                        if key in topdown_solution:result['selected_grasp'][key]=topdown_solution[key]
                result['pregrasp_offset_m']=pregrasp_offset.tolist()
                selected_R=R; target_center[:3]=center; break
            except RuntimeError as exc:
                result['pregrasp_candidates'].append({'tilt':tilt,'yaw':yaw,'error':str(exc)})
                if records:raise
        else:raise RuntimeError('All pregrasp pose candidates failed')
        phase='approach'
        if use_pregrasp_phase:
            cart(target_center[:3])
        else:
            # Shelf: stop short along the same approach axis used for hover.
            axis=np.asarray(result['pregrasp_offset_m'],dtype=float)
            axis=axis/max(float(np.linalg.norm(axis)),1e-6)
            contact_standoff=target_center[:3]+axis*0.02
            result['approach_contact_standoff_m']=contact_standoff.tolist()
            # Banana feasibility: MotionGen transfer IK_FAIL on hover->contact; cartesian_line succeeds near tol.
            cart(contact_standoff, force_phase=('transfer' if PICK=='lemon' else 'cartesian_line'))
        if args.grasp_search=='b03-topdown':result['close_contacts']=close_with_feedback(5)
        else:
            phase='close';grip=1.
            for _ in range(30):step(q,grip)
            result['close_contacts']=contacts.summarize(list(TASK_SPEC.grasp_body_z_range),'close',last_actions=5)
        held=snapshot()
        if not result['close_contacts']['both_fingers_near_body']:raise RuntimeError('No bilateral physical lime grasp')
        world.attachment=np.linalg.inv(kin.fk(held['q'][ids]))@transform(held['lime'][:3],held['lime'][3:7])
        result['planning_attachment_tcp_to_lime']=world.attachment.tolist()
        held_points=world.lime_vertices@Rotation.from_quat(held['lime'][3:7]).as_matrix().T+held['lime'][:3]
        held_min_z=float(held_points[:,2].min())
        if held_min_z<world.source_resting_min_z-.0005:raise RuntimeError('Closed grasp went below the original calibrated source support')
        world.source_lift_floor_z=world.source_resting_min_z-JITTER_M
        result['lift_support_contact_model']={'measured_closed_grasp_min_z_m':held_min_z,
            'mode':world.source_support_mode,
            'initial_lower_bound_m':world.source_lift_floor_z,'jitter_tolerance_m':.0005,
            'xy_anchor_m':world.source_resting_xy.tolist(),'max_xy_displacement_m':.05,
            'policy':'Only exact original source surface and 50 mm xy region, collision hull bottom at least initial native rest minus 0.5 mm (asset origin is not a penetration measure); geometric separation plus zero contacts permanently revokes exception.'}
        if len(task_picks(TASK_SPEC))>1:
            phase='lift';cart(held['tcp']+np.array([0.,0.,.10 if args.grasp_search=='b03-topdown' else .12]),force_phase='lift')
        elif use_pregrasp_phase:
            result['commanded_initial_lift_m']=float(TASK_SPEC.lift_height_m)
            phase='lift';cart(held['tcp']+[0,0,TASK_SPEC.lift_height_m])
        else:
            # Shelf extract: scan template waypoints if provided; else preclear→retract→rise.
            phase='lift'
            if args.shelf_lift_template is not None:
                tmpl=json.loads(Path(args.shelf_lift_template).read_text())
                waypoints=[np.asarray(w,dtype=float) for w in tmpl['waypoints_delta_m']]
                result['shelf_lift_retract_m']={'template':str(args.shelf_lift_template),
                    'waypoints':[w.tolist() for w in waypoints], **{k:tmpl[k] for k in tmpl if k!='waypoints_delta_m'}}
            else:
                n=max(1,int(args.shelf_retract_steps))
                y_final=float(args.shelf_retract_y); z_rise=float(args.shelf_rise_z)
                z_clear=float(args.shelf_retract_clear_z); z_pre=float(args.shelf_preclear_z)
                waypoints=[]
                if z_pre>0:
                    waypoints.append(np.array([0.0, 0.0, z_pre]))
                hold_z=max(z_pre, z_clear)
                for i in range(1,n+1):
                    frac=i/n
                    t=0.5*frac+0.5*(frac*frac)
                    waypoints.append(np.array([0.0, y_final*t, hold_z]))
                waypoints.append(np.array([0.0, y_final, z_rise]))
                result['shelf_lift_retract_m']={'y_final':y_final,'z_rise':z_rise,'z_clear':z_clear,
                    'z_preclear':z_pre,'steps':n,'waypoints':[w.tolist() for w in waypoints]}
            for wi,w in enumerate(waypoints):
                try:
                    # Phase from consecutive step, not absolute offset (rise keeps prior Y).
                    prev=np.zeros(3) if wi==0 else np.asarray(waypoints[wi-1],dtype=float)
                    step_delta=np.asarray(w,dtype=float)-prev
                    if abs(float(step_delta[0]))<1e-9 and abs(float(step_delta[1]))<1e-9:
                        force='lift'
                    else:
                        force='cartesian_line'
                    cart(held['tcp']+w, force_phase=force)
                except RuntimeError as exc:
                    result['shelf_lift_failed_waypoint']= {'index':wi,'delta_m':w.tolist(),
                        'step_m':step_delta.tolist(),'error':str(exc),'plan_phase':force}
                    raise
        lifted=snapshot();result['lift_m']=float(lifted['lime'][2]-p[2]);result['lift_contacts']=contacts.summarize(list(TASK_SPEC.grasp_body_z_range),'lift',last_actions=5)
        if result['lift_m']<.08 or not result['lift_contacts']['both_fingers_on_body']:raise RuntimeError('Lime failed physical lift/contact check')
        dest=snapshot()['wooden_bowl'];offset=lifted['tcp']-lifted['lime'][:3]
        # Release above destination then let gravity settle the fruit.
        desired_root=dest[:3]+np.array([(-.06 if PICK==TASK_SPEC.pick_object else .06) if len(task_picks(TASK_SPEC))==2 else 0,0,TASK_SPEC.transfer_z_m])
        phase='transfer';cart(desired_root+offset)
        if float(args.place_clear_rise_z)>0 and (not use_pregrasp_phase):
            phase='transfer'
            cart(snapshot()['tcp']+np.array([0.0,0.0,float(args.place_clear_rise_z)]), force_phase='transfer')
            result['place_clear_rise_z_m']=float(args.place_clear_rise_z)
        lower_z=float(TASK_SPEC.lower_z_m if args.place_lower_z is None else args.place_lower_z)
        lower_root=dest[:3]+np.array([(-.06 if PICK==TASK_SPEC.pick_object else .06) if len(task_picks(TASK_SPEC))==2 else 0,0.0,lower_z])
        xfer_tcp=desired_root+offset
        placement_state=snapshot();lower_tcp=lower_root+(placement_state['tcp']-placement_state['lime'][:3])
        result['place_lower']={'mode':args.place_lower_mode,'lower_z_m':lower_z,
            'transfer_tcp_m':xfer_tcp.tolist(),'lower_tcp_m':lower_tcp.tolist(),
            'drop_m':float(xfer_tcp[2]-lower_tcp[2])}
        phase='lower'
        if args.place_lower_mode=='skip' or (args.place_lower_mode=='steps' and abs(xfer_tcp[2]-lower_tcp[2])<1e-4):
            result['place_lower']['skipped']=True
        elif args.place_lower_mode=='steps':
            n=max(1,int(args.place_lower_steps))
            for i in range(1,n+1):
                frac=i/n
                mid=xfer_tcp*(1.0-frac)+lower_tcp*frac
                cart(mid, force_phase='cartesian_line')
        elif args.place_lower_mode=='cartesian_line':
            cart(lower_tcp, force_phase='cartesian_line')
        else:
            cart(lower_tcp, force_phase='transfer')
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
            if terminal.get('terms',{}).get('success',False) and all_current_native_predicates():break
        capture_placement_checkpoint(PICK)
        # DUAL_PICK_LOOP: place remaining picks (B03 banana_2) with same skill template.
        remaining=list(task_picks(TASK_SPEC))[1:]
        result['pick_objects']=list(task_picks(TASK_SPEC))
        for pick_name in remaining:
            pick_action_ranges[PICK]['end']=len(records)
            PICK=pick_name
            pick_action_ranges[PICK]={'start':len(records)}
            world.bind_pick(PICK)
            contacts=LimeGraspContacts(scene['robot_contacts']._physics_sim_view,cfg.sim.dt,pick_object=PICK,rigid_body_path=pick_rigid_paths[PICK])
            contact_trackers[PICK]=contacts
            source_contacts=source_contact_trackers[PICK]
            held0=snapshot();p=held0['lime'][:3].copy()
            points=world.lime_vertices@Rotation.from_quat(held0['lime'][3:7]).as_matrix().T+p
            world.source_resting_min_z=float(points[:,2].min());world.source_resting_xy=p[:2].copy()
            world.source_resting_root=p.copy();world.source_detached=False
            world.source_lift_floor_z=world.source_resting_min_z-JITTER_M
            result.setdefault('extra_source_support_models',{})[PICK]={'source_resting_min_z_m':world.source_resting_min_z,'source_resting_root_m':p.tolist(),'source_surface_top_m':float(world.source_surface_top),'jitter_tolerance_m':JITTER_M}
            if not source_contacts.sample()['count']:raise RuntimeError(f'No measured original source support for {PICK}')
            target_center=transform(held0['lime'][:3],held0['lime'][3:7])@np.r_[world.object_center,1]
            target_center[2]+=args.grasp_height_offset
            target_center[0]+=args.grasp_lateral_x
            R=Rotation.from_euler('z',args.grasp_yaw,degrees=True).as_matrix()@Rotation.from_euler('x',args.grasp_tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]],dtype=float)
            center=target_center[:3]+float(args.grasp_finger_bias)*R[:,1]
            if use_pregrasp_phase:
                pregrasp_offset=(np.array([0.,0.,.08]) if args.grasp_search=='b03-topdown' else np.asarray(TASK_SPEC.pregrasp_offset_m,dtype=float))
            else:
                # Prefer local approach when flag set; else TASK_SPEC offset.
                if args.grasp_local_approach:
                    approach=R[:,2]; approach=approach/max(float(np.linalg.norm(approach)),1e-6)
                    pregrasp_offset=-(args.pregrasp_standoff if args.pregrasp_standoff is not None else (.17 if PICK=='lemon' else .18))*approach
                else:
                    pregrasp_offset=(np.array([0.,0.,.08]) if args.grasp_search=='b03-topdown' else np.asarray(TASK_SPEC.pregrasp_offset_m,dtype=float))
            if args.grasp_search=='b03-topdown':
                for name in RIGID:
                    if name!=PICK:world.set_obstacle_pose(name,held0[name][:7])
                solution,attempts=search_topdown(world,kin,held0['q'][ids],held0['lime'][:7],bias_x_m=args.grasp_lateral_x,finger_bias_m=args.grasp_finger_bias,dt=e.step_dt,seed=seed)
                R=solution['rotation'];center=solution['center'];pregrasp_offset=solution['offset']
                result['grasp_solutions'][PICK]={k:v for k,v in solution.items() if k not in ('rotation','center','offset')};result['grasp_search_attempts'][PICK]=attempts
            phase='pregrasp' if use_pregrasp_phase else 'approach'
            cart(center+pregrasp_offset,rotation=R,force_phase=('transfer' if args.pregrasp_planner=='motion_gen' else 'cartesian_line' if args.pregrasp_planner=='cartesian_line' else None))
            phase='approach'
            if args.grasp_local_approach and not use_pregrasp_phase:
                axis=pregrasp_offset/np.linalg.norm(pregrasp_offset)
                cart(center+axis*.02,force_phase='cartesian_line')
            else:cart(center,rotation=R)
            if args.grasp_search=='b03-topdown':extra_close=close_with_feedback(8)
            else:
                phase='close';grip=1.
                for _ in range(40):step(q,grip)
                extra_close=contacts.summarize(list(TASK_SPEC.grasp_body_z_range),'close',last_actions=8)
            held=snapshot();result.setdefault('extra_close_contacts',[]).append(extra_close)
            if not result['extra_close_contacts'][-1]['both_fingers_near_body']:
                raise RuntimeError(f'No bilateral physical grasp for {PICK}')
            world.attachment=np.linalg.inv(kin.fk(held['q'][ids]))@transform(held['lime'][:3],held['lime'][3:7])
            phase='lift';cart(held['tcp']+np.array([0.,0.,.10 if args.grasp_search=='b03-topdown' else .12]), force_phase='lift')
            lifted=snapshot();result.setdefault('extra_lift_m',[]).append(float(lifted['lime'][2]-p[2]))
            lift_contact=contacts.summarize(list(TASK_SPEC.grasp_body_z_range),'lift',last_actions=5)
            result.setdefault('extra_lift_contacts',[]).append(lift_contact)
            if result['extra_lift_m'][-1]<.08 or not lift_contact['both_fingers_on_body']:raise RuntimeError(f'Physical lift/contact check failed for {PICK}')
            dest=snapshot()['wooden_bowl'];offset=lifted['tcp']-lifted['lime'][:3]
            desired_root=dest[:3]+np.array([(-.06 if PICK==TASK_SPEC.pick_object else .06) if len(task_picks(TASK_SPEC))==2 else 0,0,TASK_SPEC.transfer_z_m])
            phase='transfer';cart(desired_root+offset)
            if float(args.place_clear_rise_z)>0 and (not use_pregrasp_phase):
                phase='transfer';cart(snapshot()['tcp']+np.array([0.0,0.0,float(args.place_clear_rise_z)]), force_phase='transfer')
            lower_z=float(TASK_SPEC.lower_z_m if args.place_lower_z is None else args.place_lower_z)
            lower_root=dest[:3]+np.array([(-.06 if PICK==TASK_SPEC.pick_object else .06) if len(task_picks(TASK_SPEC))==2 else 0,0.0,lower_z])
            xfer_tcp=desired_root+offset;placement_state=snapshot();lower_tcp=lower_root+(placement_state['tcp']-placement_state['lime'][:3])
            phase='lower'
            if args.place_lower_mode=='skip' or abs(xfer_tcp[2]-lower_tcp[2])<1e-4:
                pass
            elif args.place_lower_mode=='cartesian_line':
                cart(lower_tcp, force_phase='cartesian_line')
            else:
                cart(lower_tcp, force_phase='transfer')
            phase='release';grip=0.;world.attachment=None
            for _ in range(35):step(q,grip)
            phase='retreat';before_retreat=len(records)
            try:cart(snapshot()['tcp']+[0,0,.14])
            except RuntimeError as exc:
                if len(records)!=before_retreat:raise
                result.setdefault('extra_optional_retreat_skipped',{})[PICK]={'reason':str(exc),'executed_actions':0}
            phase='settle_final'
            for _ in range(120):
                step(q,grip)
                if terminal.get('terms',{}).get('success',False) and all_current_native_predicates():break
            capture_placement_checkpoint(PICK)
        final=snapshot();result['final_lime']=final['lime'].tolist();result['terminal_terms']=terminal.get('terms',{})
        result['final_live_success']=bool(arr(e.termination_manager.get_term('success'))[0])
        # Re-evaluate the original support/velocity predicate at the saved final
        # physical state: a previously completed predicate sequence can latch.
        from isaaclab.managers import SceneEntityCfg
        from isaaclab_arena.tasks.predicates.spatial import object_on_destination
        native_tasks=getattr(envdef.task,'subtasks',[envdef.task])
        native={}
        for task in native_tasks:
            name=task.pick_up_object.name
            native[name]={'passed':bool(arr(object_on_destination(e,
                object_cfg=SceneEntityCfg(name),destination_cfg=SceneEntityCfg(task.destination_location.name),
                contact_sensor_cfg=task.contact_sensor_cfg,force_threshold=task.force_threshold,
                velocity_threshold=task.velocity_threshold,support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0]),
                'force_matrix_w_n':arr(scene[task.contact_sensor_cfg.name].data.force_matrix_w).tolist(),
                'final_state':final[name].tolist()}
        result['final_native_destination_checks']=native
        result['final_on_destination']=bool(set(native)==set(task_picks(TASK_SPEC)) and all(x['passed'] for x in native.values()))
        result['final_destination_contact_force_matrix_w_n']=native[TASK_SPEC.pick_object]['force_matrix_w_n']
        result['final_lime']=final[TASK_SPEC.pick_object].tolist()
        result['success']=bool(result['terminal_terms'].get('success',False) and result['final_live_success'] and result['final_on_destination'] and abs(final['q'][robot.joint_names.index('finger_joint')])<.05)
        if args.variant=='property':
            result['property_displacement_m']=float(np.linalg.norm(final['hazard_pose'][:3]-initial['hazard_pose'][:3]))
            result['success'] &= result['property_displacement_m']<.01
    except Exception as exc:
        result['error']=repr(exc);result['traceback']=traceback.format_exc();print(result['traceback'],flush=True)
    finally:
        for sig,handler in previous_signal_handlers.items():signal.signal(sig,handler)
        scene.update=original_update
        if writer:writer.close()
        if overview_writer:
            overview_writer.close()
            (out/'overview/frame_alignment.json').write_text(json.dumps({
                'scope':'Primary and supplementary RGB captured before the same actual action, without advancing physics between cameras.',
                'measurements':overview_alignment},indent=2))
            if overview_writer.frames:
                (out/'overview_camera.mp4').hardlink_to(out/'overview/overview_camera.mp4')
            result['overview_capture_alignment']=bool(len(overview_alignment)==len(records) and all(
                x['primary_frame_index']==x['overview_frame_index']==x['action_index'] and
                x['physics_step_before']==x['physics_step_after'] for x in overview_alignment))
            result['success'] &= result['overview_capture_alignment']
        result['frames']=len(records);result['physics_safety_checks']=physics_checks;result['wall_s']=time.perf_counter()-started
        result['plans']=plans;result['clearance_checks']=len(clearances)
        result['cartesian_diagnostics']=getattr(planner,'cartesian_diagnostics',[])
        result['full_world_audit_passed']=bool(physics_audits and all(x.get('valid') is True for x in physics_audits))
        (out/'full_world_audit.json').write_text(json.dumps({'passed':result['full_world_audit_passed'],
            'scope':'Every physics update: actual arm, fingers and original lime convex hull against all colliders. Only exact source counter permits native-rest collision-hull lower bound minus 0.5 mm and 50 mm xy, until >=2 mm separation and loss of source contacts permanently revoke it. Fruit/bowl support allowed only during placement. All external safety distances unchanged.',
            'minimum':min((x for x in physics_audits if x.get('valid') is not None),key=lambda x:x['slack_m'],default=None),
            'pending_samples':sum(x.get('valid') is None for x in physics_audits),
            'measurements':physics_audits},indent=2))
        if clearances:result['minimum_clearance']=min((x for x in clearances if x.get('valid') is not None),key=lambda x:x['slack_m'],default=None)
        if args.defer_added_hazard_audit:result['minimum_clearance_scope']='Live15Hz checkpoints; complete120Hz added-hazard measurements required in canonical clearances.json before acceptance'
        # Persist result/trajectory before slow MP4 decode so host timeouts still leave usable artifacts.
        (out/'result.json').write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
        (out/'clearances.json').write_text(json.dumps(clearances,indent=2))
        pick_action_ranges[PICK]['end']=len(records)
        result['pick_action_ranges']=pick_action_ranges
        (out/'result.json').write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
        (out/'contacts.json').write_text(json.dumps(contact_trackers[TASK_SPEC.pick_object].records,indent=2))
        if len(contact_trackers)>1:
            for name,tracker in contact_trackers.items():(out/('contacts_'+name+'.json')).write_text(json.dumps(tracker.records,indent=2))
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
        if overview_writer and overview_writer.frames and (out/'overview_camera.mp4').exists():
            result['overview_video_validation']=decode_video(out/'overview_camera.mp4',len(records),expected_size=tuple(overview_spec['resolution']),expected_dt=e.step_dt)
        if writer and writer.frames or (overview_writer and overview_writer.frames):
            (out/'result.json').write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
        print('COLLECT_RESULT',json.dumps({k:v for k,v in result.items() if k not in ('plans','traceback','close_contacts','lift_contacts')}),flush=True)
        env.close()
    if not result['success']:raise SystemExit(2)
