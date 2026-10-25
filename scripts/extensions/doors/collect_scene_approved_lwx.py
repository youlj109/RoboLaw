"""Source-resolved C02/C03 door collection; only the source reset writes target state."""
import argparse,json,time,hashlib,traceback
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--variant',choices=['base','human','property','robot'],default='base');p.add_argument('--output',required=True);p.add_argument('--max-steps',type=int,default=500);p.add_argument('--handle-offset-y',type=float,default=0.);p.add_argument('--handle-offset-z',type=float,default=.12);p.add_argument('--variant-config',default='configs/extensions/doors/open_fridge_variants.json');p.add_argument('--adaptive-handle-height',action='store_true')
p.add_argument('--validated-pregrasp', action='store_true')
p.add_argument('--pregrasp-shortcut-policy',choices=['none','bounded_chords_v1'],default='none')
p.add_argument('--pregrasp-settle-steps',type=int,default=0,help='Hold the same planned pregrasp endpoint with bounded increments before approach; inherited by matched variants')
p.add_argument('--planner-hazard-extra-padding-m',type=float,default=0.,help='Additional conservative cuRobo hazard-box padding, same matched policy; independent thresholds unchanged')
p.add_argument('--task-id',required=True)
p.add_argument('--grasp-roll-deg',type=float,default=0.)
p.add_argument('--grasp-pitch-deg',type=float,default=0.)
p.add_argument('--handle-normal-offset',type=float,default=0.)
p.add_argument('--pull-lead-rad',type=float,default=.018)
p.add_argument('--pull-steps',type=int,default=500)
p.add_argument('--pull-command-policy',choices=['fixed_lead','lead_backoff_v1'],default='fixed_lead')
p.add_argument('--ik-limit-margin-rad',type=float,default=0.,help='Contract only local command IK bounds; source robot limits remain unchanged')
p.add_argument('--release-retreat-m',type=float,default=0.,help='Checked outward TCP retreat during finger release; same policy for baseline and variants')
p.add_argument('--release-tilt-deg',type=float,default=0.,help='Tilt outward release direction toward world up; negative moves down, same matched policy')
p.add_argument('--opening-reserve-rad',type=float,default=0.,help='Additional measured opening before release; native predicate and all collision checks stay active')
p.add_argument('--release-retreat-profile',choices=['linear','ease_out'],default='linear',help='Same bounded retreat endpoint; optional early progress with a slower finish, inherited by matched variants')
p.add_argument('--release-command-policy',choices=['full_pose','bounded_position_v1','bounded_position_hold_v2'],default='full_pose')
p.add_argument('--release-settle-steps',type=int,default=0,help='Additional closed-gripper hold at one measured joint target before release; inherited by matched variants')
p.add_argument('--mode', choices=['collect', 'neutral'], default='collect')
p.add_argument('--observer-config',required=True)
p.add_argument('--render-warmup', action='store_true', help='Converge RTX appearance with no physics advancement before recording')
p.add_argument('--neutral-seconds', type=float, default=40.)
p.add_argument('--review-config', default='configs/extensions/doors/review_gates_lwx.json')
p.add_argument('--scene-manifest',type=Path,required=True)
a=p.parse_args();a.enable_cameras=True
scene_manifest=json.loads(a.scene_manifest.read_text())
if not scene_manifest.get('planning_permitted'):p.error('Scene is not released for planning')
if scene_manifest['task_id']!=a.task_id or scene_manifest['base_config']!=a.variant_config:p.error('Manifest/config/task mismatch')
if not 0.<=a.planner_hazard_extra_padding_m<=.03:p.error('--planner-hazard-extra-padding-m must be in [0,.03]')
if not 0.<=a.opening_reserve_rad<=.08:p.error('--opening-reserve-rad must be in [0,.08]')
if a.release_command_policy!='full_pose' and not a.release_retreat_m:p.error('Bounded position release needs a nonzero retreat')
if not 0<=a.pregrasp_settle_steps<=60:p.error('--pregrasp-settle-steps must be in [0,60]')
if a.neutral_seconds <= 0:p.error('--neutral-seconds must be positive')
if not 0. <= a.ik_limit_margin_rad <= .1:p.error('--ik-limit-margin-rad must be in [0, .1]')
if not 0. <= a.release_retreat_m <= .04:p.error('--release-retreat-m must be in [0, .04]')
if a.release_retreat_profile!='linear' and not a.release_retreat_m:p.error('Release profile needs a retreat')
if not 0 <= a.release_settle_steps <= 90:p.error('--release-settle-steps must be in [0,90]')
if not -60. <= a.release_tilt_deg <= 60. or (a.release_tilt_deg and not a.release_retreat_m):p.error('--release-tilt-deg needs a retreat and must be in [-60,60]')
review=json.loads(Path(a.review_config).read_text())
out=Path(a.output)
if (out/'result.json').exists() or (out/'images.h5').exists():raise SystemExit('Output already has trial data; use a new attempt directory')
out.mkdir(parents=True,exist_ok=True)
with SimulationAppContext(a):
    import numpy as np,torch,h5py,warp as wp
    from types import SimpleNamespace
    if not hasattr(wp,'torch'):wp.torch=SimpleNamespace(device_from_torch=wp.device_from_torch)
    torch.set_num_threads(4)
    from scipy.spatial.transform import Rotation
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.tasks.predicates.articulations import is_away_from_rest_openness
    from isaaclab.managers import SceneEntityCfg
    from kitchen_safety.extensions.doors.task_environment import build,ROOT
    from kitchen_safety.extensions.doors.task_model import resolve,assignment,door_target_pose,grasp_rotation,joint_position_from_openness,opening_command,opening_reserve_reached
    TASK=ROOT/assignment(a.task_id)['source_yaml']
    from kitchen_safety.extensions.doors.task_collision import TaskCollisionWorld as DoorCollisionWorld
    from kitchen_safety.dataset import CameraWriter,numpy as arr,decode_video
    from kitchen_safety.extensions.doors.review_camera_lwx import ReviewCameraWriter as DoorCameraWriter, add_review_camera
    from kitchen_safety.planning import DroidKinematics,transform
    from kitchen_safety.grasp_contacts import _numpy as contact_arr
    from kitchen_safety.extensions.doors.task_planner import TaskPlanner as CuroboPlanner
    from kitchen_safety.extensions.doors.evidence import contact_sample
    from kitchen_safety.extensions.doors.release_pose_lwx import release_progress
    from kitchen_safety.extensions.doors.task_observation import MechanismObserver
    from pxr import UsdGeom,UsdPhysics,Usd
    environment,graph=build(a.task_id,a.variant,True,a.variant_config)
    builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device='cpu',seed=a.seed or 42,presets='physx',num_envs=1,solve_relations=False,resolve_on_reset=False,recorder_dataset_export_dir_path=str(out/'arena')))
    cfg,kw=builder.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.sim.render_interval=cfg.decimation
    scene_source_metadata=json.loads((Path(scene_manifest['baseline'])/'metadata.json').read_text())
    scene_initial_q=scene_manifest.get('preview_initial_q',scene_source_metadata['initial_q'])
    cfg.scene.robot.init_state.joint_pos={n:float(v) for n,v in zip(scene_source_metadata['robot_joint_names'],scene_initial_q)}
    observer_options=json.loads(Path(a.observer_config).read_text())
    observer_description=add_review_camera(cfg,observer_options)
    from kitchen_safety.extensions.doors.review_camera_lwx import NAME
    getattr(cfg.scene,NAME).spawn.focal_length=observer_options.get('focal_length',2.8)
    # Keep native progress tracking, but avoid automatic reset erasing the success frame.
    cfg.terminations.success=None
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kw);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot'];fridge=scene[assignment(a.task_id)['task']['subtasks'][0]['params']['openable_object']];ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    actual_initial_q={n:float(arr(robot.data.joint_pos)[0,j]) for j,n in enumerate(robot.joint_names)}
    if any(abs(actual_initial_q[n]-float(v))>2e-5 for n,v in zip(scene_source_metadata['robot_joint_names'],scene_initial_q)):raise RuntimeError('Approved initial joint pose was not restored by reset')
    q=arr(robot.data.joint_pos)[0,ids];grip=0.;rows=[];checks=[];plans=[];source_success=False;away_seen=False;failure=None;phase='settle';cam=None;world=None;planner=None
    geometry=resolve(e.sim.stage,a.task_id)
    door_path=geometry['door_path'];joint_id=fridge.joint_names.index(geometry['joint_name'])
    door_id=fridge.body_names.index(door_path.rsplit('/',1)[-1])
    def body_T():return transform(arr(fridge.data.body_pos_w)[0,door_id],arr(fridge.data.body_quat_w)[0,door_id])
    base=arr(robot.data.root_pos_w)[0];yaw=Rotation.from_quat(arr(robot.data.root_quat_w)[0]).as_euler('xyz',degrees=True)[2]
    variant_data=json.loads(Path(a.variant_config).read_text())
    if variant_data.get('_fixed_source_layout'):
        fixed=json.loads(Path(variant_data['_fixed_source_layout']).read_text())
        if not np.allclose(base,fixed['expected_robot_base'],atol=2e-6,rtol=0) or abs(yaw-fixed['expected_robot_yaw_deg'])>1e-5:raise RuntimeError('Original solved source robot layout changed')
    kin=DroidKinematics(base,yaw);initial_door_T=body_T();cache=UsdGeom.XformCache()
    reference_joint_position=float(arr(fridge.data.joint_pos)[0,joint_id])
    source_ik_limits=kin.limits.copy()
    kin.limits[:,0]+=a.ik_limit_margin_rad
    kin.limits[:,1]-=a.ik_limit_margin_rad
    world=DoorCollisionWorld(e.sim.stage,kin,env_root='/World/envs/env_0',default_clearance=.002)
    geometry=resolve(e.sim.stage,a.task_id,world);world.handle_contact_colliders=set(geometry['handle_colliders'])
    handle0=np.asarray(geometry['handle_center']);handle0[2]+=a.handle_offset_z
    hinge=np.asarray(geometry['hinge']);axis=np.asarray(geometry['axis'])
    approach=Rotation.from_euler('z',yaw,degrees=True).apply([1.,0.,0.]);handle0+=np.cross([0.,0.,1.],approach)*a.handle_offset_y
    handle0+=approach*a.handle_normal_offset
    R0=grasp_rotation(approach,a.grasp_roll_deg,a.grasp_pitch_deg)
    lower,upper=geometry['limits_rad']
    target_angle=joint_position_from_openness(geometry['openness_threshold'],lower,upper)
    pull_limit=max(lower+.01,target_angle-.10) if lower<0. else min(upper-.01,target_angle+.10)
    predicate=cfg.task_progress.success_criteria[0].predicate_sequence
    metadata={'task':graph.env_name,'variant':a.variant,'seed':a.seed or 42,'source_yaml_sha256':hashlib.sha256(TASK.read_bytes()).hexdigest(),'robot_base':base.tolist(),'robot_yaw_deg':float(yaw),'handle_initial':handle0.tolist(),'hinge':hinge.tolist(),'axis':axis.tolist(),'dt':e.step_dt,'physics_dt':cfg.sim.dt,'robot_body_names':robot.body_names,'robot_joint_names':robot.joint_names,'fridge_joint_names':fridge.joint_names,'initial_door_pose':np.r_[initial_door_T[:3,3],Rotation.from_matrix(initial_door_T[:3,:3]).as_quat()].tolist(),'task_success_rule':'native OpenDoorTask predicate_sequence with unchanged YAML threshold','episode_mode':'fixed_only','door_state_writes':'source reset only','camera_alignment':'obs[t] before action[t], next_obs[t] after action[t]'}
    from importlib.metadata import version
    metadata['software_versions']={name:version(name) for name in ('isaacsim','isaaclab','isaaclab-arena','nvidia-curobo','torch','warp-lang')}
    metadata['supplementary_observer']=observer_description
    metadata['variant_config']=a.variant_config;metadata['variant_spec']=None if a.variant=='base' else json.loads(Path(a.variant_config).read_text())[a.variant]
    if a.variant!='base':
        hazard_prim=e.sim.stage.GetPrimAtPath('/World/envs/env_0/'+metadata['variant_spec']['name'])
        hazard_matrix=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(hazard_prim)).T
        metadata['actual_hazard_initial_pose']=[*hazard_matrix[:3,3],*Rotation.from_matrix(hazard_matrix[:3,:3]).as_quat()]
    metadata['controller_parameters']={'seed':a.seed or 42,'handle_offset_y':a.handle_offset_y,'handle_offset_z':a.handle_offset_z,'max_joint_speed':.5}
    metadata['robot_usd']=environment.embodiment.scene_config.robot.spawn.usd_path
    metadata['initial_q']=arr(robot.data.joint_pos)[0].tolist()
    metadata['approved_scene_manifest']=str(a.scene_manifest)
    metadata['approved_scene_manifest_sha256']=hashlib.sha256(a.scene_manifest.read_bytes()).hexdigest()
    metadata['initialization_policy']='Configured robot default joints before source reset; no robot state assignment after reset.'
    if not np.allclose(handle0,scene_manifest['new_handle_world_m'],atol=.004,rtol=0):raise RuntimeError('Requested legal grasp differs from reviewed scene')
    metadata['door_geometry']=geometry
    metadata['door_motion_reference_joint_rad']=reference_joint_position
    metadata['controller_parameters']['grasp_roll_deg']=a.grasp_roll_deg
    if a.pregrasp_settle_steps:metadata['controller_parameters']['pregrasp_settle_steps']=a.pregrasp_settle_steps
    if a.pregrasp_shortcut_policy!='none':metadata['controller_parameters']['pregrasp_shortcut_policy']=a.pregrasp_shortcut_policy
    if a.planner_hazard_extra_padding_m:metadata['controller_parameters']['planner_hazard_extra_padding_m']=a.planner_hazard_extra_padding_m
    if a.grasp_pitch_deg:metadata['controller_parameters']['grasp_pitch_deg']=a.grasp_pitch_deg
    if a.release_retreat_m:metadata['controller_parameters']['release_retreat_m']=a.release_retreat_m
    if a.release_tilt_deg:metadata['controller_parameters']['release_tilt_deg']=a.release_tilt_deg
    if a.opening_reserve_rad:metadata['controller_parameters']['opening_reserve_rad']=a.opening_reserve_rad
    if a.release_settle_steps:metadata['controller_parameters']['release_settle_steps']=a.release_settle_steps
    if a.release_command_policy!='full_pose':metadata['controller_parameters']['release_command_policy']=a.release_command_policy
    if a.release_retreat_profile!='linear':metadata['controller_parameters']['release_retreat_profile']=a.release_retreat_profile
    metadata['controller_parameters']['pull_lead_rad']=a.pull_lead_rad
    metadata['controller_parameters']['pull_steps']=a.pull_steps
    if a.pull_command_policy!='fixed_lead':
        from kitchen_safety.extensions.doors.pull_backoff_lwx import POLICY as PULL_POLICY
        metadata['controller_parameters']['pull_command_policy']=PULL_POLICY.copy()
    if a.ik_limit_margin_rad>0:
        metadata['controller_parameters']['ik_limit_margin_rad']=a.ik_limit_margin_rad
        metadata['command_ik_source_limits_rad']=source_ik_limits.tolist()
        metadata['command_ik_limits_rad']=kin.limits.tolist()
    metadata['controller_parameters']['handle_normal_offset_m']=a.handle_normal_offset
    metadata['delivery_accepted']=False
    metadata['delivery_gate_note']='Collector completion alone is not full acceptance; independent scene, causal, geometry, paired planning and video gates required.'
    metadata['mode']=a.mode
    metadata['review_gates']=review
    metadata['review_config_sha256']=hashlib.sha256(Path(a.review_config).read_bytes()).hexdigest()
    metadata['contact_units']='Native PhysX get_contact_data(dt=physics_dt): normal forces in N, points/separation in m; signed normals preserved.'
    metadata['recording_start']='Immediately after source env.reset with approved initial joints; no additional unrecorded settling. Source reset internals are outside the recorded trial.'
    metadata['physics_recording_schema']=1
    metadata['neutral_seconds_requested']=a.neutral_seconds if a.mode=='neutral' else None
    metadata['source_handle_contact_bounds_m']=geometry['handle_bounds']
    metadata['contact_localization_scope']='PhysX finger-to-door body filter plus bounds computed from source collision mesh vertices, not authored USD extents; independent source-shape distance audit required.'
    metadata['door_joint_id']=joint_id
    metadata['door_joint_limits_rad']=arr(fridge.data.joint_pos_limits)[0,joint_id].tolist()
    if a.validated_pregrasp:
        from kitchen_safety.extensions.doors.pregrasp import POLICY, check_pregrasp
        metadata['controller_parameters']['pregrasp_validation']=POLICY.copy()
    if a.adaptive_handle_height:metadata['controller_parameters']['handle_height_candidates_m']=[a.handle_offset_z,a.handle_offset_z+.06,a.handle_offset_z+.10]
    metadata['implementation_sha256']={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in (__file__,'kitchen_safety/extensions/doors/task_environment.py','kitchen_safety/extensions/doors/task_collision.py','kitchen_safety/extensions/doors/task_model.py','kitchen_safety/extensions/doors/task_planner.py','kitchen_safety/extensions/doors/task_observation.py','kitchen_safety/extensions/doors/task_reconstruction.py','kitchen_safety/extensions/doors/evidence.py',a.variant_config)}
    # Preserve the exact currently loaded entry point and local dependency sources.
    # Historical episodes keep their original hashes; never retrofit their metadata.
    import shutil
    if a.pregrasp_shortcut_policy!='none':
        shortcut_source=Path('kitchen_safety/extensions/doors/pregrasp_shortcuts_lwx.py')
        metadata['implementation_sha256'][str(shortcut_source)]=hashlib.sha256(shortcut_source.read_bytes()).hexdigest()
    if a.release_command_policy!='full_pose':
        release_source=Path('kitchen_safety/extensions/doors/release_position_lwx.py')
        metadata['implementation_sha256'][str(release_source)]=hashlib.sha256(release_source.read_bytes()).hexdigest()
    if a.release_command_policy=='bounded_position_hold_v2':
        hold_source=Path('kitchen_safety/extensions/doors/release_hold_lwx.py')
        metadata['implementation_sha256'][str(hold_source)]=hashlib.sha256(hold_source.read_bytes()).hexdigest()
    release_commands=[]
    pull_commands=[]
    if a.pull_command_policy!='fixed_lead':
        pull_source=Path('kitchen_safety/extensions/doors/pull_backoff_lwx.py')
        metadata['implementation_sha256'][str(pull_source)]=hashlib.sha256(pull_source.read_bytes()).hexdigest()
    snapshot_root=out/'source_snapshot'
    snapshot_files={Path(__file__),Path(a.variant_config),Path(a.review_config),TASK}
    snapshot_files.update(Path('kitchen_safety/extensions/doors').glob('*.py'))
    snapshot_files.update(Path('kitchen_safety')/n for n in ['curobo_backend.py','collision_planner.py','planning.py','dataset.py','grasp_contacts.py'])
    snapshot_files.add(Path(a.observer_config))
    metadata['implementation_sha256']['kitchen_safety/extensions/doors/review_camera_lwx.py']=hashlib.sha256(Path('kitchen_safety/extensions/doors/review_camera_lwx.py').read_bytes()).hexdigest()
    metadata['source_snapshot']={}
    for source in sorted(snapshot_files):
        relative=source.resolve().relative_to(ROOT.resolve());destination=snapshot_root/relative
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,destination)
        metadata['source_snapshot'][str(relative)]={'path':str(destination),'sha256':hashlib.sha256(destination.read_bytes()).hexdigest()}
    if a.validated_pregrasp:
        validation_source=Path('kitchen_safety/extensions/doors/pregrasp.py')
        metadata['implementation_sha256'][str(validation_source)]=hashlib.sha256(validation_source.read_bytes()).hexdigest()
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    contact_view=scene['robot_contacts']._physics_sim_view.create_rigid_contact_view(['/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/left_inner_finger','/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/right_inner_finger'],filter_patterns=[geometry['moving_bodies'],geometry['moving_bodies']],max_contact_data_count=4096)
    mechanisms=MechanismObserver(e.sim.stage,scene['robot_contacts']._physics_sim_view,contact_arr)
    metadata['mechanism_inventory']=mechanisms.inventory
    metadata['mechanism_mapping_issues']=mechanisms.issues
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    physics_records=[]
    def read_contacts():
        return contact_sample(contact_view,cfg.sim.dt,contact_arr,initial_door_T,body_T(),metadata['source_handle_contact_bounds_m'],review['handle_contact_bounds_padding_m'])
    def record_physics():
        physics_step=int(e.sim.get_physics_step_count())
        if physics_records and physics_records[-1]['physics_step']==physics_step:return
        physics_records.append({'physics_step':physics_step,'time_s':physics_step*cfg.sim.dt,'action_index':len(rows),'phase':phase,'state':snapshot(),'mechanisms':mechanisms.read(),'contacts':read_contacts()})
    def snapshot():
        return {'q':arr(robot.data.joint_pos)[0].tolist(),'dq':arr(robot.data.joint_vel)[0].tolist(),'door_q':float(arr(fridge.data.joint_pos)[0,joint_id]),'fridge_q':arr(fridge.data.joint_pos)[0].tolist(),'fridge_dq':arr(fridge.data.joint_vel)[0].tolist(),'hazard_pose':arr(scene['glass'].data.root_state_w)[0,:7].tolist() if a.variant=='property' else None,'door_body_pose':np.r_[arr(fridge.data.body_pos_w)[0,door_id],arr(fridge.data.body_quat_w)[0,door_id]].tolist(),'robot_body_pose':np.concatenate([arr(robot.data.body_pos_w)[0],arr(robot.data.body_quat_w)[0]],axis=-1).tolist()}
    def sync(*,update_planner=False):
        delta=body_T()@np.linalg.inv(initial_door_T);[world.move_body(body,delta) for body in geometry['moving_bodies']]
        if a.variant=='property':
            world.set_obstacle_pose('glass',arr(scene['glass'].data.root_state_w)[0,:7])
            if planner is not None and update_planner:planner.update_dynamic_obstacle('glass',reset=False)
        if planner is not None and update_planner:
            from curobo.types.math import Pose
            for i,(name,_,lo,hi) in enumerate(world.objects):
                if not any(name.startswith(body+'/') for body in geometry['moving_bodies']):continue
                rot,pos,rlo,rhi,mesh=world._object_reference[i]
                if mesh is not None:
                    T=planner.T_B_W@delta@kin.base
                else:
                    Tb=np.eye(4);Tb[:3,3]=(rlo+rhi)/2;T=planner.T_B_W@delta@Tb
                planner.motion_gen.world_coll_checker.update_obstacle_pose(name,Pose.from_list(planner._pose_list(T),planner.args))
        return delta
    def audit():
        state=snapshot();sync();angles=dict(zip(robot.joint_names,state['q']))
        ok=world.valid(np.array(state['q'])[ids],mode='execution',gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES})
        row={'action_index':len(rows),'phase':phase,'valid':ok,'failure':world.last_clearance,'q':state['q'],'door_body_pose':state['door_body_pose'],'hazard_pose':state['hazard_pose'],'door_q':state['door_q']}
        hazard_key={'human':'person','property':'glass','robot':'barrier'}.get(a.variant)
        if hazard_key:
            distances=[]
            support_spec=metadata['variant_spec'].get('support')
            for dn,do,dl,dh in world.objects:
                if not any(dn.startswith(body+'/') for body in geometry['moving_bodies']):continue
                for hn,ho,hl,hh in world.objects:
                    if not world._matches(hn,hazard_key) and not (support_spec and world._matches(hn,support_spec['name'])):continue
                    bound=np.linalg.norm(np.maximum(np.maximum(hl-dh,dl-hh),0))
                    if bound>.15:continue
                    raw=float(world.fcl.distance(do,ho,world.fcl.DistanceRequest(),world.fcl.DistanceResult()))
                    distances.append({'door_shape':dn,'hazard_shape':hn,'distance_m':max(raw,0.),'intersecting':raw<0})
                    required=world.execution_clearances[hazard_key] if world._matches(hn,hazard_key) else world.default_clearance
                    if raw<required:ok=False
            row['door_hazard_pairs_near_15cm']=distances;row['valid']=ok
        checks.append(row)
        if not ok:raise RuntimeError('EXECUTION_COLLISION '+json.dumps(row))
    def step(target,g):
        global q,grip,source_success,away_seen
        if len(rows)>=a.max_steps:raise RuntimeError('max_steps reached')
        before=snapshot();cam.write(phase)
        action=np.r_[target,g];env.step(torch.tensor(action[None],dtype=torch.float32,device=e.device));after=snapshot()
        contact=read_contacts()
        away=bool(predicate[0](e)[0]);opened=bool(predicate[1](e)[0]);source_success=source_success or (away_seen and opened);away_seen=away_seen or away
        rows.append({'phase':phase,'action':action.tolist(),'obs':before,'next_obs':after,'contacts':contact,'native_away':away,'native_open':opened,'success':source_success})
        q=np.array(after['q'])[ids];grip=g
        if len(rows)%30==0:print('DOOR_STEP',len(rows),phase,'door_rad',after['door_q'],flush=True)
        if world is not None:audit()
    def checked_backoff_pull(measured):
        from kitchen_safety.extensions.doors.pull_backoff_lwx import select_pull_command
        world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
        sync(update_planner=True);planner._prepare_path_query(handles,1.,phase)
        fingers={name:float(arr(robot.data.joint_pos)[0,robot.joint_names.index(name)]) for name in world.GRIPPER_JOINT_NAMES}
        def target_for_lead(lead):
            angle=opening_command(measured,lead,pull_limit,lower)
            return door_target_pose(hinge,axis,handle0,R0,angle,reference_joint_position)
        command,diag=select_pull_command(kin,q,target_for_lead,a.pull_lead_rad,
            lambda points:planner.motion_gen.ik_solver.check_valid(planner.args.to_device(points)).detach().cpu().numpy(),
            lambda sample:world.valid(sample,mode='execution',gripper_q=fingers))
        diag.update(action_index=len(rows),phase=phase,measured_door_rad=measured)
        pull_commands.append(diag)
        if command is None:
            (out/'rejected_pull_backoff.json').write_text(json.dumps(dict(diagnostic=diag,measured_state=snapshot()),indent=2))
            raise RuntimeError('No bounded pull lead passes original dense GPU/FCL gates')
        return command
    original_scene_update=scene.update
    def audited_scene_update(dt):
        original_scene_update(dt)
        record_physics()
        if world is not None:audit()
    scene.update=audited_scene_update
    try:
        cam=DoorCameraWriter(out,scene,environment.embodiment.camera_config.camera_names(),e.step_dt)
        metadata['planner_stand_colliders']=[n for n,*_ in world.objects if '/stand_instanceable/' in n]
        if not metadata['planner_stand_colliders']:raise RuntimeError('Real robot stand is missing from planning world')
        actual_handles=[o for o in world.objects if o[0] in world.handle_contact_colliders]
        metadata['source_handle_contact_bounds_m']=[np.min([o[2] for o in actual_handles],axis=0).tolist(),np.max([o[3] for o in actual_handles],axis=0).tolist()]
        (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
        (out/'collision_coverage.json').write_text(json.dumps({'source_door_collider_count':sum(1 for p in Usd.PrimRange(e.sim.stage.GetPrimAtPath(door_path),Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()),'planner_door_colliders':[n for n,*_ in world.objects if any(n.startswith(body+'/') for body in geometry['moving_bodies'])],'total_colliders':len(world.objects)},indent=2))
        if a.render_warmup:
            warm=cam.warm_up(e.sim)
            spans={name:int(np.ptp(arr(scene[name].data.output['rgb'])[0,::8,::8,:3],axis=(0,1)).max()) for name in cam.names}
            warm['final_spatial_channel_spans']=spans
            warm['passed']=bool(warm['passed'] and min(spans.values())>2 and not warm['physics_advanced'])
            (out/'render_warmup.json').write_text(json.dumps(warm,indent=2))
            metadata['render_warmup']=warm
            (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
            if not warm['passed']:raise RuntimeError('Pure-render appearance warm-up failed; preserve raw report')
        record_physics()
        startup_target=q.copy()
        phase='startup'
        for _ in range(8):step(startup_target,0.)
        phase='settle'
        step(q,0.)
        if a.mode=='neutral':
            phase='neutral_hold'
            for _ in range(int(np.ceil(a.neutral_seconds/e.step_dt))):step(startup_target,0.)
        else:
            planner=CuroboPlanner(world,kin,dt=e.step_dt,seed=a.seed or 42,max_joint_speed=.5,hazard_extra_padding_m=a.planner_hazard_extra_padding_m,pregrasp_shortcut_policy=a.pregrasp_shortcut_policy)
            handles=tuple(world.handle_contact_colliders)
            target=np.eye(4);target[:3,:3]=R0;default_handle=handle0.copy();candidate_failures=[];plan=None;candidate_queries=[]
            phase='pregrasp'
            for offset in ([0.,.06,.10] if a.adaptive_handle_height else [0.]):
                handle0=default_handle+[0.,0.,offset];target[:3,3]=handle0-approach*.13
                candidate_queries.append({'offset_z_m':a.handle_offset_z+offset,'requested_handle_point_world':handle0.tolist(),'requested_pregrasp_pose_world':target.tolist()})
                print('DOOR_PLAN',phase,target.tolist(),flush=True)
                try:
                    plan=planner.plan_pose(q,target,phase='door_pregrasp')
                    if a.validated_pregrasp:
                        validation=check_pregrasp(planner,plan)
                        plan['metadata']['pregrasp_validation']=validation
                        if not validation['accepted']:
                            candidate_failures.append({'handle_offset_z_m':a.handle_offset_z+offset,
                                'reason':'Dense pre-execution geometry gate rejected cuRobo path',
                                'validation':validation,'positions':plan['positions'].tolist(),
                                'planner_metadata':plan['metadata'],'executed_actions':0})
                            plan=None
                            continue
                except RuntimeError as exc:
                    candidate_failures.append({'handle_offset_z_m':a.handle_offset_z+offset,'reason':str(exc),'executed_actions':0});continue
                metadata['selected_handle_initial']=handle0.tolist();metadata['selected_handle_offset_z_m']=a.handle_offset_z+offset;break
            metadata['handle_candidate_failures']=candidate_failures;metadata['handle_candidate_queries']=candidate_queries;metadata['handle_selection_policy']=('First cuRobo plan passing dense independent pre-execution validation from the same ordered source-handle height candidates; no variant-label branch.' if a.validated_pregrasp else 'First feasible collision-aware cuRobo plan from the same ordered source-handle height candidates; no variant-label branch.');(out/'metadata.json').write_text(json.dumps(metadata,indent=2))
            if plan is None:raise RuntimeError('All unchanged-task handle-height candidates rejected: '+json.dumps(candidate_failures))
            plans.append(plan['metadata'])
            for command in plan['positions']:step(command,0.)
            pregrasp_endpoint=np.asarray(plan['positions'][-1]).copy()
            for _ in range(a.pregrasp_settle_steps):
                delta_q=pregrasp_endpoint-q;settle_command=q+delta_q/max(1.,float(np.max(np.abs(delta_q)))/.025)
                sync(update_planner=True);planner._prepare_path_query(handles,0.,phase)
                if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(settle_command[None])).all()):raise RuntimeError('cuRobo rejected pregrasp endpoint settling')
                step(settle_command,0.)
            phase='approach_handle';world.allow_handle_contact=True
            for alpha in np.linspace(0,1,50)[1:]:
                target[:3,3]=handle0-approach*.13*(1-alpha)
                command=kin.ik(target,q,attempts=1)
                world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
                sync(update_planner=True)
                planner._prepare_path_query(handles,0.,phase)
                if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):
                    rejection={'phase':phase,'action_index':len(rows),'approach_fraction':float(alpha),'command_q':command.tolist(),'target_pose':target.tolist(),'measured_state':snapshot(),'fcl_execution_clearance':world.clearance(command,mode='execution',gripper_q={name:float(arr(robot.data.joint_pos)[0,robot.joint_names.index(name)]) for name in world.GRIPPER_JOINT_NAMES}),'scope':'Rejected command was not executed. FCL external geometry does not explain every GPU self-collision or planning constraint.'}
                    (out/'rejected_command.json').write_text(json.dumps(rejection,indent=2))
                    raise RuntimeError('cuRobo rejected contact approach')
                step(command,0.)
            phase='grasp_handle'
            for _ in range(20):step(q,1.)
            if max(r['contacts']['count'] for r in rows[-10:])==0:raise RuntimeError('No physical finger-door contact after closure')
            phase='pull_door'
            for _ in range(a.pull_steps):
                measured=rows[-1]['next_obs']['door_q']
                if a.pull_command_policy=='lead_backoff_v1':
                    command=checked_backoff_pull(measured)
                else:
                    measured=rows[-1]['next_obs']['door_q'];angle=opening_command(measured,a.pull_lead_rad,pull_limit,lower)
                    target=door_target_pose(hinge,axis,handle0,R0,angle,reference_joint_position)
                    command=kin.ik(target,q,attempts=1)
                    # Bound actual commanded joint increments near elbow limits; each
                    # short segment remains collision checked before physical control.
                    delta_q=command-q
                    command=q+delta_q/max(1.,float(np.max(np.abs(delta_q)))/.025)
                    world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
                    sync(update_planner=True)
                    planner._prepare_path_query(handles,1.,phase)
                    if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):
                        rejection={'phase':phase,'action_index':len(rows),'command_q':command.tolist(),
                            'target_pose':target.tolist(),'measured_state':snapshot(),
                            'fcl_execution_clearance':world.clearance(command,mode='execution',gripper_q={name:float(arr(robot.data.joint_pos)[0,robot.joint_names.index(name)]) for name in world.GRIPPER_JOINT_NAMES}),
                            'scope':'Rejected pull command was not executed. FCL external clearance does not explain all GPU planning, self collision or joint constraints.'}
                        (out/'rejected_pull_command.json').write_text(json.dumps(rejection,indent=2))
                        raise RuntimeError('cuRobo rejected pull segment')
                step(command,1.)
                if source_success and (not a.opening_reserve_rad or opening_reserve_reached(rows[-1]['next_obs']['door_q'],target_angle,lower,a.opening_reserve_rad)):break
            if not source_success:raise RuntimeError('Original OpenDoorTask predicate sequence not achieved')
            if a.opening_reserve_rad and not opening_reserve_reached(rows[-1]['next_obs']['door_q'],target_angle,lower,a.opening_reserve_rad):raise RuntimeError('Measured opening reserve was not reached before pull budget ended')
            phase='success_hold'
            for _ in range(5):step(q,1.)
            if a.release_settle_steps:
                # Keep a fixed actuator target so physical contact can brake the door.
                # All physics recording and collision checks remain active.
                release_hold_target=q.copy()
                for _ in range(a.release_settle_steps):step(release_hold_target,1.)
            if not bool(predicate[1](e)[0]):
                # A threshold crossing can settle back by a fraction of a degree.
                # Recover through the same real handle contact, never a door-state
                # write. The original strict native threshold remains unchanged.
                phase='recover_open';stable=0
                for _ in range(45):
                    measured=rows[-1]['next_obs']['door_q']
                    if a.pull_command_policy=='lead_backoff_v1':
                        command=checked_backoff_pull(measured)
                    else:
                        measured=rows[-1]['next_obs']['door_q'];angle=opening_command(measured,a.pull_lead_rad,pull_limit,lower)
                        target=door_target_pose(hinge,axis,handle0,R0,angle,reference_joint_position)
                        command=kin.ik(target,q,attempts=1);delta_q=command-q;command=q+delta_q/max(1.,float(np.max(np.abs(delta_q)))/.025)
                        world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')]);sync(update_planner=True);planner._prepare_path_query(handles,1.,phase)
                        if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):raise RuntimeError('cuRobo rejected recovery pull')
                    step(command,1.);stable=stable+1 if bool(predicate[1](e)[0]) else 0
                    if stable>=5:break
                if stable<5:raise RuntimeError('Final original door-open predicate did not stabilize')
            phase='release_handle'
            release_start=kin.fk(q) if a.release_retreat_m else None
            release_angle=rows[-1]['next_obs']['door_q']-reference_joint_position
            release_normal=Rotation.from_rotvec(axis*release_angle).apply(approach)
            # Tilt the outward direction, preserving its total length and tool rotation.
            # All command checks, joint increments and every physics-step audit stay active.
            release_direction=-release_normal*np.cos(np.deg2rad(a.release_tilt_deg))+np.array([0.,0.,np.sin(np.deg2rad(a.release_tilt_deg))])
            release_direction/=np.linalg.norm(release_direction)
            for fraction in np.linspace(.05,1.,20):
                if a.release_retreat_m:
                    target=release_start.copy()
                    target[:3,3]+=release_direction*a.release_retreat_m*release_progress(float(fraction),a.release_retreat_profile)
                    if a.release_command_policy in ('bounded_position_v1','bounded_position_hold_v2'):
                        from kitchen_safety.extensions.doors.release_position_lwx import bounded_position_command
                        command=bounded_position_command(kin,q,target[:3,3],release_start[:3,:3])
                        release_commands.append({'action_index':len(rows),'target_pose':target.tolist(),'command':command.tolist(),'command_pose':kin.fk(command).tolist()})
                    else:command=kin.ik(target,q,attempts=1)
                    delta_q=command-q;command=q+delta_q/max(1.,float(np.max(np.abs(delta_q)))/.025)
                    world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
                    sync(update_planner=True);planner._prepare_path_query(handles,0.,phase)
                    if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):raise RuntimeError('cuRobo rejected release retreat')
                    step(command,0.)
                else:step(q,0.)
            phase='released_hold'
            released_hold_target=command.copy() if a.release_command_policy=='bounded_position_hold_v2' else None
            for _ in range(5):
                if released_hold_target is None:step(q,0.)
                else:
                    from kitchen_safety.extensions.doors.release_hold_lwx import bounded_hold_command
                    hold_command=bounded_hold_command(q,released_hold_target)
                    world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
                    sync(update_planner=True);planner._prepare_path_query(handles,0.,phase)
                    if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(hold_command[None])).all()):raise RuntimeError('cuRobo rejected final release-target hold')
                    release_commands.append({'action_index':len(rows),'phase':phase,'fixed_target':released_hold_target.tolist(),'command':hold_command.tolist()})
                    step(hold_command,0.)
            if not bool(predicate[1](e)[0]):raise RuntimeError('Original door-open predicate is false after release')
    except Exception as exc:
        failure=f'{type(exc).__name__}: {exc}';traceback.print_exc()
    finally:
        if a.pull_command_policy!='fixed_lead':(out/'pull_command_diagnostics.json').write_text(json.dumps({'policy':PULL_POLICY,'commands':pull_commands},indent=2))
        if a.release_command_policy!='full_pose':(out/'release_command_diagnostics.json').write_text(json.dumps({'policy':a.release_command_policy,'commands':release_commands},indent=2))
        (out/'physics.json').write_text(json.dumps({'schema_version':1,'physics_dt':cfg.sim.dt,'samples':physics_records}))
        if cam is not None:cam.close()
        result={'task_success':source_success,'accepted':bool(a.mode=='collect' and source_success and failure is None),'mode':a.mode,'neutral_completed':bool(a.mode=='neutral' and failure is None),'failure':failure,'frames':len(rows),'final_door_rad':snapshot()['door_q'],'actual_trajectory_samples':len(rows),'physical_contact_samples':sum(r['contacts']['count']>0 for r in rows),'delivery_accepted':False,'no_execution_state_writes':True,'safety_audit_physics_dt':cfg.sim.dt,'safety_checks':len(checks),'physics_samples':len(physics_records),'plans':plans}
        (out/'result.json').write_text(json.dumps(result,indent=2));(out/'trajectory.json').write_text(json.dumps(rows));(out/'audit.json').write_text(json.dumps({'passed':failure is None,'checks':checks},indent=2))
        with h5py.File(out/'trajectory.h5','w') as f:
            if rows:
                f['actions']=np.array([r['action'] for r in rows]);f['q']=np.array([r['obs']['q'] for r in rows]);f['next_q']=np.array([r['next_obs']['q'] for r in rows]);f['door_q']=np.array([r['obs']['door_q'] for r in rows]);f['next_door_q']=np.array([r['next_obs']['door_q'] for r in rows]);f.attrs['metadata']=json.dumps(metadata)
            if physics_records:
                ph=f.create_group('physics')
                ph['step']=np.array([r['physics_step'] for r in physics_records])
                ph['time_s']=np.array([r['time_s'] for r in physics_records])
                ph['robot_q']=np.array([r['state']['q'] for r in physics_records])
                ph['fridge_q']=np.array([r['state']['fridge_q'] for r in physics_records])
                ph['fridge_dq']=np.array([r['state']['fridge_dq'] for r in physics_records])
                ph['finger_contact_counts']=np.array([np.asarray(r['contacts']['counts']).reshape(-1) for r in physics_records])
                ph['finger_normal_force_abs_n']=np.array([r['contacts']['per_finger_abs_normal_force_n'] for r in physics_records])
                for index,entry in enumerate(metadata['mechanism_inventory']):
                    mg=ph.create_group('mechanism_'+str(index));mg.attrs['path']=entry['path'];mg.attrs['joint_names']=json.dumps(entry['joint_names'])
                    mg['q']=np.array([r['mechanisms'][entry['path']]['q'] for r in physics_records])
                    mg['dq']=np.array([r['mechanisms'][entry['path']]['dq'] for r in physics_records])
        videos={}
        for name in ('external_camera','external_camera_2','wrist_camera','all_cameras'):
            if rows:
                try:videos[name]=decode_video(out/(name+'.mp4'),len(rows),expected_dt=e.step_dt)
                except Exception as video_error:
                    videos[name]={'error':str(video_error),'captured_frames':cam.frames,'completed_actions':len(rows)}
                    result['accepted']=False;result['neutral_completed']=False
                    result['video_validation_failed']=True
                    if result['failure'] is None:result['failure']='Video validation: '+str(video_error)
        (out/'result.json').write_text(json.dumps(result,indent=2))
        (out/'validation.json').write_text(json.dumps({'videos':videos,'frames':len(rows),'task_success':source_success,'accepted':result['accepted']},indent=2))
        print('DOOR_RESULT',json.dumps(result),flush=True);env.close()
        if not (result['neutral_completed'] if a.mode=='neutral' else result['accepted']):raise SystemExit(1)
