"""Native hard human execution. Isolated adaptation of original doors collect.py.
No application-level GPU lock: caller must hold reports/skill_trials/gpu.lock.
"""
import argparse,json,time,hashlib,traceback
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--variant',choices=['base','human','property','robot'],default='base');p.add_argument('--output',required=True);p.add_argument('--max-steps',type=int,default=1400);p.add_argument('--handle-offset-y',type=float,default=0.);p.add_argument('--handle-offset-z',type=float,default=.18);p.add_argument('--variant-config',default='configs/extensions/doors/hard_execution_v1/open_fridge.json');p.add_argument('--adaptive-handle-height',action='store_true')
p.add_argument('--mode',choices=['smoke','neutral','execute'],default='smoke');p.add_argument('--duration-seconds',type=float,default=4.)
a=p.parse_args();a.enable_cameras=True
if a.variant!='base':raise SystemExit('Hard human uses original fixed base task plus its dedicated event actor; pass --variant base')
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
    from kitchen_safety.extensions.doors.environment import build,TASK
    from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
    from kitchen_safety.dataset import CameraWriter,numpy as arr,decode_video
    from kitchen_safety.planning import DroidKinematics,transform
    from kitchen_safety.grasp_contacts import _numpy as contact_arr
    from kitchen_safety.curobo_backend import CuroboPlanner
    from pxr import UsdGeom,UsdPhysics,Usd,Gf
    from kitchen_safety.extensions.doors.hard_execution_v1.person import PersonAsset,load_spec
    from kitchen_safety.extensions.doors.hard_execution_v1.events import IntrusionEvents
    from kitchen_safety.extensions.doors.hard_execution_v1.geometry import MeasuredDoorGeometry
    from kitchen_safety.extensions.doors.hard_execution_v1.background import mechanism_metadata,mechanism_snapshot
    from kitchen_safety.extensions.doors.hard_execution_v1.native_mechanisms import NativeMechanisms
    from kitchen_safety.extensions.doors.hard_execution_v1.completion import OpeningCompletion
    from kitchen_safety.geometry_audit import minimum_distance
    spec=load_spec(a.variant_config);person=PersonAsset(spec);events=IntrusionEvents(spec);completion=OpeningCompletion(spec)
    pull_target_cap=float(spec.get('pull_target_cap_rad',.40))
    if pull_target_cap<completion.required_rad:raise ValueError('Pull cap cannot be below measured hard completion goal')
    def opening_target(measured):
        cap=pull_target_cap
        if events.reactive_stop_frame is None and events.started and not events.completed:
            cap=spec['pause_door_rad']
        return min(measured+.018,cap)
    if 'handle_offset_z_m' in spec and not np.isclose(a.handle_offset_z,spec['handle_offset_z_m']):raise ValueError('CLI handle height disagrees with configured validated hard grasp height')
    class NeutralComplete(Exception):pass
    environment,graph=build('base',True,a.variant_config)
    builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device='cpu',seed=a.seed or 42,presets='physx',num_envs=1,solve_relations=True,resolve_on_reset=False,recorder_dataset_export_dir_path=str(out/'arena')))
    cfg,kw=builder.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.sim.render_interval=cfg.decimation
    person.configure_scene(cfg)
    from isaaclab.sensors import CameraCfg
    import isaaclab.sim as sim_utils
    overview_matrix=np.asarray(Gf.Matrix4d().SetLookAt(Gf.Vec3d(4.,-4.6,2.15),Gf.Vec3d(3.65,-1.,.93),Gf.Vec3d(0,0,1)).GetInverse()).T
    cfg.scene.overview_camera=CameraCfg(prim_path='{ENV_REGEX_NS}/hard_overview_camera',height=720,width=960,data_types=['rgb'],spawn=sim_utils.PinholeCameraCfg(focal_length=24.,horizontal_aperture=20.955,vertical_aperture=15.71625),offset=CameraCfg.OffsetCfg(pos=tuple(overview_matrix[:3,3]),rot=tuple(Rotation.from_matrix(overview_matrix[:3,:3]).as_quat()),convention='opengl'))
    # Keep native progress tracking, but avoid automatic reset erasing the success frame.
    cfg.terminations.success=None
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kw);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot'];fridge=scene['fridge'];ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    person.apply(e.sim.stage,scene,0)
    simulation_seconds=0.;hold_target=None;last_contact={'count':0,'counts':[[0],[0]]};neutral_initial=None
    q=arr(robot.data.joint_pos)[0,ids];grip=0.;rows=[];checks=[];plans=[];source_success=False;away_seen=False;failure=None;phase='settle';cam=None;world=None;planner=None
    door_id=next(i for i,n in enumerate(fridge.body_names) if n=='Refrigerator032_fridge_door')
    joint_id=fridge.joint_names.index('fridge_door_joint')
    door_path='/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door'
    def body_T():return transform(arr(fridge.data.body_pos_w)[0,door_id],arr(fridge.data.body_quat_w)[0,door_id])
    contact_view=scene['robot_contacts']._physics_sim_view.create_rigid_contact_view(['/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/left_inner_finger','/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/right_inner_finger'],filter_patterns=[[door_path],[door_path]],max_contact_data_count=4096)
    native_mechanisms=NativeMechanisms(e.sim.stage,scene['robot_contacts']._physics_sim_view)
    def contact_snapshot():
        forces,points,normals,separations,counts,starts=map(contact_arr,contact_view.get_contact_data(dt=cfg.sim.dt))
        chunks=[np.arange(int(start),int(start+count)) for count,start in zip(counts.reshape(-1),starts.reshape(-1))]
        indices=np.concatenate(chunks).astype(int) if chunks else np.array([],dtype=int)
        return {'per_finger_abs_normal_force_n':[float(np.abs(forces[chunk.astype(int)]).sum()) for chunk in chunks],'per_finger_signed_normal_force_n':[float(forces[chunk.astype(int)].sum()) for chunk in chunks],'count':len(indices),'normal_force_sum_n':float(forces[indices].sum()),'counts':counts.tolist(),'points_world_m':points[indices].reshape(-1,3).tolist(),'normal_forces_n':forces[indices].reshape(-1).tolist(),'separations_m':separations[indices].reshape(-1).tolist()}
    startup=[]
    def startup_sample(kind):
        startup.append({'kind':kind,'physics_step':int(e.sim.get_physics_step_count()),'simulation_seconds':float(e.sim.get_physics_step_count())*cfg.sim.dt,'q':arr(robot.data.joint_pos)[0].tolist(),'dq':arr(robot.data.joint_vel)[0].tolist(),'fridge_q':arr(fridge.data.joint_pos)[0].tolist(),'fridge_dq':arr(fridge.data.joint_vel)[0].tolist(),'mechanisms':mechanism_snapshot(scene),'native_mechanisms':native_mechanisms.sample(),'contacts':contact_snapshot()})
    startup_sample('native_reset_returned')
    startup_update=scene.update
    def recorded_startup_update(dt):
        startup_update(dt);startup_sample('warmup_physics_substep')
    scene.update=recorded_startup_update
    for _ in range(8):env.step(torch.tensor(np.r_[q,0.][None],dtype=torch.float32,device=e.device))
    scene.update=startup_update
    (out/'startup.json').write_text(json.dumps({'scope':'Explicit native reset return plus all physics substeps of 8 warmup actions; internal reset implementation substeps are not claimed sampled.','reset_internal_substeps_sampled':False,'native_mechanism_inventory':native_mechanisms.metadata,'samples':startup},indent=2))
    base=arr(robot.data.root_pos_w)[0];yaw=Rotation.from_quat(arr(robot.data.root_quat_w)[0]).as_euler('xyz',degrees=True)[2]
    variant_data=json.loads(Path(a.variant_config).read_text())
    if variant_data.get('_fixed_source_layout'):
        fixed=json.loads(Path(variant_data['_fixed_source_layout']).read_text())
        if not np.allclose(base,fixed['expected_robot_base'],atol=2e-6,rtol=0) or abs(yaw-fixed['expected_robot_yaw_deg'])>1e-5:raise RuntimeError('Original solved source robot layout changed')
    kin=DroidKinematics(base,yaw);initial_door_T=body_T();cache=UsdGeom.XformCache()
    handle_prim=e.sim.stage.GetPrimAtPath(door_path+'/Collisions/fridge_door_handle_main')
    handle_T=np.array(cache.GetLocalToWorldTransform(handle_prim)).T
    handle0=handle_T[:3,3].copy();handle0[1]+=a.handle_offset_y;handle0[2]+=a.handle_offset_z
    j=UsdPhysics.RevoluteJoint(e.sim.stage.GetPrimAtPath(door_path+'/fridge_door_joint'))
    if not (np.radians(j.GetLowerLimitAttr().Get())<=completion.required_rad<=pull_target_cap<=np.radians(j.GetUpperLimitAttr().Get())):raise ValueError('Additional opening target outside original native door limits')
    body0=e.sim.stage.GetPrimAtPath(str(j.GetBody0Rel().GetTargets()[0]));T0=np.array(cache.GetLocalToWorldTransform(body0)).T
    lq=j.GetLocalRot0Attr().Get();local=transform(j.GetLocalPos0Attr().Get(),[*lq.GetImaginary(),lq.GetReal()]);hingeT=T0@local
    hinge=hingeT[:3,3];axis=hingeT[:3,'XYZ'.index(str(j.GetAxisAttr().Get()))];axis=axis/np.linalg.norm(axis)
    R0=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    exact_door=MeasuredDoorGeometry(e.sim.stage,door_path)
    predicate=cfg.task_progress.success_criteria[0].predicate_sequence
    metadata={'task':graph.env_name,'variant':a.variant,'seed':a.seed or 42,'source_yaml_sha256':hashlib.sha256(TASK.read_bytes()).hexdigest(),'robot_base':base.tolist(),'robot_yaw_deg':float(yaw),'handle_initial':handle0.tolist(),'hinge':hinge.tolist(),'axis':axis.tolist(),'dt':e.step_dt,'physics_dt':cfg.sim.dt,'robot_body_names':robot.body_names,'robot_joint_names':robot.joint_names,'fridge_joint_names':fridge.joint_names,'initial_door_pose':np.r_[initial_door_T[:3,3],Rotation.from_matrix(initial_door_T[:3,:3]).as_quat()].tolist(),'task_success_rule':'native OpenDoorTask predicate_sequence; move >0.05 before open >0.2','episode_mode':'fixed_only','door_state_writes':'source reset only','camera_alignment':'obs[t] before action[t], next_obs[t] after action[t]'}
    from importlib.metadata import version
    metadata['software_versions']={name:version(name) for name in ('isaacsim','isaaclab','isaaclab-arena','nvidia-curobo','torch','warp-lang')}
    metadata['mechanisms']=mechanism_metadata(scene)
    metadata['operated_mechanism']={'articulation':'fridge','joint':'fridge_door_joint'}
    metadata['mechanism_observation_scope']='Every non-robot articulation registered in the native scene, from reset return through all recorded physics substeps. No mechanism state is written by this observer.'
    metadata['native_mechanisms']=native_mechanisms.metadata
    metadata['additional_measured_opening_requirement']={**completion.snapshot(),'pull_target_cap_rad':pull_target_cap,'native_predicate_unchanged':True,'hold_actions_are_fixed_arm_target_actions':True,'hold_gate_readbacks':'Minimum measured door q and maximum absolute door dq across every physics substep in each hold action'}
    metadata['variant_config']=a.variant_config;metadata['variant_spec']=None if a.variant=='base' else json.loads(Path(a.variant_config).read_text())[a.variant]
    if a.variant!='base':
        hazard_prim=e.sim.stage.GetPrimAtPath('/World/envs/env_0/'+metadata['variant_spec']['name'])
        hazard_matrix=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(hazard_prim)).T
        metadata['actual_hazard_initial_pose']=[*hazard_matrix[:3,3],*Rotation.from_matrix(hazard_matrix[:3,:3]).as_quat()]
    metadata['hard_execution']={'mode':a.mode,'spec':spec,'human_collision_model':'PhysX kinematic segment capsules; separate source human triangle mesh vs source robot convex hull/gripper and exact affine source door cuboids at every physics substep. Proxies are not asserted identical to source meshes.','human_pose_rate_hz':12,'native_rendering':True,'proxy_collision_coverage_complete':False,'door_driver':'Measured-joint closed loop robot handle contact only','substep_contact_causality_boundary':'States and contact buffers are sampled after the same physics step; substep-internal ordering is not resolved.'}
    metadata['controller_parameters']={'seed':a.seed or 42,'handle_offset_y':a.handle_offset_y,'handle_offset_z':a.handle_offset_z,'max_joint_speed':.5}
    if a.adaptive_handle_height:metadata['controller_parameters']['handle_height_candidates_m']=[a.handle_offset_z,a.handle_offset_z+.06,a.handle_offset_z+.10]
    metadata['implementation_sha256']={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in (__file__,'kitchen_safety/extensions/doors/environment.py','kitchen_safety/extensions/doors/collision.py','kitchen_safety/extensions/doors/hard_execution_v1/person.py','kitchen_safety/extensions/doors/hard_execution_v1/events.py','kitchen_safety/extensions/doors/hard_execution_v1/geometry.py','kitchen_safety/extensions/doors/hard_execution_v1/background.py','kitchen_safety/extensions/doors/hard_execution_v1/native_mechanisms.py','kitchen_safety/extensions/doors/hard_execution_v1/completion.py',a.variant_config)}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    def snapshot():
        return {'q':arr(robot.data.joint_pos)[0].tolist(),'dq':arr(robot.data.joint_vel)[0].tolist(),'door_q':float(arr(fridge.data.joint_pos)[0,joint_id]),'door_dq':float(arr(fridge.data.joint_vel)[0,joint_id]),'human_frame':person.frame,'intrusion':events.snapshot(),'fridge_q':arr(fridge.data.joint_pos)[0].tolist(),'fridge_dq':arr(fridge.data.joint_vel)[0].tolist(),'mechanisms':mechanism_snapshot(scene),'native_mechanisms':native_mechanisms.sample(),'hazard_pose':arr(scene['glass'].data.root_state_w)[0,:7].tolist() if a.variant=='property' else None,'door_body_pose':np.r_[arr(fridge.data.body_pos_w)[0,door_id],arr(fridge.data.body_quat_w)[0,door_id]].tolist(),'robot_body_pose':np.concatenate([arr(robot.data.body_pos_w)[0],arr(robot.data.body_quat_w)[0]],axis=-1).tolist()}
    def sync(*,update_planner=False):
        delta=body_T()@np.linalg.inv(initial_door_T);world.move_body(door_path,delta)
        if a.variant=='property':
            world.set_obstacle_pose('glass',arr(scene['glass'].data.root_state_w)[0,:7])
            if planner is not None and update_planner:planner.update_dynamic_obstacle('glass',reset=False)
        if planner is not None and update_planner:
            from curobo.types.math import Pose
            for i,(name,_,lo,hi) in enumerate(world.objects):
                if not name.startswith(door_path+'/'):continue
                rot,pos,rlo,rhi,mesh=world._object_reference[i]
                if mesh is not None:
                    T=planner.T_B_W@delta@kin.base
                else:
                    Tb=np.eye(4);Tb[:3,3]=(rlo+rhi)/2;T=planner.T_B_W@delta@Tb
                planner.motion_gen.world_coll_checker.update_obstacle_pose(name,Pose.from_list(planner._pose_list(T),planner.args))
        return delta
    def audit(sample_kind="post_action"):
        state=snapshot();sync();angles=dict(zip(robot.joint_names,state['q']))
        ok=world.valid(np.array(state['q'])[ids],mode='execution',gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES})
        row={'action_index':len(rows),'phase':phase,'valid':ok,'failure':world.last_clearance,'q':state['q'],'door_body_pose':state['door_body_pose'],'hazard_pose':state['hazard_pose'],'door_q':state['door_q']}
        hazard_key={'human':'person','property':'glass','robot':'barrier'}.get(a.variant)
        if hazard_key:
            distances=[]
            support_spec=metadata['variant_spec'].get('support')
            for dn,do,dl,dh in world.objects:
                if not dn.startswith(door_path+'/'):continue
                for hn,ho,hl,hh in world.objects:
                    if not world._matches(hn,hazard_key) and not (support_spec and world._matches(hn,support_spec['name'])):continue
                    bound=np.linalg.norm(np.maximum(np.maximum(hl-dh,dl-hh),0))
                    if bound>.15:continue
                    raw=float(world.fcl.distance(do,ho,world.fcl.DistanceRequest(),world.fcl.DistanceResult()))
                    distances.append({'door_shape':dn,'hazard_shape':hn,'distance_m':max(raw,0.),'intersecting':raw<0})
                    required=world.execution_clearances[hazard_key] if world._matches(hn,hazard_key) else world.default_clearance
                    if raw<required:ok=False
            row['door_hazard_pairs_near_15cm']=distances;row['valid']=ok
        row.update({'sample_kind':sample_kind,'physics_step':int(e.sim.get_physics_step_count()),'simulation_seconds':float(e.sim.get_physics_step_count())*cfg.sim.dt,'robot_dq':state['dq'],'fridge_q':state['fridge_q'],'fridge_dq':state['fridge_dq'],'mechanisms':state['mechanisms'],'native_mechanisms':state['native_mechanisms'],'contacts':contact_snapshot(),'human_frame':person.frame,'intrusion':events.snapshot()})
        robot_source_shapes=[r for r in world.shapes(np.array(state['q'])[ids],gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES}) if r[0]=='robot_hull' or r[0].startswith('gripper/')]
        row['source_human_to_robot_hulls_and_gripper']=minimum_distance(person.shapes,robot_source_shapes,spec['source_mesh_human_clearance_m'])
        row['source_human_to_measured_source_door']=minimum_distance(person.shapes,exact_door.measured(body_T()@np.linalg.inv(initial_door_T)),spec['source_mesh_human_clearance_m'])
        ok=ok and row['source_human_to_robot_hulls_and_gripper']['valid'] and row['source_human_to_measured_source_door']['valid'];row['valid']=ok
        checks.append(row)
        if not ok:raise RuntimeError('EXECUTION_COLLISION '+json.dumps(row))
    def step(target,g):
        global q,grip,source_success,away_seen,simulation_seconds,hold_target,last_contact
        current=snapshot();loads=last_contact.get('per_finger_abs_normal_force_n',[0.,0.]);both=bool(len(loads)>=2 and min(loads[:2])>=spec['loaded_contact_min_abs_force_per_finger_n'])
        events.advance(e.step_dt,door_q=current['door_q'],door_dq=current['door_dq'],arm_dq_max=float(np.max(np.abs(np.array(current['dq'])[ids]))),both_fingers=both,at=simulation_seconds,enabled=a.mode=='execute' and phase in ('pull_door','human_wait','recover_open'))
        changed=person.apply(e.sim.stage,scene,events.frame)
        if changed and world is not None:
            person.bind_world(world)
            if planner is not None:person.sync_planner(planner)
        if events.hold:
            if hold_target is None:hold_target=np.array(current['q'])[ids].copy()
            target=hold_target
        elif events.completed:hold_target=None
        if world is not None:
            angles=dict(zip(robot.joint_names,current['q']))
            if not world.valid(np.array(target),mode='execution',gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES}):raise RuntimeError('Human-aware next command rejected before physics: '+str(world.last_clearance))
        if len(rows)>=a.max_steps:raise RuntimeError('max_steps reached')
        before=snapshot();cam.write(phase)
        action=np.r_[target,g];env.step(torch.tensor(action[None],dtype=torch.float32,device=e.device));after=snapshot()
        contact=contact_snapshot();last_contact=contact
        away=bool(predicate[0](e)[0]);opened=bool(predicate[1](e)[0]);source_success=source_success or (away_seen and opened);away_seen=away_seen or away
        rows.append({'phase':('human_wait' if events.hold else phase),'intrusion':events.snapshot(),'human_frame':person.frame,'action':action.tolist(),'obs':before,'next_obs':after,'contacts':contact,'native_away':away,'native_open':opened,'success':source_success})
        if len(rows)%50==0:
            print('HARD_HUMAN_PROGRESS',json.dumps({'actions':len(rows),'physics_step':int(e.sim.get_physics_step_count()),'phase':rows[-1]['phase'],'door_deg':float(np.degrees(after['door_q'])),'human_frame':person.frame,'intrusion':events.snapshot()}),flush=True)
        q=np.array(after['q'])[ids];grip=g
        if world is not None:audit()
    original_scene_update=scene.update
    def audited_scene_update(dt):
        global simulation_seconds
        original_scene_update(dt);simulation_seconds+=dt
        if world is not None:audit('physics_substep')
    scene.update=audited_scene_update
    try:
        camera_names=[*environment.embodiment.camera_config.camera_names(),'overview_camera']
        cam=CameraWriter(out,scene,camera_names,e.step_dt)
        world=DoorCollisionWorld(e.sim.stage,kin,env_root='/World/envs/env_0',default_clearance=.002)
        person.bind_world(world)
        neutral_initial=snapshot()
        (out/'collision_coverage.json').write_text(json.dumps({'source_door_collider_count':sum(1 for p in Usd.PrimRange(e.sim.stage.GetPrimAtPath(door_path),Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()),'planner_door_colliders':[n for n,*_ in world.objects if n.startswith(door_path+'/')],'total_colliders':len(world.objects)},indent=2))
        step(q,0.)
        if a.mode!='execute':
            phase='native_'+a.mode
            for _ in range(max(1,int(a.duration_seconds/e.step_dt))):step(q,0.)
            raise NeutralComplete()
        planner=CuroboPlanner(world,kin,dt=e.step_dt,seed=a.seed or 42,max_joint_speed=.5)
        handles=tuple(n.rsplit('/',1)[-1] for n,*_ in world.objects if '/fridge_door_handle' in n)
        target=np.eye(4);target[:3,:3]=R0;default_handle=handle0.copy();candidate_failures=[];plan=None;candidate_queries=[]
        phase='pregrasp'
        for offset in ([0.,.06,.10] if a.adaptive_handle_height else [0.]):
            handle0=default_handle+[0.,0.,offset];target[:3,3]=handle0+[0,-.13,0]
            candidate_queries.append({'offset_z_m':a.handle_offset_z+offset,'requested_handle_point_world':handle0.tolist(),'requested_pregrasp_pose_world':target.tolist()})
            print('DOOR_PLAN',phase,target.tolist(),flush=True)
            try:plan=planner.plan_pose(q,target,phase='door_pregrasp')
            except RuntimeError as exc:
                candidate_failures.append({'handle_offset_z_m':a.handle_offset_z+offset,'reason':str(exc),'executed_actions':0});continue
            metadata['selected_handle_initial']=handle0.tolist();metadata['selected_handle_offset_z_m']=a.handle_offset_z+offset;break
        metadata['handle_candidate_failures']=candidate_failures;metadata['handle_candidate_queries']=candidate_queries;metadata['handle_selection_policy']='First feasible collision-aware cuRobo plan from the same ordered source-handle height candidates; no variant-label branch.';(out/'metadata.json').write_text(json.dumps(metadata,indent=2))
        if plan is None:raise RuntimeError('All unchanged-task handle-height candidates rejected: '+json.dumps(candidate_failures))
        plans.append(plan['metadata'])
        for command in plan['positions']:step(command,0.)
        phase='approach_handle';world.allow_handle_contact=True
        for alpha in np.linspace(0,1,50)[1:]:
            target[:3,3]=handle0+[0,-.13*(1-alpha),0]
            command=kin.ik(target,q,attempts=1)
            world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
            sync(update_planner=True)
            planner._prepare_path_query(handles,0.,phase)
            if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):raise RuntimeError('cuRobo rejected contact approach')
            step(command,0.)
        phase='grasp_handle'
        for _ in range(20):step(q,1.)
        if not any(min(r['contacts']['per_finger_abs_normal_force_n'][:2])>=spec['loaded_contact_min_abs_force_per_finger_n'] for r in rows[-10:]):raise RuntimeError('No loaded physical contact on both fingers after closure')
        phase='pull_door'
        for _ in range(650):
            if events.hold:
                step(hold_target if hold_target is not None else q,1.)
                continue
            # Walking toward the task does not stop the opening. IntrusionEvents
            # requests the hold only once the hand is visibly being raised.
            measured=rows[-1]['next_obs']['door_q'];angle=opening_target(measured)
            rr=Rotation.from_rotvec(axis*angle).as_matrix();target[:3,:3]=rr@R0;target[:3,3]=hinge+rr@(handle0-hinge)
            command=kin.ik(target,q,attempts=1)
            # Bound actual commanded joint increments near elbow limits; each
            # short segment remains collision checked before physical control.
            delta_q=command-q
            command=q+delta_q/max(1.,float(np.max(np.abs(delta_q)))/.025)
            world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')])
            sync(update_planner=True)
            planner._prepare_path_query(handles,1.,phase)
            if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):raise RuntimeError('cuRobo rejected pull segment')
            step(command,1.)
            if source_success and completion.reached(rows[-1]['next_obs']['door_q'],rows[-1]['native_open'],events.completed):break
        if not source_success:raise RuntimeError('Original OpenDoorTask predicate sequence not achieved')
        final_hold_target=q.copy()
        # Final completion requires ACTUAL opening to remain above the goal,
        # with low measured door speed, for consecutive fixed-target actions.
        # If settling drops below it, recover by physical handle control only.
        for _ in range(120):
            current=snapshot()
            if not completion.reached(current['door_q'],bool(predicate[1](e)[0]),events.completed):
                phase='recover_open';completion.stable_actions=0
                measured=current['door_q'];angle=opening_target(measured)
                rr=Rotation.from_rotvec(axis*angle).as_matrix();target[:3,:3]=rr@R0;target[:3,3]=hinge+rr@(handle0-hinge)
                command=kin.ik(target,q,attempts=1);delta_q=command-q;command=q+delta_q/max(1.,float(np.max(np.abs(delta_q)))/.025)
                world.gripper_joint_position=float(arr(robot.data.joint_pos)[0,robot.joint_names.index('finger_joint')]);sync(update_planner=True);planner._prepare_path_query(handles,1.,phase)
                if not bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all()):raise RuntimeError('cuRobo rejected recovery pull')
                step(command,1.);final_hold_target=q.copy()
            else:
                phase='success_hold';check_start=len(checks);step(final_hold_target,1.)
                measured=snapshot()
                substeps=[sample for sample in checks[check_start:] if sample['sample_kind']=='physics_substep']
                if len(substeps)!=round(e.step_dt/cfg.sim.dt):raise RuntimeError('Final hold lacks complete native physics sampling')
                held_q=min(sample['door_q'] for sample in substeps)
                held_dq=max(abs(sample['fridge_dq'][joint_id]) for sample in substeps)
                if completion.observe(door_q=held_q,door_dq=held_dq,native_open=bool(predicate[1](e)[0]),events_complete=events.completed):break
        if not completion.complete:raise RuntimeError('Measured final opening did not reach and hold the configured goal with original native predicate true')
    except NeutralComplete:
        pass
    except Exception as exc:
        failure=f'{type(exc).__name__}: {exc}';traceback.print_exc()
    finally:
        if cam is not None:cam.close()
        result={'task_success':source_success,'accepted':False,'failure':failure,'frames':len(rows),'final_door_rad':snapshot()['door_q'],'actual_trajectory_samples':len(rows),'physical_contact_samples':sum(r['contacts']['count']>0 for r in rows),'no_execution_state_writes':True,'safety_audit_physics_dt':cfg.sim.dt,'safety_checks':len(checks),'plans':plans}
        result.update({'mode':a.mode,'run_completed':failure is None and (a.mode!='execute' or (source_success and events.completed and completion.complete)),'hard_interruption_completed':events.completed,'human_final_frame':person.frame,'native_rendering':True,'physical_acceptance':'not independently validated','neutral_joint_drift_rad':float(np.max(np.abs(np.asarray(snapshot()['fridge_q'])-np.asarray(neutral_initial['fridge_q'])))) if neutral_initial else None})
        result['additional_measured_opening']=completion.snapshot()
        result['native_mechanism_recording_complete']=bool(native_mechanisms.metadata['all_authored_roots_read'] and not any(r['native_mechanisms']['read_errors'] for r in startup+checks))
        result['demonstration_completed']=bool(source_success and failure is None and events.completed and completion.complete)
        result['acceptance_pending_independent_audit']=True
        (out/'events.json').write_text(json.dumps({'spec':spec,'events':events.events,'final':events.snapshot()},indent=2))
        (out/'result.json').write_text(json.dumps(result,indent=2));(out/'trajectory.json').write_text(json.dumps(rows));(out/'audit.json').write_text(json.dumps({'passed':failure is None,'checks':checks},indent=2))
        with h5py.File(out/'trajectory.h5','w') as f:
            if rows:
                f['actions']=np.array([r['action'] for r in rows]);f['q']=np.array([r['obs']['q'] for r in rows]);f['next_q']=np.array([r['next_obs']['q'] for r in rows]);f['door_q']=np.array([r['obs']['door_q'] for r in rows]);f['next_door_q']=np.array([r['next_obs']['door_q'] for r in rows]);f.attrs['metadata']=json.dumps(metadata)
        videos={}
        for name in ('external_camera','external_camera_2','wrist_camera','overview_camera','all_cameras'):
            if rows:videos[name]=decode_video(out/(name+'.mp4'),len(rows),expected_dt=e.step_dt)
        (out/'validation.json').write_text(json.dumps({'videos':videos,'frames':len(rows),'task_success':source_success,'accepted':result['accepted']},indent=2))
        print('DOOR_RESULT',json.dumps(result),flush=True);env.close()
        if not result['run_completed']:raise SystemExit(1)
