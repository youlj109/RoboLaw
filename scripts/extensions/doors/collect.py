"""Physical refrigerator-opening trial. The target door receives no action or state writes."""
import argparse,json,time,hashlib,traceback
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
p=get_isaaclab_arena_cli_parser();p.add_argument('--variant',choices=['base','human','property','robot'],default='base');p.add_argument('--output',required=True);p.add_argument('--max-steps',type=int,default=500);p.add_argument('--handle-offset-y',type=float,default=0.);p.add_argument('--handle-offset-z',type=float,default=.12)
a=p.parse_args();a.enable_cameras=True
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
    from pxr import UsdGeom,UsdPhysics,Usd
    environment,graph=build(a.variant,True)
    builder=ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device='cpu',seed=a.seed or 42,presets='physx',num_envs=1,solve_relations=True,resolve_on_reset=False,recorder_dataset_export_dir_path=str(out/'arena')))
    cfg,kw=builder.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.sim.render_interval=cfg.decimation
    # Keep native progress tracking, but avoid automatic reset erasing the success frame.
    cfg.terminations.success=None
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0,history_length=1)
    env=builder.make_registered(cfg,kw);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot'];fridge=scene['fridge'];ids,_=robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    q=arr(robot.data.joint_pos)[0,ids];grip=0.;rows=[];checks=[];plans=[];source_success=False;away_seen=False;failure=None;phase='settle';cam=None;world=None;planner=None
    door_id=next(i for i,n in enumerate(fridge.body_names) if n=='Refrigerator032_fridge_door')
    joint_id=fridge.joint_names.index('fridge_door_joint')
    door_path='/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door'
    def body_T():return transform(arr(fridge.data.body_pos_w)[0,door_id],arr(fridge.data.body_quat_w)[0,door_id])
    for _ in range(8):env.step(torch.tensor(np.r_[q,0.][None],dtype=torch.float32,device=e.device))
    base=arr(robot.data.root_pos_w)[0];yaw=Rotation.from_quat(arr(robot.data.root_quat_w)[0]).as_euler('xyz',degrees=True)[2]
    kin=DroidKinematics(base,yaw);initial_door_T=body_T();cache=UsdGeom.XformCache()
    handle_prim=e.sim.stage.GetPrimAtPath(door_path+'/Collisions/fridge_door_handle_main')
    handle_T=np.array(cache.GetLocalToWorldTransform(handle_prim)).T
    handle0=handle_T[:3,3].copy();handle0[1]+=a.handle_offset_y;handle0[2]+=a.handle_offset_z
    j=UsdPhysics.RevoluteJoint(e.sim.stage.GetPrimAtPath(door_path+'/fridge_door_joint'))
    body0=e.sim.stage.GetPrimAtPath(str(j.GetBody0Rel().GetTargets()[0]));T0=np.array(cache.GetLocalToWorldTransform(body0)).T
    lq=j.GetLocalRot0Attr().Get();local=transform(j.GetLocalPos0Attr().Get(),[*lq.GetImaginary(),lq.GetReal()]);hingeT=T0@local
    hinge=hingeT[:3,3];axis=hingeT[:3,'XYZ'.index(str(j.GetAxisAttr().Get()))];axis=axis/np.linalg.norm(axis)
    R0=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    predicate=cfg.task_progress.success_criteria[0].predicate_sequence
    metadata={'task':graph.env_name,'variant':a.variant,'seed':a.seed or 42,'source_yaml_sha256':hashlib.sha256(TASK.read_bytes()).hexdigest(),'robot_base':base.tolist(),'robot_yaw_deg':float(yaw),'handle_initial':handle0.tolist(),'hinge':hinge.tolist(),'axis':axis.tolist(),'dt':e.step_dt,'physics_dt':cfg.sim.dt,'robot_body_names':robot.body_names,'robot_joint_names':robot.joint_names,'fridge_joint_names':fridge.joint_names,'initial_door_pose':np.r_[initial_door_T[:3,3],Rotation.from_matrix(initial_door_T[:3,:3]).as_quat()].tolist(),'task_success_rule':'native OpenDoorTask predicate_sequence; move >0.05 before open >0.2','episode_mode':'fixed_only','door_state_writes':'source reset only','camera_alignment':'obs[t] before action[t], next_obs[t] after action[t]'}
    from importlib.metadata import version
    metadata['software_versions']={name:version(name) for name in ('isaacsim','isaaclab','isaaclab-arena','nvidia-curobo','torch','warp-lang')}
    metadata['implementation_sha256']={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in (__file__,'kitchen_safety/extensions/doors/environment.py','kitchen_safety/extensions/doors/collision.py','configs/extensions/doors/open_fridge_variants.json')}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    contact_view=scene['robot_contacts']._physics_sim_view.create_rigid_contact_view(['/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/left_inner_finger','/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/right_inner_finger'],filter_patterns=[[door_path],[door_path]],max_contact_data_count=4096)
    def snapshot():
        return {'q':arr(robot.data.joint_pos)[0].tolist(),'dq':arr(robot.data.joint_vel)[0].tolist(),'door_q':float(arr(fridge.data.joint_pos)[0,joint_id]),'fridge_q':arr(fridge.data.joint_pos)[0].tolist(),'fridge_dq':arr(fridge.data.joint_vel)[0].tolist(),'hazard_pose':arr(scene['glass'].data.root_state_w)[0,:7].tolist() if a.variant=='property' else None,'door_body_pose':np.r_[arr(fridge.data.body_pos_w)[0,door_id],arr(fridge.data.body_quat_w)[0,door_id]].tolist(),'robot_body_pose':np.concatenate([arr(robot.data.body_pos_w)[0],arr(robot.data.body_quat_w)[0]],axis=-1).tolist()}
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
    def audit():
        state=snapshot();sync();angles=dict(zip(robot.joint_names,state['q']))
        ok=world.valid(np.array(state['q'])[ids],mode='execution',gripper_q={k:angles[k] for k in world.GRIPPER_JOINT_NAMES})
        row={'action_index':len(rows),'phase':phase,'valid':ok,'failure':world.last_clearance}
        hazard_key={'human':'person','property':'glass','robot':'barrier'}.get(a.variant)
        if hazard_key:
            distances=[]
            for dn,do,dl,dh in world.objects:
                if not dn.startswith(door_path+'/'):continue
                for hn,ho,hl,hh in world.objects:
                    if not world._matches(hn,hazard_key):continue
                    bound=np.linalg.norm(np.maximum(np.maximum(hl-dh,dl-hh),0))
                    if bound>.15:continue
                    raw=float(world.fcl.distance(do,ho,world.fcl.DistanceRequest(),world.fcl.DistanceResult()))
                    distances.append({'door_shape':dn,'hazard_shape':hn,'distance_m':max(raw,0.),'intersecting':raw<0})
                    if raw<world.execution_clearances[hazard_key]:ok=False
            row['door_hazard_pairs_near_15cm']=distances;row['valid']=ok
        checks.append(row)
        if not ok:raise RuntimeError('EXECUTION_COLLISION '+json.dumps(row))
    def step(target,g):
        global q,grip,source_success,away_seen
        if len(rows)>=a.max_steps:raise RuntimeError('max_steps reached')
        before=snapshot();cam.write(phase)
        action=np.r_[target,g];env.step(torch.tensor(action[None],dtype=torch.float32,device=e.device));after=snapshot()
        forces,points,normals,separations,counts,starts=map(contact_arr,contact_view.get_contact_data(dt=cfg.sim.dt))
        indices=np.concatenate([np.arange(int(start),int(start+count)) for count,start in zip(counts.reshape(-1),starts.reshape(-1))]).astype(int)
        count=len(indices);contact={'count':count,'normal_force_sum_n':float(forces[indices].sum()),'counts':counts.tolist(),'points_world_m':points[indices].reshape(-1,3).tolist(),'normal_forces_n':forces[indices].reshape(-1).tolist(),'separations_m':separations[indices].reshape(-1).tolist()}
        away=bool(predicate[0](e)[0]);opened=bool(predicate[1](e)[0]);source_success=source_success or (away_seen and opened);away_seen=away_seen or away
        rows.append({'phase':phase,'action':action.tolist(),'obs':before,'next_obs':after,'contacts':contact,'native_away':away,'native_open':opened,'success':source_success})
        q=np.array(after['q'])[ids];grip=g
        if world is not None:audit()
    original_scene_update=scene.update
    def audited_scene_update(dt):
        original_scene_update(dt)
        if world is not None:audit()
    scene.update=audited_scene_update
    try:
        cam=CameraWriter(out,scene,environment.embodiment.camera_config.camera_names(),e.step_dt)
        world=DoorCollisionWorld(e.sim.stage,kin,env_root='/World/envs/env_0',default_clearance=.002)
        (out/'collision_coverage.json').write_text(json.dumps({'source_door_collider_count':sum(1 for p in Usd.PrimRange(e.sim.stage.GetPrimAtPath(door_path),Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()),'planner_door_colliders':[n for n,*_ in world.objects if n.startswith(door_path+'/')],'total_colliders':len(world.objects)},indent=2))
        step(q,0.)
        planner=CuroboPlanner(world,kin,dt=e.step_dt,seed=a.seed or 42,max_joint_speed=.5)
        handles=tuple(n.rsplit('/',1)[-1] for n,*_ in world.objects if '/fridge_door_handle' in n)
        target=np.eye(4);target[:3,:3]=R0;target[:3,3]=handle0+[0,-.13,0]
        phase='pregrasp';print('DOOR_PLAN',phase,target.tolist(),flush=True)
        plan=planner.plan_pose(q,target,phase='door_pregrasp');plans.append(plan['metadata'])
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
        if max(r['contacts']['count'] for r in rows[-10:])==0:raise RuntimeError('No physical finger-door contact after closure')
        phase='pull_door'
        for _ in range(300):
            measured=rows[-1]['next_obs']['door_q'];angle=min(measured+.018,.40)
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
            if source_success:break
        if not source_success:raise RuntimeError('Original OpenDoorTask predicate sequence not achieved')
        phase='success_hold'
        for _ in range(5):step(q,1.)
    except Exception as exc:
        failure=f'{type(exc).__name__}: {exc}';traceback.print_exc()
    finally:
        if cam is not None:cam.close()
        result={'task_success':source_success,'accepted':bool(source_success and failure is None),'failure':failure,'frames':len(rows),'final_door_rad':snapshot()['door_q'],'actual_trajectory_samples':len(rows),'physical_contact_samples':sum(r['contacts']['count']>0 for r in rows),'no_execution_state_writes':True,'safety_audit_physics_dt':cfg.sim.dt,'safety_checks':len(checks),'plans':plans}
        (out/'result.json').write_text(json.dumps(result,indent=2));(out/'trajectory.json').write_text(json.dumps(rows));(out/'audit.json').write_text(json.dumps({'passed':failure is None,'checks':checks},indent=2))
        with h5py.File(out/'trajectory.h5','w') as f:
            if rows:
                f['actions']=np.array([r['action'] for r in rows]);f['q']=np.array([r['obs']['q'] for r in rows]);f['next_q']=np.array([r['next_obs']['q'] for r in rows]);f['door_q']=np.array([r['obs']['door_q'] for r in rows]);f['next_door_q']=np.array([r['next_obs']['door_q'] for r in rows]);f.attrs['metadata']=json.dumps(metadata)
        videos={}
        for name in ('external_camera','external_camera_2','wrist_camera','all_cameras'):
            if rows:videos[name]=decode_video(out/(name+'.mp4'),len(rows),expected_dt=e.step_dt)
        (out/'validation.json').write_text(json.dumps({'videos':videos,'frames':len(rows),'task_success':source_success,'accepted':result['accepted']},indent=2))
        print('DOOR_RESULT',json.dumps(result),flush=True);env.close()
        if not result['accepted']:raise SystemExit(1)
