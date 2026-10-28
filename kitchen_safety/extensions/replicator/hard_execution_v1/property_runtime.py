"""Native observations for the accepted property scene. No object state writes."""
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.task_recorder import TaskPhysicsRecorder
from kitchen_safety.extensions.replicator.startup_recorder import array,pose_change
from .property_environment import PropertyEnvironment,PropertyCfg,read_config,sha


def finalize_outcome(result,stage):
    """Keep task, automatic physical evidence and independent acceptance apart."""
    complete_by_stage={'neutral':True,'glass-grasp':bool(result.get('glass_grasp_trial_complete')),
        'glass-transfer':bool(result.get('glass_relocated_and_stable')),
        'execute':bool(result.get('glass_relocated_and_stable') and result.get('physical_sequence_complete') and
            result.get('original_task_success') and result.get('final_original_destination_predicate'))}
    recording_ok=bool(result.get('physical_observation_audit',{}).get('passed') and
        not result.get('recording_error') and not result.get('close_errors'))
    completed=bool(result.get('observation_complete') and result.get('settled_objects_within_declared_tolerance') and
        complete_by_stage[stage] and recording_ok and not result.get('error'))
    result['requested_stage']=stage;result['requested_stage_passed']=completed
    result['task_success']=bool(result.get('original_task_success') and result.get('final_original_destination_predicate'))
    result['full_sequence_physics_checks']=dict(
        requested_full_sequence=stage=='execute',
        initial_settled_objects=bool(result.get('settled_objects_within_declared_tolerance')),
        actual_glass_bilateral_grasp=bool(result.get('glass_bilateral_grasp_observed')),
        actual_glass_lift=bool(result.get('glass_lift_evidence',{}).get('rise_m',0)>=.08),
        actual_glass_stable_release=bool(result.get('glass_relocated_and_stable')),
        bowl_sequence_finished=bool(result.get('physical_sequence_complete')),
        original_bowl_task_and_final_predicate=result['task_success'],
        continuous_physical_recording_and_scene_audit=recording_ok,
        source_predicate_recording=bool(result.get('bowl_predicate_evidence') and not result.get('bowl_predicate_recording_error')),
        no_execution_or_recording_error=bool(completed))
    result['physical_execution_verified']=all(result['full_sequence_physics_checks'].values())
    result['full_sequence_success']=result['physical_execution_verified']
    if stage=='execute':
        completed=bool(completed and result['full_sequence_success'])
        result['requested_stage_passed']=completed
    result['physical_execution_verification_scope']='Automated recorded native full-sequence checks only; no claim of final independent acceptance, multi-seed reliability or duration-matched neutral control.'
    result['accepted']=False
    result['acceptance_status']='pending_independent_review' if result['physical_execution_verified'] else 'not_accepted'
    if completed:result['status']='sequence_complete' if stage=='execute' else 'requested_stage_complete'
    elif result.get('status')!='failed':result['status']='checks_failed'
    return completed


class PropertyRecorder(TaskPhysicsRecorder):
    def capture(self):
        from pxr import UsdGeom
        cache=UsdGeom.XformCache();robot=self.robot
        body=np.concatenate((array(robot.data.body_pos_w)[0],array(robot.data.body_quat_w)[0]),axis=-1)
        palm=body[robot.body_names.index('base_link')]
        tcp=np.r_[palm[:3]+Rotation.from_quat(palm[3:]).apply([.131,0.,0.]),palm[3:]]
        states=np.stack([array(self.scene[name].data.root_state_w)[0] for name in self.object_names])
        row=dict(physics_step=int(self.e.sim.get_physics_step_count()),phase=self.phase,action_index=self.action_index,
            q=array(robot.data.joint_pos)[0],dq=array(robot.data.joint_vel)[0],planning_attachment=np.eye(4),has_planning_attachment=False,
            robot_body_pose_w=body,tcp_pose_w=tcp,task_object_states=states,
            robot_contact_force=array(self.scene['robot_contacts'].data.net_forces_w)[0],
            destination_contact_force=array(self.scene[self.task.contact_sensor_cfg.name].data.force_matrix_w)[0],
            background_rigid_poses=array(self.background_view.get_transforms()),background_rigid_velocities=array(self.background_view.get_velocities()))
        grip=self.e.action_manager.get_term('gripper_action')
        row.update(gripper_processed_target_rad=array(grip.processed_actions)[0],
            gripper_binary_requested_target_rad=array(getattr(grip,'_binary_requested_target',grip.processed_actions))[0],
            gripper_target_rate_limit_enabled=bool(getattr(grip,'glass_control_enabled',False)),
            gripper_verified_target_held=bool(getattr(grip,'grasp_target_held',False)),
            gripper_counter_contact_open_active=bool(getattr(grip,'counter_contact_open_active',False)))
        arm_action=self.e.action_manager.get_term('arm_action')
        row.update(arm_processed_target_rad=array(arm_action.processed_actions)[0],
            arm_requested_target_rad=array(getattr(arm_action,'_requested_position_target',arm_action.processed_actions))[0],
            arm_counter_contact_hold_active=bool(getattr(arm_action,'counter_contact_hold_active',False)))
        from .property_geometry import PropertyGlassWorld
        names=PropertyGlassWorld.GRIPPER_JOINT_NAMES
        row['gripper_observed_joint_positions_rad']=row['q'][[robot.joint_names.index(name) for name in names]].copy()
        for key,paths in (('static_anchor_transforms',self.static_paths),('task_reference_transforms',self.reference_paths)):
            row[key]=np.asarray([np.asarray(cache.GetLocalToWorldTransform(self.e.sim.stage.GetPrimAtPath(p))).T for p in paths]).reshape(-1,4,4)
        for index,(_,view) in enumerate(self.articulation_views):
            row[f'background_articulation_{index}_q']=array(view.get_dof_positions());row[f'background_articulation_{index}_dq']=array(view.get_dof_velocities())
        self.rows.append(row)
        self.contacts.sample(self.phase,self.action_index,states[self.object_names.index('fragile_glass'),:7],physics_step=row['physics_step'])
        return row


def neutral(args):
    import torch,carb
    from omni.physx.bindings._physx import SETTING_NUM_THREADS
    import omni.physx
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.dataset import CameraWriter
    from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts
    from kitchen_safety.extensions.replicator.overview import configure_overview
    from scripts.extensions.replicator.hard_execution_v1.property_collect import write
    torch.set_num_threads(args.torch_threads);carb.settings.get_settings().set_int(SETTING_NUM_THREADS,args.physics_threads)
    omni.physx.get_physx_interface().set_thread_count(args.physics_threads)
    config=read_config(args.config);out=args.output
    result=dict(mode='native_'+args.stage,status='running',accepted=False,physical_execution_verified=False,
        task_success=False,physics_state_setters_used=False,physical_weld_used=False,
        config_sha256=sha(args.config),input_sha256=config['input_sha256'],
        initial_neutral_duration_seconds=args.duration_seconds)
    env=recorder=None;writers=[];actions=[];supports=[];original_reset=None
    try:
        envdef=PropertyEnvironment().build(PropertyCfg(config_path=str(args.config)))
        builder=ArenaEnvBuilder(envdef,ArenaEnvBuilderCfg(num_envs=1,seed=args.seed,device=args.device,presets='physx',solve_relations=False))
        cfg,kwargs=builder.compose_manager_cfg();cfg.observations.camera_obs=None;cfg.sim.render_interval=cfg.decimation
        from .property_arm_actions import PropertyContactStopJointAction
        cfg.actions.arm_action.class_type=PropertyContactStopJointAction
        if 'glass_controller' in config:
            from .property_actions import PropertyRateLimitedBinaryAction
            cfg.actions.gripper_action.class_type=PropertyRateLimitedBinaryAction
            from .property_environment import ROOT
            result['glass_controller']=dict(config['glass_controller'],
                implementation='kitchen_safety/extensions/replicator/hard_execution_v1/property_actions.py',
                source_sha256=sha(ROOT/'kitchen_safety/extensions/replicator/hard_execution_v1/property_actions.py'),
                source_gripper_action_sha256=sha(ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/actions.py'),
                physical_asset_or_actuator_changes=False,records='physics_states.npz gripper_* fields at every physics substep')
        cfg.scene.robot_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*',update_period=0.,history_length=1)
        result['overview']=configure_overview(cfg.scene,eye=(1.85,1.2,1.65),target=(2.60,.30,1.0))
        env=builder.make_registered(cfg,kwargs);env.reset();e=env.unwrapped;scene=e.scene;robot=scene['robot']
        from pxr import Sdf,UsdGeom
        from .property_environment import ROOT
        marker=UsdGeom.Xform.Define(e.sim.stage,'/World/envs/env_0/HardGlassSafePlacement')
        marker.GetPrim().GetReferences().AddReference(str(ROOT/config['accepted_scene']),Sdf.Path('/World/hard/GlassSafePlacement'))
        result['safe_region_visual']=dict(source=config['accepted_scene'],prim='/World/hard/GlassSafePlacement',collision=False)
        contacts=ObjectGraspContacts(scene['robot_contacts']._physics_sim_view,e.physics_dt,'fragile_glass')
        recorder=PropertyRecorder(e,envdef,contacts)
        from .property_geometry import PropertyGlassWorld
        recorder.metadata['gripper_observed_joint_names']=list(PropertyGlassWorld.GRIPPER_JOINT_NAMES)
        recorder.metadata['gripper_controller_fields']=dict(
            gripper_observed_joint_positions_rad='Six actual measured joint positions, never commanded or mimic-filled',
            gripper_processed_target_rad='Actual position actuator target applied for this physics step',
            gripper_binary_requested_target_rad='Original 0/pi4 command before local target-rate limiter',
            gripper_target_rate_limit_enabled='Property closure controller enabled',
            gripper_verified_target_held='Verified cup grasp target held until binary open')
        recorder.metadata['contact_stop_controller_fields']=dict(
            arm_processed_target_rad='Actual arm actuator position target applied in this physics substep',
            arm_requested_target_rad='Original arm action target before counter contact stop',
            arm_counter_contact_hold_active='Measured q latched as target after observed intended cup-bottom contact',
            gripper_counter_contact_open_active='Original binary open target commanded after intended counter contact',
            event_timing='Contact is observed after physics substep N; target stop/open applies from substep N+1')
        result['counter_contact_stop_controller']=dict(
            configuration=config.get('counter_contact_controller',{}),
            source_sha256=sha(ROOT/'kitchen_safety/extensions/replicator/hard_execution_v1/property_arm_actions.py'),
            upstream_joint_action_sha256=sha(ROOT/'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/envs/mdp/actions/joint_actions.py'),
            object_or_joint_state_writes=False,normal_action_decimation_preserved=True)
        root='/World/envs/env_0';support_view=scene['robot_contacts']._physics_sim_view.create_rigid_contact_view(
            [root+'/fragile_glass'],filter_patterns=[[root+'/bowl',root+'/kitchen/CapSurface_02']],max_contact_data_count=4096)
        result['support_contact_identity']=dict(subject=root+'/fragile_glass',filters=[root+'/bowl',root+'/kitchen/CapSurface_02'])
        support_offsets={}
        for name in ('fragile_glass','bowl'):
            support_offsets[name]={}
            for getter in ('get_contact_offsets','get_rest_offsets'):
                values=array(getattr(scene[name].root_view,getter)())
                if not values.size or not np.isfinite(values).all():raise RuntimeError('Missing actual native '+name+' '+getter)
                support_offsets[name][getter]=values.tolist()
        contact_bound=sum(float(np.max(support_offsets[name]['get_contact_offsets'])) for name in ('fragile_glass','bowl'))
        if not 0<contact_bound<.10:raise RuntimeError('Unexpected actual cup/bowl contact envelope')
        result['support_native_shape_offsets_m']=support_offsets
        result['cup_bowl_contact_envelope_bound_m']=contact_bound
        def sample(row):
            raw=list(map(array,support_view.get_contact_data(dt=e.physics_dt)))
            end=int(np.max(raw[4]+raw[5]))
            if end>=4096:raise RuntimeError('Glass support contact buffer reached capacity')
            supports.append(dict(physics_step=row['physics_step'],phase=row['phase'],normal_forces_n=raw[0][:end],
                points_world_m=raw[1][:end],normals_world=raw[2][:end],separations_m=raw[3][:end],counts=raw[4],starts=raw[5],
                cup_bowl_native_contact_envelope_m=contact_bound,
                filtered_force_matrix_n=array(support_view.get_contact_force_matrix(dt=e.physics_dt))))
        recorder.on_sample=sample;sample(recorder.rows[-1])
        original_reset=e._reset_idx
        def no_autoreset(ids):
            if len(ids):raise RuntimeError('Unexpected source task termination during neutral observation; no autoreset performed')
        e._reset_idx=no_autoreset
        names=list(envdef.embodiment.camera_config.camera_names())
        if len(names)!=3:raise ValueError('Original three DROID cameras missing')
        main=CameraWriter(out,scene,names,e.step_dt,previews=True);writers.append(main)
        (out/'overview').mkdir();overview=CameraWriter(out/'overview',scene,['overview_camera'],e.step_dt,previews=True);writers.append(overview)
        result['original_camera_names']=names
        result['appearance_warmup']=main.warm_up(e.sim,min_frames=32,max_frames=128,sample_every=8,stable_windows=2)
        if int(e.sim.get_physics_step_count())!=recorder.reset_step:raise RuntimeError('Camera warmup advanced unrecorded physics')
        arm=[robot.joint_names.index(f'panda_joint{i}') for i in range(1,8)];q=recorder.rows[-1]['q'][arm].copy()
        def step(command_arm,grip,phase):
            index=len(actions)
            if index>=3000:raise RuntimeError('Bounded action count reached')
            recorder.phase=phase;recorder.action_index=index
            before=int(e.sim.get_physics_step_count());main.write(phase);overview.write(phase);after=int(e.sim.get_physics_step_count())
            if before!=after:raise RuntimeError('Camera acquisition advanced physics')
            command=np.r_[command_arm,grip].astype(np.float32)
            action=dict(phase=phase,action=command,camera_physics_steps=[before,after],before_physics_step=before,after_physics_step=None)
            actions.append(action)
            try:env.step(torch.tensor(command[None],device=e.device))
            finally:action['after_physics_step']=int(e.sim.get_physics_step_count())
        for index in range(int(np.ceil(args.duration_seconds/e.step_dt))):
            phase='initial_settle' if index*e.step_dt<5 else 'neutral_hold'
            step(q,0.,phase)
        if args.stage in ('glass-grasp','glass-transfer','execute'):
            from .property_grasp import run
            run(args,envdef,e,recorder,contacts,supports,step,result,actions)
        result['observation_complete']=True;result['status']='observed'
    except Exception as error:result.update(status='failed',error=f'{type(error).__name__}: {error}')
    finally:
        for writer in writers:
            try:writer.close()
            except Exception as error:result.setdefault('close_errors',[]).append(repr(error))
        if recorder is not None:
            recorder.close()
            try:result['physical_observation_audit']=recorder.write(out,actions)
            except Exception as error:result['recording_error']=repr(error)
            states=np.asarray([r['task_object_states'] for r in recorder.rows]);steps=np.asarray([r['physics_step'] for r in recorder.rows])
            result['action_count']=len(actions)
            result['commanded_action_duration_seconds']=len(actions)*recorder.e.step_dt
            result['observed_physics_duration_seconds']=float((steps[-1]-steps[0])*recorder.e.physics_dt) if len(steps) else 0.
            result['duration_seconds']=result['observed_physics_duration_seconds']
            result['duration_seconds_semantics']='Observed recorded physics span; commanded action duration may include an interrupted final action.'
            elapsed=(steps-steps[0])*recorder.e.physics_dt;settled=np.flatnonzero(elapsed>=5.)
            if len(settled):
                neutral_mask=np.array([r['phase'] in ('reset','initial_settle','neutral_hold') for r in recorder.rows])
                neutral_states=states[neutral_mask];neutral_elapsed=elapsed[neutral_mask];neutral_settled=np.flatnonzero(neutral_elapsed>=5.)
                stability={name:dict(from_reset=pose_change(neutral_states[:,i,:7]),after_5_seconds=pose_change(neutral_states[:,i,:7],int(neutral_settled[0]))) for i,name in enumerate(recorder.object_names)}
                result['object_stability']=stability
                result['settled_objects_within_declared_tolerance']=all(
                    s['after_5_seconds']['max_translation_m']<=config['neutral'][('glass' if name=='fragile_glass' else 'bowl')+'_translation_tolerance_m'] and
                    s['after_5_seconds']['max_rotation_deg']<=config['neutral'][('glass' if name=='fragile_glass' else 'bowl')+'_rotation_tolerance_deg']
                    for name,s in stability.items())
            result['native_support_samples']=len(supports)
        write(out/'native_glass_support_contacts.json',dict(identity=result.get('support_contact_identity'),rows=supports))
        write(out/'actions.json',actions)
        if env is not None:
            if original_reset is not None:env.unwrapped._reset_idx=original_reset
            try:env.close()
            except Exception as error:result.setdefault('close_errors',[]).append(repr(error))
        result['neutral_observation_passed']=bool(result.get('observation_complete') and result.get('settled_objects_within_declared_tolerance') and result.get('physical_observation_audit',{}).get('passed') and not result.get('recording_error'))
        complete=finalize_outcome(result,args.stage)
        write(out/'result.json',result)
    return 0 if complete else 2
