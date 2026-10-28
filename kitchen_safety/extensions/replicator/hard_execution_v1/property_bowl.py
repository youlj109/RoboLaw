"""Same reviewed bowl sequence resumed after observed safe glass release.

The move and bowl phase code is a local copy of bowl_supported_collect.py;
its source/rim/destination bindings, thresholds and profile are unchanged.
No reset, object state write or simulation attachment is introduced.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.hard_execution_v1.property_environment import ROOT,sha
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from kitchen_safety.extensions.replicator.startup_recorder import array


def run(args,envdef,e,recorder,step,actions,result):
    from isaaclab.managers import SceneEntityCfg
    from isaaclab_arena.tasks.predicates.spatial import object_on_destination
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.bowl_target_support import SupportedDestinationWorld,NativeTargetContacts,read_policy,release_boundary
    from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
    from kitchen_safety.extensions.replicator.bowl_transport_binding import lift_motion_evidence,observe_transport_binding,apply_transport_binding,transport_target
    from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts
    from kitchen_safety.extensions.replicator.bowl_controller_profile import read_profile,require_grasp_profile,grasp_target,sink_target
    from kitchen_safety.extensions.replicator.task_collection_evidence import TaskPredicateRecorder
    from scripts.extensions.replicator.hard_execution_v1.property_collect import write as json_write
    config=envdef.hard_execution_config;layout=envdef.resolved_task_layout;out=args.output;robot=e.scene['robot'];task=envdef.task
    profile=read_profile(ROOT/config['bowl_controller_profile'],ROOT/config['source_layout']);require_grasp_profile(profile)
    support_policy=read_policy(ROOT/config['bowl_support_policy'])
    ids=[robot.joint_names.index(f'panda_joint{i}') for i in range(1,8)]
    finger_ids=[robot.joint_names.index(n) for n in SupportedDestinationWorld.GRIPPER_JOINT_NAMES]
    payload_index=recorder.object_names.index('bowl');glass_index=recorder.object_names.index('fragile_glass')
    kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
    world=SupportedDestinationWorld(e.sim.stage,kin,payload_usd=ROOT/layout['objects']['bowl']['usd_path'],env_root='/World/envs/env_0',clearances={},execution_clearances={},default_clearance=.002)
    contacts=ObjectGraspContacts(e.scene['robot_contacts']._physics_sim_view,e.physics_dt,'bowl')
    predicates=TaskPredicateRecorder(e,envdef)
    predicates.metadata['scope']+=' Recording begins at verified glass-to-bowl handoff; this is not another reset.'
    physical_step=step
    def step(q,grip,phase):
        physical_step(q,grip,phase)
        predicates.sample_action(phase,recorder.action_index)
    native_target=NativeTargetContacts(e.scene['robot_contacts']._physics_sim_view,e.physics_dt,predicates.metadata['sensor_identity'],e.scene['bowl'].root_view)
    result['plans']=[];result['bowl_sequence_controller']='Local preserved bowl_supported_collect sequence'
    result['bowl_support_policy']=support_policy
    terminal={};transport_context=None
    old_callback=recorder.on_sample;old_reset=e._reset_idx
    glass_reference=recorder.rows[-1]['task_object_states'][glass_index,:7].copy()
    def sync_world(row):
        world.set_phase(row['phase']);world.gripper_joint_positions=row['q'][finger_ids].copy()
        world.gripper_joint_position=float(world.gripper_joint_positions[0]);world.payload_pose=pose_matrix(row['task_object_states'][payload_index,:7])
        world.sync_background(recorder.background_paths,row['background_rigid_poses'])
        for name,index in (('bowl',payload_index),('fragile_glass',glass_index)):world.set_obstacle_pose(name,row['task_object_states'][index,:7])
    def observe(row):
        old_callback(row)
        contacts.sample(row['phase'],row['action_index'],row['task_object_states'][payload_index,:7],physics_step=row['physics_step'])
        predicates.sample_physics(row['phase'],row['action_index']);sync_world(row);world.observe(row,native_target.sample(row))
        row['has_planning_attachment']=world.attachment is not None
        if world.attachment is not None:row['planning_attachment']=world.attachment.copy()
        if not world.valid(row['q'][ids],mode='execution',gripper_q=row['q'][finger_ids],object_pose=row['task_object_states'][payload_index,:7]):
            result['bowl_safety_failure']=dict(physics_step=row['physics_step'],phase=row['phase'],**world.last_clearance)
            raise RuntimeError('Actual bowl safety check failed: '+str(world.last_clearance))
        cup=row['task_object_states'][glass_index,:7]
        if np.linalg.norm(cup[:3]-glass_reference[:3])>.005 or (Rotation.from_quat(cup[3:])*Rotation.from_quat(glass_reference[3:]).inv()).magnitude()>np.deg2rad(2):
            raise RuntimeError('Previously released fragile glass moved during bowl task')
    def keep_terminal_state(env_ids):
        if len(env_ids):
            terms={name:bool(array(e.termination_manager.get_term(name))[0]) for name in e.termination_manager.active_terms}
            entry=dict(terms=terms,physics_step=int(e.sim.get_physics_step_count()),phase=recorder.phase,action_index=recorder.action_index)
            if terms.get('success') and 'first_success' not in terminal:
                terminal['first_success']=entry
                np.savez_compressed(out/'first_source_success_state.npz',**recorder.rows[-1])
            if any(value for name,value in terms.items() if name!='success'):
                terminal['failure']=entry;raise RuntimeError('Original source task failure: '+str(terms))
    def hold(q,gripper,phase,seconds):
        for _ in range(max(1,int(np.ceil(seconds/e.step_dt)))):step(q,gripper,phase)
    recorder.on_sample=observe;e._reset_idx=keep_terminal_state
    initial=dict(recorder.rows[-1]);initial['phase']='initial_settle';recorder.phase='initial_settle'
    # Record current contacts once without duplicating the outer support stream.
    contacts.sample('initial_settle',recorder.action_index,initial['task_object_states'][payload_index,:7],physics_step=initial['physics_step'])
    sync_world(initial);world.observe(initial,native_target.sample(initial))
    predicates.sample_physics('initial_settle',recorder.action_index)
    try:
        def move(target, gripper, phase):
            recorder.phase = phase
            current = recorder.rows[-1]
            sync_world(current)
            world.set_phase(phase)
            # Rebuild from measured background and destination every segment;
            # never reuse a stale world after dynamic objects have moved.
            planner = BowlTransportCuroboPlanner(world, kin, dt=e.step_dt, seed=args.seed, max_joint_speed=.4)
            plan_record = dict(phase=phase, start_physics_step=current["physics_step"], target_tcp=target,
                               measured_planning_attachment=world.attachment, status="requested")
            result["plans"].append(plan_record)
            action_start = len(actions)
            try:
                if phase == 'lift':
                    plan_record['source_departure_binding'] = planner.bind_source_departure(
                        physics_step=current['physics_step'], arm_q=current['q'][ids],
                        measured_tcp=pose_matrix(current['tcp_pose_w']),
                        measured_payload=pose_matrix(current['task_object_states'][payload_index, :7]),
                        raw_contact_rows=contacts.records)
                elif phase == 'transport':
                    plan_record['transport_binding'] = planner.bind_transport(profile, transport_context)
                elif phase == 'lower':
                    plan_record['sink_lowering_binding'] = planner.bind_sink_lowering(profile,
                        physics_step=current['physics_step'], arm_q=current['q'][ids],
                        measured_tcp=pose_matrix(current['tcp_pose_w']),
                        measured_payload=pose_matrix(current['task_object_states'][payload_index, :7]),
                        raw_contact_rows=contacts.records)
                if phase == 'retreat':
                    plan_record['observed_target_support_certificate'] = world.issue_planning_certificate()
                plan = planner.plan_pose(current["q"][ids], target, grip_binary=gripper, phase=phase)
                positions = np.asarray(plan["positions"])
                if len(positions) < 2 or positions.shape[1:] != (7,) or not np.isfinite(positions).all():
                    raise RuntimeError("Planner returned an empty/invalid path")
                plan_record.update(metadata=plan["metadata"], candidate_contact_mask=planner.candidate_contact_mask,
                                   payload_geometry=getattr(planner, "payload_geometry_report", None))
                np.savez_compressed(out/f"plan_{len(result['plans']):02d}_{phase}.npz", positions=positions,
                                    goal_tcp=target, start_q=current["q"][ids])
                validation = planner.validate_path(positions, grip_binary=gripper, phase=phase, resolution=.02)
                plan_record["validation"] = validation
                if phase == 'transport':
                    np.savez_compressed(out/'transport_dense_validation.npz',
                        positions=planner.last_transport_dense_samples,
                        actual_six_fingers=current['q'][finger_ids], measured_attachment=world.attachment)
                if not validation["valid"]:
                    raise RuntimeError("Dense cuRobo + full-pair FCL validation rejected path")
                # The inherited validator supplies no observed free-object pose;
                # world.payload_pose and the measured fingers provide that state.
                plan_record["status"] = "validated"
                for command in positions[1:]:
                    step(command, gripper, phase)
                if phase == 'lower':
                    # Execute the complete original path. Only its redundant
                    # hold may stop at an already legal observed action boundary.
                    boundaries = []
                    for hold_index in range(max(1, int(np.ceil(.3/e.step_dt)))+1):
                        row = recorder.rows[-1]
                        source = world.source_floor_observation(pose_matrix(row['task_object_states'][payload_index,:7]))
                        landing_now, _ = sink_target(row['task_object_states'][payload_index,:7], world, profile)
                        actual_pose = pose_matrix(row['task_object_states'][payload_index,:7])
                        xy = float(np.linalg.norm(landing_now[:2,3]-actual_pose[:2,3]))
                        summary = contacts.summarize(*grasp_info['contact_region_local_m'], phase='lower', last_steps=10)
                        boundary = release_boundary(source, True, xy, summary, int(row['physics_step']))
                        boundaries.append(boundary)
                        if boundary['passed']:
                            plan_record['release_boundary'] = boundary
                            break
                        if hold_index == max(1, int(np.ceil(.3/e.step_dt))):
                            raise RuntimeError('No original legal source gap/XY/bilateral release boundary after fixed lower hold')
                        step(positions[-1], gripper, phase)
                    plan_record['lower_boundary_candidates'] = boundaries
                else:
                    hold(positions[-1], gripper, phase, .3)
                plan_record["status"] = "executed"
                return positions[-1].copy()
            except Exception as error:
                plan_record.update(status="failed", error=repr(error))
                if hasattr(planner, 'last_source_support_validation'):
                    plan_record['source_support_validation'] = planner.last_source_support_validation
                if hasattr(planner, 'last_destination_validation'):
                    plan_record['destination_validation'] = planner.last_destination_validation
                if hasattr(planner, 'last_transport_validation'):
                    plan_record['transport_validation'] = planner.last_transport_validation
                candidate = getattr(planner, "last_cartesian_candidate", None)
                if candidate is not None:
                    observed_actions = [row for row in actions[action_start:]
                        if row["after_physics_step"] is not None and row["after_physics_step"] > row["before_physics_step"]]
                    filename = f"rejected_plan_{len(result['plans']):02d}_{phase}.npz"
                    np.savez_compressed(out/filename, **{key: candidate[key] for key in
                        ("positions", "ik_waypoint_positions", "poses", "effective_goal_pose")},
                        start_q=current["q"][ids], requested_target=target,
                        measured_gripper_angles=current["q"][finger_ids])
                    plan_record.update(rejected_candidate_file=filename,
                        rejected_candidate_sha256=sha(out/filename),
                        candidate_constraints=getattr(planner, "last_constraint_diagnostics", None),
                        candidate_contact_mask=getattr(planner, "candidate_contact_mask", None),
                        rejected_candidate_executed=bool(observed_actions),
                        observed_candidate_actions=len(observed_actions),
                        observed_candidate_physics_steps=sum(row["after_physics_step"]-row["before_physics_step"]
                            for row in observed_actions),
                        failure_stage="execution" if observed_actions else "planning_or_validation")
                raise
    
        target, grasp_info = grasp_target(initial["task_object_states"][payload_index, :7], world, profile)
        result["grasp_target"] = grasp_info
        pregrasp = target.copy()
        pregrasp[2, 3] += profile['pregrasp_height_m']
        q = move(pregrasp, 0., "pregrasp")
        q = move(target, 0., "approach")
        hold(q, 1., "close", 2.)
        region = grasp_info["contact_region_local_m"]
        result["close_contacts"] = contacts.summarize(*region, phase="close", last_steps=10)
        if not (result["close_contacts"]["both_fingers_loaded"] and result["close_contacts"]["both_fingers_near"]):
            raise RuntimeError("No simultaneous source upper-wall/rim bilateral physical grasp")
        actual = recorder.rows[-1]
        measured_tcp = pose_matrix(actual["tcp_pose_w"])
        world.attachment = np.linalg.inv(measured_tcp) @ pose_matrix(actual["task_object_states"][payload_index, :7])
        result["measured_tcp_to_bowl"] = world.attachment.copy()
        lift = measured_tcp.copy()
        lift[2, 3] += profile['lift_height_m']
        q = move(lift, 1., "lift")
        result["lift_contacts"] = contacts.summarize(*region, phase="lift", last_steps=10)
        result["observed_lift_m"] = float(recorder.rows[-1]["task_object_states"][payload_index, 2]-initial["task_object_states"][payload_index, 2])
        if (result["observed_lift_m"] < .05 or not result["lift_contacts"]["both_fingers_loaded"] or
            not result["lift_contacts"]["both_fingers_near"]):
            raise RuntimeError("Held source bowl did not physically lift with bilateral upper-wall/rim contacts")
        state = recorder.rows[-1]
        sync_world(state)
        # Preserve every actual close-to-lift relative pose, including slip.
        # Latest loaded contacts authorize re-observation, never a rigidity
        # claim or a physical constraint. The new planner rebuilds payload
        # spheres and generates/checks a fresh complete transport trajectory.
        lift_motion = lift_motion_evidence(recorder.rows, payload_index)
        json_write(out/'lift_attachment_motion.json', lift_motion)
        result['lift_attachment_motion'] = {key: value for key, value in lift_motion.items() if key != 'samples'}
        result['lift_attachment_motion']['file_sha256'] = sha(out/'lift_attachment_motion.json')
        transport_context = observe_transport_binding(world, kin, profile,
            physics_step=state['physics_step'], arm_q=state['q'][ids],
            measured_tcp=pose_matrix(state['tcp_pose_w']),
            measured_payload=pose_matrix(state['task_object_states'][payload_index, :7]),
            raw_contact_rows=contacts.records, lift_motion=lift_motion)
        result['observed_transport_binding'] = apply_transport_binding(world, profile, transport_context)
        world.set_phase('transport')
        above, transport_info = transport_target(world, profile, transport_context)
        result['source_sink_landing'] = transport_info['original_sink_landing']
        result['fresh_transport_target'] = transport_info
        q = move(above, 1., "transport")
        state = recorder.rows[-1]
        landing, lower_info = sink_target(state["task_object_states"][payload_index, :7], world, profile)
        result["source_supported_lower_target"] = lower_info
        current_tcp = pose_matrix(state["tcp_pose_w"])
        # Rebind planning geometry to actual post-transport observations. This
        # never welds or repositions the physical bowl, and the latest complete
        # bilateral transport contact window is checked by bind_sink_lowering.
        world.attachment = np.linalg.inv(current_tcp) @ pose_matrix(state["task_object_states"][payload_index, :7])
        result["measured_tcp_to_bowl_before_lower"] = world.attachment.copy()
        requested_tcp = landing @ np.linalg.inv(world.attachment)
        xy_error = float(np.linalg.norm(requested_tcp[:2, 3]-current_tcp[:2, 3]))
        rotation_error = float(np.rad2deg(Rotation.from_matrix(
            requested_tcp[:3, :3]@current_tcp[:3, :3].T).magnitude()))
        result["lower_alignment"] = dict(target_xy_error_m=xy_error, target_rotation_error_deg=rotation_error)
        if xy_error > .005 or rotation_error > 2.:
            raise RuntimeError("Actual carried bowl missed selected sink landing column/orientation")
        lower = current_tcp.copy()
        lower[2, 3] = requested_tcp[2, 3]
        q = move(lower, 1., "lower")
        # Planning attachment is removed at opening; the real bowl is never
        # constrained or repositioned. Every free-object pose is still checked.
        world.attachment = None
        hold(q, 0., "release", support_policy["release_seconds"])
        result["released_finger_joint_rad"] = float(recorder.rows[-1]["q"][finger_ids[0]])
        retreat = pose_matrix(recorder.rows[-1]["tcp_pose_w"])
        retreat[2, 3] += support_policy["retreat_height_m"]
        q = move(retreat, 0., "retreat")
        hold(q, 0., "final_settle", support_policy["final_settle_seconds"])
        result["original_task_success"] = "first_success" in terminal
        result["final_original_destination_predicate"] = bool(array(object_on_destination(e,
            object_cfg=SceneEntityCfg("bowl"), destination_cfg=SceneEntityCfg("sink"),
            contact_sensor_cfg=task.contact_sensor_cfg, force_threshold=task.force_threshold,
            velocity_threshold=task.velocity_threshold, support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
        result["physical_sequence_complete"] = True
        result["status"] = "sequence_complete"
    finally:
        recorder.on_sample=old_callback;e._reset_idx=old_reset
        result['bowl_terminal']=terminal
        try:result['bowl_predicate_evidence']=predicates.finish(out)
        except Exception as error:result['bowl_predicate_recording_error']=repr(error)
        json_write(out/'bowl_grasp_contacts.json',contacts.records)
        json_write(out/'bowl_target_native_contacts.json',dict(identity=native_target.identity,offsets=native_target.offsets,rows=native_target.rows))
        json_write(out/'bowl_target_support_observations.json',dict(policy=support_policy,records=world.support_records,certificates=world.support_certificates))
        json_write(out/'bowl_target_source_intersection_ledger.json',world.source_intersection_ledger)
