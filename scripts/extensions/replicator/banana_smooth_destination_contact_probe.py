#!/usr/bin/env python3
"""Independent bounded banana/bowl source/native contact diagnostic.

Never an accepted dataset sample. Retains original grasp/transport/target,
adds actual >=2mm lower guard and permits only declared original banana/bowl
support observations for exactly 30 release actions. Original strict rejects,
raw predicate and every actual substep/video remain. --dry-run is CPU only.
"""
import argparse
import json
from pathlib import Path
import time

TASK = "kitchen_bench_replicator_g_shape_banana_bowl"
ROOT = Path(__file__).resolve().parents[3]


def json_write(path, value):
    def convert(value):
        if hasattr(value, "tolist"):
            return value.tolist()
        if isinstance(value, Path):
            return str(value)
        raise TypeError(type(value).__name__)
    Path(path).write_text(json.dumps(value, indent=2, default=convert, allow_nan=False)+"\n")


def preflight(args):
    from kitchen_safety.extensions.replicator.task_environment import resolve_task_layout
    from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
    from kitchen_safety.extensions.replicator.task_contact_policy import BananaContactPolicy
    from kitchen_safety.extensions.replicator.task_motion import banana_midbody
    from scripts.extensions.replicator.audit_task_source_relation import check_layout
    from kitchen_safety.extensions.replicator.banana_controller_profile import read_profile, PROFILE
    from kitchen_safety.extensions.replicator.banana_halfcenter_candidate import read_candidate, CANDIDATE
    from kitchen_safety.extensions.replicator.banana_transport_height import read_candidate as read_transport_candidate, CANDIDATE as TRANSPORT_CANDIDATE
    from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import read_declaration
    declaration=read_declaration(args.probe_declaration)
    layout = resolve_task_layout(args.task_id)
    profile = read_profile(getattr(args, 'controller_profile', PROFILE))
    candidate = read_candidate(getattr(args, 'grasp_candidate_profile', CANDIDATE), getattr(args, 'controller_profile', PROFILE))
    transport_candidate = read_transport_candidate(getattr(args, 'transport_candidate_profile', TRANSPORT_CANDIDATE),
        getattr(args, 'controller_profile', PROFILE))
    geometry = PayloadGeometry(ROOT/layout["objects"]["banana"]["usd_path"])
    audit = check_layout(layout)
    return dict(task_id=args.task_id, cpu_preflight_passed=audit["initial_source_relations_passed"],
        physics_launched=False, execution_verified=False, diagnostic_only=True, accepted=False, physical_trial_passed=False, task_success=False, declaration=declaration, layout=layout, source_relations=audit,
        grasp_geometry=banana_midbody(geometry), contact_policy=BananaContactPolicy().report(), controller_profile=profile,
        grasp_candidate_profile=candidate,
        transport_candidate_profile=transport_candidate,
        physical_phases=["warmup", "initial_settle", "pregrasp", "approach", "close", "lift", "transport",
                         "lower", "bounded_release_observation"],
        pending=["Root-scheduled actual diagnostic and all original checks",
                 "Independent native/source contact review; diagnostic is never eligible for acceptance"])


def run(args, provenance):
    import numpy as np
    import torch
    if args.torch_threads is not None:
        torch.set_num_threads(args.torch_threads)
    import carb
    if args.physics_threads is not None:
        import omni.physx
        from omni.physx.bindings._physx import SETTING_NUM_THREADS
        carb.settings.get_settings().set_int(SETTING_NUM_THREADS, args.physics_threads)
        omni.physx.get_physx_interface().set_thread_count(args.physics_threads)
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.managers import SceneEntityCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.tasks.predicates.spatial import object_on_destination
    from kitchen_safety.dataset import CameraWriter, decode_video
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.task_environment import AssignedTaskCfg, AssignedTaskEnvironment
    from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
    from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import (DiagnosticDestinationWorld as PayloadCollisionWorld,
        DestinationNativeContacts, read_declaration, release_boundary, audit_declaration_archive)
    from kitchen_safety.extensions.replicator.source_support_curobo import SourceSupportCuroboPlanner
    from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts
    from kitchen_safety.extensions.replicator.task_recorder import TaskPhysicsRecorder
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix, banana_bowl_lower_target
    from kitchen_safety.extensions.replicator.banana_controller_profile import read_profile
    from kitchen_safety.extensions.replicator.banana_halfcenter_candidate import read_candidate, grasp_target
    from kitchen_safety.extensions.replicator.banana_transport_height import read_candidate as read_transport_candidate, transport_target
    from kitchen_safety.extensions.replicator.task_collection_evidence import TaskPredicateRecorder
    from kitchen_safety.extensions.replicator.startup_recorder import array, sha
    from kitchen_safety.extensions.replicator.overview import configure_overview
    from kitchen_safety.extensions.replicator.banana_release_schedule_candidate import ReleaseSchedule, original_binary_target, INTERFACE
    from kitchen_safety.extensions.replicator.banana_command_recording import CommandRecorder, JointBounds
    from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg

    declaration=read_declaration(args.probe_declaration)
    started = time.perf_counter()
    out = args.output
    profile = read_profile(args.controller_profile)
    candidate = read_candidate(args.grasp_candidate_profile, args.controller_profile)
    transport_candidate = read_transport_candidate(args.transport_candidate_profile, args.controller_profile)
    result = dict(task_id=TASK, variant="base", seed=args.seed, physical_trial_passed=False, accepted=False,
        fixed_only=True, physics_state_setters_used=False, physical_weld_used=False,
        planning_attachment_only=True, original_task_success=False, task_success=False, diagnostic_only=True, diagnostic_completed=False, declaration=declaration, diagnostic_declaration_binding=provenance.diagnostic_declaration_binding, final_original_destination_predicate=False,
        status="running", diagnostic_limits=["Never eligible for accepted data or task success",
            "All original strict destination rejections retained", "At most 240 release physics steps"],
        plans=[], safety=dict(checked_physics_samples=0, failure=None), source_hashes={"collector": sha(__file__)},
        controller_profile=profile, controller_profile_sha256=sha(args.controller_profile),
        grasp_candidate_profile=candidate, grasp_candidate_binding=provenance.grasp_candidate_binding,
        transport_candidate_profile=transport_candidate, transport_candidate_binding=provenance.transport_candidate_binding)
    result["runtime_threads"] = dict(requested_physics_threads=args.physics_threads,
        requested_torch_threads=args.torch_threads,
        live_physics_threads=carb.settings.get_settings().get("/persistent/physics/numThreads"),
        live_torch_threads=torch.get_num_threads(), requested_physics_device=args.device)
    env, recorder, cameras, overview_writer, original_reset = None, None, None, None, None
    predicates = native_destination = command_trace = bounds_audit = None
    actions, terminal = [], {}
    world = None
    try:
        envdef = AssignedTaskEnvironment().build(AssignedTaskCfg(task_id=TASK, variant="base", enable_cameras=True,
            episode_length_s=args.episode_length_s))
        envdef.embodiment.action_config.gripper_action=JointPositionActionCfg(asset_name="robot",joint_names=["finger_joint"],scale=1.,offset=0.,use_default_offset=False,clip=None)
        result["action_interface"]=INTERFACE
        provenance.bind_environment(envdef)
        builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=args.seed, device=args.device,
            presets="physx", solve_relations=False))
        cfg, kwargs = builder.compose_manager_cfg()
        cfg.observations.camera_obs = None
        cfg.sim.render_interval = cfg.decimation
        cfg.scene.robot_contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", update_period=0., history_length=1)
        overview = configure_overview(cfg.scene, eye=(-.8, .3, 2.2), target=(-2.05, 2.1, .95))
        env = builder.make_registered(cfg, kwargs)
        env.reset()
        e, scene, robot = env.unwrapped, env.unwrapped.scene, env.unwrapped.scene["robot"]
        result["runtime_threads"]["actual_environment_device"] = str(e.device)
        predicates = TaskPredicateRecorder(e, envdef)
        contacts = ObjectGraspContacts(scene["robot_contacts"]._physics_sim_view, e.physics_dt, "banana")
        recorder = TaskPhysicsRecorder(e, envdef, contacts)
        recorder.metadata["grasp_candidate_binding"] = dict(provenance.grasp_candidate_binding)
        recorder.metadata["transport_candidate_binding"] = dict(provenance.transport_candidate_binding)
        recorder.metadata['diagnostic_declaration_binding']=dict(provenance.diagnostic_declaration_binding)
        recorder.metadata['action_interface']=INTERFACE
        recorder.metadata['action_joint_names']=[f'panda_joint{i}' for i in range(1,8)]+['finger_joint']
        expected_contacts = {name for name in robot.body_names if name.startswith("panda_link") or name == "base_link"}
        if not expected_contacts <= set(scene["robot_contacts"].body_names):
            raise RuntimeError("Arm/palm contact bodies are missing from the actual contact sensor")
        layout, task = envdef.resolved_task_layout, envdef.task
        result.update(source_relations=envdef.source_relation_audit, layout=layout, overview=overview)
        json_write(out/"source_relation_audit.json", envdef.source_relation_audit)
        ids = [robot.joint_names.index(f"panda_joint{i}") for i in range(1, 8)]
        finger_ids = [robot.joint_names.index(name) for name in PayloadCollisionWorld.GRIPPER_JOINT_NAMES]
        banana_index, bowl_index = (recorder.object_names.index(name) for name in ("banana", "bowl"))
        base = np.asarray(layout["robot_position"]).copy()
        base[2] += layout["stand_height"]
        kin = DroidKinematics(base, layout["robot_yaw_deg"])
        world = PayloadCollisionWorld(e.sim.stage, kin, payload_name="banana",
            payload_usd=ROOT/layout["objects"]["banana"]["usd_path"], env_root="/World/envs/env_0",
            clearances={}, execution_clearances={}, default_clearance=.002)
        bowl_geometry = PayloadGeometry(ROOT/layout["objects"]["bowl"]["usd_path"])
        result["contact_policy"] = world.contact_policy.report()
        world.gripper_joint_positions = recorder.rows[-1]["q"][finger_ids]
        if (task.pick_up_object.name!='banana' or task.destination_location.name!='bowl' or
            task.force_threshold!=.1 or task.velocity_threshold!=.003 or
            task.support_cone_half_angle_rad!=declaration['support_cone_half_angle_rad'] or task.placement_consecutive_steps!=1):
            raise ValueError('Diagnostic must retain exact original task roles/predicate')
        if e.physics_dt!=declaration['physics_dt_s'] or e.step_dt!=declaration['action_dt_s'] or cfg.decimation!=8:
            raise ValueError('Diagnostic requires original 120Hz physics / 15Hz actions')
        world.bind_spawned_bounds(predicates.metadata['actual_spawned_local_bounds'])
        native_destination=DestinationNativeContacts(scene['robot_contacts']._physics_sim_view,e.physics_dt,
            predicates.metadata['sensor_identity'],scene['banana'].root_view,scene['bowl'].root_view)
        result['native_destination_identity']=native_destination.identity
        result['native_collision_offsets']=native_destination.offsets
        result['source_destination_binding']=world.probe_source_binding
        bounds_audit=JointBounds(robot.joint_names)
        command_trace=CommandRecorder(e,robot,recorder,actions)

        def sync_world(row):
            world.set_phase(recorder.phase)
            world.gripper_joint_positions = row["q"][finger_ids].copy()
            world.gripper_joint_position = float(world.gripper_joint_positions[0])
            world.payload_pose = pose_matrix(row["task_object_states"][banana_index, :7])
            world.sync_background(recorder.background_paths, row["background_rigid_poses"])
            world.set_obstacle_pose("bowl", row["task_object_states"][bowl_index, :7])
            world.probe_bowl_pose=row['task_object_states'][bowl_index,:7].copy()
            world.set_obstacle_pose("banana", row["task_object_states"][banana_index, :7])

        def check_actual(row):
            # Observe the exact same substep before any safety failure can
            # stop the callback; this never evaluates stateful objects_settled.
            predicates.sample_physics(row['phase'], row['action_index'])
            world.probe_row=row
            world.probe_native=native_destination.sample(row)
            sync_world(row)
            command_trace.observe(row)
            joint_check=bounds_audit.sample(row)
            if not joint_check["passed"]:
                result["safety"]["failure"]=joint_check
                raise RuntimeError("Original actual joint bound violation: "+json.dumps(joint_check))
            row["has_planning_attachment"] = world.attachment is not None
            if world.attachment is not None:
                row["planning_attachment"] = world.attachment.copy()
            result["safety"]["checked_physics_samples"] += 1
            if not world.valid(row["q"][ids], mode="execution", gripper_q=row["q"][finger_ids],
                               object_pose=row["task_object_states"][banana_index, :7]):
                failure = dict(world.last_clearance, physics_step=row["physics_step"], phase=recorder.phase,
                               action_index=recorder.action_index)
                result["safety"]["failure"] = failure
                raise RuntimeError("Actual selective-pair safety check failed: " + json.dumps(failure))

        recorder.on_sample = check_actual
        check_actual(recorder.rows[-1])
        original_reset = e._reset_idx

        def keep_terminal_state(env_ids):
            if len(env_ids):
                terms = {name: bool(array(e.termination_manager.get_term(name))[0])
                         for name in e.termination_manager.active_terms}
                entry = dict(terms=terms, physics_step=int(e.sim.get_physics_step_count()),
                             phase=recorder.phase, action_index=recorder.action_index)
                if terms.get("success") and "first_success" not in terminal:
                    terminal["first_success"] = entry
                    terminal["success_state"] = {key: value.copy() if hasattr(value, "copy") else value
                        for key, value in recorder.rows[-1].items()}
                if any(value for name, value in terms.items() if name != "success"):
                    terminal["failure"] = entry
            # No post-terminal autoreset writes: retain the actual physical state.
        e._reset_idx = keep_terminal_state
        original_names = list(envdef.embodiment.camera_config.camera_names())
        if len(original_names) != 3:
            raise RuntimeError("Source DROID must retain all three original cameras")
        cameras = CameraWriter(out, scene, original_names, e.step_dt, previews=True)
        (out/"overview").mkdir()
        overview_writer = CameraWriter(out/"overview", scene, ["overview_camera"], e.step_dt, previews=True)
        result["original_camera_names"] = original_names
        result["appearance_warmup"] = cameras.warm_up(e.sim, min_frames=32, max_frames=128,
            sample_every=8, stable_windows=2)
        if int(e.sim.get_physics_step_count()) != recorder.reset_step:
            raise RuntimeError("Camera initialization advanced physics outside recorder")

        def step(command, gripper, phase, *, explicit_gripper_target=None):
            if len(actions) >= args.max_steps:
                raise RuntimeError("Bounded maximum action count reached")
            recorder.phase, recorder.action_index = phase, len(actions)
            world.set_phase(phase)
            before = int(e.sim.get_physics_step_count())
            cameras.write(phase)
            overview_writer.write(phase)
            after_capture = int(e.sim.get_physics_step_count())
            if phase=="release":
                if explicit_gripper_target is None:raise ValueError("Release requires an explicit declared radian target")
                target=float(explicit_gripper_target)
            else:
                if explicit_gripper_target is not None:raise ValueError("Only release uses the declared schedule")
                target=original_binary_target(float(gripper))
            action = np.r_[command, target].astype(np.float32)
            record = dict(phase=phase, action=action, source_binary_intent=float(gripper), gripper_target_rad=target, action_interface=INTERFACE, camera_physics_steps=[before, after_capture],
                          before_physics_step=before, after_physics_step=None)
            actions.append(record)
            if after_capture != before:
                raise RuntimeError("Camera acquisition advanced physics")
            try:
                _, _, done, truncated, _ = env.step(torch.tensor(action[None], dtype=torch.float32, device=e.device))
                predicates.sample_action(phase, len(actions)-1)
                if terminal.get("failure") or bool(array(truncated)[0]):
                    raise RuntimeError("Original task failure/timeout: " + str(terminal.get("failure")))
                if bool(array(done)[0]) and "first_success" not in terminal:
                    raise RuntimeError("Unexplained source-task termination")
            finally:
                record["after_physics_step"] = int(e.sim.get_physics_step_count())
            if len(actions) % 30 == 0:
                print("BANANA_STEP", len(actions), phase, recorder.rows[-1]["task_object_states"][banana_index, :3], flush=True)

        def hold(command, gripper, phase, seconds):
            for _ in range(max(1, int(np.ceil(seconds/e.step_dt)))):
                step(command, gripper, phase)

        q = recorder.rows[-1]["q"][ids].copy()
        for _ in range(8):
            step(q, 0., "warmup")
        hold(q, 0., "initial_settle", 5.)
        initial = recorder.rows[-1]
        result["settled_state"] = {key: initial[key] for key in ("q", "robot_body_pose_w", "task_object_states", "tcp_pose_w")}
        fk = kin.fk(initial["q"][ids])
        measured = pose_matrix(initial["tcp_pose_w"])
        fk_error = dict(position_m=float(np.linalg.norm(fk[:3, 3]-measured[:3, 3])),
            rotation_deg=float(np.rad2deg(Rotation.from_matrix(fk[:3, :3]@measured[:3, :3].T).magnitude())))
        result["fk_measured_tcp_error"] = fk_error
        if fk_error["position_m"] > .002 or fk_error["rotation_deg"] > 2.:
            raise RuntimeError("Source base/FK differs from observed TCP; reject planning attachment")

        def move(target, gripper, phase):
            recorder.phase = phase
            current = recorder.rows[-1]
            sync_world(current)
            # Rebuild from measured background and destination every segment;
            # never reuse a stale world after dynamic objects have moved.
            planner = SourceSupportCuroboPlanner(world, kin, dt=e.step_dt, seed=args.seed, max_joint_speed=.4)
            plan_record = dict(phase=phase, start_physics_step=current["physics_step"], target_tcp=target,
                               measured_planning_attachment=world.attachment, status="requested")
            result["plans"].append(plan_record)
            action_start = len(actions)
            try:
                if phase == 'lift':
                    plan_record['source_departure_binding'] = planner.bind_source_departure(
                        physics_step=current['physics_step'], arm_q=current['q'][ids],
                        measured_tcp=pose_matrix(current['tcp_pose_w']),
                        measured_payload=pose_matrix(current['task_object_states'][banana_index, :7]),
                        raw_contact_rows=contacts.records)
                plan = planner.plan_pose(current["q"][ids], target, grip_binary=gripper, phase=phase)
                positions = np.asarray(plan["positions"])
                if len(positions) < 2 or positions.shape[1:] != (7,) or not np.isfinite(positions).all():
                    raise RuntimeError("Planner returned an empty/invalid path")
                plan_record.update(metadata=plan["metadata"], candidate_contact_mask=planner.candidate_contact_mask,
                                   payload_geometry=getattr(planner, "payload_geometry_report", None))
                validation = planner.validate_path(positions, grip_binary=gripper, phase=phase, resolution=.02)
                plan_record["validation"] = validation
                np.savez_compressed(out/f"plan_{len(result['plans']):02d}_{phase}.npz", positions=positions,
                                    goal_tcp=target, start_q=current["q"][ids])
                if not validation["valid"]:
                    raise RuntimeError("Dense cuRobo + full-pair FCL validation rejected path")
                # The inherited validator supplies no observed free-object pose;
                # world.payload_pose and the measured fingers provide that state.
                plan_record["status"] = "validated"
                for command in positions[1:]:
                    step(command, gripper, phase)
                if phase=='lower':
                    boundaries=[]
                    for boundary_index in range(1+declaration['maximum_extra_lower_hold_actions']):
                        actual=recorder.rows[-1]
                        source=world.source_bowl_observation(pose_matrix(actual['task_object_states'][banana_index,:7]))
                        window=contacts.summarize(*region,phase='lower',last_steps=10)
                        boundary=release_boundary(source,plan_complete=True,contact_summary=window,
                            current_step=int(actual['physics_step']),reset_step=recorder.reset_step,
                            original_other_checks_passed=result['safety']['failure'] is None)
                        boundaries.append(boundary);plan_record['diagnostic_release_boundaries']=boundaries
                        if boundary['passed']:
                            result['release_start_boundary']=boundary
                            break
                        if boundary_index==declaration['maximum_extra_lower_hold_actions']:
                            raise RuntimeError('No legal declared [3,5] mm source-distance release boundary after original lower plan')
                        step(positions[-1],gripper,phase)
                else:
                    hold(positions[-1], gripper, phase, .3)
                plan_record["status"] = "executed"
                return positions[-1].copy()
            except Exception as error:
                plan_record.update(status="failed", error=repr(error))
                if hasattr(planner, 'last_source_support_validation'):
                    plan_record['source_support_validation'] = planner.last_source_support_validation
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

        target, grasp_info = grasp_target(initial["task_object_states"][banana_index, :7], world.payload_geometry, world, profile, candidate)
        result["grasp_target"] = grasp_info
        pregrasp = target.copy()
        pregrasp[2, 3] += profile['pregrasp_height_m']
        q = move(pregrasp, 0., "pregrasp")
        q = move(target, 0., "approach")
        hold(q, 1., "close", 2.)
        region = grasp_info["contact_region_local_m"]
        result["close_contacts"] = contacts.summarize(*region, phase="close", last_steps=10)
        if not result["close_contacts"]["both_fingers_loaded"]:
            raise RuntimeError("No simultaneous central-band bilateral physical grasp")
        actual = recorder.rows[-1]
        measured_tcp = pose_matrix(actual["tcp_pose_w"])
        world.attachment = np.linalg.inv(measured_tcp) @ pose_matrix(actual["task_object_states"][banana_index, :7])
        result["measured_tcp_to_banana"] = world.attachment.copy()
        lift = measured_tcp.copy()
        lift[2, 3] += profile['lift_height_m']
        q = move(lift, 1., "lift")
        result["lift_contacts"] = contacts.summarize(*region, phase="lift", last_steps=10)
        result["observed_lift_m"] = float(recorder.rows[-1]["task_object_states"][banana_index, 2]-initial["task_object_states"][banana_index, 2])
        if result["observed_lift_m"] < .05 or not result["lift_contacts"]["both_fingers_loaded"]:
            raise RuntimeError("Held banana did not physically lift with central-band bilateral contacts")
        state = recorder.rows[-1]
        above, transport_info = transport_target(pose_matrix(state["task_object_states"][banana_index, :7]),
            pose_matrix(state["task_object_states"][bowl_index, :7]), world.attachment,
            candidate=transport_candidate, source_object_sha256=recorder.metadata["source_hashes"])
        transport_info["observed_physics_step"] = int(state["physics_step"])
        result["transport_height_target"] = transport_info
        q = move(above, 1., "transport")
        state = recorder.rows[-1]
        delta, lower_info = banana_bowl_lower_target(state["task_object_states"][banana_index, :7],
            state["task_object_states"][bowl_index, :7], world.payload_geometry, bowl_geometry, gap=profile['release_gap_m'])
        result["source_supported_lower_target"] = lower_info
        lower = pose_matrix(state["tcp_pose_w"])
        lower[2, 3] += delta
        q = move(lower, 1., "lower")
        # Planning attachment is removed at opening; the real banana is never
        # constrained or repositioned. Every free-object pose is still checked.
        world.attachment = None
        world.release_start_step=int(recorder.rows[-1]['physics_step'])
        result['release_start_physics_step']=world.release_start_step
        driver=robot.joint_names.index('finger_joint')
        release_schedule=ReleaseSchedule(float(recorder.rows[-1]['q'][driver]),float(recorder.rows[-1]['dq'][driver]))
        result['release_schedule']=release_schedule.describe()
        for release_index in range(declaration['maximum_release_actions']):
            step(q,0.,'release',explicit_gripper_target=release_schedule.target_at_action(release_index))
        observed=int(recorder.rows[-1]['physics_step'])-world.release_start_step
        if observed!=declaration['maximum_release_physics_steps']:
            raise RuntimeError('Bounded release did not record exactly 240 actual substeps')
        result['observed_release_physics_steps']=observed
        result['released_finger_joint_rad']=float(recorder.rows[-1]['q'][finger_ids[0]])
        result["original_task_success"] = "first_success" in terminal
        result["final_original_destination_predicate"] = bool(array(object_on_destination(e,
            object_cfg=SceneEntityCfg("banana"), destination_cfg=SceneEntityCfg("bowl"),
            contact_sensor_cfg=task.contact_sensor_cfg, force_threshold=task.force_threshold,
            velocity_threshold=task.velocity_threshold, support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
        result["physical_sequence_complete"] = False
        result["diagnostic_completed"] = True
        result["status"] = "bounded_observation_complete"
    except Exception as error:
        result.update(status="failed", error=f"{type(error).__name__}: {error}")
    finally:
        if command_trace is not None:
            command_trace.close()
            result["command_recording"]=command_trace.report()
            json_write(out/"actual_command_targets.json",result["command_recording"])
        if bounds_audit is not None:
            result["actual_joint_bound_audit"]=dict(passed=all(x["passed"] for x in bounds_audit.rows),rows=bounds_audit.rows,allowance_rad=1e-5)
            json_write(out/"actual_joint_bound_audit.json",result["actual_joint_bound_audit"])
        if native_destination is not None:
            json_write(out/'destination_native_contacts.json',dict(accepted=False,diagnostic_only=True,
                identity=native_destination.identity,offsets=native_destination.offsets,rows=native_destination.rows))
        if world is not None:
            json_write(out/'destination_source_observations.json',dict(accepted=False,diagnostic_only=True,
                source_binding=world.probe_source_binding,rows=world.probe_records))
            json_write(out/'destination_strict_failure_ledger.json',dict(accepted=False,diagnostic_only=True,
                failures=world.strict_failure_ledger))
            result['original_strict_destination_failure_count']=len(world.strict_failure_ledger)
        result["terminal"] = terminal
        if "success_state" in result["terminal"]:
            # Preserve the state separately so terminal metadata remains readable.
            np.savez_compressed(out/"first_success_state.npz", **result["terminal"].pop("success_state"))
        for writer in (cameras, overview_writer):
            if writer is not None:
                try:
                    writer.close()
                except Exception as error:
                    result.update(status="failed", video_error=repr(error))
        if recorder is not None:
            recorder.close()
            try:
                result["physical_evidence_audit"] = recorder.write(out, actions)
                json_write(out/"physical_evidence_audit.json", result["physical_evidence_audit"])
            except Exception as error:
                result.update(status="failed", recording_audit_error=repr(error))
        if actions:
            # Versioned8D interface: original7 arm radians plus explicit finger_joint radians.
            np.savez_compressed(out/"trajectory.npz", actions=np.asarray([row["action"] for row in actions]),
                phase=np.asarray([row["phase"] for row in actions]),
                before_physics_step=np.asarray([row["before_physics_step"] for row in actions]),
                after_physics_step=np.asarray([row["after_physics_step"] for row in actions]),
                camera_physics_steps=np.asarray([row["camera_physics_steps"] for row in actions]))
        json_write(out/"actions.json", actions)
        if predicates is not None:
            try:
                predicates.finish(out)
                result['predicate_recording'] = dict(saved=True, metadata_sha256=sha(out/'task_predicate_metadata.json'),
                    independent_audit_required=True)
            except Exception as error:
                result.update(status='failed', predicate_recording_error=repr(error))
        if env is not None and provenance.scene_bound:
            try:
                check = provenance.finish_task(out, int(env.unwrapped.sim.get_physics_step_count()))
                result['diagnostic_declaration_archive']=audit_declaration_archive(check,provenance.diagnostic_declaration_binding)
                result['collection_provenance'] = dict(passed=check['passed'],
                    sha256=sha(out/'task_collection_provenance.json'))
                if not check['passed']:
                    result.update(status='failed', provenance_error='Execution inputs changed during collection')
            except Exception as error:
                result.update(status='failed', provenance_error=repr(error))
        if env is not None:
            if original_reset is not None:
                env.unwrapped._reset_idx = original_reset
            try:
                env.close()
            except Exception as error:
                result.update(status="failed", close_error=repr(error))
    if cameras is not None and cameras.frames:
        try:
            decoded = {}
            for relative, size in [(name+".mp4", (1280, 720)) for name in result["original_camera_names"]] + [
                ("all_cameras.mp4", (1920, 360)), ("overview/overview_camera.mp4", (1280, 720)),
                ("overview/all_cameras.mp4", (640, 360))]:
                decoded[relative] = dict(frames=decode_video(out/relative, cameras.frames, expected_size=size,
                    expected_dt=cameras.dt), sha256=sha(out/relative))
            result["video_decode"] = decoded
        except Exception as error:
            result.update(status="failed", video_error=repr(error))
    result['accepted']=False
    result['physical_trial_passed']=False
    result['task_success']=False
    result["wall_seconds"] = time.perf_counter()-started
    json_write(out/"result.json", result)
    print(json.dumps(dict(output=str(out), status=result["status"], physical_trial_passed=result["physical_trial_passed"],
                         accepted=False, error=result.get("error")), indent=2), flush=True)
    return 0 if (result['diagnostic_completed'] and result['status']=='bounded_observation_complete'
        and result.get('physical_evidence_audit',{}).get('passed') and result.get('collection_provenance',{}).get('passed')
        and result.get('diagnostic_declaration_archive',{}).get('passed') and len(result.get('video_decode',{}))==6
        and result.get('actual_joint_bound_audit',{}).get('passed') and result.get('command_recording',{}).get('passed')) else 2


def make_provenance(args):
    from kitchen_safety.extensions.replicator.banana_controller_profile import LAYOUT
    from kitchen_safety.extensions.replicator.task_collection_evidence import TaskRunProvenance,input_key
    provenance=TaskRunProvenance(LAYOUT,collector_path=__file__,input_paths=[args.controller_profile])
    module='kitchen_safety/extensions/replicator/banana_smooth_destination_probe.py'
    provenance.capture(ROOT/module,'execution_source',snapshot=True)
    for extra in [
        'kitchen_safety/extensions/replicator/banana_destination_probe.py',
        'kitchen_safety/extensions/replicator/banana_release_schedule_candidate.py',
        'kitchen_safety/extensions/replicator/banana_command_recording.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/envs/mdp/actions/joint_actions.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/envs/mdp/actions/actions_cfg.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/managers/action_manager.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/assets/articulation/base_articulation_data.py',
        'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/articulation/actuator_control.py']:
        provenance.capture(ROOT/extra,'execution_source',snapshot=True)
    provenance.capture(ROOT/'assets/curobo/droid/urdf/franka_robotiq_2f_85_zero.urdf','original_joint_limits',expected_sha256='0aad7ada239f91360d57376247dc894e9c32a818957ee3099cbb6027ed18c2af')
    provenance.capture(args.probe_declaration,'diagnostic_declaration_input',snapshot=True)
    key=input_key(args.probe_declaration,ROOT)
    provenance.diagnostic_declaration_binding=dict(path=key,sha256=provenance.entries[key]['initial_sha256'],
        module_path=module,module_sha256=provenance.entries[module]['initial_sha256'])
    native=list((ROOT/'.venv-arena/lib/python3.12/site-packages/isaacsim/extscache').glob('omni.physics.tensors-*/omni/physics/tensors/api.py'))
    if len(native)!=1:raise ValueError('Require unique installed native contact API source')
    provenance.capture(native[0],'execution_source',snapshot=True)
    provenance.capture(ROOT/'third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/rigid_object/rigid_object.py','execution_source',snapshot=True)
    provenance.bind_grasp_candidate(args.grasp_candidate_profile)
    provenance.bind_transport_candidate(args.transport_candidate_profile)
    # Capture every new leaf and the declaration before their first import/use.
    from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import read_declaration
    read_declaration(args.probe_declaration)
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-id", choices=[TASK], default=TASK)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, choices=[42, 43, 44], default=42)
    parser.add_argument("--device", choices=["cpu"], default="cpu", help="Keep reviewed CPU PhysX; cuRobo/rendering still require GPU")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--max-steps", type=int, default=2400)
    parser.add_argument("--physics-threads", type=int, choices=[2], default=2)
    parser.add_argument("--torch-threads", type=int, choices=[2], default=2)
    parser.add_argument("--episode-length-s", type=float, default=180.)
    parser.add_argument("--dry-run", action="store_true", help="Source/geometry CPU checks; do not launch Kit or CUDA")
    parser.add_argument('--controller-profile', type=Path,
        default=ROOT/'configs/extensions/replicator/g_shape_banana_bowl_controller.json')
    parser.add_argument('--grasp-candidate-profile', type=Path,
        default=ROOT/'configs/extensions/replicator/g_shape_banana_bowl_halfcenter_candidate.json')
    parser.add_argument('--transport-candidate-profile', type=Path,
        default=ROOT/'configs/extensions/replicator/g_shape_banana_bowl_transport_height_candidate.json')
    parser.add_argument('--probe-declaration',type=Path,default=ROOT/'configs/extensions/replicator/g_banana_smooth_destination_contact_probe.json')
    args = parser.parse_args()
    if args.max_steps < 1 or not 0 < args.episode_length_s < float("inf"):
        parser.error("Require a positive action limit and finite episode duration")
    if args.physics_threads is not None and args.physics_threads < 0:
        parser.error("--physics-threads must be nonnegative")
    if args.torch_threads is not None and args.torch_threads < 1:
        parser.error("--torch-threads must be positive")
    args.output = args.output.resolve()
    if not args.output.is_relative_to((ROOT/'reports').resolve()):
        parser.error('Diagnostic outputs must remain under reports, never datasets/success')
    args.probe_declaration=args.probe_declaration.resolve()
    args.controller_profile = args.controller_profile.resolve()
    args.grasp_candidate_profile = args.grasp_candidate_profile.resolve()
    args.transport_candidate_profile = args.transport_candidate_profile.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.dry_run:
        provenance=make_provenance(args)
        report = preflight(args)
        from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import audit_declaration_archive
        captured=provenance.check()
        report['preapp_archive_check']=audit_declaration_archive(captured,provenance.diagnostic_declaration_binding)
        report['preapp_controller_inputs']=[k for k,v in captured['entries'].items() if v['kind']=='controller_input']
        if not captured['passed'] or len(report['preapp_controller_inputs'])!=1:
            raise ValueError('Pre-App provenance contains changed inputs or ambiguous controller role')
        report['preapp_capture']={k:dict(kind=v['kind'],sha256=v['initial_sha256']) for k,v in provenance.entries.items()}
        report['diagnostic_declaration_binding']=provenance.diagnostic_declaration_binding
        json_write(args.output/"preflight.json", report)
        print(json.dumps(dict(task_id=TASK, cpu_preflight_passed=report["cpu_preflight_passed"],
            execution_verified=False, output=str(args.output)), indent=2))
        return 0 if report["cpu_preflight_passed"] else 2
    # One canonical shared lock; never create a different per-clone lock.
    import fcntl
    lock_path = Path("/workspace/reports/skill_trials/gpu.lock")
    if ROOT != Path("/workspace") or not lock_path.exists():
        raise RuntimeError("Actual collection requires the verified /workspace mount and existing shared GPU lock")
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        provenance=make_provenance(args)
        from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
        from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
        # Arena's current parser takes headless mode from HEADLESS, not a CLI
        # --headless argument. Keep this wrapper's explicit headless interface.
        import os
        os.environ['HEADLESS'] = '1' if args.headless else '0'
        runtime_args = get_isaaclab_arena_cli_parser().parse_args(["--enable_cameras", "--device", args.device,
                                                                "--seed", str(args.seed)])
        with SimulationAppContext(runtime_args):
            code = run(args, provenance)
            if code:
                # Kit may force exit 0 on a normal context exit. Raise inside
                # it, after preserving the report, for an honest failure status.
                raise RuntimeError("Bounded banana/bowl diagnostic stopped; inspect " + str(args.output/"result.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
