#!/usr/bin/env python3
"""Independent additional-mustard neutral/physical trial with cuRobo and selective source-geometry checks.

--dry-run is CPU-only. Actual invocation holds the shared GPU lock internally.
Every physics step from reset is recorded; failures remain in a new output
directory. A physical candidate is not delivery acceptance: independent neutral,
fixed-base, actual-self-collision and camera/visual sidecars remain mandatory.
"""
import argparse
import json
from pathlib import Path
import time

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
    from kitchen_safety.extensions.replicator.mustard_family_controller import load_controller
    controller, prepared = load_controller(args.controller_profile)
    return dict(task_id=controller['task_id'], cpu_preflight_passed=True,
        physics_launched=False, execution_verified=False, controller=controller, preparation=prepared,
        neutral_required_before_physical=True, pending=['Actual neutral/mobility', 'cuRobo and physical task', 'Independent sidecars'])


def plan_execution_evidence(actions, first_action_index, first_physics_step, current_physics_step):
    """Read action counters, including a step that raised after advancing physics."""
    rows=[]
    expected=int(first_physics_step)
    complete=True
    advanced=False
    for index in range(first_action_index,len(actions)):
        item=actions[index]
        before,after=item['before_physics_step'],item['after_physics_step']
        valid=(before is not None and after is not None and int(after)>=int(before))
        count=int(after)-int(before) if valid else None
        advanced=advanced or bool(valid and count>0)
        complete=complete and valid and before==expected
        expected=int(after) if valid else None
        rows.append(dict(action_index=index,before_physics_step=before,after_physics_step=after,
            observed_physics_substeps=count))
    complete=bool(complete and expected==current_physics_step)
    executed=True if advanced else (False if complete else None)
    return dict(command_executed=executed,execution_status=('observed_physics_execution' if executed is True
        else 'no_physics_execution' if executed is False else 'unknown_incomplete_counters'),
        action_counter_coverage_complete=complete,first_action_index=int(first_action_index),
        start_physics_step=int(first_physics_step),current_physics_step=int(current_physics_step),actions=rows,
        scope='Measured per-command before/after counters; an exception after advancing physics is execution. No joint displacement, path completion or task success inference.')


def run(args, provenance, on_entry=None):
    if on_entry is not None:
        on_entry()
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
    from kitchen_safety.extensions.replicator.mustard_family_environment import MustardFamilyCfg, MustardFamilyEnvironment
    from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
    from kitchen_safety.extensions.replicator.mustard_family_collision import MustardFamilyCollisionWorld
    from kitchen_safety.extensions.replicator.mustard_family_curobo import MustardSourceSupportPlanner
    from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts
    from kitchen_safety.extensions.replicator.mustard_family_recorder import MustardFamilyRecorder
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from kitchen_safety.placement_geometry import bottle_bowl_lower_target
    from kitchen_safety.extensions.replicator.mustard_family_controller import load_controller, grasp_target, transport_target
    from kitchen_safety.extensions.replicator.task_collection_evidence import TaskPredicateRecorder
    from kitchen_safety.extensions.replicator.startup_recorder import array, sha
    from kitchen_safety.extensions.replicator.overview import configure_overview

    started = time.perf_counter()
    out = args.output
    profile, prepared = load_controller(args.controller_profile)
    task_id = profile["task_id"]
    result = dict(task_id=task_id, variant="base", seed=args.seed, physical_trial_passed=False, accepted=False,
        fixed_only=True, physics_state_setters_used=False, physical_weld_used=False,
        planning_attachment_only=True, original_task_success=False, final_original_destination_predicate=False,
        status="running", pending_acceptance=["Matching-duration neutral probe", "Source/fixed-base sidecar",
            "Actual unclamped q self-collision sidecar", "Camera pose/intrinsic alignment sidecar", "Visual review"],
        plans=[], safety=dict(checked_physics_samples=0, failure=None), source_hashes={"collector": sha(__file__)},
        controller_profile=profile, controller_profile_sha256=sha(args.controller_profile))
    result["runtime_threads"] = dict(requested_physics_threads=args.physics_threads,
        requested_torch_threads=args.torch_threads,
        live_physics_threads=carb.settings.get_settings().get("/persistent/physics/numThreads"),
        live_torch_threads=torch.get_num_threads(), requested_physics_device=args.device)
    env, recorder, cameras, overview_writer, original_reset = None, None, None, None, None
    predicates = None
    actions, terminal = [], {}
    world = None
    try:
        envdef = MustardFamilyEnvironment().build(MustardFamilyCfg(preparation_json=str(ROOT/profile["inputs"]["preparation"]["path"]), enable_cameras=True,
            episode_length_s=args.episode_length_s))
        provenance.bind_environment(envdef)
        builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=args.seed, device=args.device,
            presets="physx", solve_relations=False))
        cfg, kwargs = builder.compose_manager_cfg()
        cfg.observations.camera_obs = None
        cfg.sim.render_interval = cfg.decimation
        cfg.scene.robot_contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", update_period=0., history_length=1)
        camera = prepared["profile"]["extra_camera"]
        overview = configure_overview(cfg.scene, eye=tuple(camera["eye"]), target=tuple(camera["target"]))
        env = builder.make_registered(cfg, kwargs)
        env.reset()
        e, scene, robot = env.unwrapped, env.unwrapped.scene, env.unwrapped.scene["robot"]
        result["runtime_threads"]["actual_environment_device"] = str(e.device)
        predicates = TaskPredicateRecorder(e, envdef)
        contacts = ObjectGraspContacts(scene["robot_contacts"]._physics_sim_view, e.physics_dt, "mustard_bottle")
        recorder = MustardFamilyRecorder(e, envdef, contacts)
        expected_contacts = {name for name in robot.body_names if name.startswith("panda_link") or name == "base_link"}
        if not expected_contacts <= set(scene["robot_contacts"].body_names):
            raise RuntimeError("Arm/palm contact bodies are missing from the actual contact sensor")
        layout, task = envdef.resolved_task_layout, envdef.task
        result.update(source_relations=envdef.source_relation_audit, layout=layout, overview=overview)
        json_write(out/"source_relation_audit.json", envdef.source_relation_audit)
        ids = [robot.joint_names.index(f"panda_joint{i}") for i in range(1, 8)]
        finger_ids = [robot.joint_names.index(name) for name in MustardFamilyCollisionWorld.GRIPPER_JOINT_NAMES]
        payload_index, bowl_index = (recorder.object_names.index(name) for name in ("mustard_bottle", "bowl"))
        base = np.asarray(layout["robot_position"]).copy()
        base[2] += layout["stand_height"]
        kin = DroidKinematics(base, layout["robot_yaw_deg"])
        world = MustardFamilyCollisionWorld(e.sim.stage, kin, preparation=prepared, env_root="/World/envs/env_0")
        bowl_geometry = PayloadGeometry(ROOT/layout["objects"]["bowl"]["usd_path"])
        result["contact_policy"] = world.contact_policy.report()
        world.gripper_joint_positions = recorder.rows[-1]["q"][finger_ids]

        def sync_world(row):
            world.set_phase(recorder.phase)
            world.gripper_joint_positions = row["q"][finger_ids].copy()
            world.gripper_joint_position = float(world.gripper_joint_positions[0])
            world.payload_pose = pose_matrix(row["task_object_states"][payload_index, :7])
            world.sync_background(recorder.background_paths, row["background_rigid_poses"])
            world.set_obstacle_pose("bowl", row["task_object_states"][bowl_index, :7])
            world.set_obstacle_pose("mustard_bottle", row["task_object_states"][payload_index, :7])

        def check_actual(row):
            # Observe the exact same substep before any safety failure can
            # stop the callback; this never evaluates stateful objects_settled.
            predicates.sample_physics(row['phase'], row['action_index'])
            sync_world(row)
            row["has_planning_attachment"] = world.attachment is not None
            if world.attachment is not None:
                row["planning_attachment"] = world.attachment.copy()
            result["safety"]["checked_physics_samples"] += 1
            if not world.valid(row["q"][ids], mode="execution", gripper_q=row["q"][finger_ids],
                               object_pose=row["task_object_states"][payload_index, :7]):
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

        def step(command, gripper, phase):
            if len(actions) >= args.max_steps:
                raise RuntimeError("Bounded maximum action count reached")
            recorder.phase, recorder.action_index = phase, len(actions)
            world.set_phase(phase)
            before = int(e.sim.get_physics_step_count())
            cameras.write(phase)
            overview_writer.write(phase)
            after_capture = int(e.sim.get_physics_step_count())
            action = np.r_[command, gripper].astype(np.float32)
            record = dict(phase=phase, action=action, camera_physics_steps=[before, after_capture],
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
                print("MUSTARD_STEP", len(actions), phase, recorder.rows[-1]["task_object_states"][payload_index, :3], flush=True)

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
            planner = MustardSourceSupportPlanner(world, kin, preparation=prepared,
                dt=e.step_dt, seed=args.seed, max_joint_speed=.4)
            plan_record = dict(phase=phase, start_physics_step=current["physics_step"], target_tcp=target,
                               measured_planning_attachment=world.attachment, status="requested")
            first_action_index=len(actions)
            result["plans"].append(plan_record)
            try:
                if phase == 'lift':
                    plan_record['source_departure_binding'] = planner.bind_source_departure(
                        physics_step=current['physics_step'], arm_q=current['q'][ids],
                        measured_tcp=pose_matrix(current['tcp_pose_w']),
                        measured_payload=pose_matrix(current['task_object_states'][payload_index, :7]),
                        raw_contact_rows=contacts.records)
                if phase == 'pregrasp':
                    goal_q = kin.ik(target, np.asarray(profile['pregrasp_ik_seed_rad']), attempts=1)
                    actual_goal = kin.fk(goal_q)
                    if (np.linalg.norm(actual_goal[:3,3]-target[:3,3]) > .002 or
                            Rotation.from_matrix(actual_goal[:3,:3]@target[:3,:3].T).magnitude() > np.deg2rad(2.)):
                        raise RuntimeError('Observed-pose seeded IK exceeds original target gates')
                    plan_record['measured_target_seeded_goal_q'] = goal_q.tolist()
                    plan = planner.plan_joint_goal(current['q'][ids], goal_q, grip_binary=gripper, phase=phase)
                else:
                    plan = planner.plan_pose(current['q'][ids], target, grip_binary=gripper, phase=phase)
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
                hold(positions[-1], gripper, phase, .3)
                plan_record["status"] = "executed"
                return positions[-1].copy()
            except Exception as error:
                plan_record.update(status="failed", error=repr(error))
                execution=plan_execution_evidence(actions,first_action_index,current['physics_step'],int(e.sim.get_physics_step_count()))
                if hasattr(planner, 'last_source_support_validation'):
                    plan_record['source_support_validation'] = planner.last_source_support_validation
                candidate = getattr(planner, "last_cartesian_candidate", None)
                if candidate is not None:
                    filename = f"rejected_plan_{len(result['plans']):02d}_{phase}.npz"
                    np.savez_compressed(out/filename, **{key: candidate[key] for key in
                        ("positions", "ik_waypoint_positions", "poses", "effective_goal_pose")},
                        start_q=current["q"][ids], requested_target=target,
                        measured_gripper_angles=current["q"][finger_ids])
                    plan_record.update(rejected_candidate_file=filename,
                        rejected_candidate_sha256=sha(out/filename),
                        candidate_constraints=getattr(planner, "last_constraint_diagnostics", None),
                        candidate_contact_mask=getattr(planner, "candidate_contact_mask", None),
                        rejected_candidate_executed=execution['command_executed'])
                raise
            finally:
                plan_record['execution']=plan_execution_evidence(actions,first_action_index,current['physics_step'],
                    int(e.sim.get_physics_step_count()))

        if args.neutral_seconds:
            hold(q, 0., 'final_settle', args.neutral_seconds)
            result.update(neutral_sequence_complete=True, neutral_hold_seconds=args.neutral_seconds,
                status='neutral_sequence_complete')
        else:
            target, grasp_info = grasp_target(initial["task_object_states"][payload_index, :7], profile, prepared)
            result["grasp_target"] = grasp_info
            pregrasp = target.copy()
            pregrasp[2, 3] += profile['motion']['pregrasp_height_m']
            q = move(pregrasp, 0., "pregrasp")
            q = move(target, 0., "approach")
            hold(q, 1., "close", 2.)
            region = grasp_info["contact_region_local_m"]
            result["close_contacts"] = contacts.summarize(*region, phase="close", last_steps=10)
            if not result["close_contacts"]["both_fingers_loaded"]:
                raise RuntimeError("No simultaneous central-band bilateral physical grasp")
            actual = recorder.rows[-1]
            measured_tcp = pose_matrix(actual["tcp_pose_w"])
            world.attachment = np.linalg.inv(measured_tcp) @ pose_matrix(actual["task_object_states"][payload_index, :7])
            result["measured_tcp_to_mustard_bottle"] = world.attachment.copy()
            lift = measured_tcp.copy()
            lift[2, 3] += profile['motion']['lift_height_m']
            q = move(lift, 1., "lift")
            result["lift_contacts"] = contacts.summarize(*region, phase="lift", last_steps=10)
            result["observed_lift_m"] = float(recorder.rows[-1]["task_object_states"][payload_index, 2]-initial["task_object_states"][payload_index, 2])
            if result["observed_lift_m"] < .05 or not result["lift_contacts"]["both_fingers_loaded"]:
                raise RuntimeError("Held mustard bottle did not physically lift with central-band bilateral contacts")
            state = recorder.rows[-1]
            above, transport_info = transport_target(state, world.attachment, payload_index, bowl_index,
                world.payload_geometry, bowl_geometry, profile['motion']['transport_guard_above_rim_m'])
            result['source_transport_target'] = transport_info
            q = move(above, 1., "transport")
            state = recorder.rows[-1]
            lower_info = bottle_bowl_lower_target(state['task_object_states'][payload_index, :7],
                state['task_object_states'][bowl_index, :7], state['tcp_pose_w'][:3],
                bottle_usd=ROOT/layout['objects']['mustard_bottle']['usd_path'], bowl_usd=ROOT/layout['objects']['bowl']['usd_path'],
                nominal_gap_m=profile['motion']['release_gap_m'])
            result['source_supported_lower_target'] = lower_info
            lower = pose_matrix(state['tcp_pose_w'])
            lower[:3,3] = lower_info['lower_target_xyz_world_m']
            q = move(lower, 1., "lower")
            # Planning attachment is removed at opening; the real mustard bottle is never
            # constrained or repositioned. Every free-object pose is still checked.
            world.attachment = None
            hold(q, 0., "release", 2.)
            result["released_finger_joint_rad"] = float(recorder.rows[-1]["q"][finger_ids[0]])
            retreat = pose_matrix(recorder.rows[-1]["tcp_pose_w"])
            retreat[2, 3] += .15
            q = move(retreat, 0., "retreat")
            hold(q, 0., "final_settle", 4.)
            result["original_task_success"] = "first_success" in terminal
            result["final_original_destination_predicate"] = bool(array(object_on_destination(e,
                object_cfg=SceneEntityCfg("mustard_bottle"), destination_cfg=SceneEntityCfg("bowl"),
                contact_sensor_cfg=task.contact_sensor_cfg, force_threshold=task.force_threshold,
                velocity_threshold=task.velocity_threshold, support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
            result["physical_sequence_complete"] = True
            result["status"] = "sequence_complete"
    except Exception as error:
        result.update(status="failed", error=f"{type(error).__name__}: {error}")
    finally:
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
            # Original 7 arm joint targets + binary gripper action semantics.
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
    result["physical_trial_passed"] = bool(result.get("physical_sequence_complete") and result["original_task_success"]
        and result["final_original_destination_predicate"] and result.get("released_finger_joint_rad", 1.) < .03
        and result.get("physical_evidence_audit", {}).get("passed") and result["safety"]["failure"] is None
        and recorder is not None and result["safety"]["checked_physics_samples"] == len(recorder.rows)
        and result.get('predicate_recording', {}).get('saved') and result.get('collection_provenance', {}).get('passed')
        and len(result.get("video_decode", {})) == 6 and result["status"] != "failed")
    result['neutral_probe_passed'] = False
    if args.neutral_seconds and recorder is not None and recorder.rows:
        from kitchen_safety.extensions.replicator.startup_recorder import pose_change
        steps = np.asarray([row['physics_step'] for row in recorder.rows])
        eligible = np.flatnonzero((steps-steps[0])*recorder.metadata['physics_dt_s'] >= 5.-1e-8)
        stability = []
        if len(eligible):
            for index, name in enumerate(recorder.object_names):
                poses = np.asarray([row['task_object_states'][index,:7] for row in recorder.rows])
                change = pose_change(poses, int(eligible[0]))
                stability.append(dict(object_name=name, **change, passed=change['max_translation_m']<=.002 and change['max_rotation_deg']<=1.))
        result['neutral_task_objects_after_declared_5s_settle'] = stability
        result['neutral_probe_passed'] = bool(result.get('neutral_sequence_complete') and stability and
            all(row['passed'] for row in stability) and result.get('physical_evidence_audit',{}).get('passed') and
            result['safety']['failure'] is None and result['safety']['checked_physics_samples']==len(recorder.rows) and
            result.get('collection_provenance',{}).get('passed') and result.get('predicate_recording',{}).get('saved') and
            len(result.get('video_decode',{}))==6 and result['status']!='failed')
    result["wall_seconds"] = time.perf_counter()-started
    json_write(out/"result.json", result)
    print(json.dumps(dict(output=str(out), status=result["status"], physical_trial_passed=result["physical_trial_passed"],
                         neutral_probe_passed=result['neutral_probe_passed'], accepted=False, error=result.get("error")), indent=2), flush=True)
    return 0 if (result["neutral_probe_passed"] if args.neutral_seconds else result["physical_trial_passed"]) else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--neutral-seconds", type=float, default=0.)
    parser.add_argument("--neutral-episode", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, choices=[42, 43, 44], default=42)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--max-steps", type=int, default=2400)
    parser.add_argument("--physics-threads", type=int, default=None)
    parser.add_argument("--torch-threads", type=int, default=None)
    parser.add_argument("--episode-length-s", type=float, default=180.)
    parser.add_argument("--dry-run", action="store_true", help="Source/geometry CPU checks; do not launch Kit or CUDA")
    parser.add_argument('--controller-profile', type=Path, required=True)
    args = parser.parse_args()
    if args.max_steps < 1 or not 0 < args.episode_length_s < float("inf"):
        parser.error("Require a positive action limit and finite episode duration")
    if args.physics_threads is not None and args.physics_threads < 0:
        parser.error("--physics-threads must be nonnegative")
    if args.torch_threads is not None and args.torch_threads < 1:
        parser.error("--torch-threads must be positive")
    if not 0 <= args.neutral_seconds < float("inf"):
        parser.error("Neutral seconds must be finite and nonnegative")
    args.output = args.output.resolve()
    args.controller_profile = args.controller_profile.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.dry_run:
        report = preflight(args)
        json_write(args.output/"preflight.json", report)
        print(json.dumps(dict(task_id=report["task_id"], cpu_preflight_passed=report["cpu_preflight_passed"],
            execution_verified=False, output=str(args.output)), indent=2))
        return 0 if report["cpu_preflight_passed"] else 2
    # One canonical shared lock; never create a different per-clone lock.
    import fcntl
    lock_path = Path("/workspace/reports/skill_trials/gpu.lock")
    if ROOT != Path("/workspace") or not lock_path.exists():
        raise RuntimeError("Actual collection requires the verified /workspace mount and existing shared GPU lock")
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        from kitchen_safety.extensions.replicator.mustard_family_controller import read_controller_inputs, require_neutral
        from kitchen_safety.extensions.replicator.mustard_family_evidence import MustardFamilyProvenance
        import sys,traceback
        state=dict(schema_version=1,stage='prelaunch_input_hashes',events=[],kit_context_entered=False,
            collector_run_entered=False,launch_complete=False,
            scope='Observed launch milestones only. Exit code zero or incomplete launch state is never neutral/task success.')
        provenance=None
        def mark(stage):
            state['stage']=stage
            state['events'].append(dict(stage=stage,monotonic_time_s=time.monotonic()))
            json_write(args.output/'launch_state.json',state)
        def record_failure(error):
            if state['stage']=='launch_failed':
                return
            failure=dict(stage=state['stage'],exception_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),
                kit_context_entered=state['kit_context_entered'],collector_run_entered=state['collector_run_entered'],
                physical_record_file_present=(args.output/'physics_states.npz').is_file())
            json_write(args.output/'launch_failure.json',failure)
            if provenance is not None:
                json_write(args.output/'launch_exit_provenance.json',provenance.check())
            if not (args.output/'result.json').exists():
                json_write(args.output/'result.json',dict(task_id=state.get('task_id'),status='initialization_failed',
                    accepted=False,physical_trial_passed=False,neutral_probe_passed=False,launch_failure=failure,
                    scope='No physical result available; no missing reset/substep/video evidence inferred.'))
            mark('launch_failed')
        def collector_entered():
            state['collector_run_entered']=True
            mark('collector_run_entered')
        mark('prelaunch_input_hashes')
        try:
            # Do not resolve USD/source geometry here: standalone CPU USD and
            # Kit USD have different ABIs. Full gates still run inside run().
            controller,prepared=read_controller_inputs(args.controller_profile)
            state['task_id']=controller['task_id']
            if not args.neutral_seconds:
                if args.neutral_episode is None:
                    raise ValueError('A passed matching actual --neutral-episode is required before a physical task')
                neutral=require_neutral(args.neutral_episode,args.controller_profile,controller['task_id'])
                json_write(args.output/'neutral_reference.json',neutral)
            provenance=MustardFamilyProvenance(args.controller_profile,controller,prepared)
            json_write(args.output/'prelaunch_provenance.json',provenance.check())
            state['before_arena_bootstrap_modules']=[name for name in sys.modules if name=='pxr' or name.startswith('pxr.') or
                name=='omni' or name.startswith('omni.')]
            if state['before_arena_bootstrap_modules']:
                raise RuntimeError('USD/Omniverse imported by prelaunch input capture: '+str(state['before_arena_bootstrap_modules']))
            from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
            from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
            # Arena's parser takes headless mode from HEADLESS, not --headless.
            import os
            os.environ['HEADLESS']='1' if args.headless else '0'
            runtime_args=get_isaaclab_arena_cli_parser().parse_args(["--enable_cameras","--device",args.device,
                                                                    "--seed",str(args.seed)])
            state['prelaunch_usd_modules']=[name for name in sys.modules if name=='pxr' or name.startswith('pxr.') or
                name=='omni.usd' or name.startswith('omni.usd.')]
            if state['prelaunch_usd_modules']:
                raise RuntimeError('USD was imported before SimulationApp: '+str(state['prelaunch_usd_modules']))
            mark('kit_initialization_requested')
            with SimulationAppContext(runtime_args):
                state['kit_context_entered']=True
                mark('kit_context_entered')
                try:
                    code=run(args,provenance,on_entry=collector_entered)
                    state['collector_return_code']=int(code)
                    mark('collector_returned')
                    if code:
                        raise RuntimeError("Independent mustard trial failed; inspect "+str(args.output/"result.json"))
                except BaseException as error:
                    # Arena __exit__ may terminate the process immediately.
                    # Save failures while still inside the context.
                    record_failure(error)
                    raise
            state['launch_complete']=True
            mark('launch_complete')
        except BaseException as error:
            record_failure(error)
            raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
