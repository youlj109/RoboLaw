#!/usr/bin/env python3
"""Fail-closed CPU review of task-aware G banana/bowl and bowl/sink recordings.

Never steps physics, launches CUDA, changes evidence, or manufactures a visual
review. A successful contact window is explicitly separate from task success.
Missing collector provenance or predicate instrumentation remains a failed gate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
TASKS = {"kitchen_bench_replicator_g_shape_banana_bowl": ("banana", "bowl"),
         "kitchen_bench_replicator_g_shape_bowl_sink": ("bowl", "sink")}
FINGERS = ("left_inner_finger", "right_inner_finger")
SOURCE_DIR = ROOT/"third_party/IsaacLab-Arena/isaaclab_arena"
CORE_INPUTS = ("result.json", "physics_metadata.json", "physics_states.npz", "trajectory.npz", "actions.json",
               "grasp_contacts.json", "startup_physics_states.npz", "startup_audit.json",
               "source_relation_audit.json", "images.h5", "cameras.json", "overview/images.h5", "overview/cameras.json")


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def require(value, message):
    if not value:
        raise ValueError(message)


def contact_window(data, metadata, contacts, object_name, region, phase, count=10):
    """Recompute positive bilateral forces, identity and frame transforms."""
    steps = data["physics_step"]
    require(len(contacts) == len(steps), "Contact record does not cover every physics sample")
    region = np.asarray(region, dtype=float)
    require(region.shape == (2, 3) and np.isfinite(region).all() and np.all(region[0] < region[1]), "Invalid contact region")
    oi = metadata["task_object_names"].index(object_name)
    max_transform_error = 0.
    for i, row in enumerate(contacts):
        require(row["object_name"] == object_name and row["physics_step"] == int(steps[i])
                and row["phase"] == str(data["phase"][i]) and row["action_index"] == int(data["action_index"][i]),
                "Contact identity/phase/action does not match recorded physics")
        require(abs(row["physics_dt_s"]-metadata["physics_dt_s"]) < 1e-12, "Contact physics period differs")
        require(set(row["fingers"]) == set(FINGERS), "Bilateral finger inventory changed")
        pose = data["task_object_states"][i, oi, :7]
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        for finger in FINGERS:
            item = row["fingers"][finger]
            n = item["count"]
            require(type(n) is int and 0 <= n < 4096, "Invalid or saturated raw contact count")
            for key, shape in [("points_world_m", (n, 3)), ("points_environment_m", (n, 3)),
                    ("points_object_local_m", (n, 3)), ("normals_world", (n, 3)),
                    ("normals_object_local", (n, 3)), ("normal_forces_n", (n,)), ("separations_m", (n,))]:
                value = np.asarray(item[key]).reshape(shape) if n == 0 else np.asarray(item[key])
                require(value.shape == shape and np.isfinite(value).all(), "Missing/invalid raw contact array: "+key)
            if n:
                local = (np.asarray(item["points_world_m"])-pose[:3])@rotation
                error = float(np.abs(local-np.asarray(item["points_object_local_m"])).max())
                normal_error = float(np.abs(np.asarray(item["normals_world"])@rotation-item["normals_object_local"]).max())
                max_transform_error = max(max_transform_error, error, normal_error)
                require(error < 1e-6 and normal_error < 1e-6, "Raw contact object/world frame mismatch")
    indices = np.flatnonzero(data["phase"] == phase)[-count:]
    summaries = []
    for i in indices:
        fingers = {}
        for finger in FINGERS:
            item = contacts[i]["fingers"][finger]
            forces = np.asarray(item["normal_forces_n"])
            points = np.asarray(item["points_object_local_m"]).reshape(-1, 3)
            loaded = forces > .01
            inside = np.all((points >= region[0]) & (points <= region[1]), axis=-1)
            fingers[finger] = dict(loaded_in_region=bool(loaded.any() and inside[loaded].all()),
                loaded_contact_count=int(loaded.sum()), total_positive_normal_force_n=float(forces[forces > 0].sum()),
                minimum_loaded_force_n=float(forces[loaded].min()) if loaded.any() else None)
        summaries.append(dict(physics_step=int(steps[i]), fingers=fingers,
            bilateral_loaded=all(item["loaded_in_region"] for item in fingers.values())))
    passed = (len(indices) == count and np.all(np.diff(steps[indices]) == 1)
              and all(row["bilateral_loaded"] for row in summaries))
    return dict(passed=bool(passed), phase=phase, requested_consecutive_substeps=count, sampled_substeps=len(indices),
        steps=summaries, contact_region_object_local_m=region.tolist(), maximum_frame_transform_error=max_transform_error,
        threshold_n=.01, inequality="positive normal force > 0.01 N; all loaded contacts inside source-derived region",
        scope="Measured bilateral loading only; not lift, release, original task success, or global safety")


def raw_destination(state, force, object_center, destination_pose, destination_bounds, force_threshold=.1,
                    velocity_threshold=.003, cone=np.pi/4):
    """Original strict speed / inclusive support / footprint semantics, double diagnostic arithmetic."""
    center_w = state[:3]+Rotation.from_quat(state[3:7]).apply(object_center)
    center = Rotation.from_quat(destination_pose[3:7]).inv().apply(center_w-destination_pose[:3])
    magnitude, speed = float(np.linalg.norm(force)), float(np.linalg.norm(state[7:10]))
    low, high = np.asarray(destination_bounds)
    geometry = bool(np.all(center[:2] >= low[:2]) and np.all(center[:2] <= high[:2]) and center[2] >= low[2])
    support = bool(magnitude >= force_threshold and force[2] > 0. and force[2] >= magnitude*np.cos(cone))
    return dict(passed=bool(geometry and support and speed < velocity_threshold), center_in_destination_frame_m=center.tolist(),
        source_footprint=geometry, upward_support=support, linear_speed_m_s=speed, force_world_n=force.tolist(),
        force_magnitude_n=magnitude, force_threshold_n=force_threshold, velocity_threshold_m_s=velocity_threshold,
        support_cone_half_angle_rad=cone)


def sequence_audit(data, metadata, trajectory, result, close, lift):
    names = metadata["task_object_names"]
    pick, _ = TASKS[metadata["task_id"]]
    states = data["task_object_states"][:, names.index(pick)]
    phases = data["phase"]
    settle = np.flatnonzero(phases == "initial_settle")
    require(len(settle) > 0, "Missing physical initial settling phase")
    reference = int(settle[-1])
    lift_ids = np.flatnonzero(phases == "lift")
    release = np.flatnonzero(phases == "release")
    final = np.flatnonzero(phases == "final_settle")
    max_lift = float((states[lift_ids, 2]-states[reference, 2]).max()) if len(lift_ids) else None
    ordered = bool(close["passed"] and lift["passed"] and len(release) and len(final) and
                   reference < close["steps"][0]["physics_step"]-data["physics_step"][0] < lift_ids[0] < release[0] < final[0])
    actions = np.asarray(trajectory["actions"])
    open_actions = trajectory["phase"] == "release"
    finger_id = metadata["joint_names"].index("finger_joint")
    released = bool(open_actions.any() and np.all(actions[open_actions, -1] == 0.) and
                    len(final) and abs(data["q"][final[-1], finger_id]) < .03)
    terminal = result.get("terminal", {})
    success = terminal.get("first_success")
    checks = dict(physical_close_bilateral=close["passed"], physical_lift_bilateral=lift["passed"],
        physical_lift_at_least_50mm=max_lift is not None and max_lift >= .05,
        physical_close_lift_release_order=ordered, actual_release_command_and_open_finger=released,
        original_success_reported=result.get("original_task_success") is True,
        original_final_predicate_reported=result.get("final_original_destination_predicate") is True,
        sequence_completed=result.get("physical_sequence_complete") is True and result.get("status") == "sequence_complete",
        first_success_state_available=isinstance(success, dict))
    return dict(passed=all(checks.values()), checks=checks, initial_reference_physics_step=int(data["physics_step"][reference]),
        lift_physics_samples=len(lift_ids), maximum_actual_lift_m=max_lift, release_physics_samples=len(release),
        final_settle_physics_samples=len(final), final_finger_joint_rad=float(data["q"][-1, finger_id]),
        source_lift_predicate_distance_m=.01, additional_physical_lift_requirement_m=.05,
        scope="Actual recorded phase sequence. Original rest-recorder/temporal predicates require their separately bound runtime evidence.")


def first_success_audit(episode, result, data):
    terminal = result["terminal"]["first_success"]
    ids = np.flatnonzero(data["physics_step"] == terminal["physics_step"])
    require(len(ids) == 1 and terminal["terms"].get("success") is True, "Original first success has no unique actual state")
    index = int(ids[0])
    require(terminal["phase"] == data["phase"][index] and terminal["action_index"] == data["action_index"][index],
            "Original success phase/action identity mismatch")
    path = episode/"first_success_state.npz"
    with np.load(path, allow_pickle=False) as saved:
        require(set(saved.files) == set(data), "Original first-success state inventory is incomplete")
        for key, values in data.items():
            require(np.array_equal(saved[key], values[index]), "Original first success differs from recorded raw state: "+key)
    return dict(passed=True, physics_step=int(terminal["physics_step"]), sha256=sha(path))


def prefix_audit(episode, metadata, data):
    report = read(episode/"startup_audit.json")
    indices = np.flatnonzero(np.isin(data["phase"], ["reset", "warmup", "initial_settle"]))
    require(len(indices) and np.array_equal(indices, np.arange(len(indices))), "Startup is not a complete contiguous prefix")
    with np.load(episode/"startup_physics_states.npz", allow_pickle=False) as saved:
        require(set(saved.files) == set(data), "Startup/full state fields differ")
        for key in data:
            require(np.array_equal(saved[key], data[key][indices]), "Startup differs from full physical record: "+key)
    require(report["task_object_names"] == metadata["task_object_names"] and report["samples"] == len(indices)
        and report["first_physics_step"] == int(data["physics_step"][0])
        and report["last_physics_step"] == int(data["physics_step"][indices[-1]])
        and report["continuous"] is True, "Startup summary differs from exact raw prefix")
    duration = (data["physics_step"][indices[-1]]-data["physics_step"][0])*metadata["physics_dt_s"]
    require(abs(report["duration_s"]-duration) < 1e-9 and duration >= 5., "Startup duration is incomplete")
    return dict(passed=True, physics_samples=len(indices), duration_seconds=float(duration))


def timeline_audit(data, metadata, trajectory, actions):
    require(len(actions) == len(trajectory["actions"]), "Action JSON/NPZ counts differ")
    decimation = round(metadata["action_dt_s"]/metadata["physics_dt_s"])
    require(data["phase"][0] == "reset" and data["action_index"][0] == -1, "Initial sample is not reset")
    for i, row in enumerate(actions):
        for name in ("phase", "before_physics_step", "after_physics_step", "camera_physics_steps"):
            require(np.array_equal(np.asarray(row[name]), np.asarray(trajectory[name][i])), "Action JSON/NPZ mismatch: "+name)
        require(np.array_equal(np.asarray(row["action"]), trajectory["actions"][i]), "Recorded action command mismatch")
        ids = slice(1+i*decimation, 1+(i+1)*decimation)
        require(np.all(data["phase"][ids] == row["phase"]) and np.all(data["action_index"][ids] == i),
                "Recorded substep phase/action does not match executed action interval")
    return dict(passed=True, actions=len(actions), physics_samples=len(data["q"]), decimation=decimation)


def video_audit(episode, metadata, trajectory):
    import h5py
    import io
    from PIL import Image
    from kitchen_safety.dataset import CAMERA_NAMES, RGB_ALIGNMENT, decode_video
    frames = len(trajectory["actions"])
    files, image_reports = {}, {}
    for folder, names in [(episode, CAMERA_NAMES), (episode/"overview", ("overview_camera",))]:
        config = read(folder/"cameras.json")
        require(set(config) == set(names), "Camera inventory changed")
        with h5py.File(folder/"images.h5", "r") as images:
            require(int(images.attrs["frames"]) == frames and images.attrs["alignment"] == RGB_ALIGNMENT
                and abs(float(images.attrs["dt"])-metadata["action_dt_s"]) < 1e-9, "RGB timeline differs")
            for name in names:
                group, camera = images[name], config[name]
                require(len(group["jpeg"]) == frames and group["position_world"].shape == (frames, 3)
                    and group["quaternion_world_ros_xyzw"].shape == (frames, 4), "Missing RGB/frame pose samples")
                require(np.isfinite(group["position_world"][:]).all() and
                    np.isfinite(group["quaternion_world_ros_xyzw"][:]).all(), "Invalid camera poses")
                for encoded in group["jpeg"]:
                    with Image.open(io.BytesIO(np.asarray(encoded, dtype=np.uint8).tobytes())) as image:
                        image.load()
                        require(image.size == (camera["width"], camera["height"]) and image.mode == "RGB", "Raw image dimensions/mode changed")
                image_reports[str(folder.relative_to(episode)/name)] = dict(jpeg_frames_decoded=frames,
                    images_sha256=sha(folder/"images.h5"), cameras_sha256=sha(folder/"cameras.json"))
                path = folder/(name+".mp4")
                count = decode_video(path, frames, (camera["width"], camera["height"]), metadata["action_dt_s"])
                files[str(path.relative_to(episode))] = dict(frames=count, sha256=sha(path))
            path = folder/"all_cameras.mp4"
            count = decode_video(path, frames, (640*len(names), 360), metadata["action_dt_s"])
            files[str(path.relative_to(episode))] = dict(frames=count, sha256=sha(path))
    return dict(passed=True, raw_images=image_reports, video_decode=files,
                scope="Every saved JPEG and every video frame decoded; semantic visibility and task success are not inferred.")


def bound_sidecar(path, episode, kind, data, metadata):
    report = read(path)
    expected = {name: sha(episode/name) for name in ("physics_states.npz", "physics_metadata.json", "trajectory.npz")}
    require(report.get("kind") == kind and report.get("input_sha256") == expected, "Sidecar kind or exact input identity differs")
    require(report.get("passed") is True, "Sidecar records a failure")
    if kind in ("fixed", "self-collision"):
        require(report.get("physics_samples") == report.get("checked_samples") == len(data["q"]), "Sidecar omits physical samples")
    if kind == "fixed":
        require(report.get("complete") is True and report.get("failures") == []
            and report["source_and_base"]["passed"] is True, "Fixed-base source evidence is incomplete")
        from scripts.extensions.replicator.audit_fixed_base import ASSEMBLY
        require(report["assembly_sha256"] == sha(ASSEMBLY) and
            report["kitchen_source_sha256"] == sha(ROOT/metadata["source_layout"]["kitchen_usd"]), "Fixed-base source assets changed")
    if kind == "self-collision":
        require(report.get("recorded_positions_modified") is False and report.get("mimic_angles_substituted") is False,
                "Self-collision substituted or clamped actual joints")
        require(report.get("joint_names") == metadata["joint_names"] and report.get("violations") == []
            and report.get("maximum_self_collision_constraint") == 0.
            and report.get("maximum_joint_limit_excess_rad", 1.) <= 1e-5, "Actual self-collision/limit evidence failed")
        expected_fingers = {"finger_joint", "right_outer_knuckle_joint", "left_inner_finger_joint", "right_inner_finger_joint",
                           "left_inner_finger_knuckle_joint", "right_inner_finger_knuckle_joint"}
        require(set(report["locked_measured_joint_names"]) == expected_fingers, "Measured finger inventory incomplete")
        require(report["sphere_model_sha256"] == sha(ROOT/"assets/curobo/droid/franka_robotiq_2f_85_zero_curobo.yml"),
                "Self-collision sphere model changed")
    return dict(passed=True, path=str(path), sha256=sha(path), report=report)


def neutral_audit(directory, metadata, result, data):
    from scripts.extensions.replicator.review_neutral_evidence import load_neutral
    info, raw = load_neutral(directory, require_full_physics=True)
    require(info["task_id"] == metadata["task_id"] and info["layout"] == metadata["source_layout"], "Neutral source task/layout differs")
    for key in ("source_sha256", "recipe_sha256", "overlay_sha256"):
        require(info["background_physics"][key] == metadata["background_physics"][key], "Neutral physics profile differs: "+key)
    require(info["background_topology"] == metadata["background_topology"], "Neutral background topology differs")
    require(info["task_reference_paths"] == metadata["task_reference_paths"] and
            info["static_anchor_paths"] == metadata["static_anchor_paths"], "Neutral reference identity differs")
    for key in ("source_layout",):
        require(metadata[key] == result["layout"], "Result/physics layout differs")
    config, threads = info["execution_configuration"], result["runtime_threads"]
    require(config["physics_threads"] == threads["live_physics_threads"] == threads["requested_physics_threads"] and
        config["torch_threads"] == threads["live_torch_threads"] == threads["requested_torch_threads"] and
        config["simulation_device"] == threads["actual_environment_device"] == threads["requested_physics_device"],
        "Neutral runtime device/threads differ")
    require(info["seed"] == result["seed"] and info["physics_dt"] == metadata["physics_dt_s"]
        and info["video_dt"] == metadata["action_dt_s"], "Neutral seed/period differs")
    duration = float((data["physics_step"][-1]-data["physics_step"][0])*metadata["physics_dt_s"])
    require(info["duration_seconds"] >= duration, "Neutral observation is shorter than actual episode")
    return dict(passed=True, full_physics_samples=len(raw["physics_step"]), neutral_duration_s=info["duration_seconds"],
                episode_duration_s=duration, input_sha256={name: sha(directory/name) for name in ("neutral_probe.json", "neutral_states.npz")})


def review(episode, neutral, *, fixed_audit=None, self_audit=None, visual_review=None):
    from scripts.extensions.replicator.task_audit_sidecars import load_episode, source_audit
    from scripts.extensions.replicator.task_camera_audit import authored_camera_audit
    from kitchen_safety.extensions.replicator.task_recorder import audit_physics
    episode, neutral = Path(episode), Path(neutral)
    output = dict(schema_version=1, episode=str(episode), neutral=str(neutral), passed=False, accepted=False,
        checks={}, details={}, failed_or_missing={}, input_sha256={}, visual_review="Not performed by this tool.",
        limitations=["This is an independent evidence gate, not a physics run or automatic delivery acceptance.",
            "Live FCL collector counters are reported separately from independent source/fixed/self checks.",
            "Source-USD destination geometry is diagnostic until actual spawned predicate bounds and sensor identity are recorded."])
    for name in CORE_INPUTS:
        if (episode/name).is_file():
            output["input_sha256"][name] = sha(episode/name)
        else:
            output["failed_or_missing"][name] = "Required input missing"
    def gate(name, fn):
        try:
            value = fn()
            passed = value.get("passed") is True
            output["details"][name] = value
            if not passed:
                output["failed_or_missing"][name] = value.get("reason", "Recomputed check failed; see details")
        except (OSError, ValueError, KeyError, TypeError, IndexError, AssertionError, StopIteration) as exc:
            passed = False
            output["failed_or_missing"][name] = f"{type(exc).__name__}: {exc}"
        output["checks"][name] = passed
        return output["details"].get(name, {"passed": False})
    try:
        metadata, data, trajectory = load_episode(episode)
        result, actions, contacts = read(episode/"result.json"), read(episode/"actions.json"), read(episode/"grasp_contacts.json")
        require(metadata["task_id"] in TASKS and result["task_id"] == metadata["task_id"], "Unsupported or mismatched task")
        output["task_id"], output["variant"] = metadata["task_id"], result.get("variant")
        require(result.get("variant") == "base", "Added-hazard task-aware schema is not yet audited; never accept by renaming a baseline")
        output["checks"]["complete_finite_reset_physics"] = True
    except (OSError, ValueError, KeyError, TypeError) as exc:
        output["checks"]["complete_finite_reset_physics"] = False
        output["failed_or_missing"]["complete_finite_reset_physics"] = str(exc)
        return output
    pick, destination = TASKS[metadata["task_id"]]
    gate("all_action_and_physics_identities", lambda: timeline_audit(data, metadata, trajectory, actions))
    gate("startup_matches_full_reset_record", lambda: prefix_audit(episode, metadata, data))
    gate("physical_coverage_background_contacts", lambda: audit_physics(data, metadata, actions, contacts))
    gate("original_source_relations_and_fixed_pose", lambda: source_audit(metadata, data))
    gate("original_camera_mount_intrinsics_and_physics_alignment", lambda: authored_camera_audit(episode, metadata, data, trajectory))
    gate("all_raw_images_and_videos_decode", lambda: video_audit(episode, metadata, trajectory))
    gate("matching_complete_neutral", lambda: neutral_audit(neutral, metadata, result, data))
    region = None
    def source_contact_region():
        nonlocal region
        from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
        from kitchen_safety.extensions.replicator.task_motion import banana_midbody
        from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_geometry
        geometry = PayloadGeometry(ROOT/metadata["source_layout"]["objects"][pick]["usd_path"])
        info = banana_midbody(geometry) if pick == "banana" else bowl_rim_geometry(geometry)
        region = info["contact_region_local_m"]
        require(info["source_sha256"] == metadata["source_hashes"][pick] == result["grasp_target"]["source_sha256"],
                "Grasp source asset changed")
        require(region == result["grasp_target"]["contact_region_local_m"], "Recorded grasp region differs from source derivation")
        return dict(passed=True, region=region, source_sha256=info["source_sha256"])
    gate("source_contact_region", source_contact_region)
    close = gate("physical_bilateral_close_10_substeps", lambda: contact_window(data, metadata, contacts, pick, region, "close"))
    lift = gate("physical_bilateral_lift_10_substeps", lambda: contact_window(data, metadata, contacts, pick, region, "lift"))
    gate("physical_pick_lift_release_and_original_success", lambda: sequence_audit(data, metadata, trajectory, result, close, lift))
    gate("original_first_success_actual_state", lambda: first_success_audit(episode, result, data))
    for kind in ("fixed", "self-collision"):
        gate("independent_"+kind.replace("-", "_")+"_sidecar", lambda kind=kind: bound_sidecar(
            (fixed_audit if kind == "fixed" else self_audit) or episode/("task_"+kind.replace("-", "_")+"_audit.json"), episode, kind, data, metadata))
    safety = result.get("safety", {})
    output["details"]["recorded_execution_FCL"] = dict(recorded_no_failure=safety.get("failure") is None,
        recorded_checked_physics_samples=safety.get("checked_physics_samples"), actual_physics_samples=len(data["q"]),
        recorded_full_coverage=safety.get("checked_physics_samples") == len(data["q"]), failure=safety.get("failure"),
        scope="Collector's live actual-state FCL counter only; no independent replay or self-collision conclusion.")
    gate("recorded_live_execution_collision_checks", lambda: dict(passed=(safety.get("failure") is None
        and safety.get("checked_physics_samples") == len(data["q"])),
        scope="Runtime actual-state FCL counters, bound separately by the complete execution provenance. Not an independent FCL replay."))
    gate("collector_completed_physical_trial", lambda: dict(passed=(result.get("physical_trial_passed") is True
        and result.get("fixed_only") is True and result.get("physics_state_setters_used") is False
        and result.get("physical_weld_used") is False and result.get("planning_attachment_only") is True),
        scope="Collector declarations; independently measured sequence, original predicate, and code evidence remain separate gates."))
    output["details"]["rejected_plans"] = [dict(phase=row.get("phase"), status=row.get("status"), error=row.get("error"),
        candidate_file=row.get("rejected_candidate_file"), executed=row.get("rejected_candidate_executed"))
        for row in result.get("plans", []) if row.get("status") != "executed"]
    gate("raw_original_final_support_speed_geometry", lambda: final_predicate_diagnostic(metadata, data, episode))
    from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance, audit_predicate_evidence
    gate("actual_spawned_predicate_geometry_and_sensor_identity", lambda: audit_predicate_evidence(
        episode, metadata, data, trajectory, result))
    gate("complete_execution_provenance", lambda: audit_provenance(
        episode/"task_collection_provenance.json", metadata, data["physics_step"][-1]))
    gate("independent_visual_review", lambda: visual_evidence(visual_review, episode, trajectory))
    output["review_code_sha256"] = {str(Path(__file__).relative_to(ROOT)): sha(__file__),
        **{name: sha(ROOT/name) for name in (
            "scripts/extensions/replicator/task_audit_sidecars.py", "scripts/extensions/replicator/task_camera_audit.py",
            "kitchen_safety/extensions/replicator/task_collection_evidence.py", "kitchen_safety/extensions/replicator/task_recorder.py",
            "scripts/extensions/replicator/review_neutral_evidence.py")}}
    output["source_predicate_sha256"] = {str(path.relative_to(ROOT)): sha(path) for path in
        (SOURCE_DIR/"tasks/pick_and_place_task.py", SOURCE_DIR/"tasks/predicates/spatial.py", SOURCE_DIR/"tasks/predicates/object_settling.py")}
    output["passed"] = bool(output["checks"]) and all(output["checks"].values()) and not output["failed_or_missing"]
    return output


def visual_evidence(path, episode, trajectory):
    require(path is not None, "No separate actual visual inspection supplied; this tool never creates one")
    report = read(path)
    require(report.get("visual_sample_passed") is True and isinstance(report.get("scope"), str)
        and report.get("observations") and report.get("reviewer"), "Visual observations/decision absent")
    frames = report["frames"]
    require(frames and all(type(i) is int and 0 <= i < len(trajectory["actions"]) for i in frames), "Visual sample identities invalid")
    require({"external_camera_2", "wrist_camera", "overview_camera"} <= set(report["cameras"]), "Visual original/wrist/overview evidence incomplete")
    require({"close", "lift", "release", "final_settle"} <= set(trajectory["phase"][frames]), "Visual samples omit grasp/lift/release/final phases")
    for name in ("result.json", "physics_states.npz", "images.h5", "overview/images.h5"):
        require(report["sha256"][name] == sha(episode/name), "Visual evidence refers to different raw input: "+name)
    return dict(passed=True, path=str(path), sha256=sha(path), frames=frames, cameras=report["cameras"], scope=report["scope"])


def final_predicate_diagnostic(metadata, data, episode):
    from isaaclab_arena.utils.usd.helpers import compute_local_bounding_box_from_usd
    from scripts.extensions.replicator.audit_task_source_relation import source
    from scripts.extensions.replicator.review_episode import source_thresholds
    _, original = source(metadata["task_id"])
    params = original["task"]["subtasks"][0]["params"]
    pick, destination = TASKS[metadata["task_id"]]
    require(params["pick_up_object"] == pick and params["destination_location"] == destination, "Original task role mismatch")
    if destination not in metadata["task_object_names"]:
        # A sink is an original ObjectReference. Never substitute a bowl state.
        info = read(episode/"task_predicate_metadata.json")
        require(info["task_id"] == metadata["task_id"] and info["destination_location"] == destination,
                "Static destination evidence belongs to a different original task")
        references = {row["id"]: "/World/envs/env_0/kitchen/"+row["prim_path"] for row in original["object_references"]}
        matrix = data["task_reference_transforms"][-1, metadata["task_reference_paths"].index(references[destination])]
        pose = np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()]
        b = info["actual_spawned_local_bounds"]
        center = (np.asarray(b[pick]["min"])+b[pick]["max"])/2
        defaults = source_thresholds()
        value = raw_destination(data["task_object_states"][-1, metadata["task_object_names"].index(pick)],
            data["destination_contact_force"][-1, 0, 0], center, pose,
            [b[destination]["min"], b[destination]["max"]], defaults.force_threshold, defaults.velocity_threshold,
            defaults.support_cone_half_angle_rad)
        value["scope"] = "Actual static ObjectReference transform and recorded spawned bounds; identity and original temporal success require the separate complete predicate-evidence gate."
        return value
    bounds = {}
    for name in (pick, destination):
        box = compute_local_bounding_box_from_usd(str(ROOT/metadata["source_layout"]["objects"][name]["usd_path"]))
        bounds[name] = np.stack([box.min_point.numpy()[0], box.max_point.numpy()[0]])
    thresholds = source_thresholds()
    value = raw_destination(data["task_object_states"][-1, metadata["task_object_names"].index(pick)],
        data["destination_contact_force"][-1, 0, 0], bounds[pick].mean(axis=0),
        data["task_object_states"][-1, metadata["task_object_names"].index(destination), :7], bounds[destination],
        params.get("force_threshold", thresholds.force_threshold), params.get("velocity_threshold", thresholds.velocity_threshold),
        params.get("support_cone_half_angle_rad", thresholds.support_cone_half_angle_rad))
    value["scope"] = "Raw final support/speed and original source-USD geometry diagnostic; not equivalence to omitted runtime spawned bounds or original temporal sequence."
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--neutral", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixed-audit", type=Path)
    parser.add_argument("--self-audit", type=Path)
    parser.add_argument("--visual-review", type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("Refuse to overwrite prior evidence")
    report = review(args.episode, args.neutral, fixed_audit=args.fixed_audit, self_audit=args.self_audit, visual_review=args.visual_review)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(passed=report["passed"], accepted=False, checks=report["checks"],
        failed_or_missing=report["failed_or_missing"], output=str(args.output))))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
