#!/usr/bin/env python3
"""Replay retained bowl lift PLANNING constraints, never physical actions.

Reconstruct the actual close-end state, six fingers, measured attachment and
background. An unchanged recorded hazard layout uses its measured pose. A new
placement requires --hypothetical-placement and never claims a measured scene.
All original support partitions and source FCL gates remain authoritative.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_paired_query import ROOT, TASK, digest, inventory, require, sha, write
from scripts.extensions.replicator.paired_bowl_safety_plans import execution_inputs, frozen_input_check, source_asset_inputs


def lift_boundary(result, data):
    plans = [(i, row) for i, row in enumerate(result["plans"], 1) if row["phase"] == "lift"]
    require(len(plans) == 1, "Exactly one original lift query required")
    number, plan = plans[0]
    indices = np.flatnonzero(data["physics_step"] == plan["start_physics_step"])
    require(len(indices) == 1, "Lift boundary must match a unique actual state")
    index = int(indices[0])
    closes = np.flatnonzero(data["phase"] == "close")
    require(len(closes) and index == int(closes[-1]) and str(data["phase"][index]) == "close",
            "Lift query must use the final close state, not the first lift state")
    if plan["status"] == "failed":
        require(plan.get("rejected_candidate_executed") is False and plan.get("observed_candidate_actions") == 0 and
            plan.get("observed_candidate_physics_steps") == 0 and plan.get("failure_stage") == "planning_or_validation",
            "Only an unexecuted, retained planning rejection is supported here")
        require(index == len(data["physics_step"]) - 1, "Rejected lift must end at its actual closed query state")
    else:
        require(plan["status"] == "executed" and result.get("status") == "sequence_complete" and
                result.get("physical_trial_passed") is True and result.get("original_task_success") is True,
                "Successful baseline must complete the original physical task")
        require(index + 1 < len(data["phase"]) and str(data["phase"][index + 1]) == "lift",
                "Successful lift does not follow its actual boundary")
    return index, number, plan


def candidate_path(episode, number, plan, q, fingers):
    if plan["status"] == "failed":
        name = plan["rejected_candidate_file"]
        require(Path(name).name == name and name.endswith(".npz"), "Rejected candidate must be an episode-local NPZ")
        path = episode / name
        require(sha(path) == plan["rejected_candidate_sha256"], "Retained rejected candidate SHA differs")
    else:
        path = episode / f"plan_{number:02d}_lift.npz"
    with np.load(path, allow_pickle=False) as raw:
        arrays = {name: raw[name].copy() for name in raw.files}
    positions = arrays["positions"]
    require(positions.ndim == 2 and positions.shape[1] == 7 and len(positions) >= 2 and np.isfinite(positions).all(),
            "Retained finite original seven-arm candidate required")
    require(np.array_equal(arrays["start_q"], q) and np.array_equal(positions[0], q),
            "Candidate starts at another joint state; no endpoint splicing allowed")
    if plan["status"] == "failed":
        require(np.array_equal(arrays["measured_gripper_angles"], fingers), "Rejected candidate actual six-finger binding differs")
        require(np.array_equal(arrays["requested_target"], plan["target_tcp"]), "Rejected candidate target changed")
    else:
        require(np.array_equal(arrays["goal_tcp"], plan["target_tcp"]), "Successful retained lift target changed")
    return path, arrays


def placement_mode(layout, recorded_layout, hypothetical):
    same = layout == recorded_layout
    require(same != bool(hypothetical),
            "Changed layout requires explicit hypothetical placement; unchanged layout must use the actual hazard pose")
    return "actual_recorded_hazard" if same else "hypothetical_placement_only"


def obstacle_groups(enabled, support, hazard_names):
    require(len(enabled) == len(set(enabled)) and support in enabled, "Original enabled-world inventory/support missing")
    hazards = [name for name in enabled if name in set(hazard_names)]
    require(hazards and support not in hazards, "Nonempty exact hazard set must not include the source support")
    others = [name for name in enabled if name != support and name not in hazards]
    require(not set(hazards) & set(others) and set(hazards + others) == set(enabled) - {support},
            "Payload/non-support split must be disjoint and exhaustive")
    return hazards, others


def isolated_payload_costs(planner, state, original_audit, hazard_name):
    """Read-only attribution; restore every original CUDA obstacle in finally."""
    rollout = planner.motion_gen.ik_solver.rollout_fn
    cost = rollout.primitive_collision_constraint
    checker = cost.world_coll_checker
    config = rollout.dynamics_model.robot_model.kinematics_config
    reverse = {value: key for key, value in config.link_name_to_idx_map.items()}
    labels = [reverse[int(i)] for i in config.link_sphere_idx_map.detach().cpu().numpy()]
    payload = [i for i, label in enumerate(labels) if label == "attached_object"]
    robot = [i for i, label in enumerate(labels) if label != "attached_object"]
    require(payload and robot and sorted(payload + robot) == list(range(len(labels))), "Incomplete original sphere partition")
    masks = planner.candidate_contact_mask["object_switches"]
    require(masks == ["bowl"], "Only original duplicate carried-bowl candidate mask is allowed")
    enabled = [name for name in planner.obstacle_names if not planner._ignored(name, masks)]
    support = planner.world.contact_policy.source_support
    hazards, others = obstacle_groups(enabled, support,
        [name for name in enabled if planner.world._matches(name, hazard_name)])
    rows, restored = {}, False

    def evaluate(names):
        query = state.robot_spheres.clone()
        query[..., robot, 3] = -100.  # Diagnostic payload-only query, never the authoritative gate.
        for name in planner.obstacle_names:
            checker.enable_obstacle(name, enable=name in names)
        values = cost.forward(query.contiguous(), env_query_idx=None).detach().cpu().numpy()
        values = values.reshape(len(values), -1).max(axis=1)
        require(np.isfinite(values).all(), "Nonfinite isolated original CUDA cost")
        return dict(costs=values.tolist(), maximum_cost=float(values.max()),
            nonzero_samples=np.flatnonzero(values != 0.).tolist(), enabled_obstacles=list(names),
            active_sphere_ids=payload)

    try:
        rows["payload_hazard"] = evaluate(hazards)
        rows["payload_other_non_support"] = evaluate(others)
        per_collider = {name: evaluate([name]) for name in hazards} if rows["payload_hazard"]["nonzero_samples"] else {}
    finally:
        for name in planner.obstacle_names:
            checker.enable_obstacle(name, enable=name in enabled)
        restored = True
    union = np.logical_or(np.asarray(rows["payload_hazard"]["costs"]) != 0.,
                          np.asarray(rows["payload_other_non_support"]["costs"]) != 0.)
    original = original_audit["original_world_pair_partition"]["rows"]["payload_non_support"]
    require(np.array_equal(union, np.asarray(original["costs"]) != 0.),
            "Hazard/non-hazard attribution does not reproduce the original payload/non-support gate")
    if per_collider:
        require(np.array_equal(np.logical_or.reduce([np.asarray(row["costs"]) != 0. for row in per_collider.values()]),
                               np.asarray(rows["payload_hazard"]["costs"]) != 0.),
                "Per-collider attribution differs from the original hazard subset")
    return dict(rows=rows, per_hazard_collider=per_collider, disjoint_exhaustive=True,
        zero_nonzero_equivalence=True, original_enabled_restored=restored,
        authoritative_gate_unchanged=True, scope="Attribution only. An isolated pass never excuses any original rejection.")


def prepare(args):
    from kitchen_safety.extensions.replicator.bowl_safety_policy import load_layout
    from kitchen_safety.extensions.replicator.bowl_safety_cpu import build_world
    from kitchen_safety.extensions.replicator.bowl_safety_recorder import audit_hazard
    from kitchen_safety.extensions.replicator.bowl_controller_profile import read_profile
    from kitchen_safety.extensions.replicator.bowl_source_support_query import bind_observed_departure, source_departure_geometry
    from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from scripts.extensions.replicator.task_audit_sidecars import load_episode
    meta, data, _ = load_episode(args.episode)
    result = json.loads((args.episode / "result.json").read_text())
    require(meta["task_id"] == result["task_id"] == TASK and meta["task_object_names"] == ["bowl"], "Original bowl schema required")
    require(meta["action_dt_s"] == 1/15 and meta["physics_dt_s"] == 1/120,
            "Original physical/action sampling required")
    require(result["physics_state_setters_used"] is False and result["physical_weld_used"] is False,
            "No setter/weld episode may define this diagnostic")
    profile = read_profile()
    require(result["controller_profile"] == profile, "Original controller input changed")
    candidate = json.loads(args.layout.read_text())
    variant = candidate["obstacle"]["category"]
    layout, policy = load_layout(args.layout, variant)
    canonical = {k: v for k, v in layout.items() if k != "obstacle"}
    require({k: v for k, v in meta["source_layout"].items() if k != "obstacle"} == canonical,
            "Original source layout changed")
    mode = placement_mode(layout, result["layout"], args.hypothetical_placement)
    index, number, plan = lift_boundary(result, data)
    if "obstacle" in meta["source_layout"]:
        from kitchen_safety.extensions.replicator.bowl_safety_provenance_review import audit
        provenance = audit(args.episode / "task_collection_provenance.json", meta, result,
            data["physics_step"][-1], args.output / "derived_original_task_provenance.json")
    else:
        provenance = audit_provenance(args.episode / "task_collection_provenance.json", meta, data["physics_step"][-1])
    world, kin, _ = build_world(layout, policy)
    arm_ids = [meta["joint_names"].index(f"panda_joint{i}") for i in range(1, 8)]
    finger_ids = [meta["joint_names"].index(name) for name in world.GRIPPER_JOINT_NAMES]
    q, fingers = data["q"][index, arm_ids].copy(), data["q"][index, finger_ids].copy()
    path_file, arrays = candidate_path(args.episode, number, plan, q, fingers)
    world.sync_background(meta["background_paths"], data["background_rigid_poses"][index])
    for name, pose in zip(meta["background_paths"], data["background_rigid_poses"][index]):
        if any(world._matches(row[0], name) for row in world.objects):
            require(name in world._background_poses and np.array_equal(world._background_poses[name], pose),
                    "Collision-bearing measured background was not synchronized")
    actual_bowl = data["task_object_states"][index, 0, :7]
    actual_tcp = pose_matrix(data["tcp_pose_w"][index])
    world.payload_pose = pose_matrix(actual_bowl)
    world.set_obstacle_pose("bowl", actual_bowl)
    world.gripper_joint_positions, world.gripper_joint_position = fingers.copy(), float(fingers[0])
    world.attachment = np.linalg.inv(actual_tcp) @ world.payload_pose
    require(np.allclose(world.attachment, plan["measured_planning_attachment"], atol=1e-12, rtol=0.),
            "Saved lift attachment differs from actual TCP-to-bowl relation")
    world.set_phase("lift")
    hazard_evidence = dict(mode=mode, hypothetical=mode != "actual_recorded_hazard")
    if mode == "actual_recorded_hazard":
        require(result["variant"] == variant, "Recorded hazard category differs")
        saved = json.loads((args.episode / "hazard_audit.json").read_text())
        raw = json.loads((args.episode / "hazard_contacts.json").read_text())
        require(raw["metadata"]["policy"] == result["safety_policy"] == policy, "Recorded native hazard policy differs")
        audit = audit_hazard(data, raw["metadata"], raw["rows"])
        require(saved == audit and audit["passed"] is True, "Recorded prefix hazard completeness/stability audit failed")
        hazard_pose = data["hazard_pose_w"][index].copy()
        hazard_evidence.update(independent_hazard_prefix_audit=audit, physics_step=int(data["physics_step"][index]),
            pose_source="Retained actual closed-boundary hazard observation; failed task prefix is not a successful episode")
    else:
        obstacle = layout["obstacle"]
        hazard_pose = np.r_[obstacle["position"], Rotation.from_euler("z", obstacle.get("yaw_deg", 0), degrees=True).as_quat()]
        hazard_evidence.update(pose_source="Explicit hypothetical candidate placement; all robot/payload/background observations unchanged",
                               actual_hazard_state_claimed=False)
    world.set_obstacle_pose(policy["hazard_name"], hazard_pose)
    contacts = json.loads((args.episode / "grasp_contacts.json").read_text())
    contacts = [row for row in contacts if row["physics_step"] <= int(data["physics_step"][index])]
    context = bind_observed_departure(world, kin, physics_step=int(data["physics_step"][index]), arm_q=q,
        measured_tcp=actual_tcp, measured_payload=world.payload_pose, raw_contact_rows=contacts)
    require(json.loads(json.dumps(context.evidence())) == plan["source_departure_binding"],
            "Actual source-departure binding differs from the retained original")
    positions = arrays["positions"]
    source = source_departure_geometry(world, kin, positions)
    fcl = []
    for i, state in enumerate(positions):
        valid = world.valid(state, mode="planning", gripper_q=fingers)
        fcl.append(dict(sample=i, passed=bool(valid), measurement=deepcopy(world.last_clearance)))
    report = dict(accepted=False, physical_execution=False, task_success=False,
        candidate_kind="retained_rejected_lift" if plan["status"] == "failed" else "successful_baseline_lift",
        original_episode_status=result["status"], original_lift_status=plan["status"],
        actual_start_physics_step=int(data["physics_step"][index]), actual_array_row=index,
        actual_source_binding=context.evidence(), original_plan=plan, candidate_file=str(path_file), candidate_sha256=sha(path_file),
        candidate_samples=len(positions), source_support_geometry=source, source_fcl=dict(rows=fcl, checked=len(fcl),
            passed=all(r["passed"] for r in fcl)), hazard_evidence=hazard_evidence, hazard_pose=hazard_pose,
        layout=layout, policy=policy, background_paths=meta["background_paths"],
        background_actual_pose_sha256=digest(data["background_rigid_poses"][index].tolist()),
        provenance=provenance, input_sha256=inventory(args.episode), layout_sha256=sha(args.layout),
        source_padding_rule="Lift has original zero transport-only padding; all original robot/self and non-support payload spheres remain unchanged",
        gripper_model_scope="Original lift CUDA uses measured finger_joint with its unchanged mimic rule; source FCL uses all six independently measured fingers. The actual-six CUDA transport override does not apply to lift.",
        scope="Planning reconstruction of actual measured close-end state, not new physical execution. A failed original episode remains failed.")
    return world, kin, context, arrays, report


def run(args):
    captured = execution_inputs()
    write(args.output / "preflight_inputs.json", dict(execution_source_sha256=captured,
        input_sha256=inventory(args.episode), layout_sha256=sha(args.layout), accepted=False))
    world, kin, context, arrays, report = prepare(args)
    report["execution_source_sha256"] = captured
    report["source_asset_sha256"] = source_asset_inputs(SimpleNamespace(
        source_layout={k:v for k,v in report["layout"].items() if k!="obstacle"},
        layout=report["layout"], policy=report["policy"]))
    write(args.output / "cpu_reconstruction.json", report)
    np.savez_compressed(args.output / "retained_lift_query.npz", **arrays,
        actual_tcp=np.asarray(context.measured_tcp).reshape(4,4), actual_payload=np.asarray(context.measured_payload).reshape(4,4),
        actual_attachment=np.asarray(context.measured_attachment).reshape(4,4), hazard_pose=report["hazard_pose"])
    if args.dry_run:
        require("torch" not in sys.modules and not any(name.startswith(("omni", "isaacsim")) for name in sys.modules),
                "CPU dry-run unexpectedly initialized a GPU/Kit framework")
        report.update(status="cpu_reconstruction_complete", gpu_run=False, cuda_validated=False)
    else:
        from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
        from kitchen_safety.extensions.replicator.bowl_source_support_query import audit_source_departure
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components
        planner = BowlTransportCuroboPlanner(world, kin, dt=1/15, seed=42, max_joint_speed=.4,
            transport_object_padding_m=.015 if report["policy"]["variant"] == "human" else 0.)
        planner.departure_context = context
        planner._prepare_path_query((), 1, "lift")
        original, state = constraint_components(planner, arrays["positions"])
        exact = audit_source_departure(planner, arrays["positions"], context)
        report["original_retained_candidate_audit"] = exact
        report["original_payload_geometry"] = planner.payload_geometry_report
        report["attribution"] = isolated_payload_costs(planner, state, exact, report["policy"]["hazard_name"])
        restored, _ = constraint_components(planner, arrays["positions"])
        require(restored == original, "Isolated diagnostics did not exactly restore original enabled CUDA queries")
        report["attribution"]["restored_original_components_exactly_equal"] = True
        report["original_dense_validation"] = planner.validate_path(arrays["positions"], grip_binary=1, phase="lift", resolution=.02)
        report.update(status="original_cuda_diagnostic_complete", gpu_run=True,
                      cuda_validated=report["original_dense_validation"]["valid"])
    report["execution_source_end_mismatches"] = frozen_input_check(captured)
    report["source_asset_end_mismatches"] = frozen_input_check(report["source_asset_sha256"])
    report["episode_inputs_unchanged"] = inventory(args.episode) == report["input_sha256"]
    report["layout_unchanged"] = sha(args.layout) == report["layout_sha256"]
    require(not report["execution_source_end_mismatches"] and not report["source_asset_end_mismatches"] and
            report["episode_inputs_unchanged"] and report["layout_unchanged"],
            "Diagnostic inputs changed")
    write(args.output / "diagnostic.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hypothetical-placement", action="store_true")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--gpu", action="store_true", help="Root-only scheduled original CUDA constraints, no Kit/physics")
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock = ROOT / "reports/skill_trials/gpu.lock"
            require(ROOT == Path("/workspace") and lock.is_file(), "Verified container/canonical existing lock required")
            with lock.open("a+") as handle:
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                report = run(args)
        else:
            report = run(args)
    except Exception as error:
        write(args.output / "failure.json", dict(error=repr(error), accepted=False, physical_execution=False))
        raise
    print(json.dumps(dict(output=str(args.output), status=report["status"], cuda_validated=report["cuda_validated"],
                          accepted=False, physical_execution=False)), flush=True)
    # A reproduced original rejection is a completed diagnostic, never a pass.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
