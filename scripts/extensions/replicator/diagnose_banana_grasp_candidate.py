#!/usr/bin/env python3
"""Offline source-pinch candidate, with an exhaustive cuRobo pair partition.

No physical execution or runtime edits. The changed grasp and derived attachment
are explicitly hypothetical. Default CPU; --gpu is for the root's scheduled
diagnostic. Original episode, margins, robot/payload spheres and self gates stay
unchanged. Non-support payload pairs must still pass the original sphere gate.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.diagnose_task_plan import reconstruct, cpu_sphere_audit
from scripts.extensions.replicator.search_banana_grasp_envelope_cpu import GripperSphereModel, sphere_clearance
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.task_collect import json_write
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate


def partition_world(planner, state, original):
    """Prove zero/nonzero equivalence before interpreting a support exception.

    R x W, P x (W-S), P x S is a disjoint, exhaustive partition of the exact
    original candidate query. S contains only the originally declared support.
    R/P use exact original padded sphere arrays, including inactive sentinels.
    World objects are restored in finally; joint/self always use full R+P.
    """
    rollout = planner.motion_gen.ik_solver.rollout_fn
    cost = rollout.primitive_collision_constraint
    checker = cost.world_coll_checker
    config = rollout.dynamics_model.robot_model.kinematics_config
    reverse = {value: key for key, value in config.link_name_to_idx_map.items()}
    labels = [reverse[int(i)] for i in config.link_sphere_idx_map.detach().cpu().numpy()]
    payload_ids = [i for i, name in enumerate(labels) if name == "attached_object"]
    robot_ids = [i for i, name in enumerate(labels) if name != "attached_object"]
    if not payload_ids or not robot_ids or set(payload_ids) & set(robot_ids):
        raise ValueError("Missing or overlapping original sphere partitions")
    masks = planner.candidate_contact_mask["object_switches"]
    enabled = [name for name in planner.obstacle_names if not planner._ignored(name, masks)]
    support = planner.world.contact_policy.source_support
    if support not in enabled or len(enabled) != len(set(enabled)):
        raise ValueError("Original support missing or duplicate collider identity")
    rows = {}
    groups = (("robot_all_world", payload_ids, enabled),
              ("payload_non_support", robot_ids, [name for name in enabled if name != support]),
              ("payload_source_support", robot_ids, [support]))
    try:
        for name, disabled_ids, obstacles in groups:
            query = state.robot_spheres.clone()
            query[..., disabled_ids, 3] = -100.
            active = set(obstacles)
            for obstacle in planner.obstacle_names:
                checker.enable_obstacle(obstacle, enable=obstacle in active)
            costs = cost.forward(query.contiguous(), env_query_idx=None).detach().cpu().numpy()
            costs = costs.reshape(len(costs), -1).max(axis=1)
            if not np.isfinite(costs).all():
                raise ValueError("Nonfinite original partition cost")
            rows[name] = dict(costs=costs, maximum_cost=float(costs.max()),
                nonzero_samples=np.flatnonzero(costs != 0.), enabled_obstacles=obstacles,
                active_sphere_ids=robot_ids if name == "robot_all_world" else payload_ids)
    finally:
        for obstacle in planner.obstacle_names:
            checker.enable_obstacle(obstacle, enable=obstacle in enabled)
    union = np.logical_or.reduce([np.asarray(row["costs"]) != 0. for row in rows.values()])
    original_bad = np.asarray(original["components"]["world_collision"]["costs"]) != 0.
    if not np.array_equal(union, original_bad):
        raise ValueError("Pair partition differs from original full-world zero/nonzero gate")
    noncontact = all(not len(rows[name]["nonzero_samples"]) for name in ("robot_all_world", "payload_non_support"))
    return dict(rows=rows, original_world_zero_nonzero_equivalent=True,
        noncontact_sphere_gate_passed=noncontact,
        support_pair=["attached_object", support], support_exception_applied_to_execution=False,
        sphere_partition_disjoint_exhaustive=sorted(robot_ids+payload_ids) == list(range(len(labels))),
        pair_counts=dict(robot_world=len(robot_ids)*len(enabled),
            payload_non_support=len(payload_ids)*(len(enabled)-1), payload_support=len(payload_ids),
            original_total=len(labels)*len(enabled)),
        unchanged=["All original sphere centers/radii and padding", "All enabled non-contact world pairs",
                   "Original full robot+payload self collision and joint costs"],
        coverage="R×W ∪ P×(W−S) ∪ P×S = (R∪P)×W, mutually disjoint; same original CUDA kernels and activation. Necessity of source contact does not authorize other payload pairs.")


def fcl_path(world, positions):
    rows = []
    for index, q in enumerate(positions):
        valid = world.valid(q)
        rows.append(dict(sample=index, passed=valid, violation=deepcopy(world.last_clearance)))
    return dict(passed=all(row["passed"] for row in rows), checked=len(rows), rows=rows)


def source_contacts(world, kin, q, meshes, region):
    """Source triangles at this exact candidate FK, never inferred from a pose."""
    import fcl
    source = world.payload_pose
    inverse = np.linalg.inv(source)
    payload = fcl.CollisionObject(world.payload_triangle_model,
        fcl.Transform(source[:3, :3], source[:3, 3]))
    palm = kin.links(q)[world.gripper_root]
    links = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, world.gripper_joint_positions)))
    low, high = map(np.asarray, region)
    result = {}
    for body, parts in meshes.items():
        transform = palm@links[body]
        points = []
        for _, model in parts:
            response = fcl.CollisionResult()
            fcl.collide(fcl.CollisionObject(model, fcl.Transform(transform[:3, :3], transform[:3, 3])), payload,
                fcl.CollisionRequest(num_max_contacts=4096, enable_contact=True), response)
            if len(response.contacts) >= 4096:
                raise ValueError("Source finger contact buffer saturated")
            points.extend(inverse[:3, :3]@contact.pos+inverse[:3, 3] for contact in response.contacts)
        points = np.asarray(points).reshape(-1, 3)
        result[body.rsplit("/", 1)[-1]] = dict(intersections=len(points),
            all_central=bool(len(points) and np.all(points >= low) and np.all(points <= high)),
            local_bounds=[points.min(0), points.max(0)] if len(points) else None)
    return dict(fingers=result, bilateral=all(item["intersections"] for item in result.values()),
        bilateral_all_central=all(item["all_central"] for item in result.values()))


def run(args):
    search = json.loads(args.search_report.read_text())
    if search["selected_index"] is None or not search["candidate_found"]:
        raise ValueError("No selected source-pinch candidate")
    selected = search["candidates"][search["selected_index"]]
    if not selected["passed"]:
        raise ValueError("Selected candidate failed the original CPU source gates")
    world, kin, start, _, metadata, reconstruction = reconstruct(args.episode, "approach")
    if search["reconstruction"]["input_sha256"] != reconstruction["input_sha256"]:
        raise ValueError("Candidate source episode has changed")
    target = np.asarray(selected["target_tcp"])
    source_pose = world.payload_pose.copy()
    approach = local_cartesian_candidate(kin, start, target, "approach", metadata["action_dt_s"], .4)
    report = dict(accepted=False, physical_execution=False, runtime_changed=False,
        hypothetical_attachment_only=True, reconstruction=reconstruction,
        candidate_input_sha256={str(args.search_report):sha(args.search_report)},
        selected_index=search["selected_index"], target_tcp=target, target_offset_m=selected["tcp_height_offset_m"],
        approach=dict(metadata=approach["metadata"], full_source_fcl=fcl_path(world, approach["positions"])))
    report["approach"]["cpu_spheres"], _ = cpu_sphere_audit(world, kin, approach["positions"])
    qclose = approach["positions"][-1]
    tcpclose = kin.fk(qclose)
    signs = np.asarray([1., 1., -1., 1., -1., -1.])
    angles = np.arange(0., float(selected["first_bilateral_angle_rad"])+.0025, .005)
    model = GripperSphereModel()
    meshes, finger_provenance = source_finger_meshes(world)
    report["source_finger_provenance"] = finger_provenance
    world.set_phase("close")
    close_rows = []
    for angle in angles:
        world.gripper_joint_positions = signs*angle
        world.gripper_joint_position = float(angle)
        source_valid = world.valid(qclose)
        spheres = sphere_clearance(model.at(tcpclose, dict(zip(world.GRIPPER_JOINT_NAMES, signs*angle))), world)
        contacts = source_contacts(world, kin, qclose, meshes, search["grasp_geometry"]["contact_region_local_m"])
        close_rows.append(dict(angle_rad=float(angle), source_noncontact_fcl_passed=source_valid,
            source_noncontact_violation=deepcopy(world.last_clearance), original_gripper_spheres=spheres,
            source_contacts=contacts,
            passed=bool(source_valid and spheres["passed"])))
    report["close"] = dict(passed=bool(all(row["passed"] for row in close_rows)
        and close_rows[-1]["source_contacts"]["bilateral_all_central"]), rows=close_rows,
        limit="Kinematic closure, original 5 mrad sampling. Actual physics can move the free banana; no force/closure certification.")
    world.attachment = np.linalg.inv(tcpclose)@source_pose
    world.set_phase("lift")
    lift_target = tcpclose.copy()
    lift_target[2, 3] += .15
    lift = local_cartesian_candidate(kin, qclose, lift_target, "lift", metadata["action_dt_s"], .4)
    report["hypothetical_tcp_to_banana"] = world.attachment.copy()
    report["lift"] = dict(metadata=lift["metadata"], full_source_fcl=fcl_path(world, lift["positions"]))
    report["lift"]["cpu_spheres"], _ = cpu_sphere_audit(world, kin, lift["positions"])
    np.savez_compressed(args.output/"hypothetical_paths.npz", approach=approach["positions"],
        close_q=qclose, close_angles_rad=angles, lift=lift["positions"], target_tcp=target,
        hypothetical_tcp_to_banana=world.attachment, observed_preapproach_banana=source_pose)
    if args.gpu:
        import fcntl
        from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components, sphere_pair_diagnostics
        with (ROOT/"reports/skill_trials/gpu.lock").open("a+") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            planner = PayloadCuroboPlanner(world, kin, dt=metadata["action_dt_s"], seed=42, max_joint_speed=.4)
            attachment = world.attachment.copy()
            world.attachment = None
            world.gripper_joint_positions = np.zeros(6)
            world.gripper_joint_position = 0.
            planner._prepare_path_query((), 0, "approach")
            report["approach"]["gpu_components"], _ = constraint_components(planner, approach["positions"])
            world.attachment = attachment
            world.gripper_joint_positions = signs*angles[-1]
            world.gripper_joint_position = float(angles[-1])
            planner._prepare_path_query((), 1, "lift")
            report["lift"]["gpu_components"], state = constraint_components(planner, lift["positions"])
            report["lift"]["gpu_sphere_pairs"], _ = sphere_pair_diagnostics(planner, state, report["lift"]["gpu_components"])
            report["lift"]["world_pair_partition"] = partition_world(planner, state, report["lift"]["gpu_components"])
    report["remaining"] = "No runtime/controller/canonical edits. Reject until all original non-contact sphere pairs pass; source-support pair-only permission still requires review. Real measured attachment, bilateral contacts, lift and original task success remain unverified."
    report["diagnostic_code_sha256"] = {str(Path(__file__)):sha(Path(__file__))}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--search-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gpu", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to replace prior diagnostic evidence")
    args.output.mkdir(parents=True)
    try:
        report = run(args)
    except Exception as error:
        json_write(args.output/"failure.json", dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json", report)
    print(json.dumps(dict(output=str(args.output), close_cpu_passed=report["close"]["passed"],
        gpu_requested=args.gpu, accepted=False)))


if __name__ == "__main__":
    main()
