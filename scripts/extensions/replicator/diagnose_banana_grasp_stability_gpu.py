#!/usr/bin/env python3
"""Scheduled CUDA prerequisite diagnosis for one predeclared grasp candidate.

Never launches Kit or executes a control action. The attachment is hypothetical,
so a source-support partition is diagnostic only and cannot bind permission to
execute a departure. Actual contacts and a measured attachment remain required.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.search_banana_grasp_stability_cpu import observed_world, path_audit, write

_GPU_LOCK = None


def load_candidate(directory, candidate_id, episode):
    declaration = json.loads((directory / "declaration.json").read_text())
    summary = json.loads((directory / "search_report.json").read_text())
    if sha(directory / "declaration.json") != summary["declaration_sha256"]:
        raise ValueError("Candidate declaration hash differs")
    if candidate_id not in [row["id"] for row in declaration["candidates"]]:
        raise ValueError("Candidate was not predeclared")
    identity = next(row for row in summary["candidates"] if row["id"] == candidate_id)
    path = directory / (candidate_id + ".json")
    if sha(path) != identity["sha256"]:
        raise ValueError("Candidate source audit hash differs")
    record = json.loads(path.read_text())
    if record["passed_cpu_geometric_prerequisites"] is not True:
        raise ValueError("CPU geometric prerequisites failed")
    for name, digest in declaration["reconstruction"]["input_sha256"].items():
        if sha(episode / name) != digest:
            raise ValueError("Candidate was derived from different actual evidence: " + name)
    for name, digest in declaration["code_sha256"].items():
        if sha(ROOT / name) != digest:
            raise ValueError("Candidate-producing code changed: " + name)
    archive = directory / (candidate_id + ".npz")
    if sha(archive) != record["archive"]["sha256"]:
        raise ValueError("Candidate path archive hash differs")
    with np.load(archive, allow_pickle=False) as raw:
        arrays = {name: raw[name] for name in raw.files}
    if not all(np.isfinite(value).all() for value in arrays.values()):
        raise ValueError("Candidate archive has nonfinite values")
    return record, arrays, {str(path): sha(path) for path in
        (directory / "declaration.json", directory / "search_report.json", path, archive)}


def run(args, report):
    global _GPU_LOCK
    record, arrays, hashes = load_candidate(args.search, args.candidate, args.episode)
    report["candidate_input_sha256"] = hashes
    world, kin, _, initial_q, metadata, _, reconstruction = observed_world(args.episode)
    if not np.array_equal(initial_q, arrays["actual_initial_q"]):
        raise ValueError("Candidate initial q differs from actual recorded state")
    if not np.array_equal(world.payload_pose, arrays["observed_preapproach_banana"]):
        raise ValueError("Candidate source payload pose differs from actual observation")
    report["reconstruction"] = reconstruction
    import fcntl
    lock_path = ROOT / "reports/skill_trials/gpu.lock"
    if ROOT != Path("/workspace") or not lock_path.exists():
        raise RuntimeError("CUDA diagnostic requires verified mount and canonical existing lock")
    _GPU_LOCK = lock_path.open("a+")
    fcntl.flock(_GPU_LOCK.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
    from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components
    from scripts.extensions.replicator.diagnose_banana_grasp_candidate import partition_world
    world.gripper_joint_positions, world.gripper_joint_position = np.zeros(6), 0.
    world.attachment = None
    planner = PayloadCuroboPlanner(world, kin, dt=metadata["action_dt_s"], seed=args.seed, max_joint_speed=.4)
    plan = planner.plan_joint_goal(initial_q, arrays["pregrasp_q"], phase="pregrasp")
    report["pregrasp"] = dict(metadata=plan["metadata"],
        validation=planner.validate_path(plan["positions"], phase="pregrasp"),
        endpoint_difference_from_archived_approach_start_rad=float(np.max(np.abs(plan["positions"][-1] - arrays["approach"][0]))))
    np.savez_compressed(args.output / "planned_pregrasp.npz", positions=plan["positions"],
                        raw_positions=plan["raw_positions"], actual_initial_q=initial_q,
                        requested_q=arrays["pregrasp_q"], source_dt=plan["source_dt"])
    planner._prepare_path_query((), 0, "approach")
    report["approach"], _ = constraint_components(planner, arrays["approach"])
    report["approach"]["source_audit"] = path_audit(world, kin, arrays["approach"])
    signs = np.array([1., 1., -1., 1., -1., -1.])
    close = []
    for entry in record["close_rows"]:
        angle = entry["angle_rad"]
        world.gripper_joint_positions, world.gripper_joint_position = signs * angle, angle
        planner._prepare_path_query((), 1, "close")
        audit, _ = constraint_components(planner, arrays["close_q"][None])
        close.append(dict(angle_rad=angle, original_gpu_constraints=audit))
    report["close"] = dict(passed=all(row["original_gpu_constraints"]["valid"] for row in close), rows=close)
    world.attachment = arrays["hypothetical_tcp_to_banana"].copy()
    world.gripper_joint_position = float(arrays["close_angle_rad"])
    world.gripper_joint_positions = signs * world.gripper_joint_position
    planner._prepare_path_query((), 1, "lift")
    report["lift"], state = constraint_components(planner, arrays["lift"])
    report["lift"]["source_audit"] = path_audit(world, kin, arrays["lift"])
    report["lift"]["original_world_pair_partition"] = partition_world(planner, state, report["lift"])
    report["lift"]["source_departure_context_bound"] = False
    checks = dict(pregrasp_actual_start_path=report["pregrasp"]["validation"]["valid"],
        pregrasp_to_approach_joint_gap=report["pregrasp"]["endpoint_difference_from_archived_approach_start_rad"] <= 1e-4,
        approach_original_gpu=report["approach"]["valid"],
        approach_source_fcl=report["approach"]["source_audit"]["source_fcl_passed"],
        close_original_gpu=report["close"]["passed"],
        lift_original_full_self=report["lift"]["components"]["self_collision"]["maximum_cost"] == 0.,
        lift_original_full_joint=report["lift"]["components"]["joint_bound"]["maximum_cost"] == 0.,
        lift_all_non_support_world_pairs=report["lift"]["original_world_pair_partition"]["noncontact_sphere_gate_passed"],
        lift_partition_exhaustive=report["lift"]["original_world_pair_partition"]["sphere_partition_disjoint_exhaustive"],
        lift_partition_equivalent=report["lift"]["original_world_pair_partition"]["original_world_zero_nonzero_equivalent"],
        lift_full_source_fcl=report["lift"]["source_audit"]["source_fcl_passed"])
    report.update(checks=checks, diagnostic_prerequisites_passed=all(checks.values()),
                  actual_task_success=False, physical_grasp_stable=False,
                  source_departure_authorized=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--search", required=True, type=Path)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite any prior GPU diagnostic")
    args.output.mkdir(parents=True)
    report = dict(schema_version=1, accepted=False, physical_execution=False,
        hypothetical_attachment_only=True, candidate=args.candidate, seed=args.seed,
        diagnostic_prerequisites_passed=False, source_departure_authorized=False,
        scope="Planned initial-to-pregrasp joint path and archived source candidates only. A geometric source-support partition is diagnostic, never an observed departure permission or proof of physical stability.",
        diagnostic_code_sha256={str(Path(__file__).resolve().relative_to(ROOT)): sha(Path(__file__))})
    try:
        run(args, report)
    except Exception as exc:
        report["error"] = repr(exc)
    if (args.output / "planned_pregrasp.npz").exists():
        report["planned_pregrasp_sha256"] = sha(args.output / "planned_pregrasp.npz")
    write(args.output / "diagnostic.json", report)
    print(json.dumps({key: report[key] for key in
        ("candidate", "diagnostic_prerequisites_passed", "accepted", "source_departure_authorized")}))
    return 0 if report["diagnostic_prerequisites_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
