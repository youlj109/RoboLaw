#!/usr/bin/env python3
"""Three-seed, hazard-off/on bowl transport comparisons; no physical execution.

--dry-run reconstructs the real query and source geometry on CPU. --gpu must be
scheduled by the root operator under the shared exclusive lock. No Kit, state
setters, altered grasp, hand waypoints, or added endpoint connecting segments.
"""
import argparse
from copy import deepcopy
import gc
import json
from pathlib import Path
import sys
import time

import numpy as np

from kitchen_safety.extensions.replicator.bowl_paired_query import (
    ROOT, SEEDS, BowlPairedQuery, densify, digest, geometric_sample, inventory,
    pair_gates, path_metrics, require, sha, summarize_geometry, validate_seeds, write,
)


def execution_inputs():
    # Capture CUDA modules before their first runtime import, including all
    # inherited collision/planning helpers. Hashes only; no GPU initialization.
    paths = [Path(__file__)]
    for directory in (ROOT / "kitchen_safety", ROOT / "scripts/extensions/replicator"):
        paths.extend(directory.rglob("*.py"))
    paths.extend((ROOT / "configs/extensions/replicator/g_shape_bowl_sink_controller.json",
                  ROOT / "configs/extensions/replicator/g_shape_bowl_sink_source_layout.json"))
    return {str(p.resolve().relative_to(ROOT)): sha(p) for p in sorted(set(paths)) if p.is_file()}


def frozen_input_check(before):
    return {name: dict(expected=value, actual=sha(ROOT / name)) for name, value in before.items()
            if sha(ROOT / name) != value}


def source_asset_inputs(query):
    from pxr import Usd
    from kitchen_safety.planning import ROBOT_USD
    paths = [ROOT / query.source_layout["kitchen_usd"], ROOT / query.source_layout["objects"]["bowl"]["usd_path"],
             ROOT / query.layout["obstacle"]["usd_path"], ROBOT_USD]
    sources = dict(query.policy["asset_inputs"])
    for path in paths:
        stage = Usd.Stage.Open(str(path))
        require(stage is not None and not stage.GetCompositionErrors(), "Original source asset composition failed")
        for layer in stage.GetUsedLayers():
            if layer.realPath:
                filename = Path(layer.realPath).resolve()
                sources[str(filename.relative_to(ROOT)) if filename.is_relative_to(ROOT) else str(filename)] = sha(filename)
    return sources


def pair_difference(off, on, kin):
    """Normalized-time comparison does not modify either original path."""
    times = np.linspace(0., 1., 1001)
    a = np.stack([np.interp(times, np.linspace(0., 1., len(off)), off[:, j]) for j in range(7)], axis=1)
    b = np.stack([np.interp(times, np.linspace(0., 1., len(on)), on[:, j]) for j in range(7)], axis=1)
    ta = np.asarray([kin.fk(q)[:3, 3] for q in a])
    tb = np.asarray([kin.fk(q)[:3, 3] for q in b])
    return dict(maximum_joint_deviation_rad=float(np.max(np.linalg.norm(a - b, axis=1))),
        maximum_tcp_deviation_m=float(np.max(np.linalg.norm(ta - tb, axis=1))),
        rms_tcp_deviation_m=float(np.sqrt(np.mean(np.sum((ta - tb) ** 2, axis=1)))),
        comparison="1001 uniformly normalized-time samples; comparison only, never appended/replaced path data")


def run(args):
    from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry
    report = dict(schema_version=1, status="preparing", seeds=list(SEEDS), rows=[], passed=False,
        accepted=False, physical_execution=False, physical_multiseed_validation=False,
        scope="Only fixed measured lift-end to fixed sink-transfer TCP planning. All six seed/off-on outcomes retained. "
              "Hazard-off paths are never executed. Source-margin risk does not prove contact/injury/breakage; source "
              "triangles and conservative models do not claim PhysX cooked equivalence.",
        claim_rule="Only if all declared pairs pass: stable avoidance necessity within this tested planner, fixed transfer query, "
                   "and seeds 42/43/44. Otherwise retain failures or classify margin/redundant adjustment; no universal-path claim.",
        sampling=dict(action_hz=15, subdivisions_per_action=8, sample_hz=120, max_joint_speed_rad_s=.4,
                      source_fcl_mode="planning", non_hazard_clearance_m=.002, self_collision="original CUDA gate retained"),
        execution_source_sha256=execution_inputs())
    save = lambda: write(args.output / "paired_plans.json", report)
    save()
    query = BowlPairedQuery(args.baseline, args.layout, args.hazard_episode, args.controller_profile,
                           provenance_output_dir=args.output)
    report.update(source_asset_sha256=source_asset_inputs(query), baseline_input_sha256=query.baseline_inputs,
                  hazard_pose_evidence=query.hazard_evidence, variant=query.variant)
    save()
    off_world, kin, context, target, source_target, off_start = query.build(False)
    evidence = query.evidence(off_world, kin, context, target, source_target, off_start)
    write(args.output / "query_evidence.json", evidence)
    on_world, on_kin, on_context, on_target, _, on_start = query.build(True)
    require(np.array_equal(context.arm_q, on_context.arm_q) and np.array_equal(target, on_target) and
        context.evidence() == on_context.evidence(), "Hazard switch changed the paired start/target/attachment")
    query_identity = dict(arm_q=context.arm_q, actual_six_fingers=context.six_finger_q,
        attachment=context.measured_attachment, actual_free_payload=context.measured_payload, target_tcp=target.tolist(),
        controller=query.controls, source_layout=query.source_layout,
        background_poses=query.data["background_rigid_poses"][query.index].tolist(),
        hazard_pose=query.hazard_pose.tolist(), policy=query.policy, original_ignore_names=[], phase="transport")
    fixed_hash = digest(query_identity)
    np.savez_compressed(args.output / "measured_query.npz", actual_arm_q=np.asarray(context.arm_q),
        actual_six_fingers=np.asarray(context.six_finger_q), actual_tcp=np.asarray(context.measured_tcp).reshape(4, 4),
        actual_payload=np.asarray(context.measured_payload).reshape(4, 4),
        attachment=np.asarray(context.measured_attachment).reshape(4, 4), target_tcp=target,
        actual_background_poses=query.data["background_rigid_poses"][query.index], hazard_pose=query.hazard_pose)
    robot = SourceRobotGeometry(on_world)
    start_geometry = geometric_sample(on_world, robot, np.asarray(context.arm_q), on_context, query.policy["hazard_name"])
    report.update(query_sha256=fixed_hash, query_identity=query_identity, query_evidence_sha256=sha(args.output / "query_evidence.json"),
        baseline_transport=evidence["baseline_transport_path"], variant=query.variant,
        cpu_start_source_fcl=dict(off=off_start, on=on_start), cpu_start_hazard_geometry=start_geometry,
        payload_source=on_world.payload_geometry.provenance, robot_source=robot.provenance,
        hazard_source_colliders=[dict(path=name, bounds=[lo.tolist(), hi.tolist()],
            source_mesh_sha256=digest([on_world.meshes[name][0].tolist(), on_world.meshes[name][1].tolist()])
            if name in on_world.meshes else None) for name, obj, lo, hi in on_world.objects
            if on_world._matches(name, query.policy["hazard_name"])],
        baseline_and_hazard_inputs=query.baseline_inputs, hazard_inputs=query.hazard_inputs,
        geometry_scope="Authored robot collision triangles/primitives plus authored bowl triangles. Source hulls and "
                       "original planner capsules/envelopes are separately measured; neither substitutes for actual physics.")
    require(off_start["passed"] and on_start["passed"], "Paired common start violates original source planning margin")
    require(start_geometry["conservative_planning"]["valid"], "Actual hazard-start margin is insufficient")
    if args.dry_run:
        report.update(status="cpu_query_prepared", cpu_preflight_passed=True,
            cuda_initialized=False, cuda_query_validated=False,
            cpu_import_check=dict(torch_loaded="torch" in sys.modules, kit_loaded=any(
                name.startswith(("omni", "isaacsim")) for name in sys.modules)))
        require(not any(report["cpu_import_check"].values()), "CPU preflight imported a GPU/Kit framework")
        report["execution_source_end_mismatches"] = frozen_input_check(report["execution_source_sha256"])
        report["source_asset_end_mismatches"] = frozen_input_check(report["source_asset_sha256"])
        require(not report["execution_source_end_mismatches"], "Planning inputs changed during CPU preflight")
        require(not report["source_asset_end_mismatches"], "Source assets changed during CPU preflight")
        save()
        return report

    import torch
    from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
    from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components
    torch.set_num_threads(2)
    paths = {}
    report["status"] = "running"
    save()
    for seed in SEEDS:
        for on in (False, True):
            name = f"seed_{seed}_{'on' if on else 'off'}"
            row = dict(seed=seed, hazard_in_planning_world=on, success=False, query_sha256=fixed_hash)
            report["rows"].append(row)
            save()
            planner = None
            started = time.perf_counter()
            try:
                np.random.seed(seed)
                torch.manual_seed(seed)
                world, current_kin, current_context, current_target, _, current_start = query.build(on)
                require(current_context.evidence() == context.evidence() and np.array_equal(current_target, target),
                        "Fresh seed query altered the fixed observations/target")
                require(current_start["passed"], "Common actual start no longer source-safe")
                planner = BowlTransportCuroboPlanner(world, current_kin, dt=1 / 15, seed=seed,
                    max_joint_speed=.4, transport_object_padding_m=query.controls["padding_m"])
                row["transport_binding"] = planner.bind_transport(query.profile, current_context)
                route = planner.plan_pose(np.asarray(context.arm_q).copy(), target.copy(), grip_binary=1, phase="transport")
                path = np.asarray(route["positions"])
                dense = densify(path)
                # Never splice the measured q into a different planner start.
                row["metrics"] = path_metrics(path, current_kin, np.asarray(context.arm_q), target)
                row["planner_metadata"] = route["metadata"]
                np.savez_compressed(args.output / (name + ".npz"), positions=path, dense_positions=dense,
                    tcp=np.asarray([current_kin.fk(q) for q in path]), attachment=world.attachment,
                    actual_six_fingers=world.gripper_joint_positions, target_tcp=target)
                row.update(path_file=name + ".npz", path_sha256=sha(args.output / (name + ".npz")))
                paths[(seed, on)] = path
                save()
                # Original exact CUDA joint/self/world + full-source FCL at its
                # own dense resolution, then ALL 8 substeps separately below.
                row["full_world_validation"] = planner.validate_path(path, grip_binary=1, phase="transport", resolution=.02)
                row["dense_120hz_cuda"], _ = constraint_components(planner, dense)
                measures, all_pairs = [], []
                audit_world, _, audit_context, _, _, _ = query.build(True)
                audit_robot = SourceRobotGeometry(audit_world)
                for index, q in enumerate(dense):
                    valid = world.valid(q, mode="planning", gripper_q=world.gripper_joint_positions)
                    all_pairs.append(dict(sample=index, valid=bool(valid), measurement=deepcopy(world.last_clearance)))
                    measures.append(dict(sample=index, time_s=index / 120,
                        **geometric_sample(audit_world, audit_robot, q, audit_context, query.policy["hazard_name"])))
                row["dense_120hz_all_source_passed"] = all(r["valid"] for r in all_pairs)
                row["dense_120hz_source_samples"] = len(all_pairs)
                row["geometry"] = summarize_geometry(measures)
                write(args.output / (name + "_geometry.json"), dict(complete=True, query_sha256=fixed_hash,
                    samples=len(measures), rows=measures, original_full_source_rows=all_pairs))
                row["geometry_file"] = name + "_geometry.json"
                row["geometry_sha256"] = sha(args.output / row["geometry_file"])
                row["success"] = True  # Planning returned a path; separate gates retain every invalid candidate.
                require(row["metrics"]["fixed_query_passed"], "Planner did not retain fixed actual query endpoints")
            except Exception as error:
                row["error"] = repr(error)
                row["success"] = False
                if planner is not None:
                    row["last_planner_metadata"] = planner.last_plan
                    row["last_transport_validation"] = getattr(planner, "last_transport_validation", None)
                # No failed query is retried by this entry and none is dropped.
            finally:
                row["wall_seconds"] = time.perf_counter() - started
                save()
                del planner
                gc.collect()
                torch.cuda.empty_cache()
    report["pairs"] = pair_gates(report["rows"])
    comparisons = []
    for seed in SEEDS:
        if (seed, False) in paths and (seed, True) in paths:
            a, b = (next(r for r in report["rows"] if r["seed"] == seed and
                          r["hazard_in_planning_world"] is on) for on in (False, True))
            comparisons.append(dict(seed=seed, **pair_difference(paths[(seed, False)], paths[(seed, True)], kin),
                tcp_path_length_change_m=b["metrics"]["tcp_path_length_m"] - a["metrics"]["tcp_path_length_m"],
                duration_change_s=b["metrics"]["duration_s"] - a["metrics"]["duration_s"],
                off_vs_original_baseline_length_m=a["metrics"]["tcp_path_length_m"] -
                    evidence["baseline_transport_path"]["metrics"]["tcp_path_length_m"]))
    report["path_comparisons"] = comparisons
    report["execution_source_end_mismatches"] = frozen_input_check(report["execution_source_sha256"])
    report["source_asset_end_mismatches"] = frozen_input_check(report["source_asset_sha256"])
    report["baseline_inputs_unchanged"] = inventory(query.baseline) == query.baseline_inputs
    report["hazard_inputs_unchanged"] = args.hazard_episode is None or inventory(args.hazard_episode) == query.hazard_inputs
    report["passed"] = all(p["passed"] for p in report["pairs"]) and not report["execution_source_end_mismatches"] and \
        not report["source_asset_end_mismatches"] and \
        report["baseline_inputs_unchanged"] and report["hazard_inputs_unchanged"]
    report["status"] = "complete"
    save()
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hazard-episode", type=Path)
    parser.add_argument("--controller-profile", type=Path)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--gpu", action="store_true", help="Root-scheduled CUDA planning, never physical execution")
    args = parser.parse_args(argv)
    validate_seeds(args.seeds)
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock_path = ROOT / "reports/skill_trials/gpu.lock"
            require(ROOT == Path("/workspace") and lock_path.is_file(), "Verified container and existing shared GPU lock required")
            with lock_path.open("a+") as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = run(args)
        else:
            result = run(args)
    except Exception as error:
        write(args.output / "failure.json", dict(error=repr(error), accepted=False, physical_execution=False))
        checkpoint = args.output / "paired_plans.json"
        if checkpoint.is_file():
            retained = json.loads(checkpoint.read_text())
            retained.update(status="failed", passed=False, error=repr(error))
            write(checkpoint, retained)
        raise
    print(json.dumps(dict(output=str(args.output), status=result["status"], passed=result["passed"],
                          accepted=False, physical_execution=False)), flush=True)
    return 0 if args.dry_run or result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
