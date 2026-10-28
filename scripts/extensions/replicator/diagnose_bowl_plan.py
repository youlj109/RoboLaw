#!/usr/bin/env python3
"""Diagnose prepared G bowl paths; CPU by default, CUDA only when root schedules.

These paths and attachment are explicitly hypothetical. No physical step is
executed, no raw contacts are synthesized, and execution valid remains false.
The exact sink floor is restored to all CPU sphere checks and retained by the
independent planner, including in the final lowering phase.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from kitchen_safety.extensions.replicator.bowl_controller_profile import read_profile
from kitchen_safety.extensions.replicator.bowl_source_support_query import (
    binding_check, source_departure_geometry, audit_source_departure,
)
from kitchen_safety.extensions.replicator.bowl_destination_query import lowering_source_geometry, audit_sink_lowering
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.diagnose_task_plan import cpu_sphere_audit
from scripts.extensions.replicator.bowl_collect import json_write
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha


def retained_floor_spheres(world, spheres, labels):
    """Original padded sphere/source-floor pairs, with no destination mask."""
    floor = [(name, obstacle) for name, obstacle, _, _ in world.objects
        if world._matches(name, world.contact_policy.destination_name)]
    if len(floor) != 1:
        raise ValueError("Expected one exact original BasinBottom collider")
    failures, checked = [], 0
    for sample, items in enumerate(spheres):
        for item, label in zip(items, labels):
            shape = world.fcl.CollisionObject(world.fcl.Sphere(float(item[3])), world.fcl.Transform(item[:3]))
            for name, obstacle in floor:
                checked += 1
                distance = float(world.fcl.distance(shape, obstacle, world.fcl.DistanceRequest(), world.fcl.DistanceResult()))
                overlap = bool(world.fcl.collide(shape, obstacle, world.fcl.CollisionRequest(), world.fcl.CollisionResult()))
                if not np.isfinite(distance):
                    raise ValueError("Nonfinite sphere/source-floor distance")
                if distance < 0. or overlap:
                    failures.append(dict(sample=sample, **label, sphere_world_xyzr=item,
                        obstacle=name, source_intersecting=overlap or distance < 0., fcl_distance_m=distance))
    return dict(passed=not failures, checked_sphere_world_pairs=checked, world_intersections=failures,
        robot_failures=[row for row in failures if row["link"] != "attached_object"],
        payload_failures=[row for row in failures if row["link"] == "attached_object"],
        full_destination_retained=True, scope="CPU original sphere/source mesh overlap; original CUDA SDF cost and self/joint tests remain separate")


def load_candidates(args):
    profile = read_profile(args.controller_profile, args.layout)
    if profile.get('schema_version') != 1:
        raise ValueError('This historical path archive predates the corrected grasp; use diagnose_bowl_grasp_candidate or the captured v1 profile explicitly')
    if profile["sink_target"]["offset_xy_m"] != [.05, -.1]:
        raise ValueError("Only the prepared original sink landing is supported")
    for name, digest in profile["evidence"].items():
        if sha(ROOT/name) != digest:
            raise ValueError("Prepared source evidence changed: "+name)
    layout = json.loads(args.layout.read_text())
    grasp_path = ROOT/"reports/replicator_preparation/g_bowl_sink_grasp_cpu01/diagnostic.json"
    grasp = json.loads(grasp_path.read_text())
    search_path = ROOT/"reports/replicator_preparation/g_bowl_sink_placement_cpu01/search_report.json"
    search = json.loads(search_path.read_text())
    if (grasp["layout_sha256"] != sha(args.layout) or not grasp["closure"]["passed"] or
        search["declaration"]["layout_sha256"] != sha(args.layout) or
        search["accepted"] or search["physical_execution"]):
        raise ValueError("Candidate/source identities or hypothetical scope differ")
    source_paths = grasp_path.parent/"candidate_paths.npz"
    sink_paths = search_path.parent/"selected_candidate.npz"
    if sha(sink_paths) != search["selected_archive_sha256"]:
        raise ValueError("Selected sink lowering archive changed")
    with np.load(source_paths, allow_pickle=False) as raw:
        paths = {name: raw[name].copy() for name in ("approach", "lift")}
        attachment = raw["hypothetical_attachment"].copy()
    with np.load(sink_paths, allow_pickle=False) as raw:
        paths["lower"] = raw["lower"].copy()
        if not np.allclose(attachment, raw["hypothetical_attachment"], atol=1e-12, rtol=0.):
            raise ValueError("Prepared grasp and landing do not share the same hypothetical attachment")
    if not np.array_equal(paths["lift"][0], grasp["closure"]["q"]):
        raise ValueError("Prepared lift does not start at the prepared closure state")
    angle = grasp["closure"]["rows"][grasp["closure"]["selected_index"]]["angle_rad"]
    return layout, profile, paths, attachment, angle, {str(path): sha(path) for path in
        (args.layout, args.controller_profile, grasp_path, source_paths, search_path, sink_paths)}


def run(args):
    layout, profile, paths, attachment, angle, hashes = load_candidates(args)
    report = dict(task_id=layout["task_id"], accepted=False, physical_execution=False,
        actual_state_binding=False, execution_authorized=False, gpu_requested=args.gpu,
        source_layout=layout, controller_profile=profile, input_sha256=hashes, phases={})
    # This predeclared candidate relation never replaces measured post-grasp
    # TCP/object state in bowl_collect. There are no fabricated force samples.
    for phase in args.phases:
        world, kin, _ = build_world(layout)
        world.set_phase(phase)
        world.gripper_joint_position = 0. if phase == "approach" else angle
        world.gripper_joint_positions = np.array([1., 1., -1., 1., -1., -1.])*world.gripper_joint_position
        world.attachment = None if phase == "approach" else attachment.copy()
        if phase == "lower":
            world.payload_pose = kin.fk(paths[phase][0])@world.attachment
        positions = paths[phase]
        rows = []
        for index, q in enumerate(positions):
            valid = world.valid(q)
            rows.append(dict(sample=index, valid=valid, measurement=deepcopy(world.last_clearance)))
        spheres, values = cpu_sphere_audit(world, kin, positions)
        row = dict(samples=len(positions), full_source_fcl=dict(passed=all(item["valid"] for item in rows), rows=rows),
            cpu_original_spheres=spheres,
            original_floor_spheres=retained_floor_spheres(world, values, spheres["sphere_names"]))
        if phase == "lift":
            row.update(actual_state_binding=binding_check(world, positions, None),
                source_support_geometry=source_departure_geometry(world, kin, positions))
        if phase == "lower":
            row["selected_floor_source_geometry"] = lowering_source_geometry(world, kin, positions, profile)
        if args.gpu:
            from kitchen_safety.extensions.replicator.bowl_source_support_curobo import BowlSourceSupportCuroboPlanner
            from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components, sphere_pair_diagnostics
            planner = BowlSourceSupportCuroboPlanner(world, kin, dt=1/15, seed=42, max_joint_speed=.4)
            planner._prepare_path_query((), int(phase != "approach"), phase)
            row["candidate_contact_mask"] = planner.candidate_contact_mask
            if phase == "lift":
                row["source_pair_query"] = audit_source_departure(planner, positions, None)
                if row["source_pair_query"]["valid"]:
                    raise AssertionError("A hypothetical attachment was incorrectly authorized")
            if phase == "lower":
                row["sink_pair_query"] = audit_sink_lowering(planner, positions, profile, None)
                if row["sink_pair_query"]["valid"]:
                    raise AssertionError("A hypothetical sink attachment was incorrectly authorized")
            components, state = constraint_components(planner, positions)
            row["original_cuda_components"] = components
            row["original_cuda_sphere_pairs"], cuda_spheres = sphere_pair_diagnostics(planner, state, components)
            np.savez_compressed(args.output/(phase+"_cuda_spheres.npz"), spheres=cuda_spheres)
            del planner
        np.savez_compressed(args.output/(phase+"_candidate.npz"), positions=positions,
            original_cpu_spheres=values, hypothetical_attachment=attachment)
        report["phases"][phase] = row
        json_write(args.output/(phase+".json"), row)
    sources = (Path(__file__), ROOT/"scripts/extensions/replicator/bowl_collect.py", *[ROOT/
        ("kitchen_safety/extensions/replicator/"+name+".py") for name in ("bowl_collision", "bowl_motion",
            "bowl_controller_profile", "bowl_source_support_query", "bowl_source_support_curobo", "bowl_destination_query", "task_plan_diagnostics")])
    report["runtime_source_sha256"] = {str(path.relative_to(ROOT)): sha(path) for path in sources}
    report["pending"] = ["Fresh matching neutral", "Actual-state full pregrasp and transfer planning", "Physical source rim grasp, observed attachment and support", "Original task predicate and all independent episode gates"]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, default=ROOT/"configs/extensions/replicator/g_shape_bowl_sink_source_layout.json")
    parser.add_argument("--controller-profile", type=Path, default=ROOT/"configs/extensions/replicator/g_shape_bowl_sink_controller.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gpu", action="store_true", help="Root-scheduled CUDA only; never start physics")
    parser.add_argument("--phases", nargs="+", choices=["approach", "lift", "lower"], default=["approach", "lift", "lower"])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock_path = ROOT/"reports/skill_trials/gpu.lock"
            if ROOT != Path("/workspace") or not lock_path.exists():
                raise RuntimeError("CUDA requires the verified workspace and existing shared lock")
            with lock_path.open("a+") as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                report = run(args)
        else:
            report = run(args)
    except Exception as error:
        json_write(args.output/"failure.json", dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json", report)
    print(json.dumps(dict(output=str(args.output), gpu_requested=args.gpu, accepted=False,
        phases={name: dict(source_fcl=row["full_source_fcl"]["passed"],
            floor_robot_intersections=len(row["original_floor_spheres"]["robot_failures"]),
            floor_payload_intersections=len(row["original_floor_spheres"]["payload_failures"]))
            for name, row in report["phases"].items()})))


if __name__ == "__main__":
    main()
