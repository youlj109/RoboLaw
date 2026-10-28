#!/usr/bin/env python3
"""Bounded source-legal banana X candidates, retaining original payload spheres.

Candidate layouts only; no canonical/runtime edits or physics. The two observed
banana poses are translated as explicit predictions, never recorded as new
physical evidence. Every non-source-support sphere/world pair retains 2 mm.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import fcl
import numpy as np

from scripts.extensions.replicator.diagnose_task_plan import reconstruct
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.task_audit_sidecars import sha
from scripts.extensions.replicator.task_collect import json_write


def payload_clearance(world, pose):
    centers, radii, provenance = world.payload_geometry.sphere_cover(100)
    centers = centers@pose[:3, :3].T+pose[:3, 3]
    minimum, violations = None, []
    for sphere_id, (center, radius) in enumerate(zip(centers, radii)):
        bounds = np.linalg.norm(np.maximum(np.maximum(world.low-center, center-world.high), 0.), axis=1)-radius
        sphere = fcl.CollisionObject(fcl.Sphere(radius), fcl.Transform(center))
        for index in np.argsort(bounds):
            if minimum is not None and bounds[index] > max(.002, minimum["distance_m"]):
                break
            name, obstacle, _, _ = world.objects[index]
            if world._matches(name, world.payload_name) or name == world.contact_policy.source_support:
                continue
            overlap = bool(fcl.collide(sphere, obstacle, fcl.CollisionRequest(), fcl.CollisionResult()))
            raw = float(fcl.distance(sphere, obstacle, fcl.DistanceRequest(), fcl.DistanceResult()))
            value = dict(sphere_index=sphere_id, obstacle=name, distance_m=max(0., raw),
                intersecting=bool(overlap or raw < 0.), required_m=.002,
                valid=bool(not overlap and raw >= .002), sphere_world_xyzr=np.r_[center, radius])
            if minimum is None or value["distance_m"] < minimum["distance_m"]:
                minimum = value
            if not value["valid"]:
                violations.append(value)
    return dict(passed=not violations, minimum=minimum, violations=violations, cover=provenance,
        permitted_support_pair=["payload/banana", world.contact_policy.source_support],
        remaining="Source-support sphere pair retained separately; this is not a world-wide counter mask")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--layout", type=Path, default=Path("configs/extensions/replicator/g_shape_banana_bowl_source_layout.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--initial-object-z-offset-m", type=float, choices=[0., .0001], default=0.,
        help="Explicit source-legal initial height correction only; predicted settled physical poses are not raised")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to replace prior candidate evidence")
    args.output.mkdir(parents=True)
    layout = json.loads(args.layout.read_text())
    if layout["task_id"] != "kitchen_bench_replicator_g_shape_banana_bowl":
        raise ValueError("Search is only defined for original G banana")
    source_layout = deepcopy(layout)
    for item in layout["objects"].values():
        item["position"][2] += args.initial_object_z_offset_m
    declaration = dict(layout_sha256=sha(args.layout), source_layout=source_layout,
        initial_object_z_offset_m=args.initial_object_z_offset_m, search_initial_layout=layout,
        offsets_m=[[i*.001, 0., 0.] for i in range(31)],
        fixed=["Robot base XYZ/yaw/stand height", "banana Y/yaw/asset", "bowl XY/yaw/asset",
               "all original On/NextTo/Rotate validators", "2 mm non-contact margin", "original 86-sphere source-volume cover"],
        initial_height_policy="Only explicitly requested fixed 0 or +0.1 mm source placement correction, unchanged On 5 mm band. Predicted settled poses remain at their actual recorded heights; new neutral is mandatory.",
        ordering="0 to +30 mm banana X in 1 mm increments; first source-legal candidate passing both shifted recorded source poses",
        predicted_physics=False, canonical_changed=False, execution_ready=False)
    json_write(args.output/"declaration.json", declaration)
    scene = CandidateSearch(layout)
    worlds = {}
    reconstruction = {}
    for phase in ("approach", "lift"):
        world, _, _, _, _, info = reconstruct(args.episode, phase)
        worlds[phase] = world
        reconstruction[phase] = info
    rows, selected = [], None
    for delta in declaration["offsets_m"]:
        row = scene.evaluate("banana", delta)
        row["original_initial_source_geometry_passed"] = row["passed"]
        row["predicted_observed_pose_checks"] = {}
        for phase, world in worlds.items():
            pose = world.payload_pose.copy()
            pose[:3, 3] += delta
            row["predicted_observed_pose_checks"][phase] = dict(pose=pose, **payload_clearance(world, pose))
        row["passed"] = bool(row["passed"] and all(value["passed"] for value in row["predicted_observed_pose_checks"].values()))
        rows.append(row)
        print(json.dumps(dict(offset_m=delta, passed=row["passed"], source=row["original_initial_source_geometry_passed"],
            minimums={phase:value["minimum"]["distance_m"] for phase, value in row["predicted_observed_pose_checks"].items()})), flush=True)
        if row["passed"]:
            selected = len(rows)-1
            break
    report = dict(accepted=False, execution_ready=False, canonical_changed=False, declaration=declaration,
        reconstruction=reconstruction, candidates=rows, selected_index=selected,
        candidate_found=selected is not None, script_sha256=sha(Path(__file__)))
    if selected is not None:
        candidate = deepcopy(layout)
        candidate["objects"]["banana"]["position"] = (np.asarray(candidate["objects"]["banana"]["position"])+rows[selected]["offset_xyz_m"]).tolist()
        relations = check_layout(candidate)
        if not relations["initial_source_relations_passed"]:
            raise ValueError("All-source relation check disagrees with candidate validator")
        path = args.output/"candidate_layout.json"
        json_write(path, candidate)
        report.update(candidate_layout=str(path), candidate_layout_sha256=sha(path), original_relations=relations,
            remaining=["Independent all-object/stand source geometry", "Full initial robot/source hulls", "Full candidate pregrasp/approach/closure/lift CPU paths",
                       "Original GPU joint/world/self partition", "New neutral for new layout", "Actual measured contacts/attachment/lift and original task success"])
    json_write(args.output/"search_report.json", report)


if __name__ == "__main__":
    main()
