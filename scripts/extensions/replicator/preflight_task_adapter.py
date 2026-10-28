#!/usr/bin/env python3
"""CPU checks for the independent G banana/bowl and bowl/sink adapter.

This command does not launch SimulationApp, CUDA, physics or collection. Run
with scripts/arena_python.sh so the source stand's /workspace references resolve.
"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True, choices=[
        "kitchen_bench_replicator_g_shape_banana_bowl", "kitchen_bench_replicator_g_shape_bowl_sink"])
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    import numpy as np
    from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
    from kitchen_safety.extensions.replicator.task_environment import resolve_task_layout
    from scripts.extensions.replicator.audit_task_source_relation import check_layout, source

    layout = resolve_task_layout(args.task)
    relations = check_layout(layout)
    _, original = source(args.task)
    task_params = original["task"]["subtasks"][0]["params"]
    payload_name = task_params["pick_up_object"]
    root = Path(__file__).resolve().parents[3]
    payload = PayloadGeometry(root / layout["objects"][payload_name]["usd_path"])
    centers, radii, cover = payload.sphere_cover()
    gaps = np.min(np.linalg.norm(payload.vertices[:, None, :] - centers[None, :, :], axis=2) - radii, axis=1)
    passed = relations["initial_source_relations_passed"] and bool(np.all(gaps <= 1e-8))
    report = {"task_id": args.task, "cpu_preflight_passed": passed,
        "execution_ready": False, "physics_or_collection_launched": False,
        "scope": "Original authored relations and source-derived payload collision representation only",
        "source_relations": relations, "payload_geometry": payload.provenance,
        "payload_sphere_cover": dict(cover, maximum_source_vertex_gap_m=float(gaps.max())),
        "original_pick_up_object": payload_name, "original_destination": task_params["destination_location"],
        "pending_validation": [
            "Factory build inside the actual SimulationApp runtime (PhysxSchema unavailable in ordinary CPU interpreter)",
            "Runtime validation of the implemented G banana selective-contact collector; bowl/sink controller remains unimplemented",
            "Runtime validation of original three cameras and complete reset/warmup/settling physics/contact recording",
            "IK and cuRobo paths, original task predicate, bilateral physical grasp, release/support and collision checks",
            "Matched baseline, human/property/obstacle layouts, source-geometry avoidance and three-seed paired planning",
            "Actual RGB/video collection, decoding and visual review"],
    }
    target = args.report or root / "reports/replicator_preparation" / f"{args.task}_adapter_preflight.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"task": args.task, "cpu_preflight_passed": passed, "execution_ready": False,
                      "payload_spheres": len(centers), "report": str(target)}, indent=2))
    return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
