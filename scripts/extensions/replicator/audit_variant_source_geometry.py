"""Audit source geometry for G-mustard motion and complete recorded startup.

Requires an explicit safe/risk expectation; nonzero exit means the requested
gate failed (including a counterfactual that has only proxy violations).
"""
import argparse
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.variant_geometry import (
    ROOT, MeasuredHazardShapes, SourceRobotGeometry, assert_source_layout, code_evidence, expectation_passed,
    hazard_pose_evidence, make_hazard, measure_sample, sha, source_provenance,
    summarize_samples, variant_settings, write_report,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--expect", choices=("safe", "risk"), required=True)
    parser.add_argument("--hazard-episode", type=Path,
                        help="Successful matching variant required for a dynamic hazard counterfactual")
    args = parser.parse_args(argv)
    if args.report.exists():
        raise FileExistsError(args.report)
    report = dict(passed=False, expected=args.expect, episode=str(args.episode),
                  layout=str(args.layout), scope="Every saved main-motion sample plus startup when present, actual bottle/finger poses; "
                  "counterfactual hazards are checked offline only. No dangerous path execution.")
    try:
        from kitchen_safety.extensions.replicator.source_geometry import SourceBottleGeometry
        from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
        from kitchen_safety.planning import DroidKinematics

        layout = json.loads(args.layout.read_text())
        variant = layout["obstacle"]["category"]
        hazard, required, planning, _ = variant_settings(layout, variant)
        result = json.loads((args.episode / "result.json").read_text())
        assert_source_layout(layout, result)
        if not result.get("success") or not result.get("arena_success"):
            raise ValueError("Source geometry evidence must refer to a successful physical episode")
        actual_hazard = result.get("variant") == variant and result.get("obstacle") == layout["obstacle"]
        if args.expect == "safe" and not actual_hazard:
            raise ValueError("Safe execution audit requires the actual matching hazard episode")
        reference = args.episode if actual_hazard else args.hazard_episode
        pose, pose_evidence = hazard_pose_evidence(layout, variant, reference)
        stage, root = make_hazard(layout, variant, pose)
        kin = DroidKinematics(np.asarray(layout["robot_position"]) + [0, 0, layout["stand_height"]],
                              layout["robot_yaw_deg"])
        world = CollisionWorld(stage, kin, clearances=layout["planning"]["clearances_m"],
                               execution_clearances=layout["planning"]["execution_clearances_m"])
        mesh, hull = SourceBottleGeometry(), SourceBottleGeometry(True)
        robot = SourceRobotGeometry(world)
        shapes = MeasuredHazardShapes(stage, root, pose)
        report.update(variant=variant, required_m=required, planning_required_m=planning,
                      hazard_pose_evidence=pose_evidence, source=source_provenance(stage, root, mesh, hull, robot),
                      layout_sha256=sha(args.layout), result_sha256=sha(args.episode / "result.json"),
                      physics_sha256=sha(args.episode / "physics_states.npz"),
                      hazard_pose_method="actual pose at each physics substep" if actual_hazard
                      and layout["obstacle"].get("static") is not True else "fixed stationary hazard reference",
                      **code_evidence([Path(__file__).resolve(), ROOT / "scripts/extensions/replicator/variant_geometry.py",
                                       ROOT / "kitchen_safety/extensions/replicator/source_geometry.py",
                                       ROOT / "kitchen_safety/geometry_audit.py"]))
        rows, startup_rows = [], []
        report["rows"] = rows
        with np.load(args.episode / "physics_states.npz") as data:
            names, ids = data["joint_names"].tolist(), data["arm_ids"]
            prefix_path = args.episode / "startup_physics_states.npz"
            if prefix_path.exists():
                from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence
                continuity = audit_saved_collection_evidence(args.episode)
                if not continuity['passed']:
                    raise ValueError('Incomplete startup/main timeline or source provenance: ' + str(continuity.get('checks')))
                report['continuous_collection_evidence'] = continuity
                report['startup_input_sha256'] = {name: sha(args.episode / name) for name in (
                    'startup_physics_states.npz', 'startup_audit.json', 'motion_physics_index.npz',
                    'motion_physics_alignment.json', 'collection_provenance.json')}
                with np.load(prefix_path, allow_pickle=False) as prefix:
                    if prefix['joint_names'].tolist() != names:
                        raise ValueError('Startup/motion joint orders differ')
                    report['startup_rows'] = startup_rows
                    for i, q in enumerate(prefix['q']):
                        if actual_hazard and layout['obstacle'].get('static') is not True and not prefix['has_hazard_pose'][i]:
                            raise ValueError('Dynamic hazard has missing startup pose')
                        sample_pose = prefix['hazard_pose'][i] if actual_hazard and prefix['has_hazard_pose'][i] else pose
                        world.set_obstacle_pose(root, sample_pose)
                        measured = measure_sample(world, mesh, hull, q[ids], dict(zip(names, q)),
                                                  prefix['mustard'][i, :7], shapes.at(sample_pose), required, hazard, robot)
                        startup_rows.append(dict(stream='startup', physics_index=i,
                            absolute_physics_step=int(prefix['physics_step'][i]),
                            action_index=int(prefix['action_index'][i]), phase=str(prefix['phase'][i]),
                            startup_stage=str(prefix['startup_stage'][i]), **measured))
            for i, q in enumerate(data["q"]):
                sample_pose = data["hazard_pose"][i] if actual_hazard and data["has_hazard_pose"][i] else pose
                world.set_obstacle_pose(root, sample_pose)
                measured = measure_sample(world, mesh, hull, q[ids], dict(zip(names, q)),
                                          data["mustard"][i, :7], shapes.at(sample_pose), required, hazard, robot)
                rows.append(dict(stream='motion', physics_index=i, action_index=int(data["action_index"][i]),
                                 phase=str(data["phase"][i]), **measured))
        summary = summarize_samples(rows)
        combined = summarize_samples(startup_rows + rows)
        report.update(samples=len(rows), summary=summary,
                      sample_scope='rows/samples/summary are main motion; startup is separate and whole_episode_summary combines both',
                      startup_samples=len(startup_rows), total_physics_samples=len(startup_rows) + len(rows),
                      full_reset_to_end_source_geometry=bool(startup_rows),
                      startup_summary=summarize_samples(startup_rows) if startup_rows else None,
                      whole_episode_summary=combined if startup_rows else None,
                      phase_summaries={phase: summarize_samples([r for r in rows if r["phase"] == phase])
                                       for phase in sorted({r["phase"] for r in rows})},
                      # Necessity remains demonstrated in task motion. For a
                      # safe execution, every available startup sample must also
                      # satisfy the original source and conservative margins.
                      passed=expectation_passed(combined if args.expect == 'safe' else summary, args.expect))
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    write_report(args.report, report)
    print(json.dumps({k: v for k, v in report.items() if k in
                      ("passed", "expected", "samples", "summary", "error")}, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
