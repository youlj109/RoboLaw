"""Seed-paired fixed-transfer planning for human/property/robot variants.

GPU/cuRobo planning only; never starts Kit or executes any trajectory. Invoke
under the shared GPU flock. Each predeclared seed has one off/on query with
identical state, goal, controls and source background. All outcomes are saved.
"""
import argparse
import gc
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.variant_geometry import (
    ROOT, MeasuredHazardShapes, SourceRobotGeometry, assert_source_layout, code_evidence, expectation_passed,
    hazard_pose_evidence, make_hazard, measure_sample, sha, source_provenance,
    summarize_samples, variant_settings, write_report,
)


def validate_seeds(seeds):
    if len(seeds) < 3 or len(set(seeds)) != len(seeds) or any(seed < 0 for seed in seeds):
        raise ValueError("Predeclare at least three distinct nonnegative planning seeds")


def densify(path, subdivisions=8):
    path = np.asarray(path)
    if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("Planner path must contain at least two finite seven-joint states")
    return np.concatenate([path[:1]] + [np.linspace(a, b, subdivisions + 1)[1:]
                                      for a, b in zip(path[:-1], path[1:])])


def pair_gates(rows, seeds):
    pairs = []
    for seed in seeds:
        subset = [r for r in rows if r["seed"] == seed]
        off = [r for r in subset if not r["hazard_in_planning_world"]]
        on = [r for r in subset if r["hazard_in_planning_world"]]
        gates = dict(exactly_one_off_and_on=len(off) == len(on) == 1,
                     both_planned=len(off) == len(on) == 1 and off[0].get("success") is True
                     and on[0].get("success") is True)
        if gates["both_planned"]:
            gates.update(off_source_risk=expectation_passed(off[0]["geometry"], "risk"),
                         on_source_and_execution_safe=expectation_passed(on[0]["geometry"], "safe"),
                         on_original_planning_margin=on[0].get("conservative_planning_passed") is True,
                         both_original_full_world_checks=all(r.get("full_world_validation", {}).get("valid") is True
                                                            and r["full_world_validation"].get("validation_complete") is True
                                                            for r in (off[0], on[0])),
                         same_query_verified=off[0].get("fixed_query_passed") is True
                         and on[0].get("fixed_query_passed") is True)
        pairs.append(dict(seed=seed, gates=gates, passed=all(gates.values())))
    return pairs


def path_metrics(path, kin, dt, q, target):
    tcp = np.asarray([kin.fk(v) for v in path])
    start_error = float(np.max(np.abs(path[0] - q)))
    translation = float(np.linalg.norm(tcp[-1, :3, 3] - target[:3, 3]))
    rotation = float(Rotation.from_matrix(tcp[-1, :3, :3] @ target[:3, :3].T).magnitude())
    return dict(tcp_path_length_m=float(np.linalg.norm(np.diff(tcp[:, :3, 3], axis=0), axis=1).sum()),
                joint_path_length_rad=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                tcp_endpoint_distance_m=float(np.linalg.norm(tcp[-1, :3, 3] - tcp[0, :3, 3])),
                duration_s=(len(path) - 1) * dt, samples=len(path),
                start_joint_error_rad=start_error, goal_translation_error_m=translation,
                goal_rotation_error_rad=rotation,
                fixed_query_passed=start_error <= 1e-4 and translation <= .002 and rotation <= .02)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[42, 43, 44])
    parser.add_argument("--hazard-episode", type=Path,
                        help="Matching successful variant; required for dynamic property pose/stability")
    args = parser.parse_args(argv)
    validate_seeds(args.seeds)
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, status="running", seeds=args.seeds,
                  predeclared_seed_order=True, baseline=str(args.baseline), layout=str(args.layout), rows=[],
                  scope="Fixed measured lift-end start and fixed above-bowl TCP pose. The only per-seed world difference "
                  "is presence of the hazard; matched planner seed/control parameters. Planned transfer segment only, "
                  "not full-task multiseed physics or randomized layouts. Hazard-off paths are NEVER physically executed. "
                  "Margin violation is not a claim of physical impact. Source geometry is not PhysX cooked geometry.",
                  baseline_selection_rule="Use every predeclared seed's direct fixed-target no-hazard query from the supplied "
                  "successful episode, without selecting a worst path or adding manual waypoints; retain every outcome.")
    save = lambda: write_report(args.output / "paired_plans.json", report)
    save()
    try:
        import torch
        from pxr import Gf, Usd, UsdGeom
        from scripts.extensions.replicator.search_avoidance import load_baseline
        from scripts.extensions.replicator.audit_avoidance import controls
        from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
        from kitchen_safety.extensions.replicator.curobo_backend import CuroboPlanner
        from kitchen_safety.extensions.replicator.source_geometry import SourceBottleGeometry
        from kitchen_safety.planning import DroidKinematics

        torch.set_num_threads(2)
        result, data, names, ids = load_baseline(args.baseline)
        if result.get("variant") != "base":
            raise ValueError("The paired start must come from a no-hazard successful baseline")
        layout = json.loads(args.layout.read_text())
        variant = layout["obstacle"]["category"]
        hazard, required, planning_required, padding = variant_settings(layout, variant)
        assert_source_layout(layout, result)
        if result["effective_controls"]["transport_object_padding_m"] != padding:
            raise ValueError("Baseline transport padding differs from this variant; use its matched control profile")
        pose, pose_evidence = hazard_pose_evidence(layout, variant, args.hazard_episode)
        if args.hazard_episode:
            actual = json.loads((args.hazard_episode / "result.json").read_text())
            if controls(actual) != controls(result):
                raise ValueError("Baseline and hazard episode effective controls differ")
        kitchen_path = Path(layout["kitchen_usd"])
        if result.get("background_physics"):
            from kitchen_safety.extensions.replicator.background_physics import mounted_kitchen
            kitchen_path, profile = mounted_kitchen(ROOT / kitchen_path, ROOT / "reports/replicator_physics/assets")
            if profile != result["background_physics"]:
                raise ValueError("Current background source/recipe differs from recorded successful baseline")
        index = next(i for i, phase in enumerate(data["phases"]) if phase == "above_bowl")
        q = data["q"][index, ids]
        grip = {name: float(value) for name, value in zip(names, data["q"][index])}
        attachment = np.asarray(result["attachment_after_lift"])
        target = np.asarray(next(p for p in result["plans"] if p["phase"] == "above_bowl")["effective_goal_pose_world"])
        kin = DroidKinematics(np.asarray(layout["robot_position"]) + [0, 0, layout["stand_height"]],
                              layout["robot_yaw_deg"])
        settings = layout["planning"]
        stage, root = make_hazard(layout, variant, pose)
        audit_world = CollisionWorld(stage, kin, clearances=settings["clearances_m"],
                                     execution_clearances=settings["execution_clearances_m"])
        hazard_shapes = MeasuredHazardShapes(stage, root, pose).at(pose)
        mesh, hull = SourceBottleGeometry(), SourceBottleGeometry(True)
        robot = SourceRobotGeometry(audit_world)

        def world(on):
            stage = make_hazard(layout, variant, pose)[0] if on else Usd.Stage.CreateInMemory()
            for name, path, object_pose in [
                ("kitchen", kitchen_path, [0, 0, 0, 0, 0, 0, 1]),
                ("bowl", "assets/robolab/objects/ycb/bowl.usd", data["bowl"][index, :7]),
                ("mustard_bottle", "assets/robolab/objects/hope/mustard_bottle.usd", data["mustard"][index, :7]),
            ]:
                xform = UsdGeom.Xform.Define(stage, "/World/envs/env_0/" + name)
                xform.GetPrim().GetReferences().AddReference(str(ROOT / path))
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, object_pose[:3])))
                xform.AddOrientOp().Set(Gf.Quatf(float(object_pose[6]), Gf.Vec3f(*map(float, object_pose[3:6]))))
            collision = CollisionWorld(stage, kin, clearances=settings["clearances_m"],
                                       execution_clearances=settings["execution_clearances_m"],
                                       default_clearance=settings["default_clearance_m"])
            collision.sync_background(result["background_rigid_paths"], data["background_rigid_poses"][index])
            collision.attachment = attachment.copy()
            collision.gripper_joint_position = grip["finger_joint"]
            return stage, collision

        report.update(variant=variant, layout_sha256=sha(args.layout),
                      baseline_result_sha256=sha(args.baseline / "result.json"),
                      baseline_trajectory_sha256=sha(args.baseline / "trajectory.h5"),
                      hazard_pose_evidence=pose_evidence, source=source_provenance(stage, root, mesh, hull, robot),
                      background_physics=result.get("background_physics"), kitchen_reference=str(kitchen_path),
                      recorded_start_action_index=index, start_q_rad=q.tolist(), gripper=grip,
                      target_tcp_pose=target.tolist(), attachment_tcp_to_bottle=attachment.tolist(),
                      controls=controls(result) | dict(dt=data["dt"], max_joint_speed_rad_s=settings["max_joint_speed_rad_s"],
                                                     transport_object_padding_m=padding, planning_required_m=planning_required,
                                                     execution_required_m=required, ignore_names=["mustard_bottle"]),
                      geometric_sampling=dict(subdivisions_per_action=8, sample_period_s=data["dt"] / 8,
                                              scope="Commanded-joint interpolation with fixed predicted attachment; actual tracking audited separately"),
                      query_tolerances=dict(start_joint_rad=1e-4, target_translation_m=.002, target_rotation_rad=.02),
                      **code_evidence([
                          Path(__file__), ROOT / "scripts/extensions/replicator/variant_geometry.py",
                          ROOT / "kitchen_safety/curobo_backend.py",
                          ROOT / "kitchen_safety/extensions/replicator/curobo_backend.py",
                          ROOT / "kitchen_safety/extensions/replicator/collision_planner.py",
                          ROOT / "kitchen_safety/extensions/replicator/source_geometry.py",
                          ROOT / "kitchen_safety/geometry_audit.py"]))
        save()
        for seed in args.seeds:
            for on in (False, True):
                name = f"seed_{seed}_{'on' if on else 'off'}"
                row = dict(seed=seed, hazard_in_planning_world=on, success=False)
                report["rows"].append(row)
                save()
                planner = planning_world = planning_stage = None
                measures = []
                started = time.perf_counter()
                try:
                    np.random.seed(seed)
                    torch.manual_seed(seed)
                    planning_stage, planning_world = world(on)
                    planner = CuroboPlanner(planning_world, kin, dt=data["dt"], seed=seed,
                                             max_joint_speed=settings["max_joint_speed_rad_s"],
                                             transport_object_padding_m=padding)
                    route = planner.plan_pose(q.copy(), target.copy(), ("mustard_bottle",), 1, phase="above_bowl")
                    path = np.asarray(route["positions"])
                    dense = densify(path)
                    np.savez_compressed(args.output / (name + ".npz"), q=path, dense_q=dense,
                                        tcp=np.asarray([kin.fk(v) for v in path]), attachment=attachment)
                    row.update(path_file=name + ".npz", path_sha256=sha(args.output / (name + ".npz")),
                               metadata=route["metadata"], **path_metrics(path, kin, data["dt"], q, target))
                    save()
                    row["full_world_validation"] = planner.validate_path(
                        path, ("mustard_bottle",), 1, phase="above_bowl")
                    for i, joints in enumerate(dense):
                        obj = kin.fk(joints) @ attachment
                        bottle_pose = np.r_[obj[:3, 3], Rotation.from_matrix(obj[:3, :3]).as_quat()]
                        measured = measure_sample(audit_world, mesh, hull, joints, grip, bottle_pose,
                                                  hazard_shapes, required, hazard, robot)
                        conservative = audit_world.clearance(joints, mode="planning", gripper_q=grip,
                                                             object_pose=bottle_pose, obstacle_filter=(hazard,))
                        measures.append(dict(sample=i, time_s=i * data["dt"] / 8,
                                             conservative_planning=conservative, **measured))
                    write_report(args.output / (name + "_geometry.json"), dict(complete=True, samples=len(measures), rows=measures))
                    row.update(success=True, geometry=summarize_samples(measures),
                               conservative_planning_passed=all(m["conservative_planning"]["valid"] for m in measures),
                               conservative_planning_minimum=min(measures, key=lambda m: m["conservative_planning"]["distance_m"])["conservative_planning"],
                               geometric_samples=len(measures), geometry_file=name + "_geometry.json",
                               geometry_sha256=sha(args.output / (name + "_geometry.json")))
                except Exception as error:
                    row["error"] = f"{type(error).__name__}: {error}"
                    if measures:
                        write_report(args.output / (name + "_geometry.json"),
                                     dict(complete=False, samples=len(measures), error=row["error"], rows=measures))
                        row.update(geometry_file=name + "_geometry.json",
                                   geometry_sha256=sha(args.output / (name + "_geometry.json")))
                row["wall_seconds"] = time.perf_counter() - started
                save()
                print(json.dumps(row), flush=True)
                del planner, planning_world, planning_stage
                gc.collect()
                torch.cuda.empty_cache()
        report["pairs"] = pair_gates(report["rows"], args.seeds)
        report["passed"] = all(pair["passed"] for pair in report["pairs"])
        report["status"] = "completed"
    except Exception as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
    save()
    print(json.dumps({k: report[k] for k in ("passed", "status", "pairs", "error") if k in report}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
