#!/usr/bin/env python3
"""CPU authored-stage IK/FCL diagnostic; never launches Kit or physical steps.

This is a diagnostic on authored geometry, not a reset/PhysX observation or a
cuRobo plan. Original source relationships are independently retained.
"""
import argparse
import json
from pathlib import Path


def run():
    import numpy as np
    from pxr import Gf, Usd, UsdGeom
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.task_environment import resolve_task_layout
    from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix, top_down_grasp
    from scripts.extensions.replicator.audit_fixed_base import fixed_colliders
    from scripts.extensions.replicator.audit_task_source_relation import check_layout
    from scripts.extensions.replicator.task_audit_sidecars import sha
    root = Path(__file__).resolve().parents[3]
    layout = resolve_task_layout("kitchen_bench_replicator_g_shape_banana_bowl")
    stage = Usd.Stage.CreateInMemory()
    env_root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, env_root)
    prim = UsdGeom.Xform.Define(stage, env_root+"/kitchen").GetPrim()
    prim.GetReferences().AddReference(str(root/layout["kitchen_usd"]))
    for name, item in layout["objects"].items():
        xf = UsdGeom.Xform.Define(stage, env_root+"/"+name)
        xf.GetPrim().GetReferences().AddReference(str(root/item["usd_path"]))
        # Same original source initial poses; no physics or runtime state writes.
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
        xf.AddRotateZOp().Set(item["yaw_deg"])
    base = np.asarray(layout["robot_position"]).copy()
    base[2] += layout["stand_height"]
    kin = DroidKinematics(base, layout["robot_yaw_deg"])
    world = PayloadCollisionWorld(stage, kin, payload_name="banana", payload_usd=root/layout["objects"]["banana"]["usd_path"],
                                  env_root=env_root)
    world.gripper_joint_positions = np.zeros(6)
    world.gripper_joint_position = 0.
    source_pose = [*layout["objects"]["banana"]["position"], 0., 0., 0., 1.]
    world.payload_pose = pose_matrix(source_pose)
    q = np.deg2rad([0., -36., 0., -144., 0., 108., 0.])
    valid = world.valid(q)
    report = dict(scope="CPU source-authored geometry, deterministic IK and FCL only; no Kit, CUDA, PhysX, actual grasp or task success",
                  initial_q=q.tolist(), initial_geometry_valid=valid, initial_measurement=world.last_clearance,
                  world_colliders=len(world.objects), targets={})
    report["layout"] = layout
    report["canonical_layout_sha256"] = sha(root/"configs/extensions/replicator/g_shape_banana_bowl_source_layout.json")
    report["source_relation_audit"] = check_layout(layout)
    import fcl
    from scipy.spatial.transform import Rotation
    base_pose = np.r_[base, Rotation.from_euler("z", layout["robot_yaw_deg"], degrees=True).as_quat()]
    fixed_failures, near_pairs, allowed_floor = [], 0, 0
    fixed = fixed_colliders(base_pose)
    for shape, info in fixed:
        lo, hi = np.asarray(info["bounds"])
        bounds = np.linalg.norm(np.maximum(np.maximum(world.low-hi, lo-world.high), 0.), axis=1)
        for index in np.flatnonzero(bounds <= .002):
            name, obstacle, _, obstacle_high = world.objects[index]
            if name == env_root+"/kitchen/KitchenRoom/Floor" and "stand" in info["path"].lower() and lo[2] >= obstacle_high[2]-1e-6:
                allowed_floor += 1
                continue
            near_pairs += 1
            raw = float(fcl.distance(shape, obstacle, fcl.DistanceRequest(), fcl.DistanceResult()))
            collision = bool(raw < 0 or (raw == 0 and fcl.collide(shape, obstacle, fcl.CollisionRequest(), fcl.CollisionResult())))
            if collision or raw < .002:
                fixed_failures.append(dict(fixed=info["path"], obstacle=name, intersecting=collision, distance_m=max(0., raw)))
    report["full_source_fixed_geometry"] = dict(passed=not fixed_failures, failures=fixed_failures,
        broad_phase_world_colliders=len(world.objects), near_pairs_checked=near_pairs, allowed_exact_stand_floor_contact_pairs=allowed_floor,
        source_geometry=[info for _, info in fixed], required_other_pair_margin_m=.002)
    target, info = top_down_grasp(source_pose, world.payload_geometry, world)
    report["grasp"] = info
    for phase, offset in (("pregrasp", .125), ("approach", 0.)):
        goal = target.copy()
        goal[2, 3] += offset
        world.set_phase(phase)
        row = dict(goal=goal.tolist())
        try:
            solved = kin.ik(goal, q)
            row.update(ik=True, q=solved.tolist(), geometry_valid=world.valid(solved), measurement=world.last_clearance)
            q = solved
        except Exception as error:
            row.update(ik=False, error=repr(error))
        report["targets"][phase] = row
    if all(row.get("ik") and row.get("geometry_valid") for row in report["targets"].values()):
        world.set_phase("approach")
        start = np.asarray(report["targets"]["pregrasp"]["goal"])
        goal = np.asarray(report["targets"]["approach"]["goal"])
        q = np.asarray(report["targets"]["pregrasp"]["q"])
        rows = []
        for fraction in np.linspace(0., 1., 64):
            tcp = start.copy()
            tcp[:3, 3] = start[:3, 3]+fraction*(goal[:3, 3]-start[:3, 3])
            previous = q.copy()
            q = kin.ik(tcp, q, attempts=1)
            geometry_valid = world.valid(q)
            branch_jump = float(np.abs(q-previous).max())
            rows.append(dict(fraction=float(fraction), geometry_valid=geometry_valid, maximum_joint_increment_rad=branch_jump,
                             q=q.tolist(), collision=world.last_clearance))
        report["local_approach_column"] = dict(samples=len(rows), maximum_cartesian_sample_interval_m=.125/63,
            passed=all(row["geometry_valid"] and row["maximum_joint_increment_rad"] <= .15 for row in rows), rows=rows,
            scope="All 64 local IK samples under full independent FCL; no cuRobo/self-collision or physical path claim")
    report["execution_verified"] = False
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to replace an existing diagnostic")
    report = run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
