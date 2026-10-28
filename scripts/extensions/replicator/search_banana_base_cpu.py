#!/usr/bin/env python3
"""Bounded CPU search of G banana robot X only; source relations stay intact.

No canonical layout, source asset, yaw, stand height or object pose is changed.
All rejected candidates are retained. Source-stage checks are not PhysX success.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def run():
    import fcl
    from pxr import Gf, Usd, UsdGeom
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld
    from kitchen_safety.extensions.replicator.task_contact_policy import BananaContactPolicy
    from kitchen_safety.extensions.replicator.task_environment import resolve_task_layout
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix, top_down_grasp
    from scripts.extensions.replicator.audit_task_source_relation import check_layout
    from scripts.extensions.replicator.audit_fixed_base import fixed_colliders
    from scripts.extensions.replicator.task_audit_sidecars import sha
    root = Path(__file__).resolve().parents[3]
    source = resolve_task_layout("kitchen_bench_replicator_g_shape_banana_bowl")
    stage = Usd.Stage.CreateInMemory()
    env_root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, env_root)
    UsdGeom.Xform.Define(stage, env_root+"/kitchen").GetPrim().GetReferences().AddReference(str(root/source["kitchen_usd"]))
    for name, item in source["objects"].items():
        xf = UsdGeom.Xform.Define(stage, env_root+"/"+name)
        xf.GetPrim().GetReferences().AddReference(str(root/item["usd_path"]))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
        xf.AddRotateZOp().Set(item["yaw_deg"])
    base = np.asarray(source["robot_position"])+[0., 0., source["stand_height"]]
    kin = DroidKinematics(base, source["robot_yaw_deg"])
    world = PayloadCollisionWorld(stage, kin, payload_name="banana", payload_usd=root/source["objects"]["banana"]["usd_path"], env_root=env_root)
    world.gripper_joint_positions = np.zeros(6)
    world.gripper_joint_position = 0.
    banana_pose = np.r_[source["objects"]["banana"]["position"], 0., 0., 0., 1.]
    world.payload_pose = pose_matrix(banana_pose)
    grasp, grasp_info = top_down_grasp(banana_pose, world.payload_geometry, world)
    initial_q = np.deg2rad([0., -36., 0., -144., 0., 108., 0.])
    candidates = []
    for x in np.round(np.arange(source["robot_position"][0], source["robot_position"][0]+.501, .025), 6):
        layout = deepcopy(source)
        layout["robot_position"][0] = float(x)
        row = dict(layout=layout, x_shift_m=float(x-source["robot_position"][0]), checks={})
        candidates.append(row)
        relations = check_layout(layout)
        row["checks"]["original_relations"] = relations["initial_source_relations_passed"]
        row["relations"] = relations
        base_pose = np.r_[np.asarray(layout["robot_position"])+[0., 0., layout["stand_height"]],
                         Rotation.from_euler("z", layout["robot_yaw_deg"], degrees=True).as_quat()]
        pairs, floor_contacts = [], 0
        for shape, info in fixed_colliders(base_pose):
            low, high = np.asarray(info["bounds"])
            bounds = np.linalg.norm(np.maximum(np.maximum(world.low-high, low-world.high), 0.), axis=1)
            for index in np.flatnonzero(bounds <= .002):
                name, obstacle, _, obstacle_high = world.objects[index]
                if name == env_root+"/kitchen/KitchenRoom/Floor" and "stand" in info["path"].lower() and low[2] >= obstacle_high[2]-1e-6:
                    floor_contacts += 1
                    continue
                raw = float(fcl.distance(shape, obstacle, fcl.DistanceRequest(), fcl.DistanceResult()))
                intersecting = bool(raw < 0 or (raw == 0 and fcl.collide(shape, obstacle, fcl.CollisionRequest(), fcl.CollisionResult())))
                if intersecting or raw < .002:
                    pairs.append(dict(fixed=info["path"], obstacle=name, intersecting=intersecting,
                                      distance_m=max(0., raw), required_m=.002))
        row["checks"]["full_source_fixed_geometry"] = not pairs
        row["fixed_geometry_failures"], row["floor_support_pairs"] = pairs, floor_contacts
        if not all(row["checks"].values()):
            row["viable"] = False
            print("BASE_CANDIDATE", x, "relations", row["checks"]["original_relations"], "fixed_failures", len(pairs), flush=True)
            continue
        world.kin = DroidKinematics(base_pose[:3], layout["robot_yaw_deg"])
        world.set_phase("reset")
        row["checks"]["full_robot_initial_geometry"] = world.valid(initial_q)
        row["initial_collision"] = world.last_clearance
        row["targets"] = {}
        q = initial_q.copy()
        for phase, rise in (("pregrasp", .125), ("approach", 0.)):
            world.set_phase(phase)
            target = grasp.copy()
            target[2, 3] += rise
            detail = dict(target=target.tolist())
            row["targets"][phase] = detail
            try:
                q = world.kin.ik(target, q)
                valid = world.valid(q)
                detail.update(ik=True, q=q.tolist(), full_geometry_valid=valid, collision=world.last_clearance)
                row["checks"][phase+"_ik_geometry"] = valid
            except Exception as error:
                detail.update(ik=False, error=repr(error))
                row["checks"][phase+"_ik_geometry"] = False
                break
        row["viable"] = all(row["checks"].values()) and len(row["targets"]) == 2
        print("BASE_CANDIDATE", x, row["checks"], flush=True)
    viable = [row for row in candidates if row["viable"]]
    return dict(task_id=source["task_id"], execution_verified=False, original_layout=source,
        canonical_layout_sha256=sha(root/"configs/extensions/replicator/g_shape_banana_bowl_source_layout.json"),
        source_kitchen_sha256=sha(root/source["kitchen_usd"]), grasp=grasp_info,
        selection_rule="Smallest nonnegative X-only translation among predeclared 0..0.5 m in 0.025 m increments passing original relations, whole fixed stand/link0 and initial/pregrasp/approach robot geometry/IK",
        unchanged=["source yaw", "stand_height", "robot Y/Z", "banana/bowl poses/assets", "kitchen asset", "all relation parameters", "2 mm non-contact margin"],
        candidates=candidates, viable_candidates=len(viable), selected_layout=viable[0]["layout"] if viable else None,
        scope="CPU original stage colliders; no Kit, GPU, cuRobo path, physical grasp, dynamic evidence or canonical replacement. Stand-floor contact uses only 1 micrometre numeric tolerance.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Do not replace rejected-candidate evidence")
    report = run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    if report["selected_layout"] is not None:
        candidate = args.output.with_name(args.output.stem+"_candidate_layout.json")
        candidate.write_text(json.dumps(report["selected_layout"], indent=2)+"\n")
    print(json.dumps(dict(viable_candidates=report["viable_candidates"], selected_layout=report["selected_layout"], output=str(args.output)), indent=2))


if __name__ == "__main__":
    main()
