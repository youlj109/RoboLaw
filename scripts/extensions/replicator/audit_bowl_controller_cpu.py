#!/usr/bin/env python3
"""CPU G bowl/sink source rim target and approach; no Kit/CUDA/physical success."""
import argparse
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.bowl_collision import BowlCollisionWorld
from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_grasp, sink_placement_object_pose
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
from scripts.extensions.replicator.audit_task_source_relation import ROOT, check_layout
from scripts.extensions.replicator.task_audit_sidecars import sha
from scripts.extensions.replicator.task_collect import json_write


def build_world(layout):
    stage = Usd.Stage.CreateInMemory()
    root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, root)
    UsdGeom.Xform.Define(stage, root+"/kitchen").GetPrim().GetReferences().AddReference(str(ROOT/layout["kitchen_usd"]))
    item = layout["objects"]["bowl"]
    xf = UsdGeom.Xform.Define(stage, root+"/bowl")
    xf.GetPrim().GetReferences().AddReference(str(ROOT/item["usd_path"]))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
    xf.AddRotateZOp().Set(item["yaw_deg"])
    kin = DroidKinematics(np.asarray(layout["robot_position"])+[0., 0., layout["stand_height"]], layout["robot_yaw_deg"])
    world = BowlCollisionWorld(stage, kin, payload_usd=ROOT/item["usd_path"], env_root=root,
        clearances={}, execution_clearances={}, default_clearance=.002)
    from scipy.spatial.transform import Rotation
    pose = np.r_[item["position"], Rotation.from_euler("z", item["yaw_deg"], degrees=True).as_quat()]
    world.payload_pose = pose_matrix(pose)
    world.gripper_joint_positions, world.gripper_joint_position = np.zeros(6), 0.
    return world, kin, pose


def audit(layout_path):
    layout = json.loads(layout_path.read_text())
    if layout["task_id"] != "kitchen_bench_replicator_g_shape_bowl_sink":
        raise ValueError("This controller audit is scoped to the original G bowl/sink")
    relations = check_layout(layout)
    if not relations["initial_source_relations_passed"]:
        raise ValueError("Do not run source IK on a relationship-invalid candidate")
    world, kin, pose = build_world(layout)
    values, initial = source_initial_q()
    q = np.array([values[f"panda_joint{i}"] for i in range(1, 8)])
    initial_valid = world.valid(q)
    report = dict(accepted=False, execution_ready=False, layout_sha256=sha(layout_path), layout=layout,
        original_source_relations=relations, initial_joint_source=initial, full_kitchen_collider_count=len(world.objects),
        initial_full_fcl=dict(passed=initial_valid, violation=world.last_clearance),
        original_contact_policy=world.contact_policy.report(), payload_geometry=world.payload_geometry.provenance,
        scope="CPU source geometry and IK only; full fixed initial source geometry is separately required. No CUDA/self-collision, neutral, physical rim grasp, original success or RGB claim.")
    target, grasp = bowl_rim_grasp(pose, world.payload_geometry, world)
    report["grasp_geometry"] = grasp
    report["targets"] = {}
    for phase, height in (("pregrasp", .125), ("approach", 0.)):
        goal = target.copy()
        goal[2, 3] += height
        world.set_phase(phase)
        row = dict(target_tcp=goal)
        try:
            solved = kin.ik(goal, q)
            valid = world.valid(solved)
            row.update(ik=True, q=solved, full_fcl=valid, violation=world.last_clearance)
            q = solved
        except Exception as error:
            row.update(ik=False, error=repr(error))
        report["targets"][phase] = row
    if report["targets"]["pregrasp"].get("ik") and report["targets"]["approach"].get("ik"):
        world.set_phase("approach")
        candidate = local_cartesian_candidate(kin, report["targets"]["pregrasp"]["q"], target, "approach", 1/15, .4)
        rows = []
        for index, point in enumerate(candidate["positions"]):
            valid = world.valid(point)
            rows.append(dict(sample=index, q=point, full_fcl=valid, violation=world.last_clearance))
        report["approach_candidate"] = dict(passed=all(row["full_fcl"] for row in rows),
            samples=len(rows), metadata=candidate["metadata"], rows=rows)
    placement, source_floor = sink_placement_object_pose(pose, world.payload_geometry, world)
    report["source_sink_placement_candidate"] = dict(object_pose=placement, source=source_floor,
        note="Measured TCP-to-bowl after actual grasp is mandatory before converting this to a robot goal")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite a source controller diagnostic")
    report = audit(args.layout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    json_write(args.output, report)
    print(json.dumps(dict(output=str(args.output), initial_full_fcl=report["initial_full_fcl"],
        targets={key: {field: row.get(field) for field in ("ik", "full_fcl", "violation", "error")} for key, row in report["targets"].items()},
        approach_passed=report.get("approach_candidate", {}).get("passed"), execution_ready=False)))


if __name__ == "__main__":
    main()
