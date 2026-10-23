#!/usr/bin/env python3
"""Validate an assigned fixed layout with Arena's original relation validators.

CPU only. ``--prepare-g-shape`` measures source assets and writes initial banana
and bowl/sink candidates only after every original relation passes. It does not
claim IK, collision-free execution, physical success, or runnable collection.
Existing mustard scripts and layouts are not changed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml
from pxr import Usd, UsdGeom

from isaaclab_arena.relations.bounding_box_helpers import rotate_candidate_bboxes
from isaaclab_arena.relations.relations import NextTo, On, RotateAroundSolution
from isaaclab_arena.relations.validation.pre_physics import NextToValidator, OnRelationValidator
from isaaclab_arena.utils.bounding_box import AxisAlignedBoundingBox
from isaaclab_arena.utils.usd.helpers import (
    compute_local_bounding_box_from_usd,
    compute_local_bounding_box_from_usd_at_joint_pos,
)

ROOT = Path(__file__).resolve().parents[3]
ASSIGNMENT = ROOT / "student_skills/robolaw-replicator/references/assignment.json"
STAND = "runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd"
KITCHEN = "assets/extensions/replicator/source/Isaac/Environments/replicator_kitchen/kitchen_g_shape.usda"
BANANA = "assets/extensions/replicator/source/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/ycb/banana.usd"
BOWL = "assets/robolab/objects/ycb/bowl.usd"
MUSTARD = "assets/robolab/objects/hope/mustard_bottle.usd"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Asset:
    def __init__(self, name):
        self.name, self.relations = name, []

    def get_relations(self):
        return self.relations


def source(task_id):
    assignments = json.loads(ASSIGNMENT.read_text())["tasks"]
    entry = next((item for item in assignments if item["id"] == task_id), None)
    if entry is None:
        raise ValueError(f"Task is not assigned to A: {task_id}")
    path = ROOT / entry["source_yaml"]
    if digest(path) != entry["source_sha256"]:
        raise ValueError(f"Source YAML differs from assignment hash: {path}")
    spec = yaml.safe_load(path.read_text())
    if spec["env_name"] != task_id:
        raise ValueError("Source task ID mismatch")
    return entry, spec


def geometry(layout):
    entry, spec = source(layout["task_id"])
    kitchen_name = spec["background"]["registry_name"].removeprefix("replicator_kitchen_")
    if kitchen_name not in ("g_shape", "l_island", "l_shape", "peninsula", "u_shape"):
        raise ValueError("Unsupported assigned source kitchen registry")
    kitchen_path = ("assets/extensions/replicator/source/Isaac/Environments/replicator_kitchen/"
                    f"kitchen_{kitchen_name}.usda")
    if layout["kitchen_usd"] != kitchen_path:
        raise ValueError("Layout must preserve the original task's registered kitchen")
    if layout["stand_height"] != spec["embodiment"]["params"]["stand_height_m"]:
        raise ValueError("Robot stand height must preserve the source task")
    objects = {item["id"]: item for item in spec["objects"]}
    if set(layout["objects"]) != set(objects):
        raise ValueError("Layout must contain exactly the source task objects")
    expected = {"banana_ycb_robolab": BANANA, "bowl_ycb_robolab": BOWL,
                "mustard_bottle_hope_robolab": MUSTARD}
    for name, item in objects.items():
        if layout["objects"][name]["usd_path"] != expected[item["registry_name"]]:
            raise ValueError(f"Object {name} must use its original registry asset")
    names = [spec["embodiment"]["id"], *objects, *[ref["id"] for ref in spec["object_references"]]]
    assets = {name: Asset(name) for name in names}
    stage = Usd.Stage.Open(str(ROOT / layout["kitchen_usd"]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bounds = {}
    for ref in spec["object_references"]:
        if ref["parent_id"] != "kitchen" or ref["object_type"] != "base":
            raise ValueError("Unsupported source reference; do not silently approximate it")
        prim = stage.GetPrimAtPath(str(stage.GetDefaultPrim().GetPath()) + "/" + ref["prim_path"])
        if not prim:
            raise ValueError(f"Missing original kitchen reference: {ref['prim_path']}")
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        bounds[assets[ref["id"]]] = AxisAlignedBoundingBox(tuple(box.GetMin()), tuple(box.GetMax()))
    bounds[assets["droid"]] = compute_local_bounding_box_from_usd_at_joint_pos(
        str(ROOT / STAND), {}, prim_path="/panda/panda_link0/stand_instanceable")
    for name, item in layout["objects"].items():
        bounds[assets[name]] = compute_local_bounding_box_from_usd(str(ROOT / item["usd_path"]))
    yaw = {assets["droid"]: math.radians(layout["robot_yaw_deg"])}
    yaw.update({assets[name]: math.radians(item.get("yaw_deg", 0.0)) for name, item in layout["objects"].items()})
    bounds = rotate_candidate_bboxes(list(assets.values()), bounds, [yaw])
    for name, box in bounds.items():
        low, high = box.min_point.numpy(), box.max_point.numpy()
        if not np.isfinite([low, high]).all() or np.any(high <= low):
            raise ValueError(f"Missing or degenerate source bounds: {name.name}")
    return entry, spec, assets, bounds, yaw


def check_layout(layout):
    entry, spec, assets, bounds, yaw = geometry(layout)
    robot = assets["droid"]
    positions = {assets[ref["id"]]: (0., 0., 0.) for ref in spec["object_references"]}
    positions[robot] = tuple(np.array(layout["robot_position"]) + [0, 0, layout["stand_height"]])
    positions.update({assets[name]: tuple(item["position"]) for name, item in layout["objects"].items()})
    for row in spec["relations"]:
        cls = {"on": On, "next_to": NextTo, "rotate_around_solution": RotateAroundSolution}.get(row["kind"])
        if cls:
            args = [assets[row["reference"]]] if "reference" in row else []
            assets[row["subject"]].relations.append(cls(*args, **row["params"]))
    params = SimpleNamespace(verbose=False, on_relation_z_tolerance_m=.005)
    checks = []
    for relation in spec["relations"]:
        child, kind = assets[relation["subject"]], relation["kind"]
        row = dict(relation)
        if kind in ("on", "next_to"):
            parent = assets[relation["reference"]]
            validator = OnRelationValidator(params) if kind == "on" else NextToValidator(params)
            row.update(passed=bool(validator._validate({child: positions[child], parent: positions[parent]}, bounds)),
                       validator=type(validator).__name__)
            child_box, parent_box = bounds[child].translated(positions[child]), bounds[parent].translated(positions[parent])
            if kind == "on":
                margins = np.r_[child_box.min_point.numpy()[0, :2] - parent_box.min_point.numpy()[0, :2],
                                parent_box.max_point.numpy()[0, :2] - child_box.max_point.numpy()[0, :2]]
                row.update(required_edge_margin_m=next(r.edge_margin_m for r in child.relations if isinstance(r, On)),
                           edge_margins_xy_m=margins.tolist(),
                           vertical_gap_m=float(child_box.min_point[0, 2] - parent_box.max_point[0, 2]))
        elif kind == "rotate_around_solution":
            row.update(passed=abs(yaw[child] - relation["params"]["yaw_rad"]) < 1e-8,
                       actual_yaw_rad=yaw[child])
        elif kind == "is_anchor":
            row.update(passed=True, detail="Original kitchen/reference transforms, unchanged at identity world root")
        else:
            raise ValueError(f"Unvalidated original relation: {relation}")
        checks.append(row)
    files = [layout["kitchen_usd"], STAND, *[item["usd_path"] for item in layout["objects"].values()]]
    return {
        "task_id": layout["task_id"],
        "initial_source_relations_passed": all(item["passed"] for item in checks),
        "scope": "Authored fixed initial placement only; no IK, trajectory collision, contact physics, task success, RGB or avoidance evidence.",
        "execution_ready": False,
        "source_yaml": entry["source_yaml"], "source_yaml_sha256": entry["source_sha256"],
        "assets_sha256": {path: digest(ROOT / path) for path in files},
        "layout": layout,
        "posed_local_bounds": {asset.name: {"min": box.min_point.tolist()[0], "max": box.max_point.tolist()[0]}
                               for asset, box in bounds.items()},
        "checks": checks,
    }


def prepare(operation):
    task_id = "kitchen_bench_replicator_g_shape_" + operation
    _, spec = source(task_id)
    source_yaw = next(row["params"]["yaw_rad"] for row in spec["relations"] if row["kind"] == "rotate_around_solution")
    object_paths = {"banana": BANANA, "bowl": BOWL}
    layout = {"task_id": task_id, "task": spec["task"]["description"], "kitchen_usd": KITCHEN,
              "stand_height": .8, "robot_position": [0., 0., 0.],
              "robot_yaw_deg": math.degrees(source_yaw),
              "objects": {item["id"]: {"usd_path": object_paths[item["id"]], "position": [0., 0., 0.], "yaw_deg": 0.}
                          for item in spec["objects"]}}
    _, _, assets, bounds, _ = geometry(layout)
    def lo(name): return bounds[assets[name]].min_point.numpy()[0]
    def hi(name): return bounds[assets[name]].max_point.numpy()[0]
    if operation == "banana_bowl":
        layout["robot_position"] = [-2.1, float(lo("corner_counter")[1] - .15 - hi("droid")[1]), 0.]
        for name, x in (("banana", -2.26), ("bowl", -1.81)):
            layout["objects"][name]["position"] = [x, 2.15, float(hi("corner_counter")[2] - lo(name)[2] + .002)]
    else:
        layout["robot_position"] = [float(lo("sink")[0] - .15 - hi("droid")[0]), .10, 0.]
        layout["objects"]["bowl"]["position"] = [float((lo("sink")[0] + hi("sink")[0]) / 2),
            float(hi("sink")[1] + .10 - lo("bowl")[1]), float(hi("sink_counter")[2] - lo("bowl")[2] + .002)]
    return layout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--layout", type=Path)
    mode.add_argument("--prepare-g-shape", action="store_true")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports/replicator_preparation")
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    layouts = [prepare(operation) for operation in ("banana_bowl", "bowl_sink")] if args.prepare_g_shape else [json.loads(args.layout.read_text())]
    passed = True
    for layout in layouts:
        report = check_layout(layout)
        short = layout["task_id"].removeprefix("kitchen_bench_replicator_")
        report_path = args.report_dir / f"{short}_source_relations.json"
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        if report["initial_source_relations_passed"] and args.prepare_g_shape:
            path = ROOT / "configs/extensions/replicator" / f"{short}_source_layout.json"
            if path.exists() and json.loads(path.read_text()) != layout:
                raise FileExistsError(f"Refusing to overwrite a different existing layout: {path}")
            path.write_text(json.dumps(layout, indent=2) + "\n")
        passed &= report["initial_source_relations_passed"]
        print(json.dumps({"task": layout["task_id"], "initial_source_relations_passed": report["initial_source_relations_passed"],
                          "execution_ready": False, "report": str(report_path),
                          "failed_relations": [item for item in report["checks"] if not item["passed"]]}, indent=2))
    return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
