#!/usr/bin/env python3
"""Measure fixed initial candidates for the remaining assigned kitchen layouts.

Only original Arena relations are validated here. A saved candidate is not a
claim about reachability, actual support contacts, task success or avoidance.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.audit_task_source_relation import (
    ASSIGNMENT, BANANA, BOWL, MUSTARD, ROOT, check_layout, geometry, source,
)


def prepare(task_id):
    _, spec = source(task_id)
    kitchen = spec["background"]["registry_name"].removeprefix("replicator_kitchen_")
    yaw = next(row["params"]["yaw_rad"] for row in spec["relations"]
               if row["kind"] == "rotate_around_solution" and row["subject"] == "droid")
    paths = {"banana_ycb_robolab": BANANA, "bowl_ycb_robolab": BOWL,
             "mustard_bottle_hope_robolab": MUSTARD}
    layout = dict(task_id=task_id, task=spec["task"]["description"],
                  kitchen_usd=f"assets/extensions/replicator/source/Isaac/Environments/replicator_kitchen/kitchen_{kitchen}.usda",
                  stand_height=spec["embodiment"]["params"]["stand_height_m"],
                  robot_position=[0., 0., 0.], robot_yaw_deg=math.degrees(yaw),
                  objects={item["id"]: dict(usd_path=paths[item["registry_name"]],
                           position=[0., 0., 0.], yaw_deg=0.) for item in spec["objects"]})
    _, _, assets, bounds, _ = geometry(layout)

    def low(name): return bounds[assets[name]].min_point.numpy()[0].astype(float)
    def high(name): return bounds[assets[name]].max_point.numpy()[0].astype(float)
    def middle(name): return (low(name) + high(name)) / 2
    def relation(kind, subject):
        return next(row for row in spec["relations"] if row["kind"] == kind and row["subject"] == subject)
    def side_axes(side):
        if side not in ("negative_x", "positive_x", "negative_y", "positive_y"):
            raise ValueError("Unsupported original planar relation: " + side)
        axis = 0 if side.endswith("x") else 1
        return axis, 1 - axis, 1 if side.startswith("positive") else -1

    robot_next = relation("next_to", "droid")
    reference = robot_next["reference"]
    axis, tangent, sign = side_axes(robot_next["params"]["side"])
    robot = middle(reference)
    gap = robot_next["params"]["distance_m"]
    robot[axis] = (high(reference)[axis] + gap - low("droid")[axis] if sign > 0
                   else low(reference)[axis] - gap - high("droid")[axis])
    floor = relation("on", "droid")["reference"]
    robot[2] = high(floor)[2] - low("droid")[2] - layout["stand_height"]
    layout["robot_position"] = robot.tolist()
    task = spec["task"]["subtasks"][0]["params"]
    pickup, destination = task["pick_up_object"], task["destination_location"]
    for name, item in layout["objects"].items():
        support = relation("on", name)["reference"]
        position = middle(support)
        next_to = [row for row in spec["relations"] if row["kind"] == "next_to" and row["subject"] == name]
        if next_to:
            if len(next_to) != 1:
                raise ValueError("Multiple object next-to relations need explicit source-aware solving")
            row = next_to[0]
            near = row["reference"]
            a, t, direction = side_axes(row["params"]["side"])
            position[t] = middle(near)[t]
            distance = row["params"]["distance_m"]
            position[a] = (high(near)[a] + distance - low(name)[a] if direction > 0
                           else low(near)[a] - distance - high(name)[a])
        else:
            if destination not in layout["objects"]:
                raise ValueError("Source destination is a reference but pickup lacks a placement relation")
            # Place both objects near the robot-facing support edge, with a
            # visible gap along the counter. All source margins are rechecked.
            position[axis] = (high(support)[axis] - .05 - high(name)[axis] - .12 if sign > 0
                              else low(support)[axis] + .05 - low(name)[axis] + .12)
            position[tangent] = middle(support)[tangent] + (-.19 if name == pickup else .19)
        position[2] = high(support)[2] - low(name)[2] + .002
        item["position"] = position.tolist()
    return layout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New evidence directory")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    summaries = []
    for entry in json.loads(ASSIGNMENT.read_text())["tasks"]:
        if entry["background"]["registry_name"] == "replicator_kitchen_g_shape":
            continue
        short = entry["id"].removeprefix("kitchen_bench_replicator_")
        try:
            layout = prepare(entry["id"])
            report = check_layout(layout)
            if report["initial_source_relations_passed"]:
                config = ROOT / "configs/extensions/replicator" / (short + "_source_layout.json")
                if config.exists() and json.loads(config.read_text()) != layout:
                    raise FileExistsError("Existing layout differs; retain it: " + str(config))
                config.write_text(json.dumps(layout, indent=2) + "\n")
                report["candidate_config"] = str(config.relative_to(ROOT))
            (args.output / (short + ".json")).write_text(json.dumps(report, indent=2) + "\n")
            row = dict(task_id=entry["id"], source_relations_passed=report["initial_source_relations_passed"],
                       execution_ready=False, failed_relations=[r for r in report["checks"] if not r["passed"]])
        except Exception as error:
            row = dict(task_id=entry["id"], source_relations_passed=False, execution_ready=False,
                       error=f"{type(error).__name__}: {error}")
        summaries.append(row)
        print(json.dumps(row), flush=True)
    (args.output / "summary.json").write_text(json.dumps(dict(tasks=summaries,
        scope="Initial original relation validation only; physics, IK, support and collection not verified"), indent=2) + "\n")
    return 0 if all(row["source_relations_passed"] for row in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
