#!/usr/bin/env python3
"""Bounded G bowl/sink initial-layout candidates; canonical is never changed."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.audit_source_layout_geometry import REQUIRED_M, minimum_distance, sha
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
from scripts.extensions.replicator.audit_fixed_base import fixed_colliders
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch, nearest_grid
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, default=Path("configs/extensions/replicator/g_shape_bowl_sink_source_layout.json"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite a bounded candidate search")
    args.output.mkdir(parents=True)
    layout = json.loads(args.layout.read_text())
    if layout["task_id"] != "kitchen_bench_replicator_g_shape_bowl_sink":
        raise ValueError("This bounded search is scoped to original G bowl/sink")
    grid = nearest_grid(.30, .025, [0.])
    declaration = dict(source_layout_sha256=sha(args.layout), source_layout=layout,
        fixed=["source task/asset identities", "all yaws", "stand height", "source initial joint configuration",
               "source On/NextTo validation parameters", "2 mm original geometry margin"],
        policy=dict(stand_xy_radius_m=.30, stand_xy_step_m=.025, stand_z_offsets_m=[0.],
            bowl_xyz_offsets_m=[[0., 0., 0.], [0., 0., .0001], [0., 0., .001]],
            selection="First bowl height candidate passing original relations/support/2mm geometry; then first full source+proxy robot candidate in original nearest_grid Euclidean order. Preserve every visited failure."),
        execution_ready=False, accepted=False)
    (args.output/"declaration.json").write_text(json.dumps(declaration, indent=2)+"\n")
    search = CandidateSearch(layout)
    object_rows, selected_object = [], None
    for dz in (0., .0001, .001):
        row = search.evaluate("bowl", [0., 0., dz])
        object_rows.append(row)
        if row["passed"]:
            selected_object = np.array([0., 0., dz])
            search.offsets["bowl"] = selected_object
            break
    robot_rows, selected_robot = [], None
    values, initial = source_initial_q()
    q = np.array([values[f"panda_joint{i}"] for i in range(1, 8)])
    fingers = {name: values[name] for name in CollisionWorld.GRIPPER_JOINT_NAMES}
    empty = Usd.Stage.CreateInMemory()
    robot_world = CollisionWorld(empty, DroidKinematics(np.array(layout["robot_position"])+[0., 0., .8], 0.))
    source_robot = SourceRobotGeometry(robot_world)
    if selected_object is not None:
        objects = search.shapes["bowl"].at(selected_object)
        obstacles = search.world+objects
        for index, delta in enumerate(grid):
            row = search.evaluate("droid", delta)
            row["grid_index"] = index
            if row["passed"]:
                base = np.asarray(layout["robot_position"])+np.asarray(delta)+[0., 0., layout["stand_height"]]
                robot_world.kin = DroidKinematics(base, layout["robot_yaw_deg"])
                moving = source_robot.shapes(q, fingers)
                source = moving+search.shapes["droid"].at(delta)
                fixed = fixed_colliders(np.r_[base, Rotation.from_euler("z", layout["robot_yaw_deg"], degrees=True).as_quat()])
                fixed_rows = [(info["path"], shape, *np.asarray(info["bounds"])) for shape, info in fixed]
                proxy = robot_world.shapes(q, gripper_q=fingers)+fixed_rows
                row["full_source_initial_robot"] = minimum_distance(source, obstacles, REQUIRED_M)
                row["full_conservative_initial_robot"] = minimum_distance(proxy, obstacles, REQUIRED_M)
                row["passed"] = bool(row["full_source_initial_robot"]["valid"] and row["full_conservative_initial_robot"]["valid"])
            robot_rows.append(row)
            if row["passed"]:
                selected_robot = np.asarray(delta)
                break
    report = dict(**declaration, initial_joint_source=initial, bowl_candidates=object_rows, robot_candidates=robot_rows,
        visited_robot_candidates=len(robot_rows), predeclared_robot_candidate_count=len(grid),
        candidate_found=selected_object is not None and selected_robot is not None)
    if report["candidate_found"]:
        candidate = deepcopy(layout)
        candidate["robot_position"] = (np.asarray(candidate["robot_position"])+selected_robot).tolist()
        candidate["objects"]["bowl"]["position"] = (np.asarray(candidate["objects"]["bowl"]["position"])+selected_object).tolist()
        relations = check_layout(candidate)
        if not relations["initial_source_relations_passed"]:
            raise ValueError("Full original relationship validation unexpectedly differs")
        path = args.output/"candidate_layout.json"
        path.write_text(json.dumps(candidate, indent=2)+"\n")
        report.update(candidate_layout=str(path), candidate_layout_sha256=sha(path), original_source_relations=relations,
            selected_robot_offset_m=selected_robot.tolist(), selected_bowl_offset_m=selected_object.tolist(),
            required_remaining=["Full candidate source support report", "Bowl rim grasp/source contact feasibility", "IK/cuRobo/full path", "Neutral/PhysX/actual task success"])
    (args.output/"search_report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(dict(candidate_found=report["candidate_found"], visited_robot_candidates=len(robot_rows),
        robot_offset=report.get("selected_robot_offset_m"), bowl_offset=report.get("selected_bowl_offset_m"), execution_ready=False)))


if __name__ == "__main__":
    main()
