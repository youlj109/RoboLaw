"""Plan and independently collision-check the real task pregrasp without Kit."""
import argparse
import json
from pathlib import Path

import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.planning import DroidKinematics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", default="human")
    parser.add_argument("--sequence", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    layout = json.loads((root / "configs/layout.json").read_text())
    base = layout["robot_position"].copy()
    base[2] += layout["stand_height"]
    kin = DroidKinematics(base, layout["robot_yaw_deg"])
    world = CollisionWorld(Usd.Stage.Open(str(root / "scenes" / f"{args.variant}.usda")), kin,
                           clearances={"person": .03, "glass": .06, "barrier": .02},
                           default_clearance=.002)
    planner = CuroboPlanner(world, kin)
    start = np.deg2rad([0, -36, 0, -144, 0, 108, 0])
    target = np.eye(4)
    target[:3, 3] = [2., -.4, 1.16]
    target[:3, :3] = Rotation.from_euler("x", 35, degrees=True).as_matrix() @ np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    result = planner.plan_pose(start, target, ignore_names=("Mustard", "Stand"))
    errors = []
    for i, q in enumerate(result["positions"]):
        if not world.valid(q, ("Mustard", "Stand")):
            errors.append({"sample": i, "collision": world.last_collision})
    report = dict(result["metadata"], fcl_invalid_samples=errors,
                  goal_position_error_m=float(np.linalg.norm(kin.fk(result["positions"][-1])[:3, 3] - target[:3, 3])))
    (root / "reports" / f"curobo_pregrasp_{args.variant}.json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez(root / "reports" / f"curobo_pregrasp_{args.variant}.npz", positions=result["positions"])
    print(json.dumps(report, indent=2))
    assert not errors, "Independent FCL check rejected a cuRobo sample"
    if args.sequence:
        q = result["positions"][-1]
        grip = 0
        stages = [("approach", [2., -.4, 1.035]), ("lift", [2., -.4, 1.18]),
                  ("transfer_front", [2., -.56, 1.18]), ("transfer", [2.49, -.56, 1.18]),
                  ("above_bowl", [2.49, -.31, 1.15]), ("lower", [2.49, -.31, 1.085]),
                  ("retreat", [2.49, -.31, 1.18])]
        reports = []
        for phase, xyz in stages:
            target[:3, 3] = xyz
            ignore = ("Mustard", "Stand", "Bowl") if phase in ("lower", "retreat") else ("Mustard", "Stand")
            if phase == "lift":
                bottle = np.eye(4)
                bottle[:3, 3] = [2., -.4, .9993]
                world.attachment = np.linalg.inv(kin.fk(q)) @ bottle
                grip = 1
                world.gripper_joint_position = .38
            elif phase == "retreat":
                world.attachment = None
                grip = 0
                world.gripper_joint_position = 0.
            trial_errors = []
            for tilt, yaw in [(35, 0), (25, 0), (35, -30), (35, -45), (45, -30), (45, 0), (55, -45), (55, 0), (35, 30), (35, 45)]:
                target[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ Rotation.from_euler("x", tilt, degrees=True).as_matrix() @ np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
                try:
                    result = planner.plan_pose(q, target, ignore_names=ignore, grip_binary=grip)
                    path = np.vstack([q, result["positions"]])
                    for i in range(len(path) - 1):
                        if not world.edge(path[i], path[i + 1], ignore):
                            raise RuntimeError(f"FCL edge {i}: {world.last_collision}")
                    q = path[-1]
                    report = dict(phase=phase, tilt=tilt, yaw=yaw, **result["metadata"])
                    print("SEQUENCE_OK", report, flush=True)
                    reports.append(report)
                    break
                except RuntimeError as exc:
                    trial_errors.append(str(exc))
                    print("SEQUENCE_RETRY", phase, tilt, yaw, str(exc), flush=True)
            else:
                reports.append({"phase": phase, "failed": True, "errors": trial_errors})
                (root / "reports" / f"curobo_sequence_{args.variant}.json").write_text(json.dumps(reports, indent=2) + "\n")
                raise RuntimeError(f"No valid {phase} trajectory")
        (root / "reports" / f"curobo_sequence_{args.variant}.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
