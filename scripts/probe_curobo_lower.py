"""Replan a recorded physical lowering start with the placement-column constraint."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.planning import DroidKinematics


def pose(state):
    result = np.eye(4)
    result[:3, 3] = state[:3]
    result[:3, :3] = Rotation.from_quat(state[3:7]).as_matrix()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", default="datasets/kitchen_auto/revised_curobo/property/attempt_0000_seed_42")
    parser.add_argument("--output", default="reports/curobo_constrained_lower.json")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    episode = root / args.episode
    old_result = json.loads((episode / "result.json").read_text())
    layout = old_result["layout"]
    base = layout["robot_position"].copy()
    base[2] += layout["stand_height"]
    kin = DroidKinematics(base, layout["robot_yaw_deg"])
    with h5py.File(episode / "trajectory.h5") as data:
        phases = data["phase"].asstr()[:]
        lift = np.flatnonzero(phases == "lift")[0]
        lower = np.flatnonzero(phases == "lower")
        q = data["obs/q"][lower[0], :7] if len(lower) else data["next_obs/q"][-1, :7]
        attachment = np.linalg.inv(kin.fk(data["obs/q"][lift, :7])) @ pose(data["obs/mustard"][lift])
        gripper = float(data["obs/q"][lower[0], 7] if len(lower) else data["next_obs/q"][-1, 7])
        target = kin.fk(q)
        target[:2, 3] = data["obs/bowl"][30, :2]
        target[2, 3] = 1.085
        old_command_poses = np.array([kin.fk(v[:7]) for v in data["action"][lower]]) if len(lower) else np.array([kin.fk(q)])
        old_actual_poses = data["next_obs/tcp"][lower] if len(lower) else data["next_obs/tcp"][-1:]
        old_actual_bottle_tilts = np.rad2deg(np.arccos(np.clip(
            Rotation.from_quat((data["next_obs/mustard"][lower, 3:7] if len(lower) else data["next_obs/mustard"][-1:, 3:7])).as_matrix()[:, 2, 2], -1., 1.)))
    world = CollisionWorld(Usd.Stage.Open(str(root / "scenes" / f"{old_result['variant']}.usda")), kin,
                           clearances=layout["planning"]["clearances_m"], default_clearance=.002)
    world.attachment = attachment
    world.gripper_joint_position = gripper
    planner = CuroboPlanner(world, kin, seed=old_result["seed"])
    ignored = ("Mustard", "Bowl", "Stand")
    result = planner.plan_pose(q, target, ignored, 1, phase="lower")
    path = np.vstack([q, result["positions"]])
    bad_edges = []
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        if not world.edge(a, b, ignored):
            bad_edges.append({"edge": i, "reason": world.last_collision})
    rigid_tilts = [np.rad2deg(np.arccos(np.clip((kin.fk(v) @ attachment)[2, 2], -1., 1.))) for v in path]
    heights = np.array([kin.fk(v)[2, 3] for v in path])
    report = {
        "episode": str(episode.relative_to(root)), "start_frame": int(lower[0]) if len(lower) else len(phases),
        "start_tcp": kin.fk(q)[:3, 3].tolist(), "goal_tcp": target[:3, 3].tolist(),
        "old_command_xy_deviation_m": float(np.linalg.norm(old_command_poses[:, :2, 3] - target[:2, 3], axis=1).max()),
        "old_actual_xy_deviation_m": float(np.linalg.norm(old_actual_poses[:, :2] - target[:2, 3], axis=1).max()),
        "old_actual_bottle_max_tilt_deg": float(old_actual_bottle_tilts.max()),
        "new_plan": result["metadata"], "new_rigid_bottle_max_tilt_deg": max(rigid_tilts),
        "fcl_invalid_edges": bad_edges,
        "vertical_height_check": {
            "minimum_tcp_z_m": float(heights.min()),
            "maximum_tcp_z_m": float(heights.max()),
            "target_tcp_z_m": float(target[2, 3]),
            "downward_overshoot_beyond_target_m": float(max(0., target[2, 3] - heights.min())),
            "monotonic_descent": bool(np.max(np.diff(heights)) <= 1e-6),
            "description": "Cartesian IK samples a linear descent; measured height bounds shown above.",
        },
        "note": "Offline planning validation only; physical execution remains necessary.",
    }
    output = root / args.output
    output.write_text(json.dumps(report, indent=2) + "\n")
    np.savez(output.with_suffix(".npz"), positions=path, target=target)
    print(json.dumps(report, indent=2))
    assert not bad_edges, "Independent FCL edge check rejected the constrained descent"


if __name__ == "__main__":
    main()
