"""CPU-only source bowl/sink world with one explicitly declared hazard.

Used after --dry-run dispatch, never imported before a live SimulationApp.
No simulated stability or cooked-geometry claim is made here.
"""
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def build_world(layout, policy):
    from pxr import Gf, Usd, UsdGeom
    from scipy.spatial.transform import Rotation
    from kitchen_safety.planning import DroidKinematics
    from .bowl_collision import BowlCollisionWorld
    from .task_motion import pose_matrix

    stage = Usd.Stage.CreateInMemory()
    root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, root)
    UsdGeom.Xform.Define(stage, root + "/kitchen").GetPrim().GetReferences().AddReference(
        str(ROOT / layout["kitchen_usd"]))

    def add(name, item):
        xf = UsdGeom.Xform.Define(stage, root + "/" + name)
        xf.GetPrim().GetReferences().AddReference(str(ROOT / item["usd_path"]))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, item["position"])))
        xf.AddRotateZOp().Set(float(item.get("yaw_deg", 0.)))

    item = layout["objects"]["bowl"]
    add("bowl", item)
    if "obstacle" in layout:
        add(policy["hazard_name"], layout["obstacle"])
    if stage.GetCompositionErrors():
        raise ValueError("Source/hazard USD composition failed")
    kin = DroidKinematics(np.asarray(layout["robot_position"]) + [0., 0., layout["stand_height"]],
                          layout["robot_yaw_deg"])
    world = BowlCollisionWorld(stage, kin, payload_usd=ROOT / item["usd_path"], env_root=root,
        clearances=policy["clearances_m"], execution_clearances=policy["execution_clearances_m"],
        default_clearance=policy["default_clearance_m"])
    if "obstacle" in layout and not any(world._matches(row[0], policy["hazard_name"]) for row in world.objects):
        raise ValueError("Added hazard has no active collision geometry")
    pose = np.r_[item["position"], Rotation.from_euler("z", item["yaw_deg"], degrees=True).as_quat()]
    world.payload_pose = pose_matrix(pose)
    world.gripper_joint_positions, world.gripper_joint_position = np.zeros(6), 0.
    return world, kin, pose
