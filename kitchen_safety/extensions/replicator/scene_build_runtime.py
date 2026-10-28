"""Scene-only USD composition and evidence, separate from collection acceptance.

Source geometry is referenced without edits. This leaf does not plan or execute
trajectories and must never be used as task-success/avoidance evidence.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ROBOT = "runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd"
REQUIRED_LOAD_CHECKS = ("simulation_loaded", "render_nonempty", "source_assets_unchanged",
    "robot_articulation_present", "task_objects_rigid_present", "hazard_collision_enabled")
SCOPE = "Scene composition, actual Isaac Sim stage loading and rendered preview only; no physics stepping, task success, neutral stability, or avoidance acceptance."


def stamp():
    return datetime.now(timezone.utc).isoformat()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    os.replace(temporary, path)


def relative(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def update_progress(path, row, **changes):
    path = Path(path)
    document = json.loads(path.read_text()) if path.exists() else {
        "schema_version": 1, "stage": "rendering", "total": 154, "entries": []}
    key = row["task_id"], row["variant_id"]
    entry = next((entry for entry in document["entries"]
                  if (entry["task_id"], entry["variant_id"]) == key), None)
    if entry is None:
        entry = dict(row)
        document["entries"].append(entry)
    checks = dict(entry.get("checks", {}))
    checks.update(changes.pop("checks", {}))
    entry.update(changes, checks=checks, updated_at=stamp())
    document.update(stage="rendering", updated_at=stamp(), scope=SCOPE)
    document["counts"] = {state: sum(item.get("status") == state for item in document["entries"])
                          for state in ("pending", "generated", "validated", "loading", "ready", "failed")}
    if document["counts"]["ready"] == document["total"]:
        document["stage"] = "scene_build_complete"
    atomic_json(path, document)


def inventory(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    errors = stage.GetCompositionErrors()
    if errors:
        raise ValueError("USD composition errors: " + " | ".join(map(str, errors)))
    groups = {}
    for name, prefix in (("kitchen", "/World/kitchen"), ("robot", "/World/robot"),
                         ("objects", "/World/objects"), ("hazard", "/World/hazard")):
        root = stage.GetPrimAtPath(prefix)
        if not root:
            raise ValueError(f"Missing composed scene group: {prefix}")
        rows = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
        groups[name] = {
            "prim_count": len(rows),
            "mesh_count": sum(prim.IsA(UsdGeom.Mesh) for prim in rows),
            "collision_count": sum(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in rows),
            "enabled_collision_count": sum(prim.HasAPI(UsdPhysics.CollisionAPI) and
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False for prim in rows),
            "rigid_body_count": sum(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in rows),
            "articulation_count": sum(prim.HasAPI(UsdPhysics.ArticulationRootAPI) for prim in rows),
        }
        if groups[name]["enabled_collision_count"] < 1:
            raise ValueError(f"No enabled collision geometry: {prefix}")
    if groups["robot"]["articulation_count"] < 1:
        raise ValueError("Source DROID has no articulation")
    if groups["objects"]["rigid_body_count"] < 1:
        raise ValueError("Original task objects have no rigid body")
    return groups


def compose(layout_path, scene_path):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
    layout_path, scene_path = Path(layout_path).resolve(), Path(scene_path).resolve()
    layout = json.loads(layout_path.read_text())
    if layout["stand_height"] != .8:
        raise ValueError("This original DROID stand composition is exactly 0.8 m")
    import yaml
    assignment = json.loads((ROOT / "student_skills/robolaw-replicator/references/assignment.json").read_text())
    task_source = next(item for item in assignment["tasks"] if item["id"] == layout["task_id"])
    if sha(ROOT / task_source["source_yaml"]) != task_source["source_sha256"]:
        raise ValueError("Original task YAML changed")
    task_spec = yaml.safe_load((ROOT / task_source["source_yaml"]).read_text())
    source_paths = [task_source["source_yaml"], layout["kitchen_usd"], ROBOT, layout["obstacle"]["usd_path"],
                    *(item["usd_path"] for item in layout["objects"].values())]
    for source in source_paths:
        if not (ROOT / source).is_file():
            raise FileNotFoundError(ROOT / source)
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(scene_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.GetRootLayer().customLayerData = {
        "scene_scope": SCOPE, "task_id": layout["task_id"],
        "variant_id": layout["obstacle"]["id"], "layout_sha256": sha(layout_path),
        "source_layout_json": json.dumps(layout, ensure_ascii=False),
        "original_task_yaml_json": json.dumps(task_spec, ensure_ascii=False),
        "robot_pose_scope": "Original source USD authored joint transforms. Task controller home pose is metadata only; no physics initialization was performed.",
        "controller_initial_arm_q_rad": json.dumps([math.radians(v) for v in (0, -36, 0, -144, 0, 108, 0)]),
    }
    physics = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0., 0., -1.))
    physics.CreateGravityMagnitudeAttr(9.81)

    def reference(path, asset, position=(0., 0., 0.), yaw=0.):
        wrapper = UsdGeom.Xform.Define(stage, path)
        wrapper.AddTranslateOp().Set(Gf.Vec3d(*position))
        wrapper.AddRotateZOp().Set(float(yaw))
        child = stage.DefinePrim(path + "/Asset")
        # Relative references make the exported scene portable with the repository.
        asset_path = os.path.relpath(ROOT / asset, scene_path.parent)
        child.GetReferences().AddReference(asset_path)
        return wrapper

    reference("/World/kitchen", layout["kitchen_usd"])
    robot_position = list(layout["robot_position"])
    robot_position[2] += layout["stand_height"]
    reference("/World/robot", ROBOT, robot_position, layout["robot_yaw_deg"])
    UsdGeom.Xform.Define(stage, "/World/objects")
    for name, item in layout["objects"].items():
        reference("/World/objects/" + name, item["usd_path"], item["position"], item.get("yaw_deg", 0.))
    obstacle = layout["obstacle"]
    reference("/World/hazard", obstacle["usd_path"], obstacle["position"], obstacle.get("yaw_deg", 0.))
    sky = UsdLux.DomeLight.Define(stage, "/World/sky")
    sky.CreateIntensityAttr(1200.)
    sun = UsdLux.DistantLight.Define(stage, "/World/sun")
    sun.CreateIntensityAttr(2200.)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25., -40., 20.))
    positions = [robot_position, obstacle["position"], *(item["position"] for item in layout["objects"].values())]
    center = [sum(p[i] for p in positions) / len(positions) for i in range(2)]
    target = Gf.Vec3d(center[0], center[1], 1.05 if obstacle["category"] == "human" else 1.0)
    yaw = math.radians(layout["robot_yaw_deg"])
    front, side = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
    eye = Gf.Vec3d(robot_position[0] - front[0] * .45 + side[0] * .35,
                  robot_position[1] - front[1] * .45 + side[1] * .35, 2.50)
    camera = UsdGeom.Camera.Define(stage, "/World/overview_camera")
    camera.CreateFocalLengthAttr(22.)
    camera.CreateHorizontalApertureAttr(36.)
    camera.CreateVerticalApertureAttr(24.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 1000.))
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0., 0., 1.)).GetInverse())
    result = {
        "source_paths_sha256": {path: sha(ROOT / path) for path in source_paths},
        "layout_sha256": sha(layout_path), "camera_eye": list(eye), "camera_target": list(target),
        "inventory": inventory(stage), "robot_initial_pose_scope": "source USD authored joint transforms; controller home pose not applied without native initialization",
    }
    stage.GetRootLayer().Save()
    result["scene_sha256"] = sha(scene_path)
    return result
