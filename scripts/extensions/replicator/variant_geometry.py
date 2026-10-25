"""Source-geometry helpers for offline Replicator safety evidence only.

No Kit, CUDA, simulation state setters or planner changes are imported here.
The authored robot/finger triangle surfaces and their existing convex proxies
are separate checks; neither is represented as the PhysX cooked collider.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
HAZARDS = {"human": "person", "property": "glass", "robot": "barrier"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def code_evidence(paths):
    """Archive exact audit sources so later repository edits do not erase provenance."""
    snapshots = {str(Path(path).resolve().relative_to(ROOT)): Path(path).read_text()
                 for path in paths}
    return dict(code_sha256={path: hashlib.sha256(source.encode()).hexdigest()
                             for path, source in snapshots.items()},
                code_snapshots=snapshots)


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def variant_settings(layout, variant):
    if variant not in HAZARDS or layout["obstacle"]["category"] != variant:
        raise ValueError("Requested variant and saved obstacle category differ")
    settings = layout["planning"]
    key = HAZARDS[variant]
    required = float(settings["execution_clearances_m"][key])
    planning = float(settings["clearances_m"][key])
    padding = float(settings.get("transport_object_padding_m_by_variant", {}).get(variant, 0.))
    if not np.isfinite([required, planning, padding]).all():
        raise ValueError("Distances and transport padding must be finite")
    # The original task margins are floors, not a CLI tuning parameter.
    if required < {"human": .02, "property": .05, "robot": .01}[variant]:
        raise ValueError("Execution clearance is below the original task requirement")
    if planning < {"human": .04, "property": .06, "robot": .02}[variant]:
        raise ValueError("Planning clearance is below the original task requirement")
    if padding < (.015 if variant == "human" else 0.):
        raise ValueError("Transport padding is below the original variant requirement")
    return key, required, planning, padding


def assert_source_layout(layout, result):
    if layout.get("task_id") != "kitchen_bench_replicator_g_shape_mustard_bowl":
        raise ValueError("This source-bottle audit currently supports G-shape mustard-to-bowl only")
    if {k: v for k, v in layout.items() if k != "obstacle"} != {
            k: v for k, v in result["layout"].items() if k != "obstacle"}:
        raise ValueError("Source layout/controls differ; only the hazard may change")


def hazard_pose_evidence(layout, variant, episode=None):
    """Use measured stationary pose for dynamic props, never silently freeze one."""
    from scripts.extensions.replicator.audit import hazard_stage
    from scripts.extensions.replicator.audit_avoidance import hazard_stationarity

    obstacle = layout["obstacle"]
    authored = np.r_[obstacle["position"], Rotation.from_euler(
        "z", obstacle.get("yaw_deg", 0), degrees=True).as_quat()]
    if episode is None:
        if obstacle.get("static") is not True:
            raise ValueError("Dynamic property requires --hazard-episode with measured stable poses")
        stage, _ = hazard_stage(layout, variant, obstacle)
        from pxr import Usd, UsdPhysics
        enabled = [str(p.GetPath()) for p in Usd.PrimRange(
            stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get()]
        animated = [str(a.GetPath()) for p in stage.Traverse()
                    for a in p.GetAttributes()
                    if a.GetName().startswith("xformOp:") and a.GetNumTimeSamples()]
        if enabled or animated:
            raise ValueError("Declared static hazard has rigid dynamics or animated transforms")
        return authored, dict(method="authored static root; verified no enabled dynamics/animation",
                             passed=True, pose_xyz_xyzw=authored.tolist())
    episode = Path(episode).resolve()
    result = json.loads((episode / "result.json").read_text())
    assert_source_layout(layout, result)
    if result.get("variant") != variant or result.get("obstacle") != obstacle:
        raise ValueError("Hazard episode does not match this exact variant/layout")
    if not result.get("success") or not result.get("arena_success"):
        raise ValueError("Hazard pose reference must be a successful physical episode")
    with np.load(episode / "physics_states.npz") as data:
        has = data["has_hazard_pose"]
        if obstacle.get("static") is not True and not has.all():
            raise ValueError("Dynamic hazard has incomplete measured physics poses")
        if has.all():
            indices = np.flatnonzero(data["phase"] == "above_bowl")
            if not len(indices):
                raise ValueError("Hazard reference has no measured above_bowl phase")
            index = int(indices[0])
            pose = data["hazard_pose"][index].copy()
        else:
            index, pose = None, authored
    stage, _ = hazard_stage(layout, variant, obstacle)
    stability = hazard_stationarity(episode, result, stage, pose)
    if not stability["passed"]:
        raise ValueError("Hazard is not stationary over the recorded full physical episode")
    return pose, dict(method="fixed measured transfer-start pose, full-episode stationarity gate",
                      episode=str(episode), result_sha256=sha(episode / "result.json"),
                      physics_sha256=sha(episode / "physics_states.npz"),
                      physics_index=index, pose_xyz_xyzw=pose.tolist(), passed=True,
                      stationarity=stability)


def make_hazard(layout, variant, pose):
    from pxr import Gf, UsdGeom
    from scripts.extensions.replicator.audit import hazard_stage
    stage, _ = hazard_stage(layout, variant, layout["obstacle"])
    root = "/World/" + HAZARDS[variant].title()
    transform = UsdGeom.Xform(stage.GetPrimAtPath(root))
    transform.ClearXformOpOrder()
    transform.AddTranslateOp().Set(Gf.Vec3d(*map(float, pose[:3])))
    transform.AddOrientOp().Set(Gf.Quatf(float(pose[6]), Gf.Vec3f(*map(float, pose[3:6]))))
    return stage, root


class MeasuredHazardShapes:
    """Rigidly move source shapes from one immutable reference, retaining scale."""
    def __init__(self, stage, root, pose):
        from kitchen_safety.geometry_audit import static_collision_shapes
        self.shapes = static_collision_shapes(stage, root)
        self.reference = [(np.array(obj.getRotation()), np.array(obj.getTranslation()),
                           np.array(lo), np.array(hi)) for _, obj, lo, hi in self.shapes]
        self.initial_rotation = Rotation.from_quat(pose[3:7]).as_matrix()
        self.initial_position = np.asarray(pose[:3]).copy()

    def at(self, pose):
        import fcl
        rotation = Rotation.from_quat(pose[3:7]).as_matrix() @ self.initial_rotation.T
        translation = np.asarray(pose[:3]) - rotation @ self.initial_position
        rows = []
        for (name, obj, _, _), (r, p, lo, hi) in zip(self.shapes, self.reference):
            obj.setTransform(fcl.Transform(rotation @ r, rotation @ p + translation))
            center = rotation @ ((lo + hi) / 2) + translation
            radius = np.abs(rotation) @ ((hi - lo) / 2)
            rows.append((name, obj, center - radius, center + radius))
        return rows


class SourceRobotGeometry:
    """Authored collision mesh surfaces, including collision-Xform descendants.

    The source gripper enables collision on Xform groups containing its meshes.
    Keep these raw mesh surfaces separate from the planner's all-mesh hulls.
    Transform each collider from its authored rigid-body frame using measured q.
    """
    def __init__(self, world):
        import fcl
        from pxr import Usd, UsdGeom, UsdPhysics
        from kitchen_safety.planning import ROBOT_USD
        from kitchen_safety.geometry_audit import static_collision_shapes
        self.world = world
        self.stage = Usd.Stage.Open(str(ROBOT_USD))
        cache = UsdGeom.XformCache()
        self.parts, descriptions = [], []
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath("/panda"), Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            body = prim
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                body = body.GetParent()
            if not body:
                continue
            collision = prim
            while collision and collision != body and not collision.HasAPI(UsdPhysics.CollisionAPI):
                collision = collision.GetParent()
            if not collision or not collision.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if not UsdPhysics.CollisionAPI(collision).GetCollisionEnabledAttr().Get():
                continue
            body_matrix = np.asarray(cache.GetLocalToWorldTransform(body)).T
            if (not np.allclose(body_matrix[:3, :3].T @ body_matrix[:3, :3], np.eye(3), atol=1e-7)
                    or np.linalg.det(body_matrix[:3, :3]) < 0):
                raise ValueError("Scaled/sheared/reflected source rigid-body frame is unsupported")
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
                vertices = np.asarray(mesh.GetPointsAttr().Get()) @ matrix[:3, :3].T + matrix[:3, 3]
                indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
                faces, offset = [], 0
                for count in mesh.GetFaceVertexCountsAttr().Get():
                    polygon = indices[offset:offset + count]
                    offset += count
                    faces.extend([[polygon[0], polygon[i], polygon[i + 1]] for i in range(1, count - 1)])
                model = fcl.BVHModel()
                model.beginModel(len(vertices), len(faces))
                model.addSubModel(vertices, np.asarray(faces, dtype=np.int32))
                model.endModel()
                obj, lo, hi = fcl.CollisionObject(model), vertices.min(0), vertices.max(0)
                mesh_info = dict(vertices=len(vertices), triangles=len(faces))
            else:
                if collision != prim:
                    raise ValueError("Inherited primitive collision is not explicitly supported")
                _, obj, lo, hi = static_collision_shapes(self.stage, str(prim.GetPath()))[0]
                mesh_info = {}
            self.parts.append((str(prim.GetPath()), str(body.GetPath()), obj,
                               np.linalg.inv(body_matrix), np.array(obj.getRotation()),
                               np.array(obj.getTranslation()), np.asarray(lo), np.asarray(hi)))
            descriptions.append(dict(path=str(prim.GetPath()), body=str(body.GetPath()),
                                     collision_api_owner=str(collision.GetPath()), schema=str(prim.GetTypeName()),
                                     authored_body_to_world=body_matrix.tolist(),
                                     source_approximation=str(UsdPhysics.MeshCollisionAPI(collision).GetApproximationAttr().Get()),
                                     **mesh_info))
        if not self.parts:
            raise ValueError("No source robot collision geometry")
        self.provenance = dict(source=str(ROBOT_USD), sha256=sha(ROBOT_USD), parts=descriptions,
                               mode="authored_collision_mesh_surfaces_and_supported_primitives",
                               limitations="Collision-enabled Xform descendants are included explicitly; authored mesh triangles "
                               "are not PhysX cooked convex hulls/decomposition. No margin/contact-offset reconstruction.")

    def shapes(self, q, gripper):
        import fcl
        links = self.world.kin.links(q)
        base = links[self.world.gripper_root]
        links.update({name: base @ transform for name, transform in self.world._gripper_links(gripper).items()})
        shapes = []
        for name, body, obj, inverse, rotation, position, lo, hi in self.parts:
            delta = links[body] @ inverse
            r, t = delta[:3, :3], delta[:3, 3]
            obj.setTransform(fcl.Transform(r @ rotation, r @ position + t))
            center, radius = r @ ((lo + hi) / 2) + t, np.abs(r) @ ((hi - lo) / 2)
            shapes.append(("source_robot" + name, obj, center - radius, center + radius))
        return shapes


def source_provenance(stage, root, mesh, hull, robot):
    from pxr import Usd, UsdGeom, UsdPhysics
    from kitchen_safety.planning import ROBOT_USD
    cache = UsdGeom.XformCache()
    colliders = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        colliders.append(dict(path=str(prim.GetPath()), schema=str(prim.GetTypeName()),
                              local_to_world=np.asarray(cache.GetLocalToWorldTransform(prim)).T.tolist(),
                              source_approximation=str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())
                              if prim.IsA(UsdGeom.Mesh) else None))
    return dict(bottle_mesh=mesh.provenance, bottle_hull=hull.provenance,
                robot_source=str(ROBOT_USD), robot_sha256=sha(ROBOT_USD),
                robot_source_mesh=robot.provenance,
                robot_hull_model="source-derived per-link convex hull triangle surfaces; articulated finger solid hulls",
                source_mesh_mode="authored robot/finger collision triangle surfaces + authored bottle collision triangles",
                source_hull_mode="existing robot/finger hull proxies + source-bottle vertex convex hull surface",
                hazard_colliders=colliders,
                hazard_layers=[dict(path=layer.realPath, sha256=sha(layer.realPath))
                               for layer in stage.GetUsedLayers() if layer.realPath],
                limitations="Authored source triangle surfaces, source-derived robot/finger hulls, and supported round primitives. "
                "Bottle convex hull is a separate approximation. Neither mode reproduces PhysX cooked convex decomposition/contact offsets. "
                "A margin shortfall is not proof of physical contact or breakage.")


def measure_sample(world, mesh, hull, q, gripper, bottle_pose, hazard_shapes, required, hazard, robot):
    from kitchen_safety.extensions.replicator.source_geometry import source_clearance
    from kitchen_safety.geometry_audit import minimum_distance
    return dict(source_mesh=minimum_distance(robot.shapes(q, gripper) + mesh.shapes(bottle_pose), hazard_shapes, required),
                source_hull=source_clearance(world, hull, q, gripper, bottle_pose, hazard_shapes, required),
                conservative_execution=world.clearance(q, mode="execution", gripper_q=gripper,
                                                      object_pose=bottle_pose, obstacle_filter=(hazard,)))


def summarize_samples(rows):
    if not rows:
        raise ValueError("No geometric samples to audit")
    summaries = {}
    for mode in ("source_mesh", "source_hull", "conservative_execution"):
        if any(not np.isfinite(r[mode]["distance_m"]) for r in rows):
            raise ValueError("Nonfinite source geometry measurement")
        minimum = min(rows, key=lambda r: r[mode]["distance_m"])
        summaries[mode] = dict(minimum={k: v for k, v in minimum.items()
                                      if k not in ("source_mesh", "source_hull", "conservative_execution")}
                              | minimum[mode],
                              violations=sum(not r[mode]["valid"] for r in rows),
                              intersections=sum(r[mode]["intersecting"] for r in rows),
                              passed=all(r[mode]["valid"] for r in rows))
    return summaries


def expectation_passed(summary, expectation):
    if expectation == "safe":
        return all(summary[m]["passed"] for m in ("source_mesh", "source_hull", "conservative_execution"))
    if expectation == "risk":
        return all(summary[m]["violations"] > 0 for m in ("source_mesh", "source_hull"))
    raise ValueError("Expectation must be safe or risk")
