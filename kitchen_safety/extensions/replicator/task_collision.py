"""Source-derived payload geometry for Replicator banana and bowl tasks.

All source collision vertices are retained. A solid convex envelope supplies
conservative FCL containment/distance checks; the original triangle arrays are
available for separate source-geometry audits. Neither representation claims
to be PhysX's cooked convex decomposition. No bottle dimensions are used.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld


class PayloadGeometry:
    def __init__(self, usd_path):
        from pxr import Usd, UsdGeom, UsdPhysics

        path = Path(usd_path).resolve()
        stage = Usd.Stage.Open(str(path))
        if stage is None or not stage.GetDefaultPrim():
            raise ValueError(f"No source payload/default prim: {path}")
        if UsdGeom.GetStageMetersPerUnit(stage) != 1. or str(UsdGeom.GetStageUpAxis(stage)) != "Z":
            raise ValueError("Payload adapter requires the assigned metre/Z-up source assets")
        cache = UsdGeom.XformCache()
        root = stage.GetDefaultPrim()
        inverse = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(root)).T)
        vertices, triangles, collider_records = [], [], []
        cursor = 0
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                continue
            if not prim.IsA(UsdGeom.Mesh):
                raise ValueError(f"Unsupported source payload collider; do not drop it: {prim.GetPath()}")
            mesh = UsdGeom.Mesh(prim)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            transform = inverse @ np.asarray(cache.GetLocalToWorldTransform(prim)).T
            points = points @ transform[:3, :3].T + transform[:3, 3]
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
            start = 0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                for i in range(1, count - 1):
                    triangles.append(indices[[start, start + i, start + i + 1]] + cursor)
                start += count
            vertices.append(points)
            cursor += len(points)
            collider_records.append({"path": str(prim.GetPath()), "vertices": len(points),
                "mesh_approximation": UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()})
        if not vertices or not triangles:
            raise ValueError("No usable enabled collision mesh in the assigned payload")
        self.vertices = np.concatenate(vertices)
        self.triangles = np.asarray(triangles, dtype=np.int32)
        if not np.isfinite(self.vertices).all():
            raise ValueError("Non-finite source collision vertices")
        self.hull = ConvexHull(self.vertices)
        self.low, self.high = self.vertices.min(0), self.vertices.max(0)
        self.provenance = {"source_asset": str(path), "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "root_prim": str(root.GetPath()), "frame": "Source rigid-body/default-prim local coordinates in metres",
            "colliders": collider_records, "bounds_m": [self.low.tolist(), self.high.tolist()],
            "fcl_representation": "Solid convex envelope of all enabled source collision vertices",
            "independent_source_audit": "Original triangles retained; convex envelope fills concavities conservatively",
            "physx_cooked_shapes_verified": False}

    def sphere_cover(self, maximum_spheres=100):
        """Cover the complete source convex envelope with bounded voxel spheres.

        Every retained voxel is enclosed by its sphere. Voxels are excluded
        only when a hull separating plane proves the entire voxel is outside.
        This covers the volume, not just sampled surface points; overcoverage
        is conservative and must not be mistaken for true avoidance necessity.
        """
        if maximum_spheres < 1:
            raise ValueError("maximum_spheres must be positive")
        span = self.high - self.low
        resolution = max(float(span.max()) / 24, .002)
        planes = self.hull.equations
        for _ in range(150):
            counts = np.maximum(1, np.ceil(span / resolution).astype(int))
            size = span / counts
            centers = np.stack(np.meshgrid(*[self.low[i] + (np.arange(counts[i]) + .5) * size[i]
                                            for i in range(3)], indexing="ij"), axis=-1).reshape(-1, 3)
            # min(n*x+d) over an axis-aligned voxel: positive => entirely outside.
            minimum = centers @ planes[:, :3].T + planes[:, 3] - np.abs(planes[:, :3]) @ (size / 2)
            centers = centers[~np.any(minimum > 1e-10, axis=1)]
            if len(centers) <= maximum_spheres:
                radii = np.full(len(centers), np.linalg.norm(size / 2) + 1e-9)
                return centers, radii, {"voxel_size_m": size.tolist(), "count": len(centers),
                    "coverage": "Complete source convex hull; enclosing voxel spheres, no surface-sampling gaps"}
            resolution *= 1.08
        raise RuntimeError("Unable to construct a bounded conservative payload sphere cover")


class PayloadCollisionWorld(CollisionWorld):
    """Full kitchen/robot checker with an asset-derived carried-object envelope.

    Necessary grasp/support contacts still require an explicit phase/contact
    policy in the collector. This class introduces no blanket support exemption
    or destination mask. The source task has no bottle-specific tilt criterion.
    """
    def __init__(self, stage, kin, *, payload_name, payload_usd, **kwargs):
        self.payload_name = payload_name
        self.payload_geometry = PayloadGeometry(payload_usd)
        super().__init__(stage, kin, **kwargs)
        self.payload_model, self.payload_hull_vertices = self._convex(self.payload_geometry.vertices)
        from .task_contact_policy import BananaContactPolicy
        if payload_name != "banana":
            raise NotImplementedError("Contact policy is implemented for banana only; bowl needs a separate rim policy")
        self.contact_policy = BananaContactPolicy()
        parts = {name.rsplit("/", 1)[-1] for name, _, _ in self.gripper_models}
        if not {"base_link", "left_inner_finger", "right_inner_finger"} <= parts or len(parts) <= 3:
            raise ValueError("Selective envelope refinement requires source palm, both inner fingers and all articulated part hulls")
        self.phase = "reset"
        self.payload_pose = None
        self.gripper_joint_positions = None
        model = self.fcl.BVHModel()
        model.beginModel(len(self.payload_geometry.vertices), len(self.payload_geometry.triangles))
        model.addSubModel(self.payload_geometry.vertices, self.payload_geometry.triangles)
        model.endModel()
        self.payload_triangle_model = model

    def set_phase(self, phase):
        self.contact_policy.validate_phase(phase)
        self.phase = phase

    def shapes(self, q, gripper_q=None, object_pose=None, *, _cache_robot=False):
        if gripper_q is None:
            gripper_q = self.gripper_joint_positions
        if gripper_q is None:
            raise ValueError("Selective contact checks require all six measured gripper joint angles")
        result = super().shapes(q, gripper_q, object_pose, _cache_robot=_cache_robot)
        # The inherited hull names collapse all links. Restore stable identities
        # so only the aggregate gripper envelope can be refined by part hulls.
        cursor, renamed = 0, []
        for label, obj, lo, hi in result:
            if label == "robot_hull":
                body = self.robot_hulls[cursor][0]
                cursor += 1
                label = "robot_hull/whole_gripper" if body == self.gripper_root else "robot_hull/" + body.rsplit("/", 1)[-1]
            renamed.append((label, obj, lo, hi))
        return renamed

    def _append_bottle_shape(self, shapes, q, object_pose):
        # Inherited robot-shape cache calls this hook; replace its old bottle
        # proxy completely, including for a released object's measured pose.
        pose = self._object_pose(q, object_pose)
        if pose is not None:
            vertices = self.payload_hull_vertices @ pose[:3, :3].T + pose[:3, 3]
            shapes.append(("payload/" + self.payload_name,
                self.fcl.CollisionObject(self.payload_model, self.fcl.Transform(pose[:3, :3], pose[:3, 3])),
                vertices.min(0), vertices.max(0)))

    def _pairs(self, q, ignore, mode, gripper_q, object_pose, obstacle_filter, max_slack=float("inf")):
        if ignore:
            raise ValueError(
                f"Whole-object masks {tuple(ignore)} are forbidden in the independent FCL checker")
        pose = self._object_pose(q, object_pose)
        if pose is None:
            pose = self.payload_pose
        if pose is None:
            raise ValueError("Every query requires the observed free payload or measured planning attachment")
        from scipy.spatial.transform import Rotation
        pose = self._object_pose(q, pose)
        self.set_obstacle_pose(self.payload_name, np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_quat()])
        self._query_payload_pose = pose
        points = self.payload_geometry.vertices @ pose[:3, :3].T + pose[:3, 3]
        lo, hi = points.min(0), points.max(0)
        pairs = super()._pairs(q, (), mode, gripper_q, pose, obstacle_filter, max_slack)
        result = []
        for pair in pairs:
            _, label, name, *_ = pair
            own_object = self._matches(name, self.payload_name)
            if label == "payload/" + self.payload_name and own_object:
                continue  # A body is not tested against its own duplicate world geometry.
            if own_object and (self.contact_policy.finger_pair(label, self.phase) or
                               self.contact_policy.redundant_finger_envelope(label, self.phase)):
                continue
            if label == "payload/" + self.payload_name and name == self.contact_policy.source_support:
                support = next(row for row in self.objects if row[0] == name)
                if (lo[2] >= support[3][2] - self.contact_policy.support_band_m and
                        np.all(lo[:2] >= support[2][:2]) and np.all(hi[:2] <= support[3][:2])):
                    continue
            result.append(pair)
        return result

    def _measurement(self, pair):
        from .task_contact_policy import DESTINATION_PHASES
        _, label, name, shape, obstacle, margin = pair
        if (label == "payload/" + self.payload_name and self._matches(name, self.contact_policy.destination_name)
                and self.phase in DESTINATION_PHASES):
            pose = self._query_payload_pose
            triangle_shape = self.fcl.CollisionObject(self.payload_triangle_model,
                self.fcl.Transform(pose[:3, :3], pose[:3, 3]))
            refined = (pair[0], label, name, triangle_shape, obstacle, 0.)
            result = self._exact_measurement(refined)
            result.update(contact_pair="payload/destination", representation="Original source triangles; filled-hull contact refined",
                          noncontact_required_m=margin)
            return result
        return self._exact_measurement(pair)

    def _exact_measurement(self, pair):
        # This FCL build can return a tiny positive distance (e.g. 6e-323)
        # even when collide() is true for a sphere/mesh pair. Explicit overlap
        # must be checked also for intended zero-margin destination contact.
        _, label, name, shape, obstacle, margin = pair
        raw = float(self.fcl.distance(shape, obstacle, self.fcl.DistanceRequest(), self.fcl.DistanceResult()))
        intersecting = bool(self.fcl.collide(shape, obstacle, self.fcl.CollisionRequest(), self.fcl.CollisionResult()))
        if not np.isfinite(raw):
            raise RuntimeError("Nonfinite FCL distance; cannot certify a task pair")
        intersecting = intersecting or raw < 0
        distance = max(0., raw)
        return dict(valid=bool(not intersecting and distance >= margin), distance_m=distance,
            required_m=margin, slack_m=distance-margin, robot_part=label, obstacle=name, intersecting=intersecting)

    def valid(self, q, ignore=(), *, mode="planning", gripper_q=None, object_pose=None, obstacle_filter=None):
        self.last_collision = self.last_clearance = None
        # Preserve all world/robot checks. The prior mustard-specific vertical
        # 28-degree gate is inapplicable to a naturally horizontal banana;
        # any new task posture constraint must be explicit in its controller.
        for pair in self._pairs(q, ignore, mode, gripper_q, object_pose, obstacle_filter, max_slack=0.):
            if pair[0] > 0:
                break
            measurement = self._measurement(pair)
            if not measurement["valid"]:
                self.last_clearance = measurement
                self.last_collision = (measurement["robot_part"], measurement["obstacle"])
                return False
        return True
