"""Object-specific raw finger contacts for assigned Replicator tasks.

This reads PhysX observations only. It never writes poses, forces or joints,
and never creates a simulation attachment. Region checks are separate from
the original task success predicate and from all-body collision audits.
"""
from __future__ import annotations

import re
import numpy as np
from scipy.spatial.transform import Rotation


def _array(value):
    value = value.torch if hasattr(value, "torch") else value
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().copy()
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value).copy()


class ObjectGraspContacts:
    fingers = ("left_inner_finger", "right_inner_finger")

    def __init__(self, physics_sim_view, physics_dt, object_name, *, env_root="/World/envs/env_0", env_origin=None):
        if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", object_name):
            raise ValueError("Object contact name must be one exact scene-object identifier")
        if not np.isfinite(physics_dt) or physics_dt <= 0:
            raise ValueError("physics_dt must be positive")
        self.physics_dt, self.object_name = float(physics_dt), object_name
        self.env_origin = np.zeros(3) if env_origin is None else np.asarray(env_origin, dtype=float)
        if self.env_origin.shape != (3,) or not np.isfinite(self.env_origin).all():
            raise ValueError("Environment origin must be finite xyz")
        self.views = {finger: physics_sim_view.create_rigid_contact_view(
            [f"{env_root}/Robot/Gripper/Robotiq_2F_85/{finger}"],
            filter_patterns=[[f"{env_root}/{object_name}"]], max_contact_data_count=4096)
            for finger in self.fingers}
        self.records = []

    def sample(self, phase, action_index, object_pose_xyz_xyzw, *, physics_step):
        """Read one substep; the supplied object pose is the observed world pose."""
        pose = np.asarray(object_pose_xyz_xyzw, dtype=float)
        if pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-8:
            raise ValueError("Object pose must contain finite xyz and nonzero xyzw quaternion")
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        row = {"object_name": self.object_name, "phase": phase, "action_index": int(action_index),
               "physics_step": int(physics_step), "physics_dt_s": self.physics_dt, "fingers": {}}
        for finger, view in self.views.items():
            force, point, normal, separation, count, start = map(_array, view.get_contact_data(dt=self.physics_dt))
            n, first = int(count.reshape(-1)[0]), int(start.reshape(-1)[0])
            if first < 0 or n < 0 or first + n > len(point):
                raise RuntimeError("Finger/object contact buffer is truncated or invalid")
            if first + n >= 4096:
                raise RuntimeError("Finger contact buffer reached its declared capacity; complete raw contacts are not proven")
            selection = slice(first, first + n)
            points = point[selection].reshape(-1, 3)
            normals = normal[selection].reshape(-1, 3)
            forces, separations = force[selection].reshape(-1), separation[selection].reshape(-1)
            if not all(np.isfinite(value).all() for value in (points, normals, forces, separations)):
                raise RuntimeError("Non-finite raw finger contact data")
            row["fingers"][finger] = {
                "count": n, "points_world_m": points.tolist(),
                "points_environment_m": (points - self.env_origin).tolist(),
                "points_object_local_m": ((points - pose[:3]) @ rotation).tolist(),
                "normals_world": normals.tolist(), "normals_object_local": (normals @ rotation).tolist(),
                "normal_forces_n": forces.tolist(), "separations_m": separations.tolist(),
            }
        self.records.append(row)
        return row

    def summarize(self, region_min, region_max, *, phase, last_steps=10, minimum_force_n=.01, max_separation_m=.00025):
        """Require simultaneous bilateral contacts inside a declared object-local region.

        The region must be measured from the particular source object. Neither
        alternating contacts nor a positive force outside the region can pass.
        ``last_steps`` counts recorded physics updates, not control actions.
        """
        low, high = np.asarray(region_min, float), np.asarray(region_max, float)
        if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(high <= low):
            raise ValueError("A contact region must have finite, ordered 3D bounds")
        if last_steps < 1 or minimum_force_n <= 0 or max_separation_m < 0:
            raise ValueError("Invalid contact-window thresholds")
        rows = [row for row in self.records if row["phase"] == phase][-last_steps:]
        details = []
        for row in rows:
            fingers = {}
            for finger in self.fingers:
                contact = row["fingers"][finger]
                points = np.asarray(contact["points_object_local_m"]).reshape(-1, 3)
                force = np.asarray(contact["normal_forces_n"])
                gap = np.asarray(contact["separations_m"])
                if len(points) != contact["count"] or len(force) != len(points) or len(gap) != len(points):
                    raise RuntimeError("Raw contact array lengths differ")
                inside = np.all((points >= low) & (points <= high), axis=1)
                loaded = force > minimum_force_n
                near = gap <= max_separation_m
                fingers[finger] = {"loaded_in_region": bool(loaded.any() and inside[loaded].all()),
                                   "near_in_region": bool(near.any() and inside[near].all()),
                                   "loaded_contact_count": int(loaded.sum())}
            details.append({"physics_step": row["physics_step"], "fingers": fingers,
                            "bilateral_loaded": all(item["loaded_in_region"] for item in fingers.values()),
                            "bilateral_near": all(item["near_in_region"] for item in fingers.values())})
        enough = len(rows) == last_steps
        return {"object_name": self.object_name, "phase": phase,
                "scope": "Recorded physics-step finger contact window only, not task or global-safety success",
                "region_object_local_m": [low.tolist(), high.tolist()], "requested_steps": last_steps,
                "sampled_steps": len(rows), "minimum_force_n": minimum_force_n,
                "max_separation_m": max_separation_m, "steps": details,
                "both_fingers_loaded": enough and all(row["bilateral_loaded"] for row in details),
                "both_fingers_near": enough and all(row["bilateral_near"] for row in details)}
