"""Original-camera audit using the actual source USD orient-op semantics.

Nonunit source quaternions must pass through USD's authored transform, not a
four-component Euclidean normalization. Preserve the old result as diagnostic;
never change the original 0.02 degree / 0.1 mm tolerance or recorded images.
"""
from copy import deepcopy
import json
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.task_audit_sidecars import ROOT, CAMERA_SOURCE, cameras_audit, sha, source_camera_config


def usd_authored_quaternion(quaternion):
    from pxr import Usd, UsdGeom, Gf
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all() or np.linalg.norm(values) < 1e-8:
        raise ValueError("Invalid original camera quaternion")
    stage = Usd.Stage.CreateInMemory()
    frame = UsdGeom.Xform.Define(stage, "/camera")
    # Exact source create_prim/set_transform_attributes authoring representation.
    frame.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(values[3], *values[:3]))
    q = frame.GetLocalTransformation().ExtractRotationQuat()
    return np.r_[q.GetImaginary(), q.GetReal()]


def authored_camera_audit(episode, metadata, data, trajectory):
    from kitchen_safety.dataset import camera_pose_errors
    episode = Path(episode)
    old = cameras_audit(episode, metadata, data, trajectory)
    report = deepcopy(old)
    report["normalized_quaternion_diagnostic"] = old
    config = json.loads((episode/"cameras.json").read_text())
    source = source_camera_config()
    ids = np.searchsorted(data["physics_step"], trajectory["before_physics_step"])
    with h5py.File(episode/"images.h5", "r") as images:
        for name, camera in config.items():
            offset = camera["offset"]
            if offset["convention"] != "opengl":
                raise ValueError("This original DROID authoring proof supports its literal OpenGL camera config only")
            parent_name = camera["prim_path"].rsplit("/", 2)[-2]
            parent = data["robot_body_pose_w"][ids, metadata["robot_body_names"].index(parent_name)]
            rotation = Rotation.from_quat(parent[:, 3:])
            authored = usd_authored_quaternion(offset["quaternion_xyzw"])
            local = Rotation.from_quat(authored)*Rotation.from_quat([1., 0., 0., 0.])
            expected = dict(position_world=parent[:, :3]+rotation.apply(offset["position"]),
                quaternion_world_ros_xyzw=(rotation*local).as_quat())
            group = images[name]
            error = camera_pose_errors(group["position_world"][:], group["quaternion_world_ros_xyzw"][:], expected)
            original = old["cameras"][name]
            checks = dict(original_mount_preserved=original["original_mount_preserved"],
                original_dimensions=camera["width"] == source[name]["width"] and camera["height"] == source[name]["height"],
                original_intrinsics=original["intrinsic_max_error_pixels"] < .001,
                actual_position=error["position_max_error_m"] < .0001,
                actual_rotation=error["rotation_max_error_deg"] < .02)
            report["cameras"][name].update(error, passed=all(checks.values()), checks=checks,
                original_quaternion_xyzw=offset["quaternion_xyzw"], original_quaternion_norm=float(np.linalg.norm(offset["quaternion_xyzw"])),
                usd_authored_quaternion_xyzw=authored.tolist(), normalized_vs_authored_angle_deg=float(np.rad2deg(
                    (Rotation.from_quat(offset["quaternion_xyzw"]).inv()*Rotation.from_quat(authored)).magnitude())))
    report["passed"] = report["exact_physics_action_alignment"] and all(row["passed"] for row in report["cameras"].values())
    paths = [Path(__file__), CAMERA_SOURCE, ROOT/"kitchen_safety/dataset.py",
        ROOT/"scripts/extensions/replicator/task_audit_sidecars.py",
        ROOT/"third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab/sim/utils/transforms.py"]
    report["source_sha256"] = {str(path.relative_to(ROOT)): sha(path) for path in paths}
    report["thresholds"] = dict(position_m=.0001, rotation_deg=.02, intrinsics_pixels=.001)
    report["scope"] = "Every original camera pose reconstructed from actual parent physics pose and original literal source USD orient op + OpenGL-to-ROS conversion. Old normalization-only result retained for comparison. No tolerance or raw data changes; no visual semantic certification."
    return report
