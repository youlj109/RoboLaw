"""Unexecuted half-centroid grasp candidate, computed from current source geometry.

This leaf does not alter the current controller, physics or contact permissions.
The uniform-volume centroid is a geometric proxy, not an authored/native COM.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
CANDIDATE = ROOT / "configs/extensions/replicator/g_shape_banana_bowl_halfcenter_candidate.json"
BASE_PROFILE = ROOT / "configs/extensions/replicator/g_shape_banana_bowl_controller.json"


def read_candidate(path=CANDIDATE, base_profile_path=BASE_PROFILE):
    """Bind this finite candidate to the source and unchanged prepared controller."""
    profile = json.loads(Path(path).read_text())
    base = json.loads(Path(base_profile_path).read_text())
    if profile.get("schema_version") != 1 or profile.get("candidate_id") != "contactcenter1_jaw0":
        raise ValueError("Unreviewed banana grasp candidate")
    if (profile.get("task_id") != "kitchen_bench_replicator_g_shape_banana_bowl"
            or profile["task_id"] != base["task_id"]):
        raise ValueError("Candidate task differs from original task")
    if profile.get("base_profile_sha256") != hashlib.sha256(Path(base_profile_path).read_bytes()).hexdigest():
        raise ValueError("Prepared controller profile changed")
    if profile.get("source_banana_sha256") != base["source_object_sha256"]["banana"]:
        raise ValueError("Candidate source banana differs")
    if (profile.get("geometric_centroid_fraction_xy") != .5
            or profile.get("additional_yaw_rad") != 0.
            or profile.get("additional_world_height_m") != 0.
            or profile.get("baseline_and_variants_share_candidate") is not True):
        raise ValueError("Candidate differs from finite checked XY-only selection")
    return profile


def shift_source_xy(base_target, actual_source_transform, section_center, centroid_proxy):
    """Apply exactly half the source-local XY offset using the *current* pose.

    Preserve the already source-checked open/closed envelope height and jaw
    orientation bit-for-bit. No recorded world goal is consumed by this API.
    """
    target = np.array(base_target, dtype=float, copy=True)
    source = np.asarray(actual_source_transform, dtype=float)
    center = np.asarray(section_center, dtype=float)
    proxy = np.asarray(centroid_proxy, dtype=float)
    if (target.shape != (4, 4) or source.shape != (4, 4)
            or center.shape != (3,) or proxy.shape != (3,)
            or not all(np.isfinite(x).all() for x in (target, source, center, proxy))):
        raise ValueError("Invalid actual transform or source geometry")
    if (not np.array_equal(source[3], [0., 0., 0., 1.])
            or not np.allclose(source[:3, :3].T @ source[:3, :3], np.eye(3), atol=1e-8, rtol=0.)
            or not np.isclose(np.linalg.det(source[:3, :3]), 1., atol=1e-8, rtol=0.)):
        raise ValueError("Actual source transform is not a proper rigid transform")
    selected = center.copy()
    selected[:2] = (center[:2] + proxy[:2]) * .5
    target[:2, 3] += (source[:3, :3] @ (selected - center))[:2]
    return target, selected


def grasp_target(actual_payload_pose, geometry, world, base_profile, candidate):
    """Recompute original target and geometric centroid; update only target XY."""
    import trimesh
    from .banana_controller_profile import grasp_target as base_grasp_target
    from .task_motion import pose_matrix

    if (candidate.get("candidate_id") != "contactcenter1_jaw0"
            or candidate.get("geometric_centroid_fraction_xy") != .5
            or candidate.get("additional_yaw_rad") != 0.
            or candidate.get("additional_world_height_m") != 0.
            or candidate.get("source_banana_sha256") != geometry.provenance["source_sha256"]
            or candidate["source_banana_sha256"] != base_profile["source_object_sha256"]["banana"]):
        raise ValueError("Runtime banana differs from prepared half-center candidate")
    mesh = trimesh.Trimesh(geometry.vertices, geometry.triangles, process=True)
    if (not mesh.is_watertight or not mesh.is_winding_consistent
            or not np.isfinite(mesh.volume) or abs(mesh.volume) <= 1e-12):
        raise ValueError("Geometric centroid requires a closed, consistently wound source mesh")
    proxy = np.asarray(mesh.center_mass, dtype=float)
    if not np.isfinite(proxy).all():
        raise ValueError("Nonfinite source centroid proxy")
    base, info = base_grasp_target(actual_payload_pose, geometry, world, base_profile)
    target, selected = shift_source_xy(base, pose_matrix(actual_payload_pose), info["center_local_m"], proxy)
    # Keep center_local_m and the contact region from the original source rule.
    info.update(candidate_id=candidate["candidate_id"],
        original_section_center_local_m=list(info["center_local_m"]),
        geometric_volume_centroid_proxy_local_m=proxy.tolist(),
        selected_grasp_center_local_m=selected.tolist(),
        geometric_centroid_fraction_xy=.5,
        target_xy_shift_world_m=(target[:2, 3] - base[:2, 3]).tolist(),
        original_profile_tcp_z_m=float(base[2, 3]),
        target_tcp_world_m=target[:3, 3].tolist(),
        target_world_z_unchanged=True, jaw_orientation_unchanged=True,
        actual_pose_used_for_source_xy_projection=True,
        physical_grasp_stable=False, source_departure_authorized=False,
        selection="Halfway from original source y=0 section center to uniform-volume geometric centroid in local XY; original jaw yaw and clearance height retained. Native COM/force closure remain unverified.")
    return target, info
