#!/usr/bin/env python3
"""Six predeclared source grasp candidates after the measured rotating pinch.

Change only XY grasp center and jaw yaw. Keep source layout, target height,
contact region, all collision models/margins, materials and actuation unchanged.
This source-geometry scan cannot establish force closure or physical success.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha


def write(path, value):
    if path.exists():
        raise ValueError("Refuse to overwrite " + str(path))
    def convert(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(type(item).__name__)
    path.write_text(json.dumps(value, indent=2, allow_nan=False, default=convert) + "\n")


def finite_candidates(original_target, source_pose, section_center, mesh_centroid, long_axis):
    """Three XY centers × original/perpendicular yaw; exactly fixed world Z."""
    source_pose = np.asarray(source_pose)
    center, proxy = np.asarray(section_center), np.asarray(mesh_centroid)
    axis = np.asarray(long_axis)
    if np.linalg.norm(axis[:2]) < .5:
        raise ValueError("Source geometric long axis is not sufficiently horizontal")
    cross = np.array([-axis[1], axis[0], 0.])
    # A jaw line is unoriented; take the heading closest to original source +X.
    if cross[0] < 0:
        cross *= -1
    cross /= np.linalg.norm(cross)
    yaw = float(np.arctan2(cross[1], cross[0]))
    rows = []
    for i, alpha in enumerate([0., .5, 1.]):
        local = center.copy()
        local[:2] = (1 - alpha) * center[:2] + alpha * proxy[:2]
        for j, angle in enumerate([0., yaw]):
            target = np.array(original_target, copy=True)
            # Translation affects only XY; retain the exact old source-derived Z.
            target[:2, 3] += (source_pose[:3, :3] @ (local - center))[:2]
            jaw = source_pose[:3, :3] @ np.array([np.cos(angle), np.sin(angle), 0.])
            jaw[2] = 0.
            jaw /= np.linalg.norm(jaw)
            approach = np.array([0., 0., -1.])
            target[:3, :3] = np.column_stack((approach, jaw, np.cross(approach, jaw)))
            rows.append(dict(id=f"contactcenter{i}_jaw{j}", center_local_m=local,
                geometric_COM_fraction=alpha, jaw_rotation_about_source_z_deg=float(np.rad2deg(angle)),
                target_tcp=target, target_world_z_unchanged=True))
    return rows


def observed_world(episode):
    from pxr import Gf, Usd, UsdGeom
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance
    from scripts.extensions.replicator.audit_partial_task_episode import validate_partial
    metadata = json.loads((episode / "physics_metadata.json").read_text())
    data = dict(np.load(episode / "physics_states.npz", allow_pickle=False))
    trajectory = dict(np.load(episode / "trajectory.npz", allow_pickle=False))
    result = json.loads((episode / "result.json").read_text())
    extent = validate_partial(metadata, data, trajectory)
    provenance = audit_provenance(episode / "task_collection_provenance.json", metadata,
                                  int(data["physics_step"][-1]))
    if not provenance["passed"]:
        raise ValueError("Actual episode provenance failed")
    layout = metadata["source_layout"]
    if layout["task_id"] != "kitchen_bench_replicator_g_shape_banana_bowl":
        raise ValueError("Only this original source banana task is supported")
    plan = next(row for row in result["plans"] if row["phase"] == "approach")
    initial_plan = next(row for row in result["plans"] if row["phase"] == "pregrasp")
    index = int(np.flatnonzero(data["physics_step"] == plan["start_physics_step"]).item())
    initial = int(np.flatnonzero(data["physics_step"] == initial_plan["start_physics_step"]).item())
    stage = Usd.Stage.CreateInMemory()
    root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, root)
    UsdGeom.Xform.Define(stage, root + "/kitchen").GetPrim().GetReferences().AddReference(str(ROOT / layout["kitchen_usd"]))
    for name, item in layout["objects"].items():
        xf = UsdGeom.Xform.Define(stage, root + "/" + name)
        xf.GetPrim().GetReferences().AddReference(str(ROOT / item["usd_path"]))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
        xf.AddRotateZOp().Set(item["yaw_deg"])
    kin = DroidKinematics(np.asarray(layout["robot_position"]) + [0, 0, layout["stand_height"]],
                          layout["robot_yaw_deg"])
    world = PayloadCollisionWorld(stage, kin, payload_name="banana",
        payload_usd=ROOT / layout["objects"]["banana"]["usd_path"], env_root=root,
        clearances={}, execution_clearances={}, default_clearance=.002)
    world.sync_background(metadata["background_paths"], data["background_rigid_poses"][index])
    for name in metadata["task_object_names"]:
        pose = data["task_object_states"][index, metadata["task_object_names"].index(name), :7]
        world.set_obstacle_pose(name, pose)
        if name == "banana":
            world.payload_pose = pose_matrix(pose)
    ids = [metadata["joint_names"].index(f"panda_joint{i}") for i in range(1, 8)]
    return world, kin, data["q"][index, ids], data["q"][initial, ids], metadata, result, dict(
        actual_extent=extent, approach_start_physics_step=int(data["physics_step"][index]),
        pregrasp_start_physics_step=int(data["physics_step"][initial]),
        input_sha256={name: sha(episode / name) for name in
            ("physics_metadata.json", "physics_states.npz", "result.json", "trajectory.npz", "task_collection_provenance.json")})


def path_audit(world, kin, positions):
    from scripts.extensions.replicator.diagnose_task_plan import cpu_sphere_audit
    rows = []
    for index, q in enumerate(positions):
        passed = world.valid(q)
        rows.append(dict(sample=index, passed=passed, violation=deepcopy(world.last_clearance)))
    spheres, _ = cpu_sphere_audit(world, kin, positions)
    return dict(source_fcl_passed=all(row["passed"] for row in rows), source_fcl=rows,
                original_spheres=spheres, actual_cuda_self_checked=False)


def run(args):
    import trimesh
    from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
    from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
    from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts
    from scripts.extensions.replicator.search_banana_grasp_envelope_cpu import GripperSphereModel, sphere_clearance
    world, kin, approach_seed, initial_q, metadata, result, reconstruction = observed_world(args.episode)
    geometry = world.payload_geometry
    mesh = trimesh.Trimesh(geometry.vertices, geometry.triangles, process=True)
    if not mesh.is_watertight:
        raise ValueError("Uniform-volume source centroid proxy requires a closed source surface")
    values, axes = np.linalg.eigh(np.cov(geometry.vertices.T))
    original = np.array(next(row for row in result["plans"] if row["phase"] == "approach")["target_tcp"])
    candidates = finite_candidates(original, world.payload_pose, result["grasp_target"]["center_local_m"],
                                   mesh.center_mass, axes[:, -1])
    declaration = dict(schema_version=1, accepted=False, physical_success=False,
        hypothesis="Reduce off-center rotational loading by shifting the grasp XY toward a geometric mass proxy and/or aligning the jaw perpendicular to the projected long axis. Native COM/inertia and stable force closure remain unverified.",
        original_target=original, source_mesh_centroid_proxy=mesh.center_mass,
        source_geometric_long_axis=axes[:, -1], source_vertex_covariance_eigenvalues=values,
        candidate_order="All six candidates retained; no early stop or deeper-closure cherry picking.",
        candidate_count=6, candidates=candidates, close_angles_rad=np.arange(158) * .005,
        invariants=dict(target_world_z=float(original[2, 3]), source_layout=metadata["source_layout"],
            original_contact_region=result["grasp_target"]["contact_region_local_m"],
            noncontact_margin_m=.002, robot_sphere_padding_m=.005,
            physics_materials_and_actuation_unchanged=True), reconstruction=reconstruction,
        code_sha256={str(Path(__file__).resolve().relative_to(ROOT)): sha(Path(__file__))})
    write(args.output / "declaration.json", declaration)
    meshes, finger_provenance = source_finger_meshes(world)
    regions = SourceRegionContacts(geometry, result["grasp_target"]["contact_region_local_m"], meshes)
    sphere_model = GripperSphereModel()
    signs = np.array([1., 1., -1., 1., -1., -1.])
    source_pose = world.payload_pose.copy()
    summaries = []
    for candidate in candidates:
        row = dict(candidate=deepcopy(candidate), passed_cpu_geometric_prerequisites=False,
                   accepted=False, physical_grasp_verified=False, close_rows=[])
        target = np.array(candidate["target_tcp"])
        archive = {}
        try:
            world.attachment = None
            world.payload_pose = source_pose.copy()
            world.gripper_joint_positions = np.zeros(6)
            world.gripper_joint_position = 0.
            world.set_phase("approach")
            pre = target.copy()
            pre[2, 3] += .125
            qpre = kin.ik(pre, approach_seed, attempts=1)
            error = kin.fk(qpre)
            position_error = float(np.linalg.norm(error[:3, 3] - pre[:3, 3]))
            angle_error = float(Rotation.from_matrix(error[:3, :3] @ pre[:3, :3].T).magnitude())
            if position_error > .002 or angle_error > np.deg2rad(2):
                raise ValueError("Pregrasp IK misses original pose gates")
            approach = local_cartesian_candidate(kin, qpre, target, "approach", metadata["action_dt_s"], .4)
            qclose = approach["positions"][-1]
            tcp = kin.fk(qclose)
            row.update(pregrasp_cpu_ik=dict(q=qpre, position_error_m=position_error, rotation_error_rad=angle_error),
                       approach=path_audit(world, kin, approach["positions"]),
                       pregrasp_path_from_actual_initial_q_verified=False)
            archive.update(actual_initial_q=initial_q, pregrasp_q=qpre, approach=approach["positions"],
                           target_tcp=target, close_q=qclose, observed_preapproach_banana=source_pose)
            world.set_phase("close")
            first = None
            for angle in np.arange(158) * .005:
                world.gripper_joint_positions = signs * angle
                world.gripper_joint_position = float(angle)
                good = world.valid(qclose)
                contact = regions.at(world, kin, qclose)
                spheres = sphere_clearance(sphere_model.at(tcp, dict(zip(world.GRIPPER_JOINT_NAMES, signs * angle))), world)
                item = dict(angle_rad=float(angle), source_noncontact_passed=good,
                    source_noncontact_violation=deepcopy(world.last_clearance), original_gripper_spheres=spheres,
                    clipped_source_region_contacts=contact)
                row["close_rows"].append(item)
                bilateral = all(part["inside"] + part["outside"] > 0 for part in contact["fingers"].values())
                if bilateral:
                    first = item
                    break
            row["first_bilateral_angle_rad"] = None if first is None else first["angle_rad"]
            row["close_passed"] = bool(first and first["clipped_source_region_contacts"]["bilateral_all_central"]
                and all(item["source_noncontact_passed"] and item["original_gripper_spheres"]["passed"] for item in row["close_rows"]))
            if first:
                world.attachment = np.linalg.inv(tcp) @ source_pose
                world.set_phase("lift")
                lift_target = tcp.copy()
                lift_target[2, 3] += .15
                lift = local_cartesian_candidate(kin, qclose, lift_target, "lift", metadata["action_dt_s"], .4)
                row["hypothetical_lift"] = path_audit(world, kin, lift["positions"])
                row["hypothetical_tcp_to_banana"] = world.attachment.copy()
                archive.update(lift=lift["positions"], close_angle_rad=float(first["angle_rad"]),
                               hypothetical_tcp_to_banana=world.attachment.copy())
            row["passed_cpu_geometric_prerequisites"] = bool(row["close_passed"]
                and row["approach"]["source_fcl_passed"]
                and not row["approach"]["original_spheres"]["world_intersections"]
                and not row["approach"]["original_spheres"]["joint_urdf_violations"]
                and row.get("hypothetical_lift", {}).get("source_fcl_passed", False))
        except Exception as error:
            row["error"] = repr(error)
        if archive:
            path = args.output / (candidate["id"] + ".npz")
            np.savez_compressed(path, **archive)
            row["archive"] = dict(path=str(path), sha256=sha(path))
        write(args.output / (candidate["id"] + ".json"), row)
        summaries.append(dict(id=candidate["id"], passed_cpu_geometric_prerequisites=row["passed_cpu_geometric_prerequisites"],
            first_bilateral_angle_rad=row.get("first_bilateral_angle_rad"), error=row.get("error"),
            path=str(args.output / (candidate["id"] + ".json")),
            sha256=sha(args.output / (candidate["id"] + ".json"))))
        print(json.dumps(summaries[-1]), flush=True)
    report = dict(accepted=False, physical_success=False, candidates=summaries,
        declaration_sha256=sha(args.output / "declaration.json"), original_source_fingers=finger_provenance,
        exact_contact_region_partition=regions.provenance,
        remaining="No physical force closure or native grasp-stability proof. Actual initial-to-pregrasp motion, CUDA world/self/joint gates (including explicit source-support pair treatment), physical contacts/lift and full original success remain unverified. No candidate auto-promoted.")
    write(args.output / "search_report.json", report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite any prior candidate evidence")
    args.output.mkdir(parents=True)
    try:
        run(args)
    except Exception as exc:
        write(args.output / "failure.json", dict(accepted=False, error=repr(exc)))
        raise


if __name__ == "__main__":
    main()
