#!/usr/bin/env python3
"""CPU source-surface pinch feasibility at the corrected banana grasp target.

This never steps physics. It scans original coupled finger joint positions at
the source-derived TCP target, retains all non-contact FCL checks and records
exact source-surface intersections. Convex decomposition cooking, friction,
forces and stable physical bilateral grasp remain unverified.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.diagnose_task_plan import reconstruct
from scripts.extensions.replicator.task_audit_sidecars import sha
from scripts.extensions.replicator.task_collect import json_write


def source_finger_meshes(world):
    """Enabled source collider descendant meshes, in each rigid finger frame."""
    import fcl
    from pxr import Usd, UsdGeom, UsdPhysics
    from kitchen_safety.planning import ROBOT_USD
    stage = Usd.Stage.Open(str(ROBOT_USD))
    cache = UsdGeom.XformCache()
    result, provenance = {}, []
    for name in ("left_inner_finger", "right_inner_finger"):
        root = world.gripper_root.rsplit("/", 1)[0]+"/"+name
        body = stage.GetPrimAtPath(root)
        inverse = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(body)).T)
        meshes = []
        for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            ancestor, collider = prim, None
            while ancestor and str(ancestor.GetPath()).startswith(root):
                if ancestor.HasAPI(UsdPhysics.CollisionAPI):
                    collider = ancestor
                    break
                ancestor = ancestor.GetParent()
            if collider is None or not UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get():
                continue
            mesh = UsdGeom.Mesh(prim)
            transform = inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
            vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            vertices = vertices@transform[:3, :3].T+transform[:3, 3]
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
            cursor, triangles = 0, []
            for count in mesh.GetFaceVertexCountsAttr().Get():
                triangles.extend([indices[[cursor, cursor+i, cursor+i+1]] for i in range(1, count-1)])
                cursor += count
            triangles = np.asarray(triangles, dtype=np.int32)
            model = fcl.BVHModel()
            model.beginModel(len(vertices), len(triangles))
            model.addSubModel(vertices, triangles)
            model.endModel()
            meshes.append((str(prim.GetPath()), model))
            provenance.append(dict(finger=name, mesh=str(prim.GetPath()), collision_api=str(collider.GetPath()),
                source_approximation=UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get(),
                vertices=len(vertices), triangles=len(triangles)))
        if not meshes:
            raise ValueError("Missing enabled original finger collider meshes: " + name)
        result[root] = meshes
    return result, dict(robot_source=str(ROBOT_USD), robot_sha256=sha(ROBOT_USD), meshes=provenance,
        representation="Original enabled collision-descendant triangle surfaces; PhysX convexDecomposition not cooked/reproduced")


def audit(episode):
    import fcl
    from kitchen_safety.extensions.replicator.task_motion import top_down_grasp
    world, kin, start, old_target, metadata, reconstruction = reconstruct(episode, "approach")
    observed_pose = np.r_[world.payload_pose[:3, 3], Rotation.from_matrix(world.payload_pose[:3, :3]).as_quat()]
    target, grasp = top_down_grasp(observed_pose, world.payload_geometry, world)
    q = kin.ik(target, start, attempts=1)
    actual_tcp = kin.fk(q)
    position_error = float(np.linalg.norm(actual_tcp[:3, 3]-target[:3, 3]))
    rotation_error = float(Rotation.from_matrix(actual_tcp[:3, :3]@target[:3, :3].T).magnitude())
    if position_error > .002 or rotation_error > np.deg2rad(2.):
        raise ValueError("CPU IK target error exceeds existing gates")
    meshes, provenance = source_finger_meshes(world)
    source = world.payload_pose
    banana = fcl.CollisionObject(world.payload_triangle_model, fcl.Transform(source[:3, :3], source[:3, 3]))
    source_inverse = np.linalg.inv(source)
    region_low, region_high = map(np.asarray, grasp["contact_region_local_m"])
    palm = kin.links(q)[world.gripper_root]
    world.set_phase("close")
    rows = []
    for angle in np.linspace(0., .78, 157):
        angles = np.asarray([1., 1., -1., 1., -1., -1.])*angle
        world.gripper_joint_positions, world.gripper_joint_position = angles, float(angle)
        links = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, angles)))
        fingers = {}
        for body, body_meshes in meshes.items():
            transform = palm@links[body]
            all_contacts, collisions = [], 0
            for path, model in body_meshes:
                finger = fcl.CollisionObject(model, fcl.Transform(transform[:3, :3], transform[:3, 3]))
                request = fcl.CollisionRequest(num_max_contacts=4096, enable_contact=True)
                response = fcl.CollisionResult()
                count = fcl.collide(finger, banana, request, response)
                if count >= 4096:
                    raise ValueError("Source contact buffer saturated")
                collisions += int(count)
                for contact in response.contacts:
                    point = np.asarray(contact.pos)
                    local = source_inverse[:3, :3]@point+source_inverse[:3, 3]
                    if not np.isfinite(np.r_[point, local, contact.normal]).all():
                        raise ValueError("Invalid source intersection contact")
                    all_contacts.append(dict(finger_mesh=path, point_world_m=point.tolist(),
                        point_banana_local_m=local.tolist(), normal_world=np.asarray(contact.normal).tolist(),
                        within_source_central_region=bool(np.all(local >= region_low) and np.all(local <= region_high))))
            fingers[body.rsplit("/", 1)[-1]] = dict(source_triangle_intersections=collisions, contacts=all_contacts,
                central_intersection_present=bool(all_contacts and any(item["within_source_central_region"] for item in all_contacts)),
                all_reported_intersections_central=bool(all_contacts and all(item["within_source_central_region"] for item in all_contacts)))
        full_world_valid = world.valid(q, gripper_q=angles)
        rows.append(dict(finger_joint_rad=float(angle), all_original_coupled_angles_rad=angles.tolist(),
            full_noncontact_world_fcl_valid=full_world_valid, noncontact_violation=world.last_clearance,
            fingers=fingers, bilateral_central_source_surface_intersection=all(
                row["central_intersection_present"] for row in fingers.values())))
    candidates = [row for row in rows if row["full_noncontact_world_fcl_valid"] and row["bilateral_central_source_surface_intersection"]]
    return dict(accepted=False, physical_bilateral_grasp_verified=False,
        source_geometric_pinch_candidate_found=bool(candidates), grasp=grasp,
        correction_from_old_target_m=float(target[2, 3]-old_target[2, 3]),
        cpu_ik=dict(q=q.tolist(), position_error_m=position_error, rotation_error_rad=rotation_error),
        first_bilateral_candidate_angle_rad=candidates[0]["finger_joint_rad"] if candidates else None,
        last_bilateral_candidate_angle_rad=candidates[-1]["finger_joint_rad"] if candidates else None,
        candidate_angle_count=len(candidates), scanned_angle_count=len(rows),
        minimum_initial_source_contact_angle_rad={name: next((row["finger_joint_rad"] for row in rows
            if row["fingers"][name]["source_triangle_intersections"]), None) for name in ("left_inner_finger", "right_inner_finger")},
        original_source_finger_meshes=provenance, banana_source_geometry=world.payload_geometry.provenance,
        reconstruction=reconstruction, rows=rows,
        limits="Fixed-pose kinematic closure scan only. Source triangle intersections establish possible central finger contact, not PhysX cooked contacts, collision-free penetration, force closure, friction, lift or original task success. Actual collector still requires simultaneous bilateral force for 10 substeps and actual lift.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite an existing source-grasp diagnostic")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = audit(args.episode)
    json_write(args.output, report)
    print(json.dumps({key: report[key] for key in ("source_geometric_pinch_candidate_found",
        "first_bilateral_candidate_angle_rad", "last_bilateral_candidate_angle_rad", "candidate_angle_count",
        "scanned_angle_count", "physical_bilateral_grasp_verified")}))


if __name__ == "__main__":
    main()
