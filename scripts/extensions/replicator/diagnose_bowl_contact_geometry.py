#!/usr/bin/env python3
"""Read-only CPU source/observed-body geometry comparison for failed bowl close.

This inspects authored USD collision metadata, not PhysX's runtime cooked data.
It measures original source triangle distances at recorded body poses and at
CPU forward kinematics independently. It never alters a source asset or run.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics
import trimesh

from kitchen_safety.planning import ROBOT_USD
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.task_audit_sidecars import load_episode, sha
from scripts.extensions.replicator.diagnose_bowl_grasp_recording import synchronize
from scripts.extensions.replicator.bowl_collect import json_write


def collision_metadata(stage):
    rows = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        row = dict(path=str(prim.GetPath()), type=str(prim.GetTypeName()),
            applied_schemas=list(prim.GetAppliedSchemas()),
            collision_enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),
            approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(),
            authored_attributes={})
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if attr.HasAuthoredValueOpinion() and any(token in name.lower() for token in
                    ("offset", "approx", "convex", "cook", "hull", "sdf")):
                value = attr.Get()
                row["authored_attributes"][name] = str(value)
        rows.append(row)
    return rows


def finger_mesh_arrays(world):
    stage = Usd.Stage.Open(str(ROBOT_USD))
    cache = UsdGeom.XformCache()
    rows = {}
    for finger in ("left_inner_finger", "right_inner_finger"):
        body_path = world.gripper_root.rsplit("/", 1)[0]+"/"+finger
        body = stage.GetPrimAtPath(body_path)
        inverse = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(body)).T)
        parts = []
        for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            ancestor, collision = prim, None
            while ancestor and str(ancestor.GetPath()).startswith(body_path):
                if ancestor.HasAPI(UsdPhysics.CollisionAPI):
                    collision = ancestor
                    break
                ancestor = ancestor.GetParent()
            if collision is None or not UsdPhysics.CollisionAPI(collision).GetCollisionEnabledAttr().Get():
                continue
            mesh = UsdGeom.Mesh(prim)
            transform = inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
            points = np.asarray(mesh.GetPointsAttr().Get(), float)
            points = points@transform[:3, :3].T+transform[:3, 3]
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), int)
            start, faces = 0, []
            for count in mesh.GetFaceVertexCountsAttr().Get():
                faces.extend(indices[[start, start+i, start+i+1]] for i in range(1, count-1))
                start += count
            parts.append(dict(path=str(prim.GetPath()), vertices=points, faces=np.asarray(faces)))
        rows[finger] = dict(body_path=body_path, parts=parts)
    return rows, stage


def closest(vertices, faces, point):
    triangles = vertices[faces]
    points = np.repeat(np.asarray(point)[None], len(triangles), axis=0)
    candidates = trimesh.triangles.closest_point(triangles, points)
    distance = np.linalg.norm(candidates-point, axis=1)
    index = int(np.argmin(distance))
    return dict(distance_m=float(distance[index]), closest_point=candidates[index], triangle_index=index)


def run(args):
    meta, data, _ = load_episode(args.episode)
    contacts = json.loads((args.episode/"grasp_contacts.json").read_text())
    world, kin, _ = build_world(meta["source_layout"])
    fingers, robot_stage = finger_mesh_arrays(world)
    payload_stage = Usd.Stage.Open(world.payload_geometry.provenance["source_asset"])
    arm = [meta["joint_names"].index(f"panda_joint{i}") for i in range(1, 8)]
    close = np.flatnonzero(data["phase"] == "close")
    samples = []
    for index in [int(close[0]), int(close[len(close)//2]), int(close[-1])]:
        synchronize(world, meta, data, index)
        q = data["q"][index, arm]
        palm = kin.links(q)[world.gripper_root]
        kinematic_links = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, world.gripper_joint_positions)))
        bowl = world.payload_pose
        items = {}
        for name, model in fingers.items():
            body = pose_matrix(data["robot_body_pose_w"][index, meta["robot_body_names"].index(name)])
            calculated = palm@kinematic_links[model["body_path"]]
            item = dict(body_pose=data["robot_body_pose_w"][index, meta["robot_body_names"].index(name)],
                cpu_fk_transform=calculated,
                body_vs_cpu_translation_error_m=float(np.linalg.norm(body[:3, 3]-calculated[:3, 3])),
                body_vs_cpu_rotation_error_rad=float(Rotation.from_matrix(body[:3, :3]@calculated[:3, :3].T).magnitude()),
                contacts=[])
            raw = contacts[index]["fingers"][name]
            for j, force in enumerate(raw["normal_forces_n"]):
                point = np.asarray(raw["points_world_m"][j])
                local = np.asarray(raw["points_object_local_m"][j])
                row = dict(force_n=force, separation_m=raw["separations_m"][j], point_bowl_local_m=local,
                    source_bowl_surface=closest(world.payload_geometry.vertices, world.payload_geometry.triangles, local),
                    finger_source={})
                for label, transform in (("actual_recorded_body", body), ("cpu_joint_fk", calculated)):
                    plocal = transform[:3, :3].T@(point-transform[:3, 3])
                    parts = []
                    for part in model["parts"]:
                        value = closest(part["vertices"], part["faces"], plocal)
                        hull = ConvexHull(part["vertices"])
                        value.update(path=part["path"], point_finger_local_m=plocal,
                            convex_envelope_max_plane_m=float(np.max(hull.equations[:, :3]@plocal+hull.equations[:, 3])))
                        parts.append(value)
                    row["finger_source"][label] = parts
                item["contacts"].append(row)
            items[name] = item
        samples.append(dict(physics_step=index, fingers=items))
    return dict(accepted=False, runtime_cooked_geometry_verified=False, episode=str(args.episode),
        inputs_sha256={str(args.episode/name): sha(args.episode/name) for name in
            ("physics_metadata.json", "physics_states.npz", "grasp_contacts.json")},
        source_assets_sha256={str(ROBOT_USD):sha(ROBOT_USD),world.payload_geometry.provenance["source_asset"]:world.payload_geometry.provenance["source_sha256"]},
        source_robot_finger_colliders=[x for x in collision_metadata(robot_stage) if "inner_finger" in x["path"]],
        source_bowl_colliders=collision_metadata(payload_stage), samples=samples,
        limits="Authored metadata and original triangle distance only. No runtime PhysX cooked-shape access; convex-envelope plane distance is diagnostic, not the actual decomposition. Missing authored offsets do not imply zero runtime offset.")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--episode", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    args=p.parse_args()
    if args.output.exists():
        p.error("Refuse to overwrite contact geometry evidence")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result=run(args)
    json_write(args.output,result)
    for sample in result["samples"]:
        print(json.dumps(dict(step=sample["physics_step"], fingers={name:dict(
            body_fk_error_m=item["body_vs_cpu_translation_error_m"],
            loaded=[dict(force_n=c["force_n"], bowl_gap_m=c["source_bowl_surface"]["distance_m"],
                finger_gap_m=min(v["distance_m"] for v in c["finger_source"]["actual_recorded_body"]))
                for c in item["contacts"] if c["force_n"]>.01]) for name,item in sample["fingers"].items()})))


if __name__ == "__main__":
    main()
