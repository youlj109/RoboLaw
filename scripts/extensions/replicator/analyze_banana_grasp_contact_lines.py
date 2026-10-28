#!/usr/bin/env python3
"""Compare hypothetical source-surface pinch lines, never native force closure.

FCL supplies triangle-pair identities only. Recompute triangle intersections,
then use curve-length-weighted geometric centers; never classify approximate
FCL contact.pos as a source surface point or a measured loaded contact.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.task_audit_sidecars import sha
from scripts.extensions.replicator.search_banana_grasp_stability_cpu import observed_world, write


def plane_cut(triangle, point, normal, epsilon):
    distances = (triangle - point) @ normal
    points = [value for value, distance in zip(triangle, distances) if abs(distance) <= epsilon]
    for i, j in ((0, 1), (1, 2), (2, 0)):
        if distances[i] * distances[j] < 0:
            fraction = distances[i] / (distances[i] - distances[j])
            points.append(triangle[i] + fraction * (triangle[j] - triangle[i]))
    return np.array(points).reshape(-1, 3)


def triangle_intersection(first, second, epsilon=1e-10):
    """Floating point source-surface intersection; epsilon is diagnostic only."""
    first, second = np.asarray(first, float), np.asarray(second, float)
    na, nb = np.cross(first[1] - first[0], first[2] - first[0]), np.cross(second[1] - second[0], second[2] - second[0])
    if np.linalg.norm(na) < 1e-18 or np.linalg.norm(nb) < 1e-18:
        raise ValueError("Degenerate source triangle in reported collision pair")
    na, nb = na / np.linalg.norm(na), nb / np.linalg.norm(nb)
    direction = np.cross(na, nb)
    if np.linalg.norm(direction) < 1e-9:
        if np.max(np.abs((second - first[0]) @ na)) > epsilon:
            return np.empty((0, 3))
        # Coplanar convex triangle clipping; keep exact interpolated 3-D points.
        polygon = list(first)
        for i, j in ((0, 1), (1, 2), (2, 0)):
            inward = np.cross(nb, second[j] - second[i])
            inward /= np.linalg.norm(inward)
            if not polygon:
                break
            old, polygon = np.array(polygon), []
            for a, b in zip(old, np.roll(old, -1, axis=0)):
                da, db = (a - second[i]) @ inward, (b - second[i]) @ inward
                ia, ib = da >= -epsilon, db >= -epsilon
                if ia:
                    polygon.append(a)
                if ia != ib:
                    polygon.append(a + da / (da - db) * (b - a))
        return np.asarray(polygon).reshape(-1, 3)
    direction /= np.linalg.norm(direction)
    a, b = plane_cut(first, second[0], nb, epsilon), plane_cut(second, first[0], na, epsilon)
    if not len(a) or not len(b):
        return np.empty((0, 3))
    aa, bb = a @ direction, b @ direction
    low, high = max(aa.min(), bb.min()), min(aa.max(), bb.max())
    if high < low - epsilon:
        return np.empty((0, 3))
    high = max(low, high)
    return np.array([a[0] + (value - aa[0]) * direction for value in (low, high)])


def curve_centroid(pieces):
    lengths, centers, points = [], [], []
    for polygon in pieces:
        points.extend(polygon)
        edges = [(polygon[0], polygon[1])] if len(polygon) == 2 else list(zip(polygon, np.roll(polygon, -1, axis=0)))
        for a, b in edges:
            length = np.linalg.norm(b - a)
            if length > 1e-12:
                lengths.append(length)
                centers.append((a + b) / 2)
    if not points:
        raise ValueError("No reconstructible source intersection points")
    points = np.asarray(points)
    center = np.average(centers, axis=0, weights=lengths) if lengths else points.mean(0)
    return dict(curve_length_weighted_center_local_m=center,
                bounds_midpoint_local_m=(points.min(0) + points.max(0)) / 2,
                source_intersection_bounds_local_m=[points.min(0), points.max(0)],
                total_curve_length_m=float(sum(lengths)), vertices=len(points),
                segment_pieces=len(pieces))


def line_metrics(first, second, proxy_com, gravity_local):
    first, second = np.asarray(first, float), np.asarray(second, float)
    axis = second - first
    if np.linalg.norm(axis) < 1e-8:
        raise ValueError("Hypothetical finger contact centers coincide")
    axis /= np.linalg.norm(axis)
    midpoint = (first + second) / 2
    offset = np.asarray(proxy_com) - midpoint
    torque = np.cross(offset, gravity_local)
    return dict(first_to_second_axis_local=axis, midpoint_local_m=midpoint,
                COM_proxy_distance_to_infinite_contact_line_m=float(np.linalg.norm(np.cross(offset, axis))),
                COM_proxy_minus_midpoint_local_m=offset, gravity_torque_vector_local_Nm=torque,
                gravity_torque_along_first_to_second_axis_Nm=float(torque @ axis),
                gravity_torque_along_axis_absolute_Nm=float(abs(torque @ axis)))


def run(args):
    import fcl
    from scripts.extensions.replicator.diagnose_bowl_contact_geometry import finger_mesh_arrays
    from scripts.extensions.replicator.diagnose_banana_grasp_stability_gpu import load_candidate
    declaration = json.loads((args.search / "declaration.json").read_text())
    world, kin, _, _, _, _, reconstruction = observed_world(args.episode)
    finger_arrays, _ = finger_mesh_arrays(world)
    source = world.payload_pose.copy()
    payload = fcl.CollisionObject(world.payload_triangle_model, fcl.Transform(source[:3, :3], source[:3, 3]))
    proxy = np.array(declaration["source_mesh_centroid_proxy"])
    from pxr import Usd, UsdPhysics
    stage = Usd.Stage.Open(world.payload_geometry.provenance["source_asset"])
    mass = float(UsdPhysics.MassAPI(stage.GetDefaultPrim()).GetMassAttr().Get())
    gravity = source[:3, :3].T @ np.array([0., 0., -9.81 * mass])
    models = {}
    for name, body in finger_arrays.items():
        models[name] = []
        for part in body["parts"]:
            model = fcl.BVHModel()
            model.beginModel(len(part["vertices"]), len(part["faces"]))
            model.addSubModel(part["vertices"], part["faces"])
            model.endModel()
            models[name].append((part, model))
    rows = []
    for candidate in declaration["candidates"]:
        record, arrays, hashes = load_candidate(args.search, candidate["id"], args.episode)
        q = arrays["close_q"]
        fingers = np.array([1., 1., -1., 1., -1., -1.]) * float(arrays["close_angle_rad"])
        links = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, fingers)))
        palm = kin.links(q)[world.gripper_root]
        observed = {}
        for name, body in finger_arrays.items():
            transform = np.linalg.inv(source) @ palm @ links[body["body_path"]]
            world_transform = source @ transform
            pieces, records = [], []
            for part, model in models[name]:
                collider = fcl.CollisionObject(model, fcl.Transform(world_transform[:3, :3], world_transform[:3, 3]))
                response = fcl.CollisionResult()
                fcl.collide(collider, payload, fcl.CollisionRequest(num_max_contacts=8192, enable_contact=True), response)
                if len(response.contacts) >= 8192:
                    raise ValueError("Triangle pair buffer saturated")
                pairs = sorted({(int(contact.b1), int(contact.b2)) for contact in response.contacts})
                for first, second in pairs:
                    if not 0 <= first < len(part["faces"]) or not 0 <= second < len(world.payload_geometry.triangles):
                        raise ValueError("Invalid source triangle pair identity")
                    finger_tri = part["vertices"][part["faces"][first]] @ transform[:3, :3].T + transform[:3, 3]
                    banana_tri = world.payload_geometry.vertices[world.payload_geometry.triangles[second]]
                    points = triangle_intersection(finger_tri, banana_tri)
                    if not len(points):
                        raise ValueError("FCL source pair has no reconstructed triangle intersection")
                    pieces.append(points)
                    records.append(dict(mesh=part["path"], finger_triangle=first, banana_triangle=second,
                                        intersection_points_banana_local_m=points))
            observed[name] = dict(**curve_centroid(pieces), source_triangle_pairs=records)
        left, right = observed["left_inner_finger"], observed["right_inner_finger"]
        metrics = line_metrics(left["curve_length_weighted_center_local_m"], right["curve_length_weighted_center_local_m"], proxy, gravity)
        bounds_metrics = line_metrics(left["bounds_midpoint_local_m"], right["bounds_midpoint_local_m"], proxy, gravity)
        first_contacts = {name: next(item["angle_rad"] for item in record["close_rows"]
            if sum(item["clipped_source_region_contacts"]["fingers"][name][key] for key in ("inside", "outside")))
            for name in observed}
        rows.append(dict(id=candidate["id"], close_angle_rad=float(arrays["close_angle_rad"]),
            first_source_intersection_angle_rad=first_contacts, curve_centroid_line=metrics,
            bounds_midpoint_line_sensitivity=bounds_metrics, fingers=observed, candidate_input_sha256=hashes))
    return dict(schema_version=1, accepted=False, physical_stability_verified=False,
        source_centroid_proxy=proxy, source_authored_mass_kg=mass, native_COM_and_inertia_recorded=False,
        reconstruction=reconstruction, candidates=rows,
        ranking_by_proxy_torque=[row["id"] for row in sorted(rows,
            key=lambda row: row["curve_centroid_line"]["gravity_torque_along_axis_absolute_Nm"])],
        definition="At each candidate's first simultaneous source-surface intersection, reconstruct exact source triangle-pair intersection polylines. Use their curve-length-weighted centers per finger as a geometric pinch-line proxy; also report bounds-midpoint sensitivity. This is not a native force-weighted contact line.",
        numerical_intersection_epsilon_m=1e-10,
        limitations=["Native cooked collision shapes, mass distribution, friction/torsional impulses and physical contact motion are not represented.",
            "COM proxy assumes uniform density of the closed source triangle surface; it is not a recovered native PhysX COM.",
            "Different fingers may hit at different closure angles; a free object may move before this hypothetical fixed-object bilateral configuration.",
            "Torque ranking is an explicit geometric hypothesis, not an acceptance or proof of stability."],
        code_sha256={str(Path(__file__)): sha(Path(__file__))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--search", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite prior source contact-line evidence")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = run(args)
    write(args.output, report)
    for row in report["candidates"]:
        print(json.dumps(dict(id=row["id"], distance_m=row["curve_centroid_line"]["COM_proxy_distance_to_infinite_contact_line_m"],
            torque_Nm=row["curve_centroid_line"]["gravity_torque_along_first_to_second_axis_Nm"],
            bounds_midpoint_torque_Nm=row["bounds_midpoint_line_sensitivity"]["gravity_torque_along_first_to_second_axis_Nm"])))


if __name__ == "__main__":
    main()
