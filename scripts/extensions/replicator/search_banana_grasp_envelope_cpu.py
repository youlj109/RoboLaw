#!/usr/bin/env python3
"""CPU source pinch candidates constrained by the original closed-gripper spheres.

Only source grasp TCP height changes. Original pose/yaw/assets, 5 mm robot
sphere buffer and 2 mm non-contact separation remain fixed. This does not step
physics, change a controller, create an attachment, or accept an episode.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import fcl
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from scripts.extensions.replicator.diagnose_task_plan import reconstruct
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.task_collect import json_write
from kitchen_safety.extensions.replicator.task_motion import top_down_grasp


class GripperSphereModel:
    def __init__(self):
        asset = ROOT/"assets/curobo/droid"
        self.paths = [asset/"franka_robotiq_2f_85_zero_curobo.yml", asset/"urdf/franka_robotiq_2f_85_zero.urdf", asset/"tcp_calibration.json"]
        self.cfg = yaml.safe_load(self.paths[0].read_text())["robot_cfg"]["kinematics"]
        self.joints = ET.parse(self.paths[1]).getroot().findall("joint")
        self.calibration = np.asarray(json.loads(self.paths[2].read_text())["T_urdfEE_USDtcp"])

    def at(self, tcp, angles):
        frames, pending = {"base_link": tcp@np.linalg.inv(self.calibration)}, self.joints.copy()
        while pending:
            count = len(pending)
            for joint in pending.copy():
                parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
                if parent not in frames or child in frames:
                    continue
                transform = np.eye(4)
                origin = joint.find("origin")
                if origin is not None:
                    transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    transform[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
                if joint.get("type") in ("revolute", "continuous"):
                    local = np.eye(4)
                    local[:3, :3] = Rotation.from_rotvec(np.fromstring(joint.find("axis").get("xyz"), sep=" ")*angles[joint.get("name")]).as_matrix()
                    transform = transform@local
                frames[child] = frames[parent]@transform
                pending.remove(joint)
            if len(pending) == count:
                break
        rows = []
        for link in self.cfg["collision_link_names"]:
            if link not in frames:
                continue
            transform = frames[link]
            for index, sphere in enumerate(self.cfg["collision_spheres"][link]):
                center = transform[:3, :3]@sphere["center"]+transform[:3, 3]
                rows.append(dict(link=link, local_sphere=index, center=center, radius=float(sphere["radius"])+.005))
        return rows


def sphere_clearance(spheres, world):
    minimum, failures = None, []
    for row in spheres:
        center, radius = row["center"], row["radius"]
        bound = np.linalg.norm(np.maximum(np.maximum(world.low-center, center-world.high), 0.), axis=1)-radius
        order = np.argsort(bound)
        sphere = fcl.CollisionObject(fcl.Sphere(radius), fcl.Transform(center))
        for index in order:
            if minimum is not None and bound[index] > max(.002, minimum["distance_m"]):
                break
            name, obstacle, _, _ = world.objects[index]
            # The source payload is an intentional finger contact. This check
            # is for the original gripper-vs-kitchen/non-payload world pairs.
            if world._matches(name, world.payload_name):
                continue
            hit = bool(fcl.collide(sphere, obstacle, fcl.CollisionRequest(), fcl.CollisionResult()))
            raw = float(fcl.distance(sphere, obstacle, fcl.DistanceRequest(), fcl.DistanceResult()))
            distance = max(0., raw)
            measure = dict(link=row["link"], local_sphere=row["local_sphere"], obstacle=name,
                sphere_world_xyzr=[*center, radius], intersecting=hit or raw < 0, distance_m=distance,
                required_m=.002, valid=bool(not hit and raw >= .002))
            if minimum is None or distance < minimum["distance_m"]:
                minimum = measure
            if not measure["valid"]:
                failures.append(measure)
    return dict(passed=not failures, minimum=minimum, failures=failures)


def run(episode, out):
    world, kin, initial_q, _, _, reconstructed = reconstruct(episode, "approach")
    source = world.payload_pose.copy()
    pose = np.r_[source[:3, 3], Rotation.from_matrix(source[:3, :3]).as_quat()]
    original, info = top_down_grasp(pose, world.payload_geometry, world)
    heights = np.arange(21)*.001
    angles = np.arange(158)*.005
    policy = dict(height_offsets_m=heights.tolist(), original_tcp_target=original,
        finger_joint_candidates_rad=angles.tolist(), required_kitchen_clearance_m=.002,
        original_robot_sphere_buffer_m=.005, fixed_grasp_xy_yaw_and_contact_region=True,
        selection="Height ascending. Record complete first-contact scan, evaluate only the first simultaneous source-surface intersection angle; never select deep closure through a blocked object to improve spheres.",
        physical_success_claim=False)
    json_write(out/"declaration.json", policy)
    meshes, provenance = source_finger_meshes(world)
    sphere_model = GripperSphereModel()
    payload = fcl.CollisionObject(world.payload_triangle_model, fcl.Transform(source[:3, :3], source[:3, 3]))
    low, high = map(np.asarray, info["contact_region_local_m"])
    inverse = np.linalg.inv(source)
    world.set_phase("close")
    candidates, chosen = [], None
    for delta in heights:
        target = original.copy()
        target[2, 3] += delta
        row = dict(tcp_height_offset_m=float(delta), target_tcp=target, angle_scan=[])
        try:
            q = kin.ik(target, initial_q, attempts=1)
            tcp, palm = kin.fk(q), kin.links(q)[world.gripper_root]
            row["q"] = q
            for angle in angles:
                values = dict(zip(world.GRIPPER_JOINT_NAMES, np.array([1., 1., -1., 1., -1., -1.])*angle))
                links = world._gripper_links(values)
                fingers = {}
                for body, models in meshes.items():
                    transform = palm@links[body]
                    contacts = []
                    for _, model in models:
                        result = fcl.CollisionResult()
                        fcl.collide(fcl.CollisionObject(model, fcl.Transform(transform[:3, :3], transform[:3, 3])), payload,
                            fcl.CollisionRequest(num_max_contacts=4096, enable_contact=True), result)
                        if len(result.contacts) >= 4096:
                            raise ValueError("Truncated source contact diagnostic")
                        contacts.extend(inverse[:3, :3]@contact.pos+inverse[:3, 3] for contact in result.contacts)
                    points = np.asarray(contacts).reshape(-1, 3)
                    fingers[body.rsplit("/", 1)[-1]] = dict(intersections=len(points),
                        all_intersections_central=bool(len(points) and np.all(points >= low) and np.all(points <= high)),
                        bounds_local_m=[points.min(0), points.max(0)] if len(points) else None)
                bilateral = all(part["intersections"] for part in fingers.values())
                item = dict(angle_rad=float(angle), fingers=fingers, bilateral=bilateral)
                row["angle_scan"].append(item)
                if bilateral:
                    world.gripper_joint_positions = np.array([values[name] for name in world.GRIPPER_JOINT_NAMES])
                    world.gripper_joint_position = float(angle)
                    full_valid = world.valid(q, gripper_q=values)
                    spheres = sphere_clearance(sphere_model.at(tcp, values), world)
                    row.update(first_bilateral_angle_rad=float(angle), full_noncontact_fcl_passed=full_valid,
                        full_noncontact_fcl_violation=world.last_clearance, original_closed_gripper_spheres=spheres,
                        source_contacts_central=all(part["all_intersections_central"] for part in fingers.values()),
                        passed=bool(full_valid and spheres["passed"] and all(part["all_intersections_central"] for part in fingers.values())))
                    break
            row.setdefault("passed", False)
        except Exception as error:
            row.update(passed=False, error=repr(error))
        candidates.append(row)
        print(json.dumps(dict(height_offset_m=float(delta), passed=row["passed"], angle=row.get("first_bilateral_angle_rad"),
            sphere_minimum=row.get("original_closed_gripper_spheres", {}).get("minimum", {}).get("distance_m"))), flush=True)
        if row["passed"]:
            chosen = len(candidates)-1
            break
    report = dict(accepted=False, physical_grasp_verified=False, canonical_changed=False, controller_changed=False,
        reconstruction=reconstructed, policy=policy, grasp_geometry=info,
        source_finger_meshes=provenance, sphere_source_sha256={str(path):sha(path) for path in sphere_model.paths},
        candidates=candidates, selected_index=chosen, candidate_found=chosen is not None,
        remaining="GPU full-arm/world/self constraint checks, real contacts/force/friction and lift. Payload-support cover still requires an explicit separate pair treatment; this scan does not change payload collision rules.")
    json_write(out/"search_report.json", report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to replace prior candidates")
    args.output.mkdir(parents=True)
    run(args.episode, args.output)


if __name__ == "__main__":
    main()
