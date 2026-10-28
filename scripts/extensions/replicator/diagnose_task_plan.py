#!/usr/bin/env python3
"""Reconstruct a rejected local segment from recorded actual state, never step.

Default is CPU IK/full FCL + original URDF sphere attribution. --gpu adds the
exact cuRobo constraint and sphere/collider queries under the shared GPU lock.
Both preserve input episodes, save rejected paths, and never accept an episode.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.task_audit_sidecars import ROOT, load_episode, sha
from scripts.extensions.replicator.task_collect import json_write

_GPU_LOCK = None


def reconstruct(episode, phase):
    from pxr import Gf, Usd, UsdGeom
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    metadata, data, trajectory = load_episode(episode)
    result = json.loads((episode/"result.json").read_text())
    plan = next(row for row in reversed(result["plans"]) if row["phase"] == phase)
    indices = np.flatnonzero(data["physics_step"] == plan["start_physics_step"])
    if len(indices) != 1:
        raise ValueError("The failed query must have one exact recorded start state")
    index = int(indices[0])
    layout = metadata["source_layout"]
    stage = Usd.Stage.CreateInMemory()
    root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, root)
    UsdGeom.Xform.Define(stage, root+"/kitchen").GetPrim().GetReferences().AddReference(str(ROOT/layout["kitchen_usd"]))
    for name, item in layout["objects"].items():
        xf = UsdGeom.Xform.Define(stage, root+"/"+name)
        xf.GetPrim().GetReferences().AddReference(str(ROOT/item["usd_path"]))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
        xf.AddRotateZOp().Set(item["yaw_deg"])
    base = np.asarray(layout["robot_position"])+[0., 0., layout["stand_height"]]
    kin = DroidKinematics(base, layout["robot_yaw_deg"])
    world = PayloadCollisionWorld(stage, kin, payload_name="banana",
        payload_usd=ROOT/layout["objects"]["banana"]["usd_path"], env_root=root,
        clearances={}, execution_clearances={}, default_clearance=.002)
    world.set_phase(phase)
    names = metadata["joint_names"]
    finger_ids = [names.index(name) for name in world.GRIPPER_JOINT_NAMES]
    arm_ids = [names.index(f"panda_joint{i}") for i in range(1, 8)]
    world.gripper_joint_positions = data["q"][index, finger_ids].copy()
    world.gripper_joint_position = float(world.gripper_joint_positions[0])
    world.sync_background(metadata["background_paths"], data["background_rigid_poses"][index])
    for name in metadata["task_object_names"]:
        pose = data["task_object_states"][index, metadata["task_object_names"].index(name), :7]
        world.set_obstacle_pose(name, pose)
        if name == "banana":
            world.payload_pose = pose_matrix(pose)
    attachment = plan.get("measured_planning_attachment")
    attachment_check = None
    if attachment is not None:
        world.attachment = np.asarray(attachment, dtype=float)
        if world.attachment.shape != (4, 4) or not np.isfinite(world.attachment).all():
            raise ValueError("Invalid recorded planning attachment")
        # The close->lift attachment is formed after the final recorded close
        # substep. Its query record, not that row's earlier boolean, is the
        # authoritative candidate geometry. Verify it against measured states.
        measured = np.linalg.inv(pose_matrix(data["tcp_pose_w"][index]))@world.payload_pose
        error = float(np.abs(measured-world.attachment).max())
        if phase == "lift" and error > 1e-6:
            raise ValueError("First lift attachment differs from the actual closed-grasp TCP/object transform")
        attachment_check = dict(query_attachment=world.attachment, measured_tcp_to_payload=measured,
            maximum_matrix_difference=error, physics_row_attachment_active=bool(data["has_planning_attachment"][index]),
            source="Per-plan query geometry; close->lift creation occurs after the last physical row")
    elif data["has_planning_attachment"][index]:
        raise ValueError("Plan discarded an active recorded attachment; diagnose explicitly")
    info = dict(episode=str(episode), phase=phase, physics_index=index,
        physics_step=int(data["physics_step"][index]), original_plan_record=plan,
        actual_all_joint_names=names, actual_all_q=data["q"][index],
        gripper_source="All six actual physical angles retained for FCL; cuRobo original candidate locks use measured finger_joint and original mimic signs",
        robot_base_world=kin.base, background_pose_count=len(metadata["background_paths"]),
        full_world_collision_shapes=len(world.objects),
        planning_attachment_check=attachment_check,
        input_sha256={name: sha(episode/name) for name in ("result.json", "physics_metadata.json", "physics_states.npz")})
    return world, kin, data["q"][index, arm_ids], np.asarray(plan["target_tcp"]), metadata, info


def cpu_sphere_audit(world, kin, positions):
    """CPU URDF FK spheres vs full source FCL world; CUDA result stays pending."""
    import fcl
    import yaml
    asset = ROOT/"assets/curobo/droid"
    cfg = yaml.safe_load((asset/"franka_robotiq_2f_85_zero_curobo.yml").read_text())["robot_cfg"]["kinematics"]
    urdf = ET.parse(asset/"urdf/franka_robotiq_2f_85_zero.urdf").getroot()
    joints = urdf.findall("joint")
    calibration = np.asarray(json.loads((asset/"tcp_calibration.json").read_text())["T_urdfEE_USDtcp"])
    fingers = dict(zip(world.GRIPPER_JOINT_NAMES,
        np.asarray([1., 1., -1., 1., -1., -1.])*world.gripper_joint_position))
    masks = world.contact_policy.cuda_candidate_masks(world.phase, world.attachment is not None)
    payload_cover = world.payload_geometry.sphere_cover(cfg["extra_collision_spheres"]["attached_object"]) if world.attachment is not None else None
    all_spheres, sphere_names, errors, failures, joint_failures = [], [], [], [], []
    for sample, q in enumerate(positions):
        values = dict(zip([f"panda_joint{i}" for i in range(1, 8)], q)) | fingers
        frames, pending = {"panda": kin.base}, joints.copy()
        while pending:
            previous = len(pending)
            for joint in pending.copy():
                parent = joint.find("parent").get("link")
                if parent not in frames:
                    continue
                transform = np.eye(4)
                origin = joint.find("origin")
                if origin is not None:
                    transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    transform[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
                if joint.get("type") in ("revolute", "continuous"):
                    angle = values[joint.get("name")]
                    local = np.eye(4)
                    local[:3, :3] = Rotation.from_rotvec(np.fromstring(joint.find("axis").get("xyz"), sep=" ")*angle).as_matrix()
                    transform = transform@local
                    limit = joint.find("limit")
                    if limit is not None and not float(limit.get("lower"))-1e-5 <= angle <= float(limit.get("upper"))+1e-5:
                        joint_failures.append(dict(sample=sample, joint=joint.get("name"), angle=float(angle),
                            lower=float(limit.get("lower")), upper=float(limit.get("upper"))))
                frames[joint.find("child").get("link")] = frames[parent]@transform
                pending.remove(joint)
            if len(pending) == previous:
                raise ValueError("Unresolved original URDF kinematics")
        errors.append(float(np.linalg.norm((frames["base_link"]@calibration)[:3, 3]-kin.fk(q)[:3, 3])))
        spheres, labels = [], []
        for link in cfg["collision_link_names"]:
            if link == "attached_object":
                if payload_cover is not None:
                    centers, radii, _ = payload_cover
                    pose = frames["base_link"]@calibration@world.attachment
                    spheres.extend(np.c_[centers@pose[:3, :3].T+pose[:3, 3], radii])
                    labels.extend(dict(link=link, local_sphere=i) for i in range(len(centers)))
                continue
            for local_index, item in enumerate(cfg["collision_spheres"][link]):
                center = frames[link][:3, :3]@item["center"]+frames[link][:3, 3]
                radius = float(item["radius"])+.005
                spheres.append(np.r_[center, radius])
                labels.append(dict(link=link, local_sphere=local_index))
        for sphere_values, label in zip(spheres, labels):
            center, radius = sphere_values[:3], float(sphere_values[3])
            sphere = fcl.CollisionObject(fcl.Sphere(radius), fcl.Transform(center))
            bound = np.linalg.norm(np.maximum(np.maximum(world.low-center, center-world.high), 0.), axis=1)
            for index in np.flatnonzero(bound <= radius+1e-6):
                name, obstacle, _, _ = world.objects[index]
                if any(world._matches(name, mask) for mask in masks):
                    continue
                raw = float(fcl.distance(sphere, obstacle, fcl.DistanceRequest(), fcl.DistanceResult()))
                if raw < 0 or fcl.collide(sphere, obstacle, fcl.CollisionRequest(), fcl.CollisionResult()):
                    failures.append(dict(sample=sample, **label, sphere_world_xyzr=sphere_values,
                        obstacle=name, fcl_distance_m=raw))
        all_spheres.append(spheres)
        sphere_names = labels
    return dict(world_intersections=failures, joint_urdf_violations=joint_failures,
        maximum_usd_urdf_tcp_position_difference_m=max(errors), sphere_names=sphere_names,
        sphere_buffer_m=.005, candidate_masks=list(masks),
        attached_source_cover=None if payload_cover is None else payload_cover[2],
        scope="CPU original URDF FK and padded cuRobo sphere radii vs full source FCL world; independent evidence only, not cuRobo CUDA signed-distance/self gate"), np.asarray(all_spheres)


def main():
    global _GPU_LOCK
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase", default="approach")
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument("--recompute-grasp-target", action="store_true",
        help="Approach only: recompute the same source central grasp with current source robot envelopes; preserve old target in report")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite an existing diagnostic")
    args.output.mkdir(parents=True)
    report = dict(accepted=False, executed=False, diagnostic_complete=False, gpu_requested=args.gpu)
    try:
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
        world, kin, start, target, metadata, info = reconstruct(args.episode, args.phase)
        report.update(info)
        if args.recompute_grasp_target:
            if args.phase != "approach" or world.attachment is not None:
                raise ValueError("Target recomputation is limited to unattached source grasp approach")
            from kitchen_safety.extensions.replicator.task_motion import top_down_grasp
            pose = np.r_[world.payload_pose[:3, 3], Rotation.from_matrix(world.payload_pose[:3, :3]).as_quat()]
            target, report["recomputed_grasp_geometry"] = top_down_grasp(pose, world.payload_geometry, world)
            report["recomputed_target"] = target
        candidate = local_cartesian_candidate(kin, start, target, args.phase, metadata["action_dt_s"], .4)
        retained = report["original_plan_record"].get("rejected_candidate_file")
        if retained and not args.recompute_grasp_target:
            archive = args.episode/retained
            if sha(archive) != report["original_plan_record"]["rejected_candidate_sha256"]:
                raise ValueError("Retained rejected candidate hash changed")
            with np.load(archive, allow_pickle=False) as actual:
                if not np.array_equal(actual["start_q"], start) or not np.array_equal(actual["requested_target"], target):
                    raise ValueError("Retained rejected path is bound to a different query")
                difference = float(np.abs(candidate["positions"]-actual["positions"]).max())
                report["retained_rejected_path"] = dict(path=str(archive), sha256=sha(archive),
                    reconstructed_max_joint_difference_rad=difference, source="Exact preserved rejected candidate, not a substitute plan")
                for key in ("positions", "ik_waypoint_positions", "poses", "effective_goal_pose"):
                    candidate[key] = actual[key].copy()
        positions = candidate["positions"]
        np.savez_compressed(args.output/"rejected_candidate.npz", **{key: candidate[key] for key in
            ("positions", "ik_waypoint_positions", "poses", "effective_goal_pose")},
            start_q=start, requested_target=target, measured_gripper_angles=world.gripper_joint_positions)
        report["candidate"] = dict(metadata=candidate["metadata"], samples=len(positions),
            algorithm="Same public _cartesian_vertical IK/retiming/column gates; reconstructed from exact recorded query start",
            archive_sha256=sha(args.output/"rejected_candidate.npz"))
        fcl_rows = []
        for index, q in enumerate(positions):
            valid = world.valid(q)
            fcl_rows.append(dict(sample=index, valid=valid, measurement=world.last_clearance))
        report["full_source_fcl"] = dict(passed=all(row["valid"] for row in fcl_rows), checked=len(fcl_rows), rows=fcl_rows)
        report["cpu_spheres"], spheres = cpu_sphere_audit(world, kin, positions)
        np.savez_compressed(args.output/"cpu_robot_spheres.npz", spheres=spheres)
        if args.gpu:
            import fcntl
            lock = ROOT/"reports/skill_trials/gpu.lock"
            lock.parent.mkdir(parents=True, exist_ok=True)
            _GPU_LOCK = lock.open("a+")
            fcntl.flock(_GPU_LOCK.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
            from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components, sphere_pair_diagnostics
            planner = PayloadCuroboPlanner(world, kin, dt=metadata["action_dt_s"], seed=42, max_joint_speed=.4)
            planner._prepare_path_query((), int(world.attachment is not None), args.phase)
            report["curobo_components"], state = constraint_components(planner, positions)
            report["curobo_sphere_pairs"], spheres = sphere_pair_diagnostics(planner, state, report["curobo_components"])
            np.savez_compressed(args.output/"curobo_robot_spheres_base_frame.npz", spheres=spheres)
        report["diagnostic_complete"] = True
    except Exception as error:
        report["error"] = repr(error)
        raise
    finally:
        json_write(args.output/"diagnostic.json", report)
        print(json.dumps(dict(output=str(args.output), diagnostic_complete=report["diagnostic_complete"],
            gpu_requested=args.gpu, accepted=False)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
