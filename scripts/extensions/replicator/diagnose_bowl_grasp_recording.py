#!/usr/bin/env python3
"""CPU diagnosis/candidate search from a real rejected bowl close; no simulator.

Preserves original object poses, source rim region, all source collision meshes
and safety distances. Every candidate is hypothetical and unaccepted. Recorded
finger/body/contact observations are never synthesized into a physical result.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_grasp
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts
from scripts.extensions.replicator.search_banana_grasp_envelope_cpu import GripperSphereModel, sphere_clearance
from scripts.extensions.replicator.bowl_collect import json_write
from scripts.extensions.replicator.task_audit_sidecars import ROOT, load_episode, sha


def synchronize(world, meta, data, index):
    world.sync_background(meta["background_paths"], data["background_rigid_poses"][index])
    pose = data["task_object_states"][index, meta["task_object_names"].index("bowl"), :7]
    world.payload_pose = pose_matrix(pose)
    world.set_obstacle_pose("bowl", pose)
    world.attachment = None
    names = meta["joint_names"]
    world.gripper_joint_positions = data["q"][index, [names.index(name) for name in world.GRIPPER_JOINT_NAMES]].copy()
    world.gripper_joint_position = float(world.gripper_joint_positions[0])
    return pose


def run(args):
    meta, data, trajectory = load_episode(args.episode)
    result = json.loads((args.episode/"result.json").read_text())
    contacts = json.loads((args.episode/"grasp_contacts.json").read_text())
    if meta["task_id"] != "kitchen_bench_replicator_g_shape_bowl_sink" or result["physical_trial_passed"]:
        raise ValueError("Require the preserved unsuccessful original bowl episode")
    arm = [meta["joint_names"].index(f"panda_joint{i}") for i in range(1, 8)]
    world, kin, _ = build_world(meta["source_layout"])
    close = np.flatnonzero(data["phase"] == "close")
    approach_end = int(np.flatnonzero(data["phase"] == "approach")[-1])
    initial = int(np.flatnonzero(data["phase"] == "initial_settle")[-1])
    final = int(close[-1])
    actual_pose = synchronize(world, meta, data, final)
    region = np.asarray(result["grasp_target"]["contact_region_local_m"])
    meshes, source_meshes = source_finger_meshes(world)
    partition = SourceRegionContacts(world.payload_geometry, region, meshes)
    world.set_phase("close")
    q = data["q"][final, arm]
    source_contacts = partition.at(world, kin, q)
    actual_fcl = world.valid(q, mode="execution", gripper_q=world.gripper_joint_positions, object_pose=actual_pose)
    summary = []
    for index in close[-10:]:
        row = contacts[index]
        fingers = {}
        for name, item in row["fingers"].items():
            force, points, gaps = np.asarray(item["normal_forces_n"]), np.asarray(item["points_object_local_m"]).reshape(-1, 3), np.asarray(item["separations_m"])
            loaded = force > .01
            inside = np.all((points >= region[0]) & (points <= region[1]), axis=1)
            fingers[name] = dict(total_normal_force_n=float(force.sum()), loaded_count=int(loaded.sum()),
                loaded_outside_region=int((loaded & ~inside).sum()), loaded_points_local_m=points[loaded],
                loaded_gaps_m=gaps[loaded], region_violation_m=np.maximum(np.maximum(region[0]-points[loaded], points[loaded]-region[1]), 0.))
        summary.append(dict(physics_step=int(row["physics_step"]), fingers=fingers))
    def motion(a, b):
        return dict(tcp_translation_m=(data["tcp_pose_w"][b, :3]-data["tcp_pose_w"][a, :3]),
            bowl_translation_m=(data["task_object_states"][b, 0, :3]-data["task_object_states"][a, 0, :3]),
            tcp_minus_bowl_change_m=(data["tcp_pose_w"][b, :3]-data["task_object_states"][b, 0, :3])-(data["tcp_pose_w"][a, :3]-data["task_object_states"][a, 0, :3]))
    report = dict(accepted=False, executed=False, task_success=False, physical_candidate_verified=False,
        episode=str(args.episode), input_sha256={name: sha(args.episode/name) for name in
            ("result.json", "physics_metadata.json", "physics_states.npz", "trajectory.npz", "grasp_contacts.json")},
        phases={name: int(np.sum(data["phase"] == name)) for name in np.unique(data["phase"])},
        observed_close_motion=motion(approach_end, final), actual_closed_six_joints=dict(zip(world.GRIPPER_JOINT_NAMES, world.gripper_joint_positions)),
        raw_last10=summary, original_contact_region_m=region, source_finger_meshes=source_meshes,
        actual_closed_source_region_partition=source_contacts, actual_closed_source_fcl=actual_fcl,
        source_fcl_failure=deepcopy(world.last_clearance), source_region_provenance=partition.provenance,
        explanation="Physical bilateral forces exist, but the loaded left inner finger lies below the declared upper-wall/rim region. No lift was executed. Candidate geometry below cannot prove real force, closure stability or successful lift.")
    json_write(args.output/"actual_contact_diagnostic.json", report)
    # Candidate elevations are declared before any scan. Source object/robot
    # layout and the measured region never change. No inward/outward correction
    # or hold duration is silently tried in physical simulation.
    offsets = [0., .0025, .004, .006, .008, .010]
    angles = np.arange(158)*.005
    declaration = dict(target_raise_m=offsets, angle_step_rad=.005, closure_angles_rad=angles,
        source_objects="Exact recorded settled bowl and background; no object pose adjustment",
        selection="Evaluate all six predeclared upward offsets; keep original region and choose only after actual-state/uncertainty review",
        observed_load_deflection_probe="At each hypothetical target also test the recorded final six finger angles and measured close displacement; this is not a physical prediction or success proof")
    json_write(args.output/"candidate_declaration.json", declaration)
    actual_final_fingers = world.gripper_joint_positions.copy()
    actual_tcp_delta = data["tcp_pose_w"][final, :3]-data["tcp_pose_w"][approach_end, :3]
    initial_pose = synchronize(world, meta, data, initial)
    target, grasp = bowl_rim_grasp(initial_pose, world.payload_geometry, world)
    sphere_model = GripperSphereModel()
    candidates = []
    for candidate_index, raise_m in enumerate(offsets):
        synchronize(world, meta, data, initial)
        world.gripper_joint_positions, world.gripper_joint_position = np.zeros(6), 0.
        goal = target.copy(); goal[2, 3] += raise_m
        above = goal.copy(); above[2, 3] += .125
        row = dict(index=candidate_index, target_raise_m=raise_m, target_tcp=goal, source_grasp=grasp, passed=False)
        try:
            pregrasp = kin.ik(above, data["q"][approach_end, arm])
            world.set_phase("approach")
            path = local_cartesian_candidate(kin, pregrasp, goal, "approach", meta["action_dt_s"], .4)
            path_rows = []
            for i, sample in enumerate(path["positions"]):
                valid = world.valid(sample)
                path_rows.append(dict(sample=i, source_fcl_passed=valid, failure=deepcopy(world.last_clearance)))
            qgoal = path["positions"][-1]
            world.set_phase("close")
            closure, selected = [], None
            for angle in angles:
                world.gripper_joint_positions = np.array([1., 1., -1., 1., -1., -1.])*angle
                world.gripper_joint_position = float(angle)
                contact = partition.at(world, kin, qgoal)
                valid = world.valid(qgoal)
                spheres = sphere_clearance(sphere_model.at(kin.fk(qgoal), dict(zip(world.GRIPPER_JOINT_NAMES, world.gripper_joint_positions))), world)
                bilateral = all(item["inside"]+item["outside"] > 0 for item in contact["fingers"].values())
                closure.append(dict(angle_rad=float(angle), exact_source_contacts=contact, full_source_fcl_passed=valid,
                    failure=deepcopy(world.last_clearance), original_gripper_spheres=spheres, bilateral=bilateral))
                if bilateral:
                    selected = len(closure)-1
                    break
            closure_passed = (selected is not None and closure[selected]["exact_source_contacts"]["bilateral_all_central"] and
                all(item["full_source_fcl_passed"] and item["original_gripper_spheres"]["passed"] for item in closure))
            row.update(approach=dict(passed=all(item["source_fcl_passed"] for item in path_rows), rows=path_rows),
                closure=dict(passed=closure_passed, selected_index=selected, rows=closure))
            # Check the same measured final object and real six independent
            # finger angles under a translated version of the observed load
            # displacement. This is an explicitly hypothetical stress sample.
            synchronize(world, meta, data, final)
            world.set_phase("close")
            deflected = kin.fk(qgoal); deflected[:3, 3] += actual_tcp_delta
            qdeflect = kin.ik(deflected, qgoal)
            contact = partition.at(world, kin, qdeflect)
            valid = world.valid(qdeflect)
            row["recorded_load_deflection_probe"] = dict(exact_source_contacts=contact, full_source_fcl_passed=valid,
                source_fcl_failure=deepcopy(world.last_clearance), six_finger_q=actual_final_fingers,
                target_tcp=deflected, q=qdeflect, physical_prediction=False)
            row["passed"] = bool(row["approach"]["passed"] and closure_passed)
            np.savez_compressed(args.output/f"candidate_{candidate_index:02d}_paths.npz", approach=path["positions"],
                target_tcp=goal, stress_q=qdeflect, stress_tcp=deflected)
        except Exception as error:
            row["error"] = repr(error)
        candidates.append(row)
        json_write(args.output/f"candidate_{candidate_index:02d}.json", row)
        print(json.dumps(dict(candidate=candidate_index, raise_m=raise_m, nominal_geometry_passed=row["passed"],
            stress_contacts=row.get("recorded_load_deflection_probe", {}).get("exact_source_contacts"), error=row.get("error"))), flush=True)
    report.update(declaration=declaration, candidates=candidates,
        runtime_changes_made=False, source_sha256={str(p.relative_to(ROOT)): sha(p) for p in (Path(__file__).resolve(),
            ROOT/"kitchen_safety/extensions/replicator/bowl_motion.py", ROOT/"kitchen_safety/extensions/replicator/bowl_collision.py",
            ROOT/"scripts/extensions/replicator/banana_region_contacts_cpu.py")})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        report = run(args)
    except Exception as error:
        json_write(args.output/"failure.json", dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json", report)


if __name__ == "__main__":
    main()
