#!/usr/bin/env python3
"""Conditional G banana paths for a source-legal translated layout candidate.

All poses derived from an older episode are labeled predictions. No environment
is stepped, no canonical file is changed, and no episode can be accepted here.
The original runtime and the earlier fixed-layout diagnostic stay untouched.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.diagnose_banana_grasp_candidate import fcl_path, partition_world
from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts
from scripts.extensions.replicator.diagnose_task_plan import reconstruct, cpu_sphere_audit
from scripts.extensions.replicator.search_banana_grasp_envelope_cpu import GripperSphereModel, sphere_clearance
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.task_collect import json_write
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate


def run(args):
    source = json.loads(args.grasp_search.read_text())
    selected = source["candidates"][source["selected_index"]]
    if not selected["passed"]:
        raise ValueError("Original closed-grasp candidate did not pass")
    layout_search = json.loads(args.layout_search.read_text())
    if not layout_search["candidate_found"]:
        raise ValueError("No source-legal payload candidate found")
    layout_path = Path(layout_search["candidate_layout"])
    if sha(layout_path) != layout_search["candidate_layout_sha256"]:
        raise ValueError("Layout candidate changed since search")
    layout = json.loads(layout_path.read_text())
    relations = check_layout(layout)
    if not relations["initial_source_relations_passed"]:
        raise ValueError("Original source relations reject this candidate")
    world, kin, observed_start, _, metadata, reconstruction = reconstruct(args.episode, "approach")
    if source["reconstruction"]["input_sha256"] != reconstruction["input_sha256"]:
        raise ValueError("Grasp candidate refers to different physical evidence")
    original = metadata["source_layout"]
    delta = np.asarray(layout["objects"]["banana"]["position"])-original["objects"]["banana"]["position"]
    expected = deepcopy(original)
    expected["objects"]["banana"]["position"] = layout["objects"]["banana"]["position"]
    expected["objects"]["bowl"]["position"][2] = layout["objects"]["bowl"]["position"][2]
    if expected != layout or abs(delta[1]) > 1e-12 or delta[0] < 0. or delta[0] > .030000001:
        raise ValueError("Candidate changed fields outside the declared bounded banana X/initial height search")
    world.payload_pose[:3, 3] += [delta[0], 0., 0.]
    target = np.asarray(selected["target_tcp"]).copy()
    target[0, 3] += delta[0]
    pregrasp = target.copy()
    pregrasp[2, 3] += .125
    # This endpoint is a conditional candidate, not the old measured start and
    # not a fabricated physical transition between the two different layouts.
    start = kin.ik(pregrasp, observed_start, attempts=1)
    pregrasp_passed = world.valid(start)
    report = dict(accepted=False, execution_ready=False, physical_execution=False, canonical_changed=False,
        source_layout=str(layout_path), source_layout_sha256=sha(layout_path), original_relations=relations,
        reconstruction=reconstruction, predicted_settled_banana_pose=world.payload_pose.copy(),
        initial_pose_delta_m=delta, target_tcp=target,
        prediction_rule="Translate old measured settled banana by declared X only; do not raise a settled pose by the initial drop-height correction. Old background/bowl poses are conditional proxies pending NEW neutral.",
        pregrasp=dict(q=start, target=pregrasp, source_fcl_passed=pregrasp_passed,
            source_violation=deepcopy(world.last_clearance),
            physically_executed=False, initial_to_pregrasp_path_verified=False),
        input_sha256={str(args.grasp_search):sha(args.grasp_search), str(args.layout_search):sha(args.layout_search)})
    approach = local_cartesian_candidate(kin, start, target, "approach", metadata["action_dt_s"], .4)
    report["approach"] = dict(metadata=approach["metadata"], full_source_fcl=fcl_path(world, approach["positions"]))
    report["approach"]["cpu_spheres"], _ = cpu_sphere_audit(world, kin, approach["positions"])
    q = approach["positions"][-1]
    tcp = kin.fk(q)
    signs = np.array([1., 1., -1., 1., -1., -1.])
    angles = np.arange(0., selected["first_bilateral_angle_rad"]+.0025, .005)
    model = GripperSphereModel()
    meshes, report["finger_source_provenance"] = source_finger_meshes(world)
    contacts_model = SourceRegionContacts(world.payload_geometry,
        source["grasp_geometry"]["contact_region_local_m"], meshes)
    report["source_region_partition"] = contacts_model.provenance
    world.set_phase("close")
    closing = []
    for angle in angles:
        world.gripper_joint_positions = signs*angle
        world.gripper_joint_position = float(angle)
        valid = world.valid(q)
        spheres = sphere_clearance(model.at(tcp, dict(zip(world.GRIPPER_JOINT_NAMES, signs*angle))), world)
        contacts = contacts_model.at(world, kin, q)
        closing.append(dict(angle_rad=float(angle), source_fcl_passed=valid,
            source_violation=deepcopy(world.last_clearance), original_gripper_spheres=spheres,
            source_contacts=contacts, passed=bool(valid and spheres["passed"])))
    report["close"] = dict(rows=closing, passed=bool(all(row["passed"] for row in closing)
        and closing[-1]["source_contacts"]["bilateral_all_central"]),
        limit="Kinematic source contacts only; actual object motion, contact force and friction remain unknown.")
    world.attachment = np.linalg.inv(tcp)@world.payload_pose
    world.set_phase("lift")
    goal = tcp.copy()
    goal[2, 3] += .15
    lift = local_cartesian_candidate(kin, q, goal, "lift", metadata["action_dt_s"], .4)
    report["hypothetical_attachment"] = world.attachment.copy()
    report["lift"] = dict(metadata=lift["metadata"], full_source_fcl=fcl_path(world, lift["positions"]))
    report["lift"]["cpu_spheres"], _ = cpu_sphere_audit(world, kin, lift["positions"])
    np.savez_compressed(args.output/"hypothetical_paths.npz", pregrasp_q=start, approach=approach["positions"],
        close_angles_rad=angles, lift=lift["positions"], hypothetical_attachment=world.attachment,
        predicted_settled_banana=world.payload_pose)
    if args.gpu:
        import fcntl
        from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components, sphere_pair_diagnostics
        with (ROOT/"reports/skill_trials/gpu.lock").open("a+") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            planner = PayloadCuroboPlanner(world, kin, dt=metadata["action_dt_s"], seed=42, max_joint_speed=.4)
            attachment = world.attachment.copy()
            world.attachment = None
            world.gripper_joint_positions = np.zeros(6)
            world.gripper_joint_position = 0.
            planner._prepare_path_query((), 0, "approach")
            report["approach"]["gpu_components"], _ = constraint_components(planner, approach["positions"])
            world.attachment = attachment
            world.gripper_joint_positions = signs*angles[-1]
            world.gripper_joint_position = float(angles[-1])
            planner._prepare_path_query((), 1, "lift")
            report["lift"]["gpu_components"], state = constraint_components(planner, lift["positions"])
            report["lift"]["gpu_sphere_pairs"], _ = sphere_pair_diagnostics(planner, state, report["lift"]["gpu_components"])
            report["lift"]["world_pair_partition"] = partition_world(planner, state, report["lift"]["gpu_components"])
    report["code_sha256"] = {str(Path(__file__)):sha(Path(__file__)),
        **{str(ROOT/"scripts/extensions/replicator"/name):sha(ROOT/"scripts/extensions/replicator"/name)
           for name in ("diagnose_banana_grasp_candidate.py", "banana_region_contacts_cpu.py")}}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--grasp-search", type=Path, required=True)
    parser.add_argument("--layout-search", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gpu", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite diagnostic evidence")
    args.output.mkdir(parents=True)
    try:
        report = run(args)
    except Exception as error:
        json_write(args.output/"failure.json", dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json", report)
    print(json.dumps(dict(output=str(args.output), close_passed=report["close"]["passed"], accepted=False)))


if __name__ == "__main__":
    main()
