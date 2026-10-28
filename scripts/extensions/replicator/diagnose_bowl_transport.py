#!/usr/bin/env python3
"""Replan bowl transport from retained real lift-end observations, never physics.

CPU checks observation binding, complete measured slip history and target
construction. --gpu additionally generates a new MotionGen trajectory and
checks every original dense CUDA joint/self/world and source FCL sample.
Only the root agent schedules --gpu under the existing shared lock.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_controller_profile import read_profile
from kitchen_safety.extensions.replicator.bowl_transport_binding import (
    lift_motion_evidence, observe_transport_binding, apply_transport_binding, transport_target,
)
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.task_audit_sidecars import ROOT, load_episode, sha


def json_write(path, value):
    def encode(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        if isinstance(item, Path):
            return str(item)
        raise TypeError(type(item).__name__)
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False, default=encode)+"\n")


def recorded_inputs(episode, controller_profile=None):
    """Retained measurements only; this helper never executes recorded actions."""
    meta, data, _ = load_episode(episode)
    result = json.loads((episode/"result.json").read_text())
    profile = read_profile(controller_profile)
    if (meta["task_id"] != profile["task_id"] or result["controller_profile"] != profile or
        meta["source_hashes"]["bowl"] != profile["source_bowl_sha256"]):
        raise ValueError("Original episode/controller/source identity differs")
    world, kin, _ = build_world(meta["source_layout"])
    indices = np.flatnonzero(data["phase"] == "lift")
    if not len(indices):
        raise ValueError("No real lift observations to diagnose")
    index = int(indices[-1])
    payload_index = meta["task_object_names"].index("bowl")
    pose = data["task_object_states"][index, payload_index, :7]
    world.sync_background(meta["background_paths"], data["background_rigid_poses"][index])
    world.payload_pose = pose_matrix(pose)
    world.set_obstacle_pose("bowl", pose)
    world.set_phase("lift")
    fingers = [meta["joint_names"].index(name) for name in world.GRIPPER_JOINT_NAMES]
    arm = [meta["joint_names"].index(f"panda_joint{i}") for i in range(1, 8)]
    world.gripper_joint_positions = data["q"][index, fingers].copy()
    world.gripper_joint_position = float(world.gripper_joint_positions[0])
    world.attachment = np.asarray(result["measured_tcp_to_bowl"]).copy()
    close_end = int(np.flatnonzero(data["phase"] == "close")[-1])
    physics_rows = [{name: data[name][i] for name in ("physics_step", "phase", "tcp_pose_w", "task_object_states")}
                    for i in range(close_end, index+1)]
    motion = lift_motion_evidence(physics_rows, payload_index)
    all_contacts = json.loads((episode/"grasp_contacts.json").read_text())
    contacts = [row for row in all_contacts if row["physics_step"] <= int(data["physics_step"][index])]
    observations = dict(physics_step=int(data["physics_step"][index]), arm_q=data["q"][index, arm].copy(),
        measured_tcp=pose_matrix(data["tcp_pose_w"][index]), measured_payload=world.payload_pose.copy(),
        raw_contact_rows=contacts, lift_motion=motion)
    return world, kin, profile, observations, meta, data


def run(args):
    world, kin, profile, observed, meta, data = recorded_inputs(args.episode, args.controller_profile)
    context = observe_transport_binding(world, kin, profile, **observed)
    previous = world.attachment.copy()
    binding = apply_transport_binding(world, profile, context)
    world.set_phase("transport")
    target, info = transport_target(world, profile, context)
    sources = [Path(__file__), ROOT/"kitchen_safety/extensions/replicator/bowl_transport_binding.py",
        ROOT/"kitchen_safety/extensions/replicator/bowl_transport_curobo.py",
        ROOT/"kitchen_safety/extensions/replicator/bowl_source_support_curobo.py",
        ROOT/"kitchen_safety/extensions/replicator/task_curobo.py", ROOT/"kitchen_safety/curobo_backend.py",
        ROOT/"kitchen_safety/extensions/replicator/task_plan_diagnostics.py"]
    result = dict(accepted=False, physical_execution=False, execution_authorized=False, task_success=False,
        gpu_requested=args.gpu, episode=str(args.episode), observation_binding=binding,
        full_lift_attachment_motion=observed["lift_motion"], target_tcp=target,
        source_target=info, previous_close_attachment=previous,
        target_rotation_from_actual_tcp_deg=float(np.rad2deg(Rotation.from_matrix(
            target[:3, :3]@observed["measured_tcp"][:3, :3].T).magnitude())),
        all_actual_six_finger_joints=dict(zip(world.GRIPPER_JOINT_NAMES, context.six_finger_q)),
        input_sha256={name: sha(args.episode/name) for name in ("physics_metadata.json", "physics_states.npz", "result.json", "grasp_contacts.json")},
        source_sha256={str(path.relative_to(ROOT)): sha(path) for path in sources},
        limits=["Ten loaded samples do not demonstrate a rigid grasp", "Observed close-to-lift slip is retained",
                "Fresh transport physics, tracking, sustained grip, lower/release and original success remain unverified"],
        cpu_observation_and_target_passed=True, fresh_path_validated=False)
    json_write(args.output/"observation_binding.json", result)
    np.savez_compressed(args.output/"measured_transport_start.npz", q=np.asarray(context.arm_q),
        actual_six_fingers=np.asarray(context.six_finger_q), actual_tcp=observed["measured_tcp"],
        actual_payload=observed["measured_payload"], attachment=world.attachment,
        previous_attachment=previous, target_tcp=target)
    if not args.gpu:
        return result
    from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
    from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components, sphere_pair_diagnostics
    planner = BowlTransportCuroboPlanner(world, kin, dt=meta["action_dt_s"], seed=args.seed, max_joint_speed=.4)
    planner.bind_transport(profile, context)
    try:
        planner._prepare_path_query((), 1, "transport")
        start, state = constraint_components(planner, np.asarray([context.arm_q]))
        result["original_start_cuda_components"] = start
        if not start["valid"]:
            result["start_failed_sphere_pairs"], values = sphere_pair_diagnostics(planner, state, start)
            np.savez_compressed(args.output/"start_failed_spheres.npz", spheres=values)
        plan = planner.plan_pose(np.asarray(context.arm_q), target, grip_binary=1, phase="transport")
        path = np.asarray(plan["positions"])
        np.savez_compressed(args.output/"fresh_transport_candidate.npz", positions=path, target_tcp=target,
            actual_six_fingers=np.asarray(context.six_finger_q), measured_attachment=world.attachment)
        result["fresh_plan_metadata"] = plan["metadata"]
        validation = planner.validate_path(path, grip_binary=1, phase="transport", resolution=.02)
        result["fresh_path_validation"] = validation
        result["fresh_path_validated"] = validation["valid"]
        dense = planner.last_transport_dense_samples
        np.savez_compressed(args.output/"fresh_transport_dense.npz", positions=dense,
            measured_attachment=world.attachment, actual_six_fingers=np.asarray(context.six_finger_q))
        if not validation["original_cuda_components"]["valid"]:
            components, state = constraint_components(planner, dense)
            result["failed_sphere_pairs"], values = sphere_pair_diagnostics(planner, state, components)
            np.savez_compressed(args.output/"failed_spheres.npz", spheres=values)
    except Exception as error:
        result.update(error=repr(error), fresh_path_validated=False)
    result["payload_geometry"] = getattr(planner, "payload_geometry_report", None)
    result["source_sha256_end"] = {str(path.relative_to(ROOT)): sha(path) for path in sources}
    if result["source_sha256_end"] != result["source_sha256"]:
        raise RuntimeError("Execution source changed during independent diagnostic")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--controller-profile", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--gpu", action="store_true", help="Root-scheduled original CUDA planner only; no Kit or physics")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock_path = ROOT/"reports/skill_trials/gpu.lock"
            if ROOT != Path("/workspace") or not lock_path.exists():
                raise RuntimeError("CUDA requires the verified workspace and existing shared lock")
            with lock_path.open("a+") as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = run(args)
        else:
            result = run(args)
    except Exception as error:
        json_write(args.output/"failure.json", dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json", result)
    print(json.dumps(dict(output=str(args.output), accepted=False, physical_execution=False,
        cpu_observation_and_target_passed=result["cpu_observation_and_target_passed"],
        fresh_path_validated=result["fresh_path_validated"], error=result.get("error"))), flush=True)
    return 0 if not args.gpu or result["fresh_path_validated"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
