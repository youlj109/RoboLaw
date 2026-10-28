#!/usr/bin/env python3
"""Prepared G bowl pregrasp/transfer candidate diagnostics, never physics.

CPU mode records source endpoint states only. --gpu additionally invokes real
cuRobo MotionGen and dense original CUDA + full source FCL validation. Every
attachment is explicitly hypothetical; this cannot authorize a collected trial
or replace actual post-grasp measurements in bowl_collect.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.diagnose_bowl_plan import load_candidates
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
from scripts.extensions.replicator.bowl_collect import json_write
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha


def run(args):
    layout, profile, paths, attachment, angle, inputs = load_candidates(args)
    report = dict(task_id=layout["task_id"], accepted=False, physical_execution=False, gpu_requested=args.gpu,
        actual_state_binding=False, execution_authorized=False, input_sha256=inputs, phases={})
    for phase in args.phases:
        world, kin, _ = build_world(layout)
        world.set_phase(phase)
        if phase == "pregrasp":
            initial, source = source_initial_q()
            start = np.array([initial[f"panda_joint{i}"] for i in range(1, 8)])
            goal_q = paths["approach"][0]
            world.attachment = None
            world.gripper_joint_position = 0.
        else:
            start, goal_q = paths["lift"][-1], paths["lower"][0]
            world.attachment = attachment.copy()
            world.payload_pose = kin.fk(start)@world.attachment
            world.gripper_joint_position = angle
        world.gripper_joint_positions = np.array([1., 1., -1., 1., -1., -1.])*world.gripper_joint_position
        target = kin.fk(goal_q)
        row = dict(start_q=start, prepared_goal_q=goal_q, target_tcp=target,
            hypothetical_attachment=world.attachment, commanded_gripper=int(phase == "transport"),
            source_endpoints=[], path_generated=False, candidate_path_validated=False)
        for name, q in (("start", start), ("prepared_goal", goal_q)):
            valid = world.valid(q)
            row["source_endpoints"].append(dict(name=name, source_fcl_passed=valid, violation=deepcopy(world.last_clearance)))
        if args.gpu:
            from kitchen_safety.extensions.replicator.bowl_source_support_curobo import BowlSourceSupportCuroboPlanner
            planner = None
            try:
                planner = BowlSourceSupportCuroboPlanner(world, kin, dt=1/15, seed=42, max_joint_speed=.4)
                plan = planner.plan_pose(start, target, grip_binary=int(phase == "transport"), phase=phase)
                positions = np.asarray(plan["positions"])
                path = args.output/(phase+"_candidate.npz")
                np.savez_compressed(path, positions=positions, start_q=start, target_tcp=target,
                    hypothetical_attachment=np.eye(4) if world.attachment is None else world.attachment)
                row.update(path_generated=True, candidate_sha256=sha(path), planning=plan["metadata"],
                    candidate_contact_mask=planner.candidate_contact_mask)
                check = planner.validate_path(positions, grip_binary=int(phase == "transport"), phase=phase, resolution=.02)
                row.update(dense_validation=check, candidate_path_validated=bool(check["valid"]))
            except Exception as error:
                row["error"] = repr(error)
                if planner is not None:
                    row["last_plan"] = planner.last_plan
            finally:
                del planner
        report["phases"][phase] = row
        json_write(args.output/(phase+".json"), row)
    report["source_sha256"] = {str(path.relative_to(ROOT)): sha(path) for path in (Path(__file__),
        ROOT/"scripts/extensions/replicator/diagnose_bowl_plan.py",
        ROOT/"kitchen_safety/extensions/replicator/bowl_source_support_curobo.py",
        ROOT/"kitchen_safety/extensions/replicator/bowl_source_support_query.py",
        ROOT/"kitchen_safety/extensions/replicator/bowl_destination_query.py")}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, default=ROOT/"configs/extensions/replicator/g_shape_bowl_sink_source_layout.json")
    parser.add_argument("--controller-profile", type=Path, default=ROOT/"configs/extensions/replicator/g_shape_bowl_sink_controller.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phases", nargs="+", choices=["pregrasp", "transport"], default=["pregrasp", "transport"])
    parser.add_argument("--gpu", action="store_true")
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
                report = run(args)
        else:
            report = run(args)
    except Exception as error:
        json_write(args.output/"failure.json", dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json", report)
    print(json.dumps(dict(output=str(args.output), gpu_requested=args.gpu, accepted=False,
        phases={name:dict(path_generated=row["path_generated"], candidate_path_validated=row["candidate_path_validated"],
            error=row.get("error")) for name,row in report["phases"].items()})))


if __name__ == "__main__":
    main()
