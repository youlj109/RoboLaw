#!/usr/bin/env python3
"""Independent source-pair runtime-module diagnostic; never step physics.

Actual mode binds the failed attempt03 raw observations and must still reject
its original robot/counter sphere collisions. Candidate mode checks predicted
new-layout paths with no actual-state binding, so valid must remain false even
when candidate_geometry_valid is true. GPU is opt-in and root-scheduled.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.diagnose_task_plan import reconstruct
from scripts.extensions.replicator.task_audit_sidecars import ROOT, load_episode, sha
from scripts.extensions.replicator.task_collect import json_write
from kitchen_safety.extensions.replicator.source_support_query import (
    bind_observed_departure, binding_check, source_departure_geometry, audit_source_departure,
)
from kitchen_safety.extensions.replicator.task_motion import pose_matrix


def run(args):
    context = None
    if args.candidate_report is None:
        world,kin,start,target,metadata,reconstruction = reconstruct(args.episode,"lift")
        _,data,_ = load_episode(args.episode)
        rows = json.loads((args.episode/"grasp_contacts.json").read_text())
        context = bind_observed_departure(world,kin,physics_step=reconstruction["physics_step"],arm_q=start,
            measured_tcp=pose_matrix(data["tcp_pose_w"][reconstruction["physics_index"]]),
            measured_payload=world.payload_pose,raw_contact_rows=rows)
        path = args.episode/reconstruction["original_plan_record"]["rejected_candidate_file"]
        if sha(path)!=reconstruction["original_plan_record"]["rejected_candidate_sha256"]:
            raise ValueError("Preserved actual rejected candidate changed")
        with np.load(path,allow_pickle=False) as archive:
            positions = archive["positions"].copy()
        mode = "actual_original_rejected_lift"
        inputs = {str(path):sha(path),str(args.episode/"grasp_contacts.json"):sha(args.episode/"grasp_contacts.json")}
    else:
        candidate = json.loads(args.candidate_report.read_text())
        if candidate["physical_execution"] or candidate["accepted"]:
            raise ValueError("This mode requires explicitly hypothetical candidate data")
        world,kin,_,_,metadata,reconstruction = reconstruct(args.episode,"approach")
        if candidate["reconstruction"]["input_sha256"]!=reconstruction["input_sha256"]:
            raise ValueError("Candidate and source recorded episode differ")
        world.payload_pose = np.asarray(candidate["predicted_settled_banana_pose"])
        world.attachment = np.asarray(candidate["hypothetical_attachment"])
        angle = candidate["close"]["rows"][-1]["angle_rad"]
        world.gripper_joint_position = float(angle)
        world.gripper_joint_positions = np.array([1.,1.,-1.,1.,-1.,-1.])*angle
        world.set_phase("lift")
        path = args.candidate_report.parent/"hypothetical_paths.npz"
        with np.load(path,allow_pickle=False) as archive:
            positions = archive["lift"].copy()
            if not np.array_equal(archive["hypothetical_attachment"],world.attachment):
                raise ValueError("Hypothetical attachment report and archive differ")
        mode = "hypothetical_new_layout_unbound"
        inputs = {str(path):sha(path),str(args.candidate_report):sha(args.candidate_report)}
    report = dict(accepted=False,physical_execution=False,mode=mode,reconstruction=reconstruction,input_sha256=inputs,
        actual_state_binding=binding_check(world,positions,context),
        source_support_geometry=source_departure_geometry(world,kin,positions),gpu_requested=args.gpu)
    np.savez_compressed(args.output/"diagnosed_path.npz",positions=positions,planning_attachment=world.attachment,
        gripper_joint_positions=world.gripper_joint_positions)
    if args.gpu:
        import fcntl
        from kitchen_safety.extensions.replicator.source_support_curobo import SourceSupportCuroboPlanner
        with (ROOT/"reports/skill_trials/gpu.lock").open("a+") as lock:
            fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            planner = SourceSupportCuroboPlanner(world,kin,dt=metadata["action_dt_s"],seed=42,max_joint_speed=.4)
            planner._prepare_path_query((),1,"lift")
            report["runtime_query_diagnostic"] = audit_source_departure(planner,positions,context)
            if args.candidate_report is not None and report["runtime_query_diagnostic"]["valid"]:
                raise AssertionError("An unbound hypothetical candidate was incorrectly authorized")
            if args.candidate_report is None and report["runtime_query_diagnostic"]["valid"]:
                raise AssertionError("Known actual rejected robot/counter path unexpectedly authorized")
    report["runtime_code_sha256"] = {str(ROOT/"kitchen_safety/extensions/replicator"/name):
        sha(ROOT/"kitchen_safety/extensions/replicator"/name) for name in
        ("source_support_query.py","source_support_curobo.py","task_curobo.py","task_collision.py","task_plan_diagnostics.py")}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode",type=Path,required=True)
    parser.add_argument("--candidate-report",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--gpu",action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite diagnostic evidence")
    args.output.mkdir(parents=True)
    try:
        report = run(args)
    except Exception as error:
        json_write(args.output/"failure.json",dict(accepted=False,error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json",report)
    print(json.dumps(dict(output=str(args.output),mode=report["mode"],binding=report["actual_state_binding"]["passed"],
        source_pair=report["source_support_geometry"]["passed"],gpu_requested=args.gpu,accepted=False)))


if __name__=="__main__":
    main()
