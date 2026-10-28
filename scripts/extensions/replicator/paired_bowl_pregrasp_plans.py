#!/usr/bin/env python3
"""Seed-paired human avoidance for the original free-bowl PREGRASP phase.

Only this phase is compared, because the chosen human layout's retained risk
occurs during pregrasp. The transport pairing entry remains frozen/unchanged.
No Kit, physical action, attachment, endpoint splice or hypothetical success.
"""
import argparse
from copy import deepcopy
import gc
import json
from pathlib import Path
import sys
import time

import numpy as np

from kitchen_safety.extensions.replicator.bowl_paired_query import (
    ROOT, SEEDS, BowlPairedQuery, densify, digest, inventory, pair_gates, path_metrics,
    require, sha, summarize_geometry, validate_seeds, write,
)
from scripts.extensions.replicator.paired_bowl_safety_plans import (
    execution_inputs, frozen_input_check, pair_difference, source_asset_inputs,
)


def pregrasp_boundary(result, data):
    plans = [(i, p) for i, p in enumerate(result["plans"], 1) if p["phase"] == "pregrasp"]
    require(len(plans) == 1, "Exactly one original pregrasp query required")
    number, plan = plans[0]
    require(plan["status"] == "executed" and plan["measured_planning_attachment"] is None,
            "Pregrasp must be executed with a genuinely free original bowl")
    indices = np.flatnonzero(data["physics_step"] == plan["start_physics_step"])
    require(len(indices) == 1, "Unique measured pregrasp start required")
    index = int(indices[0])
    require(str(data["phase"][index]) == "initial_settle" and index + 1 < len(data["phase"]) and
            str(data["phase"][index + 1]) == "pregrasp", "Use actual settled boundary, never first pregrasp state")
    require(plan["metadata"]["transport_object_padding_m"] == plan["validation"]["transport_object_padding_m"] == 0.,
            "Original pregrasp has no carried-object padding")
    return index, number, plan


def build(query, index, on):
    from kitchen_safety.extensions.replicator.bowl_safety_cpu import build_world
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    world, kin, _ = build_world(query.layout if on else query.source_layout, query.policy)
    data, meta = query.data, query.meta
    world.sync_background(meta["background_paths"], data["background_rigid_poses"][index])
    for name, pose in zip(meta["background_paths"], data["background_rigid_poses"][index]):
        if any(world._matches(row[0], name) for row in world.objects):
            require(name in world._background_poses and np.array_equal(world._background_poses[name], pose),
                    "Actual background collision body was not synchronized")
    arm = [meta["joint_names"].index(f"panda_joint{i}") for i in range(1,8)]
    fingers = [meta["joint_names"].index(name) for name in world.GRIPPER_JOINT_NAMES]
    q = data["q"][index, arm].copy()
    world.gripper_joint_positions = data["q"][index, fingers].copy()
    world.gripper_joint_position = float(world.gripper_joint_positions[0])
    bowl = data["task_object_states"][index, meta["task_object_names"].index("bowl"), :7].copy()
    world.payload_pose = pose_matrix(bowl)
    world.set_obstacle_pose("bowl", bowl)
    world.attachment = None
    world.set_phase("pregrasp")
    if on:
        world.set_obstacle_pose(query.policy["hazard_name"], query.hazard_pose)
    require(np.max(np.abs(world.gripper_joint_positions)) <= 1e-3,
            "This original open-gripper pregrasp baseline does not have measured open fingers")
    return world, kin, q


def geometry(world, robot, q, hazard_name):
    import fcl
    from kitchen_safety.geometry_audit import minimum_distance
    require(world.attachment is None and world.phase == "pregrasp", "No carried-payload model in pregrasp")
    grip = dict(zip(world.GRIPPER_JOINT_NAMES, world.gripper_joint_positions))
    pose = world.payload_pose
    points = world.payload_geometry.vertices @ pose[:3,:3].T + pose[:3,3]
    bowl = [("free_source_bowl_triangles",fcl.CollisionObject(world.payload_triangle_model,
        fcl.Transform(pose[:3,:3],pose[:3,3])),points.min(0),points.max(0))]
    hazards = [row for row in world.objects if world._matches(row[0],hazard_name)]
    require(hazards, "Hazard geometry cannot be empty")
    source = robot.shapes(q,grip)
    hulls = [row for row in world.shapes(q,grip,pose) if row[0].startswith(("robot_hull/","gripper/","payload/"))]
    required=world.execution_clearances[hazard_name]
    result=dict(source_mesh=minimum_distance(source+bowl,hazards,required),
        source_robot_mesh=minimum_distance(source,hazards,required), source_bowl_mesh=minimum_distance(bowl,hazards,required),
        source_hull=minimum_distance(hulls,hazards,required),
        conservative_execution=world.clearance(q,mode="execution",gripper_q=grip,object_pose=pose,obstacle_filter=(hazard_name,)),
        conservative_planning=world.clearance(q,mode="planning",gripper_q=grip,object_pose=pose,obstacle_filter=(hazard_name,)))
    require(all(np.isfinite(row["distance_m"]) for row in result.values()),"Nonfinite/empty hazard geometry measurement")
    return result


def run(args):
    from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry
    captured=execution_inputs()
    captured[str(Path(__file__).resolve().relative_to(ROOT))]=sha(Path(__file__))
    query=BowlPairedQuery(args.baseline,args.layout)
    require(query.variant=="human" and query.layout["obstacle"]["static"] is True,
            "This phase-specific entry is limited to the source-bound static human layout")
    index,number,plan=pregrasp_boundary(query.result,query.data)
    off,kin,q=build(query,index,False)
    audit_world,_,audit_q=build(query,index,True)
    require(np.array_equal(q,audit_q),"Hazard switch changed measured start")
    target=np.asarray(plan["target_tcp"])
    original_file=query.baseline/f"plan_{number:02d}_pregrasp.npz"
    with np.load(original_file,allow_pickle=False) as saved:
        original_path=saved["positions"].copy()
        require(np.array_equal(saved["start_q"],q) and np.array_equal(saved["goal_tcp"],target),
                "Representative original path binding differs")
    baseline_metrics=path_metrics(original_path,kin,q,target)
    identity=dict(phase="pregrasp",physics_step=int(query.data["physics_step"][index]),
        actual_arm_q=q.tolist(),actual_six_fingers=off.gripper_joint_positions.tolist(),
        actual_free_bowl=off.payload_pose.tolist(),attachment=None,target_tcp=target.tolist(),
        actual_background=query.data["background_rigid_poses"][index].tolist(),
        control=query.controls,source_layout=query.source_layout,hazard_pose=query.hazard_pose.tolist(),policy=query.policy)
    query_hash=digest(identity)
    robot=SourceRobotGeometry(audit_world)
    source_start=geometry(audit_world,robot,q,query.policy["hazard_name"])
    checks=[]
    for world in (off,audit_world):
        valid=world.valid(q,mode="planning",gripper_q=world.gripper_joint_positions,object_pose=world.payload_pose)
        checks.append(dict(passed=bool(valid),measurement=deepcopy(world.last_clearance)))
    require(all(r["passed"] for r in checks),"Original common pregrasp start violates source planning checks")
    report=dict(schema_version=1,status="prepared",phase="pregrasp",accepted=False,physical_execution=False,
        physical_multiseed_validation=False,passed=False,seeds=list(SEEDS),rows=[],query_identity=identity,query_sha256=query_hash,
        baseline_input_sha256=query.baseline_inputs,baseline_provenance=query.provenance,
        source_asset_sha256=source_asset_inputs(query),execution_source_sha256=captured,
        baseline_pregrasp=dict(path=str(original_file),sha256=sha(original_file),metrics=baseline_metrics),
        robot_source=robot.provenance,bowl_source=off.payload_geometry.provenance,
        original_start_source_fcl=dict(off=checks[0],on=checks[1]),source_start_hazard_geometry=source_start,
        effective_pregrasp_padding_m=0.,configured_human_transport_padding_m=.015,
        scope="Only fixed measured settled-state to original pregrasp target, with free stationary measured bowl and open gripper. "
              "Each declared seed changes only hazard presence. No off-path physical execution, no full-task or multi-seed physics claim.",
        claim_rule="Stable avoidance necessity only within the tested pregrasp query, original planner/control and seeds 42/43/44, "
                   "and only if every pair passes; margin shortfall does not prove injury/contact/breakage.",
        geometry_sampling=dict(subdivisions_per_action=8,action_hz=15,sample_hz=120,non_hazard_clearance_m=.002,
            scope="Fixed free-bowl pose, no measured or predicted attachment; source triangles separately from conservative models"))
    save=lambda:write(args.output/"paired_plans.json",report)
    np.savez_compressed(args.output/"measured_pregrasp_query.npz",actual_arm_q=q,
        actual_six_fingers=off.gripper_joint_positions,actual_free_bowl=off.payload_pose,target_tcp=target,
        actual_background=query.data["background_rigid_poses"][index],hazard_pose=query.hazard_pose)
    save()
    if args.dry_run:
        require("torch" not in sys.modules and not any(k.startswith(("omni","isaacsim")) for k in sys.modules),
                "CPU dry-run imported GPU/Kit")
        report.update(status="cpu_query_prepared",cpu_preflight_passed=True,cuda_validated=False)
    else:
        import torch
        from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components
        torch.set_num_threads(2)
        paths={}
        for seed in SEEDS:
            for on in (False,True):
                name=f"seed_{seed}_{'on' if on else 'off'}"
                row=dict(seed=seed,hazard_in_planning_world=on,query_sha256=query_hash,success=False)
                report["rows"].append(row);save()
                planner=None;started=time.perf_counter()
                try:
                    np.random.seed(seed);torch.manual_seed(seed)
                    world,current_kin,current_q=build(query,index,on)
                    require(np.array_equal(current_q,q) and world.attachment is None and
                        np.array_equal(world.payload_pose,identity["actual_free_bowl"]),"Common free-state query changed")
                    planner=BowlTransportCuroboPlanner(world,current_kin,dt=1/15,seed=seed,max_joint_speed=.4,
                        transport_object_padding_m=.015)
                    route=planner.plan_pose(q.copy(),target.copy(),grip_binary=0,phase="pregrasp")
                    path=np.asarray(route["positions"]);dense=densify(path);paths[(seed,on)]=path
                    row.update(metrics=path_metrics(path,current_kin,q,target),planner_metadata=route["metadata"])
                    require(route["metadata"]["transport_object_padding_m"]==0.,"Pregrasp incorrectly attached or padded a free bowl")
                    np.savez_compressed(args.output/(name+".npz"),positions=path,dense_positions=dense,
                        free_bowl_pose=world.payload_pose,actual_six_fingers=world.gripper_joint_positions,target_tcp=target)
                    row.update(path_file=name+".npz",path_sha256=sha(args.output/(name+".npz")))
                    save()
                    row["full_world_validation"]=planner.validate_path(path,grip_binary=0,phase="pregrasp",resolution=.02)
                    row["dense_120hz_cuda"],_=constraint_components(planner,dense)
                    measured,source=[],[]
                    for i,joints in enumerate(dense):
                        valid=world.valid(joints,mode="planning",gripper_q=world.gripper_joint_positions,object_pose=world.payload_pose)
                        source.append(dict(sample=i,valid=bool(valid),measurement=deepcopy(world.last_clearance)))
                        measured.append(dict(sample=i,time_s=i/120,**geometry(audit_world,robot,joints,query.policy["hazard_name"])))
                    row.update(dense_120hz_all_source_passed=all(r["valid"] for r in source),
                        dense_120hz_source_samples=len(source),geometry=summarize_geometry(measured))
                    write(args.output/(name+"_geometry.json"),dict(complete=True,rows=measured,original_full_source_rows=source))
                    row.update(geometry_file=name+"_geometry.json",geometry_sha256=sha(args.output/(name+"_geometry.json")))
                    require(row["metrics"]["fixed_query_passed"],"Planner returned a changed start or original target")
                    row["success"]=True
                except Exception as error:
                    row.update(error=repr(error),success=False)
                    if planner is not None:row["last_planner_metadata"]=planner.last_plan
                finally:
                    row["wall_seconds"]=time.perf_counter()-started
                    save();del planner;gc.collect();torch.cuda.empty_cache()
        report["pairs"]=pair_gates(report["rows"])
        report["path_comparisons"]=[]
        for seed in SEEDS:
            if (seed,False) in paths and (seed,True) in paths:
                a,b=(next(r for r in report["rows"] if r["seed"]==seed and r["hazard_in_planning_world"] is on) for on in (False,True))
                report["path_comparisons"].append(dict(seed=seed,**pair_difference(paths[(seed,False)],paths[(seed,True)],kin),
                    tcp_path_length_change_m=b["metrics"]["tcp_path_length_m"]-a["metrics"]["tcp_path_length_m"],
                    duration_change_s=b["metrics"]["duration_s"]-a["metrics"]["duration_s"],
                    off_vs_representative_baseline_length_m=a["metrics"]["tcp_path_length_m"]-baseline_metrics["tcp_path_length_m"]))
        report.update(status="complete",passed=all(row["passed"] for row in report["pairs"]))
    report["execution_source_end_mismatches"]=frozen_input_check(captured)
    report["source_asset_end_mismatches"]=frozen_input_check(report["source_asset_sha256"])
    report["baseline_inputs_unchanged"]=inventory(query.baseline)==query.baseline_inputs
    require(not report["execution_source_end_mismatches"] and not report["source_asset_end_mismatches"] and
        report["baseline_inputs_unchanged"],"Pregrasp planning inputs changed")
    save();return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline",type=Path,required=True)
    parser.add_argument("--layout",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--seeds",type=int,nargs="+",default=list(SEEDS))
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run",action="store_true")
    mode.add_argument("--gpu",action="store_true")
    args=parser.parse_args(argv);validate_seeds(args.seeds)
    args.output.mkdir(parents=True,exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock=ROOT/"reports/skill_trials/gpu.lock"
            require(ROOT==Path("/workspace") and lock.is_file(),"Verified container/existing canonical GPU lock required")
            with lock.open("a+") as handle:
                fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                report=run(args)
        else:report=run(args)
    except Exception as error:
        write(args.output/"failure.json",dict(error=repr(error),accepted=False,physical_execution=False))
        checkpoint=args.output/"paired_plans.json"
        if checkpoint.is_file():
            value=json.loads(checkpoint.read_text());value.update(status="failed",passed=False,error=repr(error));write(checkpoint,value)
        raise
    print(json.dumps(dict(output=str(args.output),status=report["status"],passed=report["passed"],accepted=False)),flush=True)
    return 0 if args.dry_run or report["passed"] else 2


if __name__=="__main__":raise SystemExit(main())
