#!/usr/bin/env python3
"""CPU/full-gate CUDA diagnosis of an observed-contact bowl grasp candidate.

No Kit/physics and no changed episode. The local predeclared rim contact box
is retained exactly; its numerical bounds are not an upstream YAML rule.
Transformed recorded close is a stress test, never a force prediction.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_grasp
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.diagnose_bowl_grasp_recording import synchronize
from scripts.extensions.replicator.search_banana_grasp_envelope_cpu import GripperSphereModel,sphere_clearance
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts
from scripts.extensions.replicator.task_audit_sidecars import ROOT,load_episode,sha
from scripts.extensions.replicator.bowl_collect import json_write


def run(args):
    candidate_path=args.search/f"candidate_{args.candidate:02d}.json"
    candidate=json.loads(candidate_path.read_text())
    archive=args.search/f"candidate_{args.candidate:02d}_path.npz"
    search=json.loads((args.search/"diagnostic.json").read_text())
    if not candidate["geometry_passed"] or candidate["accepted"] or candidate["executed"]:
        raise ValueError("Require unexecuted source-geometry passing candidate")
    for name,digest in search["inputs_sha256"].items():
        path=Path(name)
        if sha(path)!=digest:
            raise ValueError("Original failed recording changed: "+name)
    meta,data,_=load_episode(args.episode)
    world,kin,_=build_world(meta["source_layout"])
    initial=int(np.flatnonzero(data["phase"]=="initial_settle")[-1])
    observed=synchronize(world,meta,data,initial)
    original,grasp=bowl_rim_grasp(observed,world.payload_geometry,world)
    target=np.asarray(candidate["target_tcp"])
    delta=target@np.linalg.inv(original)
    with np.load(archive,allow_pickle=False) as raw:
        approach=raw["approach"].copy()
        if not np.array_equal(raw["target_tcp"],target):
            raise ValueError("Candidate target archive differs")
    arm=[meta["joint_names"].index(f"panda_joint{i}") for i in range(1,8)]
    close=np.flatnonzero(data["phase"]=="close")
    sphere_model=GripperSphereModel()
    meshes,_=source_finger_meshes(world)
    partition=SourceRegionContacts(world.payload_geometry,np.asarray(grasp["contact_region_local_m"]),meshes)
    report=dict(accepted=False,physical_execution=False,execution_ready=False,gpu_requested=args.gpu,
        local_contact_policy="Exact predeclared bowl_rim_geometry box, source-derived but not a numerical upstream YAML requirement",
        original_contact_region_m=grasp["contact_region_local_m"],candidate=candidate,
        input_sha256={str(p):sha(p) for p in (candidate_path,archive,args.search/"diagnostic.json")},
        phases={},limits="Rigid target transform of actual close states, not future mechanics; physical force and original task remain unverified")
    groups={"approach":[dict(q=q,grip=np.zeros(6),step=initial) for q in approach]}
    selected=candidate["closure"]["rows"][-1]["angle_rad"]
    groups["nominal_closed"]=[dict(q=approach[-1],grip=np.array([1,1,-1,1,-1,-1])*selected,step=initial)]
    stress=[]
    for index in close:
        goal=delta@pose_matrix(data["tcp_pose_w"][index])
        q=kin.ik(goal,data["q"][index,arm],attempts=1)
        fk=kin.fk(q)
        position_error=float(np.linalg.norm(fk[:3,3]-goal[:3,3]))
        rotation_error=float(Rotation.from_matrix(fk[:3,:3]@goal[:3,:3].T).magnitude())
        if position_error>.002 or rotation_error>np.deg2rad(2):
            raise ValueError("Original IK gates rejected transformed observed close")
        grip=data["q"][index,[meta["joint_names"].index(name) for name in world.GRIPPER_JOINT_NAMES]]
        stress.append(dict(q=q,grip=grip,step=int(index),target=goal,
            ik_position_error_m=position_error,ik_rotation_error_rad=rotation_error))
    groups["observed_close_stress"]=stress
    planner=None
    if args.gpu:
        from kitchen_safety.extensions.replicator.bowl_source_support_curobo import BowlSourceSupportCuroboPlanner
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components,sphere_pair_diagnostics
        planner=BowlSourceSupportCuroboPlanner(world,kin,dt=meta["action_dt_s"],seed=42,max_joint_speed=.4)
    for name,states in groups.items():
        rows=[]
        phase="approach" if name=="approach" else "close"
        for index,item in enumerate(states):
            synchronize(world,meta,data,item["step"])
            world.set_phase(phase)
            world.gripper_joint_positions=item["grip"].copy();world.gripper_joint_position=float(item["grip"][0])
            valid=world.valid(item["q"])
            spheres=sphere_clearance(sphere_model.at(kin.fk(item["q"]),dict(zip(world.GRIPPER_JOINT_NAMES,item["grip"]))),world)
            row=dict(index=index,original_physics_step=item["step"],q=item["q"],actual_independent_finger_angles=item["grip"],
                source_fcl=valid,failure=deepcopy(world.last_clearance),original_gripper_spheres=spheres)
            if name!="approach":
                row["source_region_triangles"]=partition.at(world,kin,item["q"])
            if planner is not None:
                planner._prepare_path_query((),int(phase!="approach"),phase)
                # Parent candidate model normally couples all joints from the
                # primary finger. This independent audit restores every one
                # of the six observed angles before the original constraints.
                locked=deepcopy(planner.robot_dict["kinematics"]["lock_joints"])
                locked.update(dict(zip(world.GRIPPER_JOINT_NAMES,map(float,item["grip"]))))
                planner.motion_gen.update_locked_joints(locked,deepcopy(planner.robot_dict))
                components,state=constraint_components(planner,np.asarray([item["q"]]))
                row["original_cuda_components"]=components
                if not components["valid"]:
                    row["cuda_failed_pairs"],_=sphere_pair_diagnostics(planner,state,components)
            rows.append(row)
        phase_report=dict(samples=len(rows),full_source_fcl_passed=all(x["source_fcl"] for x in rows),
            original_gripper_spheres_passed=all(x["original_gripper_spheres"]["passed"] for x in rows),rows=rows)
        if planner is not None:
            phase_report["original_cuda_passed"]=all(x["original_cuda_components"]["valid"] for x in rows)
        report["phases"][name]=phase_report
        json_write(args.output/(name+".json"),phase_report)
        np.savez_compressed(args.output/(name+".npz"),q=np.asarray([x["q"] for x in states]),
            actual_fingers=np.asarray([x["grip"] for x in states]),original_physics_step=np.asarray([x["step"] for x in states]))
        print(json.dumps(dict(phase=name,samples=len(rows),source_fcl=phase_report["full_source_fcl_passed"],
            original_gripper_spheres=phase_report["original_gripper_spheres_passed"],cuda=phase_report.get("original_cuda_passed"))),flush=True)
    report["source_sha256"]={str(Path(__file__)):sha(Path(__file__))}
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode",required=True,type=Path)
    parser.add_argument("--search",required=True,type=Path)
    parser.add_argument("--candidate",required=True,type=int)
    parser.add_argument("--output",required=True,type=Path)
    parser.add_argument("--gpu",action="store_true",help="Root-scheduled CUDA only; no physics")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock_path=ROOT/"reports/skill_trials/gpu.lock"
            if ROOT!=Path("/workspace") or not lock_path.exists():
                raise RuntimeError("CUDA requires verified workspace and existing lock")
            with lock_path.open("a+") as lock:
                fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                result=run(args)
        else:
            result=run(args)
    except Exception as error:
        json_write(args.output/"failure.json",dict(accepted=False,error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json",result)


if __name__=="__main__":
    main()
