#!/usr/bin/env python3
"""CPU candidates constrained by original bowl region and observed close.

Observed-contact transforms are a sensitivity estimate only: changing a target
changes contact mechanics, so these estimates never establish future force or
physical grasp. Source geometry, original margin, all rejected candidates and
the actual failed episode remain separate and unchanged.
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
from scripts.extensions.replicator.diagnose_bowl_grasp_recording import synchronize
from scripts.extensions.replicator.task_audit_sidecars import load_episode, sha
from scripts.extensions.replicator.bowl_collect import json_write


def constrained_z_interval(old_target, unshifted_target, contact_rows, states, region):
    """Exact linear bounds for a world-Z translation of observed contact points."""
    delta = unshifted_target@np.linalg.inv(old_target)
    lower, upper = -float("inf"), float("inf")
    points, vectors = [], []
    for row in contact_rows:
        if row["phase"] != "close":
            continue
        transform = pose_matrix(states[row["physics_step"], 0, :7])
        vector = transform[:3, :3].T@np.array([0., 0., 1.])
        for item in row["fingers"].values():
            raw = np.asarray(item["points_world_m"]).reshape(-1, 3)
            use = (np.asarray(item["normal_forces_n"])>.01)|(np.asarray(item["separations_m"])<=.00025)
            local = ((raw[use]@delta[:3, :3].T+delta[:3, 3])-transform[:3, 3])@transform[:3, :3]
            points.extend(local)
            vectors.extend(np.repeat(vector[None], len(local), axis=0))
            for point in local:
                for axis in range(3):
                    if abs(vector[axis]) < 1e-12:
                        if not region[0, axis] <= point[axis] <= region[1, axis]:
                            return None
                        continue
                    bounds = sorted(((region[0, axis]-point[axis])/vector[axis],
                                     (region[1, axis]-point[axis])/vector[axis]))
                    lower, upper = max(lower, bounds[0]), min(upper, bounds[1])
    if not points or not lower <= upper or not np.isfinite([lower, upper]).all():
        return None
    shift=(lower+upper)/2
    moved=np.asarray(points)+shift*np.asarray(vectors)
    return dict(interval_m=[lower,upper], midpoint_m=shift, point_count=len(points),
        estimated_minimum_region_margin_m=float(np.min(np.minimum(moved-region[0],region[1]-moved))),
        estimated_points_bounds_m=[moved.min(0),moved.max(0)],
        model="Rigidly transform each recorded contact by candidate_TCP * inverse(original_target_TCP), then retain each recorded bowl pose. Sensitivity estimate only; no force prediction.")


def run(args):
    meta,data,_=load_episode(args.episode)
    original=json.loads((args.episode/"result.json").read_text())
    raw=json.loads((args.episode/"grasp_contacts.json").read_text())
    if original["physical_trial_passed"]:
        raise ValueError("Expected original failed bowl close")
    world,kin,_=build_world(meta["source_layout"])
    arm=[meta["joint_names"].index(f"panda_joint{i}") for i in range(1,8)]
    initial=int(np.flatnonzero(data["phase"]=="initial_settle")[-1])
    end=int(np.flatnonzero(data["phase"]=="approach")[-1])
    observed=synchronize(world,meta,data,initial)
    old,grasp=bowl_rim_grasp(observed,world.payload_geometry,world)
    region=np.asarray(grasp["contact_region_local_m"])
    meshes,_=source_finger_meshes(world)
    partition=SourceRegionContacts(world.payload_geometry,region,meshes)
    spheres=GripperSphereModel()
    declaration=dict(rotation_world_x_deg=args.roll_deg,
        z_rule="Center of feasible exact original-region interval for all observed close near/loaded points",
        angle_grid_rad=(np.arange(158)*.005).tolist(),
        original_safety_margin_m=world.default_clearance,
        selection="Keep every candidate; original source bilateral surfaces and all noncontact gates required. Contact-motion estimate is not a physical pass.",
        source_objects="Recorded settled pose, no scene pose or original yaw change")
    json_write(args.output/"declaration.json",declaration)
    rows=[]
    for number,angle in enumerate(declaration["rotation_world_x_deg"]):
        synchronize(world,meta,data,initial)
        world.gripper_joint_positions=np.zeros(6);world.gripper_joint_position=0.
        target=old.copy()
        target[:3,:3]=Rotation.from_euler("x",angle,degrees=True).as_matrix()@old[:3,:3]
        estimate=constrained_z_interval(old,target,raw,data["task_object_states"],region)
        row=dict(index=number,world_x_rotation_deg=angle,accepted=False,executed=False,
            physical_candidate_verified=False,contact_estimate=estimate,geometry_passed=False)
        if estimate is None:
            row["error"]="No world-Z adjustment keeps transformed original near/loaded contacts in original region"
        else:
            target[2,3]+=estimate["midpoint_m"]
            row["target_tcp"]=target
            try:
                pre=target.copy();pre[2,3]+=.125
                world.set_phase("pregrasp")
                qpre=kin.ik(pre,data["q"][end,arm],attempts=1)
                world.set_phase("approach")
                path=local_cartesian_candidate(kin,qpre,target,"approach",meta["action_dt_s"],.4)
                audits=[]
                for index,q in enumerate(path["positions"]):
                    valid=world.valid(q)
                    sphere=sphere_clearance(spheres.at(kin.fk(q),dict(zip(world.GRIPPER_JOINT_NAMES,world.gripper_joint_positions))),world)
                    audits.append(dict(index=index,source_fcl=valid,failure=deepcopy(world.last_clearance),original_gripper_spheres=sphere))
                row["approach"]=dict(passed=all(x["source_fcl"] and x["original_gripper_spheres"]["passed"] for x in audits),rows=audits)
                q=path["positions"][-1]
                world.set_phase("close")
                closure=[]
                for joint_angle in declaration["angle_grid_rad"]:
                    world.gripper_joint_positions=np.array([1,1,-1,1,-1,-1])*joint_angle
                    world.gripper_joint_position=joint_angle
                    contact=partition.at(world,kin,q)
                    valid=world.valid(q)
                    sphere=sphere_clearance(spheres.at(kin.fk(q),dict(zip(world.GRIPPER_JOINT_NAMES,world.gripper_joint_positions))),world)
                    bilateral=all(x["inside"]+x["outside"]>0 for x in contact["fingers"].values())
                    closure.append(dict(angle_rad=joint_angle,source_contacts=contact,source_fcl=valid,
                        failure=deepcopy(world.last_clearance),original_gripper_spheres=sphere,bilateral=bilateral))
                    if bilateral:
                        break
                row["closure"]=dict(passed=bool(closure[-1]["bilateral"] and closure[-1]["source_contacts"]["bilateral_all_central"] and
                    all(x["source_fcl"] and x["original_gripper_spheres"]["passed"] for x in closure)),rows=closure)
                row["geometry_passed"]=row["approach"]["passed"] and row["closure"]["passed"]
                np.savez_compressed(args.output/f"candidate_{number:02d}_path.npz",approach=path["positions"],target_tcp=target)
            except Exception as error:
                row["error"]=repr(error)
        json_write(args.output/f"candidate_{number:02d}.json",row)
        rows.append(row)
        print(json.dumps(dict(candidate=number,angle_deg=angle,estimated_region_margin_m=(estimate or {}).get("estimated_minimum_region_margin_m"),
            geometry_passed=row["geometry_passed"],error=row.get("error"))),flush=True)
    return dict(accepted=False,execution_ready=False,episode=str(args.episode),declaration=declaration,candidates=rows,
        inputs_sha256={str(args.episode/name):sha(args.episode/name) for name in ("result.json","physics_metadata.json","physics_states.npz","grasp_contacts.json")},
        diagnostic_sha256=sha(Path(__file__)),original_contact_region_m=region,
        limits="No simulator/force/cooked shape validation; no runtime changes. Dense CUDA joint/self/world and actual-state physical trial remain mandatory.")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--roll-deg",type=float,nargs="+",default=[0.,-5.,-10.,-15.,-20.,-25.,-30.],
        help="Explicit predeclared world-X candidate rotations; each must be within +/-30 degrees")
    args=parser.parse_args()
    if not args.roll_deg or not np.isfinite(args.roll_deg).all() or max(map(abs,args.roll_deg))>30:
        parser.error("Require finite predeclared rotations within +/-30 degrees")
    args.output.mkdir(parents=True,exist_ok=False)
    try:
        result=run(args)
    except Exception as error:
        json_write(args.output/"failure.json",dict(accepted=False,error=repr(error)))
        raise
    json_write(args.output/"diagnostic.json",result)


if __name__=="__main__":
    main()
