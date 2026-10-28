#!/usr/bin/env python3
"""CPU source-body pinch diagnosis for a sampled mustard approach candidate.

No physical state writes, CUDA, contact forces, or successful grasp claims.
The source surface is partitioned at the unchanged safe bottle-body planes;
FCL's approximate contact.pos values are not used to classify the body region.
"""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from scripts.extensions.replicator.search_mustard_grasp_cpu import Search,save
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts
from scripts.extensions.replicator.variant_geometry import code_evidence
from scripts.extensions.replicator.audit_source_layout_geometry import minimum_distance,REQUIRED_M


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--search-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--branch-dir',type=Path,help='Optional completed seed-search report for exactly the same target/layout')
    args=p.parse_args()
    if args.output_dir.exists():raise FileExistsError(args.output_dir)
    report=json.loads((args.search_dir/'report.json').read_text());declaration=json.loads((args.search_dir/'declaration.json').read_text())
    if not report['passed'] or not report['code_unchanged'] or sha(args.search_dir/'declaration.json')!=report['declaration_sha256']:
        raise ValueError('Require original completed CPU search evidence')
    if sha(declaration['preparation'])!=declaration['preparation_sha256']:raise ValueError('Changed input preparation')
    prepared=load_preparation(declaration['preparation']);selected=report['selected'];index=selected['candidate_index']
    trial=next(row for row in report['attempts'] if row['grid_index']==selected['grid_index'])
    trajectory=next(row['trajectory'] for row in trial['grasps'] if row['candidate_index']==index)
    branch_evidence=None
    if args.branch_dir:
        branch=json.loads((args.branch_dir/'report.json').read_text());branch_decl=json.loads((args.branch_dir/'declaration.json').read_text())
        if (not branch['passed'] or not branch['code_unchanged'] or branch['declaration_sha256']!=sha(args.branch_dir/'declaration.json')
                or branch_decl['source_search_sha256']!=sha(args.search_dir/'report.json') or branch_decl['selected_grasp']!=selected):
            raise ValueError('Mismatched branch search or incomplete branch evidence')
        for path,digest in branch_decl['code_sha256'].items():
            if sha(ROOT/path)!=digest:raise ValueError('Changed branch-search dependency '+path)
        chosen=next(row for row in branch['results'] if row['seed_index']==branch['selected_seed_index'])
        if not chosen['passed'] or chosen['approach_target_world']!=trajectory['approach_target_world']:
            raise ValueError('Branch changes the original selected grasp target')
        trajectory=chosen
        branch_evidence=dict(directory=str(args.branch_dir),report_sha256=sha(args.branch_dir/'report.json'),declaration_sha256=sha(args.branch_dir/'declaration.json'),selected_seed_index=chosen['seed_index'])
    files=[Path(__file__),ROOT/'scripts/extensions/replicator/search_mustard_grasp_cpu.py',
        ROOT/'scripts/extensions/replicator/audit_banana_grasp_cpu.py',ROOT/'scripts/extensions/replicator/banana_region_contacts_cpu.py',
        ROOT/'scripts/extensions/replicator/variant_geometry.py',ROOT/'kitchen_safety/extensions/replicator/mustard_family_collision.py',
        ROOT/'kitchen_safety/extensions/replicator/task_collision.py',ROOT/'kitchen_safety/extensions/replicator/task_contact_policy.py',
        ROOT/'kitchen_safety/extensions/replicator/collision_planner.py',ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/geometry_audit.py']
    evidence=code_evidence(files)
    for path,digest in declaration['code_sha256'].items():
        if sha(ROOT/path)!=digest:raise ValueError('Search execution dependency changed: '+path)
    angles=np.linspace(0.,.78,157);signs=np.array([1.,1.,-1.,1.,-1.,-1.])
    args.output_dir.mkdir(parents=True)
    save(args.output_dir/'declaration.json',dict(search_report_sha256=sha(args.search_dir/'report.json'),search_directory=str(args.search_dir),
        finger_joint_rad=angles.tolist(),coupled_signs=signs.tolist(),selected_grasp=selected,branch_evidence=branch_evidence,
        source_body_region=prepared['profile']['source_bottle_geometry']['safe_contact_z_range_m'],
        policy='All declared closure angles retained. Candidate requires bilateral actual source-triangle intersections inside safe body, zero outside-region intersection, and full noncontact source+conservative safety for the entire sampled prefix.',
        execution_ready=False,**evidence))
    search=Search(prepared);world,kin=search.world,search.kin
    q=np.asarray(trajectory['approach_q_rad']);bp=search.bp.copy();bp[:3,3]+=selected['delta_xyz_m']
    world.payload_pose=bp;world.set_phase('close')
    meshes,finger_source=source_finger_meshes(world)
    geometry=prepared['profile']['source_bottle_geometry'];low,high=map(np.array,geometry['mesh_bounds_m'])
    low[2],high[2]=geometry['safe_contact_z_range_m'];region=SourceRegionContacts(world.payload_geometry,[low.tolist(),high.tolist()],meshes)
    bottle=search.scene.shapes['mustard_bottle'].at(selected['delta_xyz_m']);rows=[];prefix=True
    for angle in angles:
        values=signs*angle;fingers=dict(zip(world.GRIPPER_JOINT_NAMES,values));world.gripper_joint_positions=values
        world.gripper_joint_position=float(angle)
        contacts=region.at(world,kin,q)
        source=search.robot.shapes(q,fingers)
        environment=minimum_distance(source+search.stand,search.environment,REQUIRED_M)
        own=minimum_distance([row for row in source if row[0] not in search.intended],bottle,REQUIRED_M)
        valid=world.valid(q,gripper_q=values,object_pose=bp)
        noncontact=bool(environment['valid'] and own['valid'] and valid);prefix &= noncontact
        rows.append(dict(finger_joint_rad=float(angle),all_coupled_joint_angles_rad=values.tolist(),source_region_contacts=contacts,
            source_environment=environment,source_noncontact_bottle=own,conservative_valid=valid,
            conservative_failure=None if valid else world.last_clearance,noncontact_valid=noncontact,
            preceding_closure_noncontact_valid=prefix,
            possible_body_pinch_candidate=bool(prefix and contacts['bilateral_all_central'])))
    candidates=[row for row in rows if row['possible_body_pinch_candidate']]
    unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    output=dict(source_body_pinch_candidate_found=bool(candidates and unchanged),code_unchanged=unchanged,
        first_candidate_angle_rad=candidates[0]['finger_joint_rad'] if candidates else None,
        last_candidate_angle_rad=candidates[-1]['finger_joint_rad'] if candidates else None,candidate_count=len(candidates),
        first_source_body_contact_angle={finger:next((row['finger_joint_rad'] for row in rows if row['source_region_contacts']['fingers'][finger]['inside']>0),None) for finger in ('left_inner_finger','right_inner_finger')},
        first_source_outside_contact_angle={finger:next((row['finger_joint_rad'] for row in rows if row['source_region_contacts']['fingers'][finger]['outside']>0),None) for finger in ('left_inner_finger','right_inner_finger')},
        finger_source=finger_source,source_region_partition=region.provenance,rows=rows,
        actual_grasp_validated=False,execution_ready=False,physics_launched=False,
        declaration_sha256=sha(args.output_dir/'declaration.json'),
        limitations='Fixed nominal bottle-pose kinematic source surface scan. Intersections indicate possible contacts, not acceptable physical penetration. Initial one-sided contact can move/tilt the bottle. Neither cooked PhysX surfaces, stable bilateral force, friction, self-collision, lift, cuRobo nor task success is established.')
    save(args.output_dir/'report.json',output)
    print(json.dumps({key:output[key] for key in ('source_body_pinch_candidate_found','first_candidate_angle_rad','last_candidate_angle_rad','candidate_count','first_source_body_contact_angle','first_source_outside_contact_angle','execution_ready')}))
    return int(not output['source_body_pinch_candidate_found'])


if __name__=='__main__':raise SystemExit(main())
