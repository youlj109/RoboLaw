#!/usr/bin/env python3
"""Source bowl rim closure/lift diagnostic, without physics or CUDA.

The source rim grasp is checked against exact region-clipped triangles and the
unchanged robot envelopes. A hypothetical attachment tests candidate geometry
only; it cannot establish physical grasp, source success or safe execution.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_grasp, sink_placement_object_pose
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts
from scripts.extensions.replicator.diagnose_task_plan import cpu_sphere_audit
from scripts.extensions.replicator.search_banana_grasp_envelope_cpu import GripperSphereModel, sphere_clearance
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.task_collect import json_write


def path_check(world, positions):
    rows = []
    for index, q in enumerate(positions):
        valid = world.valid(q)
        rows.append(dict(sample=index, passed=valid, violation=deepcopy(world.last_clearance)))
    return dict(passed=all(row['passed'] for row in rows), samples=len(rows), rows=rows)


def run(args):
    layout = json.loads(args.layout.read_text())
    if layout['task_id'] != 'kitchen_bench_replicator_g_shape_bowl_sink':
        raise ValueError('This diagnostic is scoped to the original G bowl/sink')
    relations = check_layout(layout)
    if not relations['initial_source_relations_passed']:
        raise ValueError('Original source relationships failed')
    angles = np.arange(158, dtype=float)*.005
    declaration = dict(finger_angles_rad=angles, closure_order='Ascending, stop at first bilateral source intersection',
        source_contact_region='Measured negative-X bowl upper wall/rim; unchanged source triangles',
        pregrasp_height_m=.125, lift_height_m=.15, original_sphere_buffer_m=.005,
        noncontact_clearance_m=.002, physically_executed=False,
        layout_path=str(args.layout), layout_sha256=sha(args.layout))
    json_write(args.output/'declaration.json', declaration)
    world, kin, pose = build_world(layout)
    joint_values, joint_source = source_initial_q()
    q0 = np.array([joint_values[f'panda_joint{i}'] for i in range(1, 8)])
    target, grasp = bowl_rim_grasp(pose, world.payload_geometry, world)
    pregrasp = target.copy()
    pregrasp[2, 3] += .125
    qpre = kin.ik(pregrasp, q0)
    world.set_phase('approach')
    approach = local_cartesian_candidate(kin, qpre, target, 'approach', 1/15, .4)
    approach_fcl = path_check(world, approach['positions'])
    approach_spheres, _ = cpu_sphere_audit(world, kin, approach['positions'])
    report = dict(accepted=False, execution_ready=False, physical_grasp_verified=False,
        layout=layout, layout_sha256=sha(args.layout), declaration=declaration,
        source_relations=relations, initial_joint_source=joint_source,
        source_geometry=world.payload_geometry.provenance, grasp=grasp,
        approach=dict(full_fcl=approach_fcl, cpu_spheres=approach_spheres, metadata=approach['metadata']))
    # All contacts use this exact generated approach end, never the requested
    # pose substituted for the actual FK of the candidate joint state.
    qclose = approach['positions'][-1]
    tcpclose = kin.fk(qclose)
    meshes, report['source_finger_meshes'] = source_finger_meshes(world)
    region = SourceRegionContacts(world.payload_geometry, grasp['contact_region_local_m'], meshes)
    report['source_region_partition'] = region.provenance
    sphere_model = GripperSphereModel()
    world.set_phase('close')
    rows, selected = [], None
    for angle in angles:
        values = np.asarray([1., 1., -1., 1., -1., -1.])*angle
        world.gripper_joint_positions, world.gripper_joint_position = values, float(angle)
        contacts = region.at(world, kin, qclose)
        bilateral = all(item['inside']+item['outside'] > 0 for item in contacts['fingers'].values())
        full_valid = world.valid(qclose)
        original_spheres = sphere_clearance(sphere_model.at(tcpclose, dict(zip(world.GRIPPER_JOINT_NAMES, values))), world)
        rows.append(dict(angle_rad=float(angle), source_contacts=contacts, bilateral_source_intersection=bilateral,
            full_noncontact_fcl_passed=full_valid, violation=deepcopy(world.last_clearance),
            original_gripper_spheres=original_spheres))
        if bilateral:
            selected = len(rows)-1
            break
    closure_passed = selected is not None and rows[selected]['source_contacts']['bilateral_all_central'] and all(
        row['full_noncontact_fcl_passed'] and row['original_gripper_spheres']['passed'] for row in rows)
    report['closure'] = dict(rows=rows, selected_index=selected, passed=closure_passed,
        no_physical_contact_or_force_claim=True, q=qclose, tcp=tcpclose)
    report['source_sink_candidate'] = dict(zip(('object_pose', 'geometry'),
        sink_placement_object_pose(pose, world.payload_geometry, world)))
    if closure_passed:
        # This is a purely geometric candidate. The collector must construct
        # its own attachment from measured post-grasp TCP and bowl states.
        world.attachment = np.linalg.inv(tcpclose)@world.payload_pose
        lift_target = tcpclose.copy()
        lift_target[2, 3] += .15
        world.set_phase('lift')
        lift = local_cartesian_candidate(kin, qclose, lift_target, 'lift', 1/15, .4)
        lift_fcl = path_check(world, lift['positions'])
        lift_spheres, _ = cpu_sphere_audit(world, kin, lift['positions'])
        report['hypothetical_lift'] = dict(full_fcl=lift_fcl, cpu_spheres=lift_spheres,
            metadata=lift['metadata'], hypothetical_attachment=world.attachment,
            source_support_sphere_failures=[row for row in lift_spheres['world_intersections']
                if row['link'] == 'attached_object' and row['obstacle'] == world.contact_policy.source_support],
            other_sphere_failures=[row for row in lift_spheres['world_intersections']
                if not (row['link'] == 'attached_object' and row['obstacle'] == world.contact_policy.source_support)],
            original_support_still_in_full_world=True, no_exception_applied=True)
        np.savez_compressed(args.output/'candidate_paths.npz', approach=approach['positions'],
            closure_angles=angles[:len(rows)], lift=lift['positions'], hypothetical_attachment=world.attachment)
    report['source_code_sha256'] = {str(path.relative_to(ROOT)): sha(path) for path in (
        Path(__file__).resolve(), ROOT/'kitchen_safety/extensions/replicator/bowl_motion.py',
        ROOT/'kitchen_safety/extensions/replicator/bowl_collision.py',
        ROOT/'kitchen_safety/extensions/replicator/task_collision.py',
        ROOT/'scripts/extensions/replicator/banana_region_contacts_cpu.py')}
    report['remaining'] = ['Actual same-profile neutral scene', 'cuRobo original full robot/self/world constraints',
        'Pregrasp and full transport/lower/release paths', 'Measured bilateral grasp, support and original success',
        'Full execution/physical/RGB/visual gates; no collected episode from this CPU diagnostic']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refuse to replace earlier diagnostic evidence')
    args.output.mkdir(parents=True)
    try:
        report = run(args)
    except Exception as error:
        json_write(args.output/'failure.json', dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/'diagnostic.json', report)
    print(json.dumps(dict(output=str(args.output), closure_passed=report['closure']['passed'],
        first_bilateral_index=report['closure']['selected_index'], accepted=False)))


if __name__ == '__main__':
    main()
