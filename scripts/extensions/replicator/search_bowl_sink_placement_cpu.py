#!/usr/bin/env python3
"""Find a legal original sink-floor landing, retaining the drain and all walls.

The source task permits the whole original basin, not only its centre. Search
nearest source-floor positions before any hazard or physical baseline exists.
No planner mask or physical-success claim is introduced by this search.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_motion import sink_placement_object_pose, sink_reference_bounds
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.audit_bowl_grasp_cpu import path_check
from scripts.extensions.replicator.diagnose_task_plan import cpu_sphere_audit
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.task_collect import json_write


def run(args):
    source = json.loads(args.grasp_report.read_text())
    if not source['closure']['passed'] or source['layout_sha256'] != sha(args.layout):
        raise ValueError('Requires matching source-rim geometric evidence')
    world, kin, _ = build_world(json.loads(args.layout.read_text()))
    qclose = np.asarray(source['closure']['q'])
    angle = source['closure']['rows'][source['closure']['selected_index']]['angle_rad']
    world.gripper_joint_position = angle
    world.gripper_joint_positions = np.asarray([1., 1., -1., 1., -1., -1.])*angle
    world.attachment = np.linalg.inv(kin.fk(qclose))@world.payload_pose
    lift_target = kin.fk(qclose)
    lift_target[2, 3] += .15
    lift = local_cartesian_candidate(kin, qclose, lift_target, 'lift', 1/15, .4)
    qlift = lift['positions'][-1]
    carried = kin.fk(qlift)@world.attachment
    carried_pose = np.r_[carried[:3, 3], Rotation.from_matrix(carried[:3, :3]).as_quat()]
    low, high = sink_reference_bounds(world)
    center = (low[:2]+high[:2])/2
    offsets = sorted([(i*.025, j*.025) for i in range(-4, 5) for j in range(-6, 7)],
        key=lambda pair: (round(pair[0]**2+pair[1]**2, 10), pair[0], pair[1]))
    declaration = dict(offsets_xy_m=offsets, selection='First passing candidate in squared-distance, X, Y order',
        source_floor_xy=center, original_sink_reference=world.contact_policy.destination_name,
        release_gap_m=.003, original_sphere_buffer_m=.005, noncontact_clearance_m=.002,
        grasp_report_sha256=sha(args.grasp_report), layout_sha256=sha(args.layout),
        no_added_hazard=True, no_layout_change=True, physical_execution=False)
    json_write(args.output/'declaration.json', declaration)
    rows, selected = [], None
    for index, offset in enumerate(offsets):
        row = dict(index=index, offset_xy_m=offset, passed=False)
        try:
            placement, geometry = sink_placement_object_pose(carried_pose, world.payload_geometry, world, center_xy=center+offset)
            above_object = placement.copy()
            above_object[2, 3] = carried[2, 3]
            above, lower = (item@np.linalg.inv(world.attachment) for item in (above_object, placement))
            qsink = kin.ik(above, qlift)
            world.set_phase('transport')
            row.update(source_floor_target=geometry, above_tcp=above, lower_tcp=lower, above_q=qsink,
                above_fcl=world.valid(qsink), above_violation=deepcopy(world.last_clearance))
            if row['above_fcl']:
                world.set_phase('lower')
                qend = kin.ik(lower, qsink)
                row.update(lower_endpoint_q=qend, endpoint_fcl=world.valid(qend), endpoint_violation=deepcopy(world.last_clearance))
                endpoint_spheres, _ = cpu_sphere_audit(world, kin, [qend])
                row['endpoint_cpu_spheres'] = endpoint_spheres
                if row['endpoint_fcl'] and not endpoint_spheres['world_intersections'] and not endpoint_spheres['joint_urdf_violations']:
                    candidate = local_cartesian_candidate(kin, qsink, lower, 'lower', 1/15, .4)
                    full_fcl = path_check(world, candidate['positions'])
                    spheres, _ = cpu_sphere_audit(world, kin, candidate['positions'])
                    row.update(full_lower_fcl=full_fcl, full_lower_cpu_spheres=spheres,
                        passed=full_fcl['passed'] and not spheres['world_intersections'] and not spheres['joint_urdf_violations'])
                    if row['passed']:
                        np.savez_compressed(args.output/'selected_candidate.npz', lower=candidate['positions'],
                            above_q=qsink, above_tcp=above, lower_tcp=lower, object_placement=placement,
                            hypothetical_attachment=world.attachment)
        except Exception as error:
            row['error'] = repr(error)
        rows.append(row)
        json_write(args.output/f'candidate_{index:03d}.json', row)
        print(json.dumps(dict(candidate=index, offset=offset, passed=row['passed'], endpoint_fcl=row.get('endpoint_fcl'),
            error=row.get('error'))), flush=True)
        if row['passed']:
            selected = index
            break
    code_paths = [Path(__file__).resolve(), ROOT/'kitchen_safety/extensions/replicator/bowl_motion.py',
        ROOT/'kitchen_safety/extensions/replicator/bowl_collision.py', ROOT/'kitchen_safety/extensions/replicator/task_collision.py']
    return dict(accepted=False, execution_ready=False, physical_execution=False, declaration=declaration,
        selected_index=selected, candidate_found=selected is not None, candidates=rows,
        selected_archive_sha256=sha(args.output/'selected_candidate.npz') if selected is not None else None,
        code_snapshots={str(p.relative_to(ROOT)):dict(sha256=sha(p), text=p.read_text()) for p in code_paths},
        remaining=['Actual cuRobo self/world/full free-transfer planning', 'Real physical rim grasp and retained bowl',
            'Matched neutral and all episode gates', 'Original task predicate and sink-floor support after release',
            'Safety variants and evidence; this is baseline target preparation only'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--grasp-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refuse to replace earlier candidates')
    args.output.mkdir(parents=True)
    report = run(args)
    json_write(args.output/'search_report.json', report)
    print(json.dumps(dict(output=str(args.output), candidate_found=report['candidate_found'], selected=report['selected_index'])))


if __name__ == '__main__':
    main()
