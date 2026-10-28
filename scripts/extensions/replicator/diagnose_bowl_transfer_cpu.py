#!/usr/bin/env python3
"""CPU sink endpoint and lowering check after a hypothetical source rim grasp.

No transfer trajectory or physical grasp is invented. The transport endpoint
has an IK candidate only; cuRobo must still plan the full free-transfer path.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_motion import sink_placement_object_pose
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
from scripts.extensions.replicator.audit_bowl_grasp_cpu import path_check
from scripts.extensions.replicator.diagnose_task_plan import cpu_sphere_audit
from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha
from scripts.extensions.replicator.task_collect import json_write


def run(args):
    source = json.loads(args.grasp_report.read_text())
    if not source['closure']['passed'] or source['layout_sha256'] != sha(args.layout):
        raise ValueError('Requires a matching CPU rim-grasp candidate, without a physical-success claim')
    layout = json.loads(args.layout.read_text())
    world, kin, _ = build_world(layout)
    close = source['closure']
    qclose = np.asarray(close['q'])
    selected = close['rows'][close['selected_index']]
    angle = selected['angle_rad']
    world.gripper_joint_position = angle
    world.gripper_joint_positions = np.asarray([1., 1., -1., 1., -1., -1.])*angle
    world.attachment = np.linalg.inv(kin.fk(qclose))@world.payload_pose
    lift_target = kin.fk(qclose)
    lift_target[2, 3] += .15
    world.set_phase('lift')
    lift = local_cartesian_candidate(kin, qclose, lift_target, 'lift', 1/15, .4)
    qlift = lift['positions'][-1]
    carried = kin.fk(qlift)@world.attachment
    carried_pose = np.r_[carried[:3, 3], Rotation.from_matrix(carried[:3, :3]).as_quat()]
    placement, geometry = sink_placement_object_pose(carried_pose, world.payload_geometry, world)
    above_object = placement.copy()
    above_object[2, 3] = carried[2, 3]
    above = above_object@np.linalg.inv(world.attachment)
    lower = placement@np.linalg.inv(world.attachment)
    declaration = dict(layout_sha256=sha(args.layout), grasp_report_sha256=sha(args.grasp_report),
        source_grasp_candidate_only=True, hypothetical_attachment=world.attachment,
        source_sink_floor_target=geometry, above_sink_object_pose=above_object, lower_object_pose=placement,
        above_sink_tcp=above, lower_tcp=lower, release_gap_m=.003,
        transport_policy='Keep source-lift object height, original sink floor XY centre and observed hypothetical bowl orientation',
        noncontact_clearance_m=.002, unchanged_robot_sphere_buffer_m=.005)
    json_write(args.output/'declaration.json', declaration)
    report = dict(accepted=False, execution_ready=False, physical_execution=False, declaration=declaration)
    world.set_phase('transport')
    qsink = kin.ik(above, qlift)
    report['above_sink_endpoint'] = dict(q=qsink, full_fcl=world.valid(qsink), violation=world.last_clearance,
        actual_fk=kin.fk(qsink), full_transfer_path_verified=False)
    report['above_sink_endpoint']['cpu_spheres'], _ = cpu_sphere_audit(world, kin, [qsink])
    world.set_phase('lower')
    candidate = local_cartesian_candidate(kin, qsink, lower, 'lower', 1/15, .4)
    report['lower'] = dict(metadata=candidate['metadata'], full_fcl=path_check(world, candidate['positions']))
    report['lower']['cpu_spheres'], _ = cpu_sphere_audit(world, kin, candidate['positions'])
    np.savez_compressed(args.output/'candidate_paths.npz', lift=lift['positions'], above_sink_endpoint=qsink,
        lower=candidate['positions'], hypothetical_attachment=world.attachment)
    report['path_archive_sha256'] = sha(args.output/'candidate_paths.npz')
    report['remaining'] = ['Full cuRobo transfer path, exact original CUDA joint/self/world gates',
        'No source-support or destination object exemption authorized by this diagnostic',
        'Fresh neutral, actual grip/lift/lower/release and original sink support predicate',
        'All physical/RGB/visual and causal-avoidance gates']
    paths = [Path(__file__).resolve(), ROOT/'kitchen_safety/extensions/replicator/bowl_motion.py',
        ROOT/'kitchen_safety/extensions/replicator/bowl_collision.py', ROOT/'kitchen_safety/extensions/replicator/task_collision.py']
    report['code_snapshots'] = {str(p.relative_to(ROOT)):dict(sha256=sha(p), text=p.read_text()) for p in paths}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--grasp-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refuse to overwrite earlier candidate evidence')
    args.output.mkdir(parents=True)
    try:
        report = run(args)
    except Exception as error:
        json_write(args.output/'failure.json', dict(accepted=False, error=repr(error)))
        raise
    json_write(args.output/'diagnostic.json', report)
    print(json.dumps(dict(output=str(args.output), endpoint_fcl=report['above_sink_endpoint']['full_fcl'],
        lower_fcl=report['lower']['full_fcl']['passed'], accepted=False)))


if __name__ == '__main__':
    main()
