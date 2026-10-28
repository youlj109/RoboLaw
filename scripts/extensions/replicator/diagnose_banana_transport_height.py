#!/usr/bin/env python3
"""Bound finite banana transport-height diagnosis; never simulation/execution.

Default preserves the recorded close attachment and changes only the explicitly
declared transport height. A measured post-lift attachment is a separate opt-in
counterfactual. All original world/self/joint/FCL checks remain required.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.diagnose_task_plan import reconstruct
from scripts.extensions.replicator.task_audit_sidecars import ROOT, load_episode, sha
from scripts.extensions.replicator.search_banana_grasp_stability_cpu import write


def bind_candidate(episode, declaration_path, candidate_id, attachment_mode):
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance
    from scripts.extensions.replicator.review_task_episode import contact_window
    declaration = json.loads(declaration_path.read_text())
    metadata, data, _ = load_episode(episode)
    provenance = audit_provenance(episode/'task_collection_provenance.json', metadata, int(data['physics_step'][-1]))
    if not provenance['passed'] or metadata['task_id'] != 'kitchen_bench_replicator_g_shape_banana_bowl':
        raise ValueError('Require original banana source task and complete execution provenance')
    for name, digest in declaration['input_sha256'].items():
        if sha(episode/name) != digest:
            raise ValueError('Finite declaration belongs to different input: '+name)
    selected = [row for row in declaration['finite_candidates'] if row['id'] == candidate_id]
    if len(selected) != 1:
        raise ValueError('Require exactly one explicitly predeclared height candidate')
    selected = selected[0]
    world, kin, start, original_target, _, reconstruction = reconstruct(episode, 'transport')
    index = reconstruction['physics_index']
    if index != len(data['physics_step'])-1 or str(data['phase'][index]) != 'lift':
        raise ValueError('Require final actual lift-end, before any transport execution')
    actual_tcp = pose_matrix(data['tcp_pose_w'][index])
    actual_payload = pose_matrix(data['task_object_states'][index, metadata['task_object_names'].index('banana'), :7])
    bowl = data['task_object_states'][index, metadata['task_object_names'].index('bowl'), :7]
    measured_attachment = np.linalg.inv(actual_tcp)@actual_payload
    original_attachment = world.attachment.copy()
    if not np.allclose(selected['planning_attachment'], measured_attachment, rtol=0., atol=1e-12):
        raise ValueError('Finite height declaration measured attachment differs')
    desired = actual_payload.copy()
    desired[:2, 3] = bowl[:2]
    desired[2, 3] = selected['payload_root_z_m']
    if not np.allclose(selected['desired_payload'], desired, rtol=0., atol=1e-12):
        raise ValueError('Finite candidate changed source bowl XY, actual orientation or declared height')
    if not any(lo < desired[2, 3] < hi for lo, hi in declaration['conservative_carried_envelope_free_bands_m']):
        raise ValueError('Candidate is outside its declared conservative carried-envelope band')
    if attachment_mode not in ('old-close', 'measured-postlift'):
        raise ValueError('Unknown attachment mode')
    world.attachment = original_attachment if attachment_mode == 'old-close' else measured_attachment
    target = desired@np.linalg.inv(world.attachment)
    result = json.loads((episode/'result.json').read_text())
    contacts = json.loads((episode/'grasp_contacts.json').read_text())
    loaded = contact_window(data, metadata, contacts, 'banana', result['grasp_target']['contact_region_local_m'], 'lift')
    if not loaded['passed']:
        raise ValueError('Actual last 10 lift substeps lack original bilateral loaded-region evidence')
    report = dict(accepted=False, task_success=False, physical_execution=False, execution_authorized=False,
        cpu_observation_and_target_passed=True, fresh_path_validated=False,
        episode=str(episode), predeclared_candidate_id=candidate_id, target_tcp=target,
        desired_payload=desired, old_target_tcp=original_target,
        original_close_attachment=original_attachment, measured_postlift_attachment=measured_attachment,
        planning_attachment=world.attachment, attachment_mode=attachment_mode,
        preserves_original_recorded_attachment=attachment_mode == 'old-close',
        actual_lift_last10_contact_evidence=loaded, reconstruction=reconstruction, provenance=provenance,
        original_source_bowl_xy=bowl[:2], unchanged_actual_payload_orientation=actual_payload[:3, :3],
        actual_six_finger_angles=world.gripper_joint_positions,
        input_sha256={n:sha(episode/n) for n in ['physics_metadata.json','physics_states.npz','trajectory.npz',
            'result.json','grasp_contacts.json','task_collection_provenance.json']},
        declaration_path=str(declaration_path), declaration_sha256=sha(declaration_path),
        scope='Finite source-bound height hypothesis only. Original close attachment is default; source XY, grasp, layout, margins and robot geometry unchanged. No rollout, source state write, task success, or acceptance.')
    return world, kin, start, target, metadata, report


def dense_samples(path, resolution=.02):
    rows = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        count = max(2, int(np.ceil(np.max(np.abs(b-a))/resolution))+1)
        rows.extend(a+(b-a)*f for f in np.linspace(0., 1., count)[1:])
    return np.asarray(rows)


def run(args):
    world, kin, start, target, metadata, report = bind_candidate(
        args.episode, args.declaration, args.candidate, args.attachment_mode)
    sources = [Path(__file__).resolve(), ROOT/'scripts/extensions/replicator/diagnose_task_plan.py',
        ROOT/'kitchen_safety/extensions/replicator/source_support_curobo.py',
        ROOT/'kitchen_safety/extensions/replicator/task_curobo.py',
        ROOT/'kitchen_safety/extensions/replicator/task_plan_diagnostics.py',
        ROOT/'kitchen_safety/extensions/replicator/task_collision.py',
        ROOT/'kitchen_safety/curobo_backend.py', ROOT/'kitchen_safety/planning.py']
    report['diagnostic_source_sha256'] = {str(p.relative_to(ROOT)):sha(p) for p in sources}
    report['gpu_requested'] = args.gpu
    write(args.output/'bound_observation.json', report)
    if not args.gpu:
        return report
    from kitchen_safety.extensions.replicator.source_support_curobo import SourceSupportCuroboPlanner
    from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components, sphere_pair_diagnostics
    planner = SourceSupportCuroboPlanner(world, kin, dt=metadata['action_dt_s'], seed=args.seed, max_joint_speed=.4)
    try:
        planner._prepare_path_query((), 1, 'transport')
        report['original_start_cuda_components'], state = constraint_components(planner, start[None])
        if not report['original_start_cuda_components']['valid']:
            report['start_sphere_pairs'], spheres = sphere_pair_diagnostics(planner, state, report['original_start_cuda_components'])
            np.savez_compressed(args.output/'start_failed_spheres.npz', spheres=spheres)
        plan = planner.plan_pose(start, target, grip_binary=1, phase='transport')
        path = np.asarray(plan['positions'])
        dense = dense_samples(path)
        np.savez_compressed(args.output/'transport_candidate.npz', positions=path, dense_positions=dense,
            requested_target=target, effective_goal_pose=plan['effective_goal_pose'], start_q=start,
            actual_six_fingers=world.gripper_joint_positions, planning_attachment=world.attachment)
        report['candidate_sha256'] = sha(args.output/'transport_candidate.npz')
        report['plan_metadata'] = plan['metadata']
        report['original_path_validation'] = planner.validate_path(path, grip_binary=1, phase='transport', resolution=.02)
        report['all_dense_original_cuda_components'], state = constraint_components(planner, dense)
        report['all_dense_source_fcl'] = []
        for i, q in enumerate(dense):
            valid = world.valid(q, mode='planning', gripper_q=world.gripper_joint_positions)
            report['all_dense_source_fcl'].append(dict(sample=i, passed=valid, failure=deepcopy(world.last_clearance)))
        if not report['all_dense_original_cuda_components']['valid']:
            report['failed_sphere_pairs'], spheres = sphere_pair_diagnostics(planner, state, report['all_dense_original_cuda_components'])
            np.savez_compressed(args.output/'failed_spheres.npz', spheres=spheres)
        actual_goal = kin.fk(path[-1])
        report['endpoint_error'] = dict(position_m=float(np.linalg.norm(actual_goal[:3,3]-target[:3,3])),
            rotation_deg=float(np.rad2deg(Rotation.from_matrix(target[:3,:3]@actual_goal[:3,:3].T).magnitude())))
        report['candidate_contact_mask'] = planner.candidate_contact_mask
        report['fresh_path_validated'] = bool(report['original_path_validation']['valid'] and
            report['original_path_validation']['validation_complete'] and
            report['all_dense_original_cuda_components']['valid'] and
            all(r['passed'] for r in report['all_dense_source_fcl']) and
            report['endpoint_error']['position_m'] < .005 and report['endpoint_error']['rotation_deg'] < 2.)
    except Exception as exc:
        report.update(error=f'{type(exc).__name__}: {exc}', fresh_path_validated=False)
    report['diagnostic_source_sha256_end'] = {str(p.relative_to(ROOT)):sha(p) for p in sources}
    if report['diagnostic_source_sha256_end'] != report['diagnostic_source_sha256']:
        report.update(error='Diagnostic execution source changed', fresh_path_validated=False)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--declaration', type=Path, required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--attachment-mode', choices=['old-close','measured-postlift'], default='old-close')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--gpu', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        if args.gpu:
            import fcntl
            lock_path = ROOT/'reports/skill_trials/gpu.lock'
            if ROOT != Path('/workspace') or not lock_path.exists():
                raise RuntimeError('CUDA diagnostic requires canonical mounted workspace and lock')
            with lock_path.open('a+') as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                report = run(args)
        else:
            report = run(args)
    except Exception as exc:
        write(args.output/'failure.json', dict(accepted=False, physical_execution=False, error=repr(exc)))
        raise
    write(args.output/'diagnostic.json', report)
    print(json.dumps(dict(output=str(args.output), cpu_observation_and_target_passed=True,
        gpu_requested=args.gpu, fresh_path_validated=report['fresh_path_validated'], error=report.get('error'))))
    return 0 if not args.gpu or report['fresh_path_validated'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
