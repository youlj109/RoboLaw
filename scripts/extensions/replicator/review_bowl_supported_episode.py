#!/usr/bin/env python3
"""Independent CPU audit of the original bowl/sink expected-support candidate.

No Kit, GPU, replayed actions or runtime support-policy evaluator is used. All
original pairs are recomputed from actual 7+6 joints, object and background
poses. Original source intersection refusals remain in the report. Only the
exact original bowl/BasinBottom pair can receive the archived, independently
recomputed expected-contact classification. This tool never grants delivery
acceptance and does not replace the separate actual-q CUDA or visual review.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BODY = '/World/envs/env_0/bowl'
FLOOR = '/World/envs/env_0/kitchen/Sink/BasinBottom'
SUPPORT = {'release', 'retreat', 'final_settle'}
TARGET_PHASES = SUPPORT | {'lower'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False,
                  default=lambda v: v.item() if isinstance(v, np.generic) else v.tolist())
        stream.write('\n')


def archived_policy(provenance, result):
    rows = [(k, v) for k, v in provenance['entries'].items()
            if v['kind'] == 'target_support_policy_input']
    require(len(rows) == 1, 'Exactly one archived target support policy required')
    name, row = rows[0]
    raw = row['source_text'].encode()
    require(hashlib.sha256(raw).hexdigest() == row['initial_sha256'] == row['checked_sha256'],
            'Archived support policy bytes/start/end differ')
    policy = json.loads(raw)
    expected = dict(schema_version=1, task_id='kitchen_bench_replicator_g_shape_bowl_sink',
        payload=BODY, target=FLOOR, noncontact_clearance_m=.002, lower_clearance_m=.002,
        release_gap_window_m=[.003, .005], landing_xy_tolerance_m=.005, release_seconds=2.,
        retreat_height_m=.15, final_settle_seconds=4., force_threshold_n=.1,
        support_cone_half_angle_rad=float(np.pi/4), velocity_threshold_m_s=.003,
        placement_consecutive_steps=1, physics_dt_s=1/120, action_dt_s=1/15, contact_capacity=4096,
        unloaded_transition_rule='complete_native_contacts_all_separations_nonnegative_and_no_downward_aggregate')
    require(all(policy.get(k) == v for k, v in expected.items()), 'Archived original rules differ')
    require(result['target_support_policy'] == policy and
            result['target_support_policy_sha256'] == row['initial_sha256'], 'Result/policy mismatch')
    for path, expected_sha in policy['evidence_sha256'].items():
        require(sha(ROOT/path) == expected_sha, 'Support lineage changed: '+path)
    return policy, dict(passed=True, path=name, sha256=row['initial_sha256'],
                       archived_bytes_verified=True, original_fixed_rules=expected)


def native_check(row, step, phase, action_index, original_force):
    """Verify raw inventory and recompute vectors; no force-difference gate."""
    n = int(row['count'])
    p = np.asarray(row['points_world_m'], float).reshape(-1, 3)
    normals = np.asarray(row['normals_world'], float).reshape(-1, 3)
    f = np.asarray(row['normal_forces_n'], float)
    gap = np.asarray(row['separations_m'], float)
    counts, starts = np.asarray(row['buffer_count']), np.asarray(row['buffer_start'])
    checks = dict(exact_pair=row.get('exact_body') == BODY and row.get('exact_filter') == FLOOR,
        exact_step=row['physics_step'] == step, exact_phase=row['phase'] == phase,
        exact_action=row['action_index'] == action_index,
        complete_finite=n >= 0 and len(p) == len(normals) == len(f) == len(gap) == n and
            all(np.isfinite(x).all() for x in (p, normals, f, gap, original_force)),
        buffer_one_original_pair=counts.size == starts.size == 1 and
            int(counts.flat[0]) == n and int(starts.flat[0]) >= 0 and
            int(starts.flat[0])+n < 4096 and row['buffer_capacity'] == 4096)
    contributions = f[:, None]*normals
    summed = contributions.sum(0)
    for key, value in [('native_contact_force_vectors_n', contributions),
                       ('native_normal_force_vector_sum_n', summed),
                       ('original_filtered_force_n', original_force),
                       ('native_minus_original_force_n', summed-original_force)]:
        actual = np.asarray(row[key], float).reshape(value.shape)
        checks[key+'_recomputed'] = bool(np.array_equal(actual, value))
    checks['force_difference_norm_recomputed'] = row['native_original_force_difference_norm_n'] == float(np.linalg.norm(summed-original_force))
    return dict(passed=all(checks.values()), checks=checks, count=n,
                recomputed_native_force_n=summed.tolist(), original_force_n=np.asarray(original_force).tolist(),
                native_sensor_difference_n=float(np.linalg.norm(summed-original_force)),
                minimum_separation_m=float(gap.min()) if n else None,
                scalar_sign_assumed=False, difference_is_diagnostic_only=True)


def classify(source, native, force, low, high):
    """Independent interpretation of the archived exact-pair contact rules."""
    p = np.asarray(native['points_world_m'], float).reshape(-1, 3)
    normal = np.asarray(native['normals_world'], float).reshape(-1, 3)
    scalar, gaps = np.asarray(native['normal_forces_n']), np.asarray(native['separations_m'])
    complete = len(p) == len(normal) == len(scalar) == len(gaps) == native['count'] and all(
        np.isfinite(x).all() for x in (p, normal, scalar, gaps, force))
    summed = (scalar[:, None]*normal).sum(0) if complete else np.zeros(3)
    def up(v):
        length = np.linalg.norm(v)
        return bool(length >= .1 and v[2] >= length*np.cos(np.pi/4))
    loaded = bool(complete and up(force) and up(summed))
    transition = bool(complete and len(p) > 0 and np.all(gaps >= 0) and force[2] >= 0 and summed[2] >= 0)
    checks = dict(exact_pair=native.get('exact_body') == BODY and native.get('exact_filter') == FLOOR,
        complete_nonempty=bool(complete and len(p)), supported_or_transition=loaded or transition,
        native_xy=bool(len(p) and np.all(p[:, :2] >= low[:2]) and np.all(p[:, :2] <= high[:2])),
        native_above_entity_bottom=bool(len(p) and np.all(p[:, 2] >= low[2])),
        source_xy=bool(source['full_bowl_xy_inside_floor']),
        source_above_entity_bottom=bool(source['minimum_source_z_m'] >= low[2]))
    return dict(passed=all(checks.values()), checks=checks,
        contact_state='loaded_support' if loaded else ('observed_nonpenetrating_contact_transition' if transition else 'rejected'),
        original_loaded_support=loaded, native_force_n=summed.tolist(),
        original_force_n=np.asarray(force).tolist(), penetration_tolerance_m=None,
        scope='Only original target pair; transition never proves task success or a retreat certificate.')


def check_certificate(cert, step, pose, floor_hash, payload_hash, native, classification):
    body = {k: v for k, v in cert.items() if k != 'certificate_sha256'}
    checks = dict(hash=cert.get('certificate_sha256') == digest(body),
        exact_step=cert['physics_step'] == step, exact_actual_free_payload=np.array_equal(cert['payload_pose'], pose),
        exact_floor=cert['floor_geometry_sha256'] == floor_hash,
        exact_payload_source=cert['payload_source_sha256'] == payload_hash,
        exact_native=cert['native_observation'] == native,
        independently_loaded=classification['passed'] and classification['contact_state'] == 'loaded_support',
        recorded_classification=cert['observed_support']['passed'] == classification['passed'] and
            cert['observed_support']['contact_state'] == classification['contact_state'])
    return dict(passed=all(checks.values()), checks=checks, physics_step=step)


def source_geometry(world, floor, pose):
    tri = world.fcl.CollisionObject(world.payload_triangle_model, world.fcl.Transform(pose[:3, :3], pose[:3, 3]))
    vertices = world.payload_geometry.vertices@pose[:3, :3].T+pose[:3, 3]
    lo, hi = vertices.min(0), vertices.max(0)
    source = dict(source_gap_m=float(lo[2]-floor[3][2]), minimum_source_z_m=float(lo[2]),
        full_bowl_xy_inside_floor=bool(np.all(lo[:2] >= floor[2][:2]) and np.all(hi[:2] <= floor[3][:2])),
        floor_bounds=[floor[2].tolist(), floor[3].tolist()])
    return tri, source


def full_pair_audit(episode, metadata, data, result, policy):
    from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from kitchen_safety.extensions.replicator.bowl_source_support_query import collider_geometry_evidence
    native_doc = read(episode/'target_native_contacts.json')
    observations = read(episode/'target_support_observations.json')
    ledger = read(episode/'target_source_intersection_ledger.json')
    native_rows = native_doc['rows']
    require([r['physics_step'] for r in native_rows] == data['physics_step'].tolist(), 'Native contacts omit/duplicate actual steps')
    recorded = {r['physics_step']: r for r in observations['records']}
    expected_steps = [int(s) for s, p in zip(data['physics_step'], data['phase']) if str(p) in TARGET_PHASES]
    require(list(recorded) == expected_steps, 'Source observations omit/duplicate/order target-phase states')
    require(len(recorded) == len(observations['records']), 'Duplicate source observation')
    require(observations['policy'] == policy, 'Observation policy differs from archived input')
    world, kin, _ = build_world(metadata['source_layout'])
    arms = [metadata['joint_names'].index('panda_joint'+str(i)) for i in range(1, 8)]
    fingers = [metadata['joint_names'].index(n) for n in world.GRIPPER_JOINT_NAMES]
    payload = metadata['task_object_names'].index('bowl')
    floor = next(x for x in world.objects if x[0] == FLOOR)
    binding = collider_geometry_evidence(world, FLOOR)
    floor_hash = binding['geometry_sha256']
    require(binding == observations['exact_floor_binding'] == result['exact_target_floor_binding'], 'Source floor geometry binding differs')
    require(world.payload_geometry.provenance['source_sha256'] == metadata['source_hashes']['bowl'], 'Source bowl changed')
    all_rows, failures, strict_failures, target_rows, native_failures, mismatches = [], [], [], [], [], []
    total_pairs = 0
    def mismatch(step, name, okay):
        if not okay:
            mismatches.append(dict(physics_step=step, field=name))
    for i, value in enumerate(data['physics_step']):
        step, phase = int(value), str(data['phase'][i])
        pose = pose_matrix(data['task_object_states'][i, payload, :7])
        force = data['destination_contact_force'][i].reshape(-1, 3).sum(0)
        native = native_rows[i]
        ncheck = native_check(native, step, phase, int(data['action_index'][i]), force)
        if not ncheck['passed']:
            native_failures.append(dict(physics_step=step, **ncheck))
        world.set_phase(phase); world.payload_pose = pose; world.attachment = None
        world.gripper_joint_positions = data['q'][i, fingers].copy()
        world.gripper_joint_position = float(world.gripper_joint_positions[0])
        world.sync_background(metadata['background_paths'], data['background_rigid_poses'][i])
        # Original _pairs enumerates and sorts all AABB lower-bound candidates.
        # Positive bounds prove separation; every nonpositive pair is exact-tested.
        pairs = world._pairs(data['q'][i, arms], (), 'execution', world.gripper_joint_positions, pose, None, max_slack=0.)
        checked = 0
        for pair in pairs:
            if pair[0] > 0:
                break
            if pair[1] == 'payload/bowl' and pair[2] == FLOOR and phase in TARGET_PHASES:
                continue
            measure = world._measurement(pair); checked += 1
            if not measure['valid']:
                failures.append(dict(physics_step=step, phase=phase, measurement=measure))
        total_pairs += checked
        row = dict(physics_step=step, phase=phase, potential_non_target_pairs_checked=checked,
                   native=ncheck)
        if phase in TARGET_PHASES:
            triangle, source = source_geometry(world, floor, pose)
            margin = .002 if phase == 'lower' else 0.
            strict = world._exact_measurement((0., 'payload/bowl', FLOOR, triangle, floor[1], margin))
            source.update(distance_m=strict['distance_m'], intersecting=strict['intersecting'], exact_floor_geometry_sha256=floor_hash)
            observed = recorded[step]
            for name in ('source_gap_m', 'minimum_source_z_m', 'full_bowl_xy_inside_floor', 'floor_bounds', 'exact_floor_geometry_sha256'):
                mismatch(step, 'source.'+name, observed['source'][name] == source[name])
            mismatch(step, 'source.distance_m', abs(observed['source']['distance_m']-source['distance_m']) <= 1e-12)
            mismatch(step, 'source.intersecting', observed['source']['intersecting'] == source['intersecting'])
            mismatch(step, 'strict.valid', observed['original_strict_measurement']['valid'] == strict['valid'])
            previous = None
            if i:
                _, previous = source_geometry(world, floor, pose_matrix(data['task_object_states'][i-1, payload, :7]))
                mismatch(step, 'previous_step', observed['previous_physics_step'] == int(data['physics_step'][i-1]))
                for name in ('source_gap_m', 'minimum_source_z_m', 'full_bowl_xy_inside_floor', 'floor_bounds'):
                    mismatch(step, 'previous_source.'+name, observed['previous_source'][name] == previous[name])
            cls = classify(source, native, force, floor[2], floor[3])
            allowed = bool(phase in SUPPORT and not strict['valid'] and ncheck['passed'] and cls['passed'])
            spatial = phase not in SUPPORT or (source['full_bowl_xy_inside_floor'] and source['minimum_source_z_m'] >= floor[2][2])
            valid = bool(spatial and (strict['valid'] or allowed))
            mismatch(step, 'allowed_original_support', observed['allowed_original_support'] == allowed)
            if not strict['valid'] and phase in SUPPORT:
                strict_failures.append(dict(physics_step=step, phase=phase, strict=strict, classification=cls))
                mismatch(step, 'observed_support.passed', observed.get('observed_support', {}).get('passed') == cls['passed'])
                mismatch(step, 'observed_support.contact_state', observed.get('observed_support', {}).get('contact_state') == cls['contact_state'])
            target = dict(physics_step=step, phase=phase, passed=valid, strict=strict, source=source,
                          previous_source=previous, classification=cls, allowed_expected_contact=allowed)
            target_rows.append(target)
            if not valid:
                failures.append(dict(physics_step=step, phase=phase, target=target))
        all_rows.append(row)
        if i % 500 == 0:
            print('INDEPENDENT_BOWL_FCL', i, '/', len(data['physics_step']), flush=True)
    ledger_steps = [int(x['physics_step']) for x in ledger['original_source_failures']]
    independent_steps = [x['physics_step'] for x in strict_failures]
    require(ledger_steps == independent_steps, 'Original strict source ledger omission/extra/ordering')
    mismatch(-1, 'ledger_full_record_identity', ledger['original_source_failures'] == [recorded[s] for s in ledger_steps])
    certs = observations['certificates']; retreats = [p for p in result['plans'] if p['phase'] == 'retreat']
    cert_reviews = []
    require(len(certs) == len(retreats), 'Retreat plan/certificate count differs')
    for cert, plan in zip(certs, retreats):
        step = int(plan['start_physics_step']); ids = np.flatnonzero(data['physics_step'] == step)
        require(len(ids) == 1, 'Retreat certificate step not actually recorded')
        i = int(ids[0]); pose = pose_matrix(data['task_object_states'][i, payload, :7])
        _, source = source_geometry(world, floor, pose)
        force = data['destination_contact_force'][i].reshape(-1, 3).sum(0)
        cls = classify(source, native_rows[i], force, floor[2], floor[3])
        item = check_certificate(cert, step, pose, floor_hash, metadata['source_hashes']['bowl'], native_rows[i], cls)
        item['checks']['plan_certificate_identical'] = cert == plan['observed_target_support_certificate']
        item['checks']['planning_attachment_absent'] = plan['measured_planning_attachment'] is None
        item['checks']['latest_actual_release_state'] = str(data['phase'][i]) == 'release' and step == max(
            int(s) for s, p in zip(data['physics_step'], data['phase']) if str(p) == 'release')
        item['passed'] = all(item['checks'].values()); cert_reviews.append(item)
    return dict(passed=not failures and not native_failures and not mismatches and all(r['passed'] for r in cert_reviews),
        physics_samples=len(all_rows), checked_samples=len(all_rows), checked_potential_non_target_pairs=total_pairs,
        all_other_original_pairs_passed=not any('measurement' in f for f in failures),
        all_lower_original_2mm_passed=all(r['strict']['valid'] for r in target_rows if r['phase'] == 'lower'),
        native_complete_all_actual_states=not native_failures, failures=failures, native_failures=native_failures,
        recorded_vs_independent_mismatches=mismatches, original_strict_source_failures=strict_failures,
        original_strict_ledger_matches=True, retreat_certificate_reviews=cert_reviews,
        actual_rows=all_rows, target_rows=target_rows, floor_binding=binding,
        scope='Actual-state CPU all-pair audit, original broad-phase and exact FCL; only intended target pair independently reclassified. No CUDA/self/visual claim.')


def audit(args):
    from scripts.extensions.replicator.review_task_episode import review
    from scripts.extensions.replicator.task_audit_sidecars import fixed_audit
    episode = args.episode.resolve()
    metadata, result = read(episode/'physics_metadata.json'), read(episode/'result.json')
    data = dict(np.load(episode/'physics_states.npz', allow_pickle=False))
    trajectory = dict(np.load(episode/'trajectory.npz', allow_pickle=False))
    require(metadata['task_id'] == 'kitchen_bench_replicator_g_shape_bowl_sink' and result['variant'] == 'base', 'Wrong audit scope')
    require(data['physics_step'][0] == 0 and np.all(np.diff(data['physics_step']) == 1), 'Actual step discontinuity')
    for key, value in data.items():
        require(len(value) == len(data['physics_step']) and (value.dtype.kind in 'US' or np.isfinite(value).all()), 'Invalid actual data '+key)
    provenance = read(episode/'task_collection_provenance.json')
    policy, binding = archived_policy(provenance, result)
    # Replay source must match the archived executed original collision code;
    # the independent policy above deliberately does not import the new leaf.
    for name in ('kitchen_safety/extensions/replicator/bowl_collision.py',
                 'kitchen_safety/extensions/replicator/task_collision.py', 'kitchen_safety/extensions/replicator/collision_planner.py',
                 'kitchen_safety/planning.py', 'kitchen_safety/extensions/replicator/bowl_motion.py'):
        require(sha(ROOT/name) == provenance['entries'][name]['initial_sha256'] == provenance['entries'][name]['checked_sha256'], 'Replay dependency changed: '+name)
    fcl = full_pair_audit(episode, metadata, data, result, policy)
    fcl_path = args.output.with_name(args.output.stem+'_all_pairs.json')
    write(fcl_path, fcl)
    fixed = fixed_audit(metadata, data)
    fixed_path = args.output.with_name(args.output.stem+'_fixed.json')
    fixed.update(kind='fixed', input_sha256={n: sha(episode/n) for n in ('physics_states.npz', 'physics_metadata.json', 'trajectory.npz')})
    write(fixed_path, fixed)
    generic = review(episode, args.neutral, fixed_audit=fixed_path, self_audit=args.self_audit, visual_review=args.visual_review)
    generic_path = args.output.with_name(args.output.stem+'_generic_unchanged.json')
    write(generic_path, generic)
    checks = dict(archived_original_policy=binding['passed'], independent_all_pair_actual_replay=fcl['passed'],
                  unchanged_generic_original_evidence=generic['passed'])
    out = dict(schema_version=1, episode=str(episode.relative_to(ROOT)), accepted=False,
        independent_evidence_passed=all(checks.values()), task_success_independently_supported=all(checks.values()),
        checks=checks, policy_binding=binding,
        all_pairs=dict(path=str(fcl_path), sha256=sha(fcl_path), passed=fcl['passed'],
            physics_samples=fcl['physics_samples'], retained_original_strict_failures=len(fcl['original_strict_source_failures'])),
        unchanged_generic=dict(path=str(generic_path), sha256=sha(generic_path), passed=generic['passed'],
            checks=generic['checks'], failed_or_missing=generic['failed_or_missing']),
        input_sha256={n: sha(episode/n) for n in ('result.json', 'physics_states.npz', 'physics_metadata.json',
            'trajectory.npz', 'target_native_contacts.json', 'target_support_observations.json',
            'target_source_intersection_ledger.json', 'task_collection_provenance.json')},
        review_source_sha256=sha(__file__),
        scope='Independent CPU support-aware evidence audit. Generic result unchanged and preserved. Delivery acceptance requires root review; missing/failed original predicate, self, visual or complete-action gates remain failures.')
    write(args.output, out)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--neutral', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--self-audit', type=Path)
    p.add_argument('--visual-review', type=Path)
    args = p.parse_args(argv)
    require(not args.output.exists(), 'Never overwrite an independent audit')
    result = audit(args)
    print(json.dumps({k: result[k] for k in ('checks', 'independent_evidence_passed', 'accepted')}))
    return 0 if result['independent_evidence_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
