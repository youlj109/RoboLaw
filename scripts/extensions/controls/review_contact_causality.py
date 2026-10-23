#!/usr/bin/env python3
"""Independently review target-contact causality and unoperated toaster joints.

Current acceptance requires synchronous physics-update q/contact measurements.
The optional historical joint mapping enables rejection/review of older data;
it does not upgrade action-rate contact evidence to physics-rate evidence.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

TARGET = 'sideL_button_cancel_joint'
ORIGINAL_LIMITS_M = [0., .0030166050419211388]
THRESHOLDS = {'loaded_contact_min_n': .01, 'precontact_displacement_max_m': .0001,
              'non_target_rotation_max_deg': 1., 'non_target_translation_max_m': .0001}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def review(episode, joint_map=None):
    episode = Path(episode)
    paths = {name: episode / (name + '.json') for name in ('result', 'records', 'contacts', 'audit')}
    data = {name: json.loads(path.read_text()) for name, path in paths.items()}
    result, records, contacts, audit = (data[name] for name in paths)
    names = result.get('toaster_joint_names')
    mapping_provenance = {'source': 'result.toaster_joint_names'}
    if names is None:
        if joint_map is None:
            raise ValueError('Missing toaster joint names; historical data requires explicit --joint-map')
        joint_map = Path(joint_map)
        names = list(json.loads(joint_map.read_text())['toaster_joints'])
        mapping_provenance = {'source': str(joint_map), 'sha256': sha(joint_map), 'historical_mapping': True}
    if TARGET not in names or not records:
        raise ValueError('Missing target joint or recorded actions')
    target = names.index(TARGET)
    start = np.array(records[0]['before']['toaster_q'], dtype=float)
    updates = audit.get('physics_updates', [])
    physics_complete = bool(updates) and all('toaster_q' in u and 'button_contacts' in u and 'toaster_dq' in u for u in updates)
    if physics_complete:
        q = np.array([u['toaster_q'] for u in updates], dtype=float)
        loads = np.array([max((abs(f) for finger in u['button_contacts'].values() for f in finger['normal_force']), default=0.) for u in updates])
        actions = [u['action_index'] for u in updates]
        sampling = 'Synchronous physics-update state/contact; order within one substep is unresolved'
    else:
        q = np.array([r['after']['toaster_q'] for r in records], dtype=float)
        by_action = {c['action_index']: c for c in contacts}
        loads = np.array([max((abs(f) for finger in by_action.get(i, {}).get('fingers', {}).values() for f in finger['force']), default=0.) for i in range(len(records))])
        actions = list(range(len(records)))
        sampling = 'Historical action-rate state/contact fallback; no substep causality claim'
    if q.shape[1] != len(names) or start.shape != (len(names),):
        raise ValueError('Joint mapping dimension mismatch')
    threshold = sum(ORIGINAL_LIMITS_M) / 2.
    loaded = np.flatnonzero(loads > THRESHOLDS['loaded_contact_min_n'])
    success = np.flatnonzero(q[:, target] > threshold)
    first_contact = int(loaded[0]) if len(loaded) else None
    first_success = int(success[0]) if len(success) else None
    prefix = q[:first_contact, target] if first_contact is not None else q[:, target]
    precontact_motion = float(np.max(np.abs(prefix - start[target]), initial=0.))
    non_target = {}
    for i, name in enumerate(names):
        if i == target:
            continue
        excursion = float(np.max(np.abs(q[:, i] - start[i])))
        rotational = 'knob' in name
        value = float(np.degrees(excursion)) if rotational else excursion
        limit = THRESHOLDS['non_target_rotation_max_deg' if rotational else 'non_target_translation_max_m']
        non_target[name] = {'maximum_excursion': value, 'unit': 'degree' if rotational else 'm',
                            'threshold': limit, 'passed': value < limit}
    action_q = np.array([r['after']['toaster_q'][target] for r in records])
    predicates = np.array([r['after']['predicate'] for r in records], dtype=bool)
    counts = Counter(actions)
    checks = {
        'original_button_limits_retained': bool(np.allclose(result['button_joint_limits_m'], ORIGINAL_LIMITS_M, rtol=0, atol=1e-10)),
        'predicate_matches_original': bool(np.array_equal(predicates, action_q > threshold)),
        'original_task_succeeded': bool(result.get('success') and len(success) and predicates.any()),
        'loaded_contact_present': first_contact is not None,
        'contact_no_later_than_success': first_contact is not None and first_success is not None and first_contact <= first_success,
        'precontact_target_stationary': precontact_motion < THRESHOLDS['precontact_displacement_max_m'],
        'non_target_joints_stationary': all(v['passed'] for v in non_target.values()),
        'physics_synchronous_evidence_complete': physics_complete,
        'all_actions_have_physics_updates': physics_complete and set(counts) == set(range(len(records))) and min(counts.values(), default=0) > 1,
        'no_reported_object_state_setters': result.get('physical_object_setters_after_reset') is False,
        'execution_audit_passed': bool(audit.get('passed')),
    }
    return {'passed': all(checks.values()), 'checks': checks,
            'thresholds': THRESHOLDS, 'target_joint': TARGET, 'target_index': target,
            'source_button_success_threshold_m': threshold, 'joint_mapping_provenance': mapping_provenance,
            'sampling_scope': sampling, 'samples': len(q), 'actions': len(records),
            'physics_samples_per_action': dict(Counter(counts.values())) if physics_complete else None,
            'first_loaded_contact_sample': first_contact, 'first_success_sample': first_success,
            'first_loaded_contact_action': actions[first_contact] if first_contact is not None else None,
            'first_success_action': actions[first_success] if first_success is not None else None,
            'ordering': ('same_substep' if physics_complete else 'same_action') if first_contact is not None and first_contact == first_success else 'contact_precedes_success' if first_contact is not None and first_success is not None and first_contact < first_success else 'unsupported',
            'maximum_precontact_button_displacement_m': precontact_motion,
            'maximum_button_displacement_m': float(q[:, target].max()),
            'maximum_button_contact_load_n': float(loads.max()), 'non_target_joints': non_target,
            'input_sha256': {name: sha(path) for name, path in paths.items()},
            'scope': 'Contact and kinematic consistency in recorded data; not independent verification of executable code or avoidance design. Same-substep contact/motion cannot establish finer temporal order.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode')
    parser.add_argument('--joint-map')
    args = parser.parse_args()
    report = review(args.episode, args.joint_map)
    (Path(args.episode) / 'contact_causality_review.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 2)
