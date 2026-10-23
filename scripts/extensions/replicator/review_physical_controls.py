"""CPU recomputation of reset-based hazard and deliberately forced door probes.

No state is changed. Saved native wrench calls establish commanded inputs only;
they are not measured solver reaction forces or robot task success.
"""
import argparse
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise ValueError(message)


def hazard_motion(poses):
    poses = np.asarray(poses)
    require(poses.ndim == 2 and poses.shape[1] == 7 and len(poses) >= 2 and
            np.isfinite(poses).all(), 'Missing finite hazard poses')
    require(np.allclose(np.linalg.norm(poses[:, 3:], axis=1), 1., atol=1e-5, rtol=0),
            'Invalid hazard quaternion')
    rotation = Rotation.from_quat(poses[:, 3:])
    translation = np.linalg.norm(poses[:, :3]-poses[0, :3], axis=1)
    angle = np.rad2deg((rotation*rotation[0].inv()).magnitude())
    upright = np.rad2deg(np.arccos(np.clip(rotation.as_matrix()[:, 2, 2], -1, 1)))
    vertical = np.abs(poses[:, 2]-poses[0, 2])
    checks = dict(translation_from_reset=float(translation.max()) < .002,
                  vertical_from_reset=float(vertical.max()) < .002,
                  rotation_from_reset=float(angle.max()) <= 2.,
                  upright_absolute=float(upright.max()) <= 2.)
    return dict(passed=all(checks.values()), checks=checks, samples=len(poses), reference='true reset initial pose',
                max_translation_m=float(translation.max()), max_translation_sample=int(translation.argmax()),
                max_vertical_m=float(vertical.max()), max_rotation_deg=float(angle.max()),
                max_upright_deg=float(upright.max()),
                thresholds=dict(translation_strict_m=.002, vertical_strict_m=.002, rotation_deg=2., upright_deg=2.))


def audit_complete_dynamic_hazard(episode):
    from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence
    episode = Path(episode)
    result = json.loads((episode/'result.json').read_text())
    require(result.get('obstacle') and result['obstacle']['static'] is False,
            'Full dynamic hazard review requires an actual dynamic obstacle')
    timeline = audit_saved_collection_evidence(episode)
    require(timeline['passed'], 'Incomplete or mismatched absolute physics timeline')
    report = json.loads((episode/'startup_audit.json').read_text())
    meta = report['metadata']
    require(report == result['startup_recording'] and report['passed'] is True and
            meta['dynamic_hazard'] is True, 'Startup/result hazard binding differs')
    with np.load(episode/'startup_physics_states.npz', allow_pickle=False) as prefix, \
         np.load(episode/'physics_states.npz', allow_pickle=False) as suffix, \
         np.load(episode/'motion_physics_index.npz', allow_pickle=False) as index:
        require(np.all(prefix['has_hazard_state']) and np.all(prefix['has_hazard_pose']) and
                np.all(suffix['has_hazard_pose']), 'Missing dynamic hazard physical sample')
        require(np.array_equal(prefix['hazard_state'][:, :7], prefix['hazard_pose']),
                'Startup dynamic state/pose mismatch')
        poses = np.concatenate([prefix['hazard_pose'], suffix['hazard_pose']])
        steps = np.concatenate([prefix['physics_step'], index['physics_step']])
        require(len(steps) == len(poses) and steps[0] == meta['reset_physics_step'] and
                np.all(np.diff(steps) == 1), 'Hazard timeline is not reset-to-end continuous')
    with h5py.File(episode/'trajectory.h5', 'r') as h:
        dt = float(h.attrs['dt'])
        decimation = round(dt/meta['physics_dt_s'])
        require(decimation > 0 and abs(dt-decimation*meta['physics_dt_s']) < 1e-10,
                'Hazard observation clock mismatch')
        boundaries = (8+np.arange(len(h['time'])+1))*decimation
        require(boundaries[-1] == len(poses)-1 and
                np.array_equal(h['obs/hazard_pose'][:], poses[boundaries[:-1]]) and
                np.array_equal(h['next_obs/hazard_pose'][:], poses[boundaries[1:]]),
                'Hazard raw physics/observation boundaries differ')
    value = hazard_motion(poses)
    value.update(schema_version=1, episode=str(episode), first_physics_step=int(steps[0]),
                 last_physics_step=int(steps[-1]), max_translation_physics_step=int(steps[value['max_translation_sample']]),
                 continuous_collection_evidence=timeline,
                 input_sha256={name: sha(episode/name) for name in ['result.json', 'startup_audit.json',
                     'startup_physics_states.npz', 'physics_states.npz', 'motion_physics_index.npz',
                     'motion_physics_alignment.json', 'collection_provenance.json', 'trajectory.h5']})
    return value


def mobility_measurements(states, trace, report):
    """Recompute original gates and every actual successful native API call."""
    require(report['schema_version'] == 2 and trace['schema_version'] == 1, 'Need newly recorded wrench trace')
    paths = states['paths']
    require(paths == trace['paths'] == report['paths'] and len(paths) == len(set(paths)), 'Mobility body map differs')
    indices = {name: next(i for i, p in enumerate(paths) if p.endswith(suffix)) for name, suffix in
               [('left', 'door_left_01_obj_00'), ('right', 'door_right_01_obj_00'), ('housing', 'body_01_obj_00')]}
    require(indices == trace['target_body_indices'] and len(set(indices.values())) == 3, 'Mobility targets differ')
    rows, physical, calls = states['rows'], states['physical_rows'], trace['calls']
    require(rows == report['rows'] and len(rows) == 90, 'Mobility action evidence differs')
    dt, physics_dt, decimation = float(trace['action_dt']), float(trace['physics_dt']), trace['decimation']
    require(dt == report['dt'] and physics_dt == report['physics_dt'] and type(decimation) is int and decimation > 0 and
            abs(dt*30-2) < 1e-10 and abs(physics_dt*decimation-dt) < 1e-10, 'Mobility clock differs')
    start, end = states['reset_physics_step'], states['final_physics_step']
    require(start == trace['reset_physics_step'] and end == trace['final_physics_step'] and end-start == 90*decimation and
            len(physical) == end-start+1 and len(calls) == end-start == report['force_applications'],
            'Missing physical state or native wrench call')
    phases = ['neutral']*30 + ['known_external_torque']*30 + ['coasting_after_torque']*30
    for j, row in enumerate(physical):
        require(row['physics_step'] == start+j and row['action_index'] == ((j-1)//decimation if j else -1) and
                row['phase'] == (phases[(j-1)//decimation] if j else 'reset_initial'), 'Physical identity/phase differs')
    poses = np.asarray([r['poses'] for r in physical])
    velocities = np.asarray([r['velocities'] for r in physical])
    q = np.asarray([r['robot_q'] for r in physical])
    require(poses.shape == (len(physical), len(paths), 7) and velocities.shape == (len(physical), len(paths), 6) and
            q.ndim == 2 and q.shape[0] == len(physical) and np.isfinite(poses).all() and np.isfinite(velocities).all() and
            np.isfinite(q).all() and np.allclose(np.linalg.norm(poses[:, :, 3:], axis=2), 1., atol=1e-5, rtol=0),
            'Invalid full physical states')
    for i, row in enumerate(rows):
        require(row['i'] == i and row['phase'] == phases[i] and row['time_s'] == i*dt and
                row['physics_step'] == start+i*decimation and
                all(row[key] == physical[i*decimation][key] for key in ['poses', 'velocities', 'robot_q']),
                'Action state does not match physical state')
    for j, call in enumerate(calls):
        i = j//decimation
        expected_torque = np.zeros((len(paths), 3), dtype=np.float32)
        if phases[i] == 'known_external_torque':
            expected_torque[indices['left'], 2] = .3
            expected_torque[indices['right'], 2] = -.3
        require(call['call_index'] == j and call['physics_step_before'] == start+j and
                call['action_index'] == i and call['phase'] == phases[i] and
                call['body_indices'] == list(range(len(paths))) and call['position_data'] is None and call['is_global'] is True and
                np.array_equal(call['force_world_n'], np.zeros((len(paths), 3))) and
                np.array_equal(call['torque_world_nm'], expected_torque), 'Native wrench input/target/step differs')
    code_sha = hashlib.sha256(trace['source_text'].encode()).hexdigest()
    require(code_sha == trace['initial_source_sha256'] == trace['checked_source_sha256'] == report['source_sha256'],
            'Mobility execution source changed')
    changes, original = {}, {}
    for name, index in indices.items():
        angle = np.rad2deg((Rotation.from_quat(poses[:, index, 3:])*Rotation.from_quat(poses[0, index, 3:]).inv()).magnitude())
        translation = np.linalg.norm(poses[:, index, :3]-poses[0, index, :3], axis=1)
        changes[name] = dict(initial_hold_max_rotation_deg=float(angle[:30*decimation+1].max()),
            rotation_at_pulse_end_deg=float(angle[60*decimation]), maximum_rotation_deg=float(angle.max()),
            maximum_translation_m=float(translation.max()))
        action_indices = np.arange(90)*decimation
        original[name] = dict(initial_hold_max_rotation_deg=float(angle[action_indices[:31]].max()),
            rotation_at_pulse_end_deg=float(angle[60*decimation]), maximum_rotation_deg=float(angle[action_indices].max()),
            maximum_translation_m=float(translation[action_indices].max()))
    require(original == report['changes'], 'Claimed mobility rotation/translation arithmetic differs')
    checks = dict(doors_neutral_before_torque=all(changes[n]['initial_hold_max_rotation_deg'] < 1 for n in ['left', 'right']),
                  doors_both_movable=all(5 < changes[n]['rotation_at_pulse_end_deg'] < 60 for n in ['left', 'right']),
                  housing_remains_installed=changes['housing']['maximum_rotation_deg'] < 1 and changes['housing']['maximum_translation_m'] < .002,
                  reset_to_end_states_and_native_calls_complete=True, execution_source_unchanged=True)
    return dict(passed=all(checks.values()), checks=checks, physical_samples=len(physical), native_calls=len(calls),
                pulse_calls=30*decimation, changes=changes, source_sha256=code_sha,
                scope='All saved physical states and actual successful native commanded-wrench calls; no solver reaction-force measurement.')


def audit_mobility(directory):
    directory = Path(directory)
    report = json.loads((directory/'mobility_probe.json').read_text())
    require(report['passed'] is True and all(value is True for value in report['checks'].values()), 'Failed original mobility probe')
    require(set(report['input_sha256']) == {'recorded_states.json', 'wrench_calls.json'}, 'Mobility raw hash binding missing')
    for name, expected in report['input_sha256'].items():
        require(sha(directory/name) == expected, 'Changed mobility raw evidence: '+name)
    states = json.loads((directory/'recorded_states.json').read_text())
    trace = json.loads((directory/'wrench_calls.json').read_text())
    require(states['background_physics_provenance'] == report['background_physics_provenance'], 'Mobility raw physical profile differs')
    fresh = mobility_measurements(states, trace, report)
    require(fresh == report['raw_physical_review'], 'Mobility raw recomputation differs')
    return fresh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--dynamic-episode', type=Path)
    group.add_argument('--mobility', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    try:
        result = audit_complete_dynamic_hazard(args.dynamic_episode) if args.dynamic_episode else audit_mobility(args.mobility)
    except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration) as error:
        result = dict(passed=False, error=f'{type(error).__name__}: {error}')
    result['review_code_sha256'] = sha(__file__)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ['continuous_collection_evidence', 'input_sha256']}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
