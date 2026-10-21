"""Read completed lower-first attempts without rerunning simulation or audits.

    bash scripts/arena_python.sh scripts/summarize_lower_first.py

Saved verification decisions remain unchanged. Current manifest acceptance and
later geometry revalidation are reported separately from historical passes.
State statistics are descriptive,
including failed attempts whose result.json could not be written. RGB is read
from completed images.h5, with CPU FFmpeg frame extraction as a fallback.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys

import h5py
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.summarize_obstacle_library_collection import CAMERAS, image_grid

CARRY_PHASES = ('lift', 'transfer_front', 'transfer', 'above_bowl', 'lower')
REQUIRED_PHASES = ('settle', 'pregrasp', 'approach', 'close', 'lift',
                   'above_bowl', 'lower', 'release', 'retreat', 'settle')


def relative(path):
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def load(path, errors):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as error:
        errors.append({'path': relative(path), 'error': f'{type(error).__name__}: {error}'})
        return None


def acceptance_context(manifest, geometry_report, geometry_report_path):
    """Join acceptance/revalidation by asset AND candidate, never by asset alone.

    A completed supplemental report can expose a pending manifest withdrawal,
    but this read-only presentation never writes an acceptance decision back.
    A historical candidate's review cannot affect a later accepted candidate.
    """
    reviews = {}
    if geometry_report and geometry_report.get('complete') is True:
        for episode in geometry_report.get('episodes', []):
            candidate = episode.get('candidate', episode.get('label'))
            if episode.get('obstacle_id') and candidate:
                reviews[(episode['obstacle_id'], candidate)] = {
                    'candidate': candidate,
                    'passed': episode.get('passed_added_geometry', episode.get('passed')),
                    'report': relative(geometry_report_path),
                    'directory': episode.get('directory'),
                    'source': 'completed supplemental geometry report',
                    **{key: episode.get(key) for key in (
                        'minimum_clearance_m', 'failed_state_count', 'intersecting_state_count',
                        'states_checked', 'scope_note', 'state_scope', 'first_failure')},
                }

    def entries(value):
        return list(value) if isinstance(value, list) else [value] if isinstance(value, dict) else []

    # Explicit manifest records take precedence for their exact candidate.
    for asset in manifest.get('assets', []):
        values = entries(asset.get('geometry_revalidation'))
        for accepted in [asset.get('accepted'), *asset.get('superseded_acceptances', [])]:
            if accepted:
                values += entries(accepted.get('geometry_revalidation'))
        for review in values:
            if review.get('candidate'):
                key = (asset['id'], review['candidate'])
                reviews[key] = {**reviews.get(key, {}), **review,
                                'source': 'manifest geometry_revalidation'}

    assets = {}
    for asset in manifest.get('assets', []):
        accepted = asset.get('accepted')
        current = reviews.get((asset['id'], accepted.get('candidate'))) if accepted else None
        pending = bool(accepted and current and current.get('passed') is False)
        assets[asset['id']] = {
            'id': asset['id'], 'accepted': accepted,
            'reported_current_accepted': None if pending else accepted,
            'acceptance_state': ('withdrawal_pending_geometry_failure' if pending else
                                 'current_accepted' if accepted else 'not_currently_accepted'),
            'superseded_acceptances': asset.get('superseded_acceptances', []),
            'geometry_revalidation': asset.get('geometry_revalidation'),
            'current_candidate_geometry_revalidation': current,
            'geometry_revalidation_history': [review for (identifier, _), review in reviews.items()
                                              if identifier == asset['id']],
        }
    return assets, reviews


def acceptance_for_run(asset, candidate, role, original_status, geometry_review):
    """Present current video acceptance while retaining the original status."""
    accepted = asset.get('accepted') or {}
    is_current = role == 'video' and accepted.get('candidate') == candidate
    old = next((entry for entry in asset.get('superseded_acceptances', [])
                if role == 'video' and entry.get('candidate') == candidate), None)
    if is_current:
        status = ('withdrawal_pending_geometry_failure'
                  if geometry_review and geometry_review.get('passed') is False else 'current_accepted')
    elif old is not None:
        status = 'superseded_acceptance'
    elif role == 'video' and geometry_review and geometry_review.get('passed') is False:
        status = 'geometry_revalidation_failed'
    elif original_status == 'verified_passed':
        status = 'historical_verification_passed' if role == 'video' else 'verified_probe'
    else:
        status = original_status
    return {'status': status, 'original_verification_status': original_status,
            'listed_as_current_accepted_in_manifest': is_current,
            'current_accepted': status == 'current_accepted',
            'superseded_acceptance': old,
            # Supplemental geometry reviews cover videos, not their probes.
            'geometry_revalidation': geometry_review if role == 'video' else None}


def tilt_degrees(quaternions):
    q = np.asarray(quaternions, dtype=float)
    lengths = np.linalg.norm(q, axis=-1, keepdims=True)
    if not np.isfinite(q).all() or np.any(lengths <= 0):
        raise ValueError('Invalid recorded bottle quaternion')
    q = q / lengths
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[..., 0] ** 2 + q[..., 1] ** 2), -1, 1)))


def phase_blocks(phases, dt):
    starts = [0] + (np.flatnonzero(phases[1:] != phases[:-1]) + 1).tolist()
    stops = starts[1:] + [len(phases)]
    return [{'phase': str(phases[a]), 'first_frame': a, 'last_frame': b - 1,
             'actions': b - a, 'duration_s': (b - a) * dt}
            for a, b in zip(starts, stops)]


def review_trajectory(directory):
    with h5py.File(directory / 'trajectory.h5') as data:
        phases = data['phase'].asstr()[:]
        n = len(phases)
        if n == 0:
            raise ValueError('No recorded trajectory actions')
        dt = float(data.attrs['dt'])
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid recorded action period')
        action_dtype = str(data['action'].dtype)
        action = np.asarray(data['action'][:, :7], dtype=float)
        obs = {name: data['obs/' + name][:] for name in ('q', 'tcp', 'mustard', 'bowl')}
        next_obs = {name: data['next_obs/' + name][:] for name in obs}
        if action.shape != (n, 7) or any(len(value) != n for value in [*obs.values(), *next_obs.values()]):
            raise ValueError('Trajectory field lengths disagree')
        names = json.loads(data.attrs['joint_names'])
        finger = names.index('finger_joint')
        times = data['time'][:] if 'time' in data else np.arange(n) * dt

    blocks = phase_blocks(phases, dt)
    wanted = iter(REQUIRED_PHASES)
    expected = next(wanted, None)
    for block in blocks:
        if block['phase'] == expected:
            expected = next(wanted, None)
    final_settle = blocks[-1]['duration_s'] if blocks[-1]['phase'] == 'settle' and 'release' in phases else 0.
    report = {
        'frames': n, 'action_dt_s': dt, 'duration_s': n * dt,
        'phase_blocks': blocks, 'last_recorded_phase': str(phases[-1]),
        'required_phase_order_present': expected is None,
        'required_phase_order': list(REQUIRED_PHASES),
        'final_settle_duration_s': final_settle,
        'note': 'Phase coverage is descriptive and does not replace the saved execution/verification result.',
    }
    speed = np.abs(np.diff(action, axis=0)) / dt
    seams = np.flatnonzero(phases[1:] != phases[:-1])

    def speed_sample(index, joint):
        sample = {'from_frame': int(index), 'to_frame': int(index + 1),
                  'from_phase': str(phases[index]), 'to_phase': str(phases[index + 1]),
                  'joint': f'panda_joint{joint + 1}',
                  'commanded_speed_rad_s': float(speed[index, joint]),
                  'is_phase_boundary': bool(phases[index] != phases[index + 1])}
        # Recover the recorded dtype before measuring its spacing. Taking the
        # ULP after conversion to float64 would understate float32 rounding.
        positions = np.asarray(action[[index, index + 1], joint], dtype=action_dtype)
        if np.issubdtype(positions.dtype, np.floating):
            ulps = np.abs(np.spacing(positions)).astype(np.float64)
            sample['stored_position_values_rad'] = positions.tolist()
            sample['stored_position_ulps_rad'] = ulps.tolist()
            sample['round_to_nearest_speed_uncertainty_bound_rad_s'] = float(ulps.sum() / (2 * dt))
        return sample

    peak = None
    if len(speed):
        index, joint = np.unravel_index(np.argmax(speed), speed.shape)
        peak = speed_sample(index, joint)
    report['commanded_joint_speed'] = {
        'definition': 'Absolute difference of consecutive saved arm position commands divided by action dt; includes all phase seams, excludes the binary gripper command.',
        'stored_action_dtype': action_dtype,
        'numerical_note': 'Raw speeds and phase boundaries are retained. For the maximum sample and each phase-boundary sample, the recorded-position round-to-nearest speed uncertainty is (ULP(q_previous) + ULP(q_current)) / (2 * dt), using the original action dtype. This describes possible recording quantization, not a new speed tolerance or acceptance decision; it does not prove the unrounded command value.',
        'first_action_note': 'The command preceding action[0] is not recorded, so its transition cannot be reconstructed.',
        'maximum_rad_s': float(speed.max()) if len(speed) else None,
        'maximum_sample': peak,
        'maximum_sample_round_to_nearest_speed_uncertainty_bound_rad_s':
            peak.get('round_to_nearest_speed_uncertainty_bound_rad_s') if peak else None,
        'per_joint_maximum_rad_s': speed.max(axis=0).tolist() if len(speed) else None,
        'phase_boundaries': [speed_sample(i, int(np.argmax(speed[i]))) for i in seams],
        'per_destination_phase_maximum_rad_s': {
            str(phase): float(speed[phases[1:] == phase].max())
            for phase in dict.fromkeys(phases[1:])},
        'recorded_timestamp_interval_range_s': [float(np.diff(times).min()), float(np.diff(times).max())]
            if n > 1 else None,
    }
    carry = np.flatnonzero(np.isin(phases, CARRY_PHASES))
    if len(carry):
        bottle = np.stack([obs['mustard'][carry], next_obs['mustard'][carry]], axis=1).reshape(-1, 13)
        tilt = tilt_degrees(bottle[:, 3:7])
        peak_index = int(np.argmax(tilt))
        action_index = int(carry[peak_index // 2])
        report['actual_carried_bottle_tilt'] = {
            'source': 'Observed rigid-body quaternions, including obs and next_obs of every carrying action; no planned attachment prediction.',
            'phases': list(CARRY_PHASES), 'observed_states_including_repeated_boundaries': len(tilt),
            'maximum_deg': float(tilt.max()), 'minimum_deg': float(tilt.min()),
            'maximum_sample': {'action_index': action_index, 'phase': str(phases[action_index]),
                               'state_source': 'next_obs' if peak_index % 2 else 'obs'},
            'constraint_scope': 'The existing 28 degree carrying limit concerns these carrying phases. After release, bottle inclination is reported separately and is not subject to a new final-upright requirement.',
        }
    count = min(n, max(1, int(np.ceil(4. / dt - 1e-9))))
    start = n - count
    terminal = {name: np.concatenate([values[start:], next_obs[name][-1:]]) for name, values in obs.items()}
    bottle, bowl = terminal['mustard'], terminal['bowl']
    delta = bottle[:, :3] - bowl[:, :3]
    linear_speed = np.linalg.norm(bottle[:, 7:10], axis=1)
    angular_speed = np.linalg.norm(bottle[:, 10:13], axis=1)
    tilt = tilt_degrees(bottle[:, 3:7])
    report['last_four_seconds'] = {
        'first_obs_frame': start, 'duration_s': count * dt,
        'includes_final_next_obs': True, 'states': len(bottle),
        'phases': list(dict.fromkeys(phases[start:].tolist())),
        'is_final_settle_window': bool(final_settle >= 4. - 1e-8 and expected is None),
        'interpretation': 'For failed/incomplete attempts this is only their final recorded window, not evidence that placement or settling completed.',
        'bottle_max_position_drift_from_window_start_m': float(np.linalg.norm(bottle[:, :3] - bottle[0, :3], axis=1).max()),
        'bottle_world_xyz_peak_to_peak_m': np.ptp(bottle[:, :3], axis=0).tolist(),
        'bottle_relative_to_bowl_xyz_peak_to_peak_m': np.ptp(delta, axis=0).tolist(),
        'bottle_linear_speed_range_m_s': [float(linear_speed.min()), float(linear_speed.max())],
        'bottle_angular_speed_range_rad_s': [float(angular_speed.min()), float(angular_speed.max())],
        'bottle_tilt_range_deg': [float(tilt.min()), float(tilt.max())],
        'final_bottle_pose_xyz_xyzw': bottle[-1, :7].tolist(),
        'final_bottle_relative_to_bowl_m': delta[-1].tolist(),
        'final_bottle_linear_speed_m_s': float(linear_speed[-1]),
        'final_bottle_tilt_deg': float(tilt[-1]),
        'final_tilt_interpretation': 'Released bottle orientation is descriptive. The existing task requires supported placement in the bowl, not an upright final bottle.',
        'final_finger_joint_rad': float(terminal['q'][-1, finger]),
        'support_limitation': 'Trajectory fields do not include filtered bowl support force. The final_support_evidence section preserves the collector\'s final predicate and force matrix when available; it does not establish continuous support over this whole window.',
    }
    for phase in ('close', 'lift', 'lower', 'release', 'retreat'):
        ids = np.flatnonzero(phases == phase)
        if len(ids):
            a, b = int(ids[0]), int(ids[-1])
            report.setdefault('phase_end_states', {})[phase] = {
                'first_frame': a, 'last_frame': b,
                'tcp_start_world_m': obs['tcp'][a].tolist(),
                'tcp_end_world_m': next_obs['tcp'][b].tolist(),
                'bottle_end_pose_xyz_xyzw': next_obs['mustard'][b, :7].tolist(),
                'bottle_end_speed_m_s': float(np.linalg.norm(next_obs['mustard'][b, 7:10])),
                'finger_joint_start_rad': float(obs['q'][a, finger]),
                'finger_joint_end_rad': float(next_obs['q'][b, finger]),
            }
    return report, phases


def support_evidence(result):
    """Describe the collector's final support observation without a new gate."""
    matrix = result.get('final_destination_contact_force_matrix_w_n')
    report = {
        'recorded_final_on_destination': result.get('final_on_destination'),
        'recorded_final_destination_contact_force_matrix_w_n': matrix,
        'source': 'Final physical-state Arena object_on_destination predicate and destination-filtered contact sensor matrix saved by the collector.',
        'interpretation': 'An upward destination contact force supports the placement observation. This is one final sample, not a time series or a new acceptance test; no final-upright constraint is added.',
    }
    if matrix is not None:
        forces = np.asarray(matrix, dtype=float)
        if forces.size and forces.shape[-1] == 3 and np.isfinite(forces).all():
            total = forces.reshape(-1, 3).sum(axis=0)
            norm = float(np.linalg.norm(total))
            report.update({
                'sum_force_world_xyz_n': total.tolist(),
                'sum_force_norm_n': norm,
                'sum_force_upward_z_n': float(total[2]),
                'sum_force_angle_to_world_up_deg': float(np.degrees(np.arccos(np.clip(total[2] / norm, -1, 1)))) if norm else None,
                'sum_note': 'Sum over the saved matrix entries; the source matrix is preserved above.',
            })
        else:
            report['force_summary_note'] = 'Empty, nonfinite, or unexpected force matrix shape; no aggregate was computed.'
    return report


def metadata_differences(expected, actual, prefix=''):
    if isinstance(expected, dict) and isinstance(actual, dict):
        differences = []
        for key in sorted(set(expected) | set(actual)):
            name = f'{prefix}.{key}' if prefix else key
            if key not in expected or key not in actual:
                differences.append(name)
            else:
                differences.extend(metadata_differences(expected[key], actual[key], name))
        return differences
    return [] if expected == actual else [prefix]


def ffmpeg_frame(video, index, destination):
    executable = shutil.which('ffmpeg')
    if not executable:
        import imageio_ffmpeg
        executable = imageio_ffmpeg.get_ffmpeg_exe()
    result = subprocess.run([
        executable, '-v', 'error', '-i', str(video), '-vf', f'select=eq(n\\,{index})',
        '-frames:v', '1', '-vsync', '0', '-y', str(destination),
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
    if result.returncode or not destination.exists():
        raise RuntimeError(f'FFmpeg extraction failed: {video}: {result.stderr.strip()}')


def contact_sheet(directory, destination, run, phases, audit, errors):
    n = len(phases)
    selected = {}
    for phase in ('close', 'lift', 'above_bowl', 'lower', 'release', 'retreat'):
        indices = np.flatnonzero(phases == phase)
        if not len(indices):
            continue
        minimum = audit.get('phase_minima', {}).get(phase, {}).get('sample_index')
        fallback = indices[len(indices) // 2] if phase in ('above_bowl', 'release') else indices[-1]
        selected[phase] = int(minimum) if minimum is not None and minimum in indices else int(fallback)
    minimum = audit.get('minimum_clearance_sample', {}).get('sample_index')
    if minimum is not None and 0 <= minimum < n:
        selected['closest_human'] = int(minimum)
    selected['final_recorded_obs'] = n - 1
    destination.mkdir(parents=True, exist_ok=True)
    image_file = None
    if (directory / 'images.h5').exists():
        try:
            image_file = h5py.File(directory / 'images.h5')
        except OSError as error:
            errors.append({'path': relative(directory / 'images.h5'), 'error': str(error)})
    rows, frames = [], {}
    try:
        for kind, index in selected.items():
            item = {'frame': index, 'phase': str(phases[index]), 'images': {}, 'sources': {}}
            for camera in CAMERAS:
                path = destination / f'{kind}_{camera}.jpg'
                try:
                    if image_file is not None and camera in image_file and index < len(image_file[camera]['jpeg']):
                        im = Image.open(io.BytesIO(image_file[camera]['jpeg'][index].tobytes())).convert('RGB')
                        im.save(path, quality=94)
                        method = 'images.h5:obs-aligned JPEG'
                    else:
                        ffmpeg_frame(directory / (camera + '.mp4'), index, path)
                        method = 'CPU FFmpeg:frame-index selection'
                    item['images'][camera] = relative(path)
                    item['sources'][camera] = method
                except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
                    errors.append({'path': relative(path), 'error': f'{type(error).__name__}: {error}'})
            frames[kind] = item
            if len(item['images']) == len(CAMERAS):
                rows.append((f'{run["status"]} | {kind}', item))
    finally:
        if image_file is not None:
            image_file.close()
    return {
        'frames': frames,
        'contact_sheet': image_grid(rows, destination / 'three_cameras.jpg',
                                  f'{run["obstacle_id"]} | {run["label"]} | {run["status"]}'),
        'selection_note': 'Use saved per-phase audit minima when available, otherwise phase midpoint/end; final RGB is obs, not final next_obs. No audit was rerun.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='datasets/kitchen_auto/human_lower_first_validation')
    parser.add_argument('--output', default='reports/human_lower_first_validation')
    parser.add_argument('--reference', default='datasets/kitchen_auto/obstructed_property_vertical/property/attempt_0001_seed_43')
    parser.add_argument('--no-images', action='store_true')
    parser.add_argument('--geometry-review', default='reports/human_lower_first_validation/guide_geometry_accepted_review_primitive183.json',
                        help='Optional completed supplemental geometry report; matched by asset and candidate.')
    args = parser.parse_args()
    source, output, reference = [(ROOT / value).resolve() for value in (args.input, args.output, args.reference)]
    errors = []
    manifest = load(source / 'manifest.json', errors) or {}
    geometry_path = (ROOT / args.geometry_review).resolve()
    geometry_report = load(geometry_path, errors)
    assets, geometry_reviews = acceptance_context(manifest, geometry_report, geometry_path)
    reference_cameras = load(reference / 'cameras.json', errors)
    embedded = {}
    for asset in manifest.get('assets', []):
        for trial in asset.get('trials', []):
            for role in ('probe', 'video'):
                if trial.get(role):
                    embedded[(asset['id'], trial['candidate'], role)] = trial[role]
    directories = {path for role in ('probe', 'video') for path in source.glob(f'*/*/{role}') if path.is_dir()}
    directories.update(source / asset / label / role for asset, label, role in embedded)
    runs = []
    for directory in sorted(directories):
        identifier, label, role = directory.parent.parent.name, directory.parent.name, directory.name
        result = load(directory / 'result.json', errors)
        verification = load(directory / 'verification.json', errors)
        verification_source = 'verification.json' if verification is not None else 'manifest embedded record'
        verification = verification or embedded.get((identifier, label, role), {})
        execution = load(directory / 'execution.json', errors) or verification.get('execution', {})
        finished = execution.get('exit_code') is not None or bool(verification) or result is not None
        status = ('in_progress_or_unfinished' if not finished else
                  'result_missing_or_unreadable' if result is None else
                  'verified_passed' if verification.get('passed') is True else
                  'physical_failed' if result.get('success') is False else
                  'verification_incomplete_or_failed')
        run = {
            'obstacle_id': identifier, 'label': label, 'role': role,
            'directory': relative(directory), 'status': status,
            'result_file_exists': (directory / 'result.json').exists(), 'result_readable': result is not None,
            'recorded_physical_success': (result or {}).get('success'),
            'recorded_arena_success': (result or {}).get('arena_success'),
            'final_support_evidence': support_evidence(result or {}),
            'verification': verification or None,
            'verification_source': verification_source if verification else None,
            'execution': execution or None, 'finished_record_present': finished,
            'recorded_failure_phase': (result or {}).get('failure_phase', verification.get('failure_phase')),
            'recorded_error': (result or {}).get('error') or verification.get('error'),
            'requested_grasp': (result or {}).get('requested_grasp'),
            'placement': (result or {}).get('placement'),
            'selected_plans': (result or {}).get('plans'),
            'physics_clearance_checks': (result or {}).get('physics_clearance_checks'),
            'recorded_max_arm_contact_force_n': (result or {}).get('max_arm_contact_force_n'),
            'videos': {name: relative(directory / (name + '.mp4'))
                       for name in (*CAMERAS, 'all_cameras') if (directory / (name + '.mp4')).exists()},
        }
        run.update(acceptance_for_run(assets.get(identifier, {}), label, role, status,
                                     geometry_reviews.get((identifier, label))))
        if not finished:
            run['read_note'] = 'No completed execution/result/verification record; trajectory and RGB are not opened.'
            runs.append(run)
            continue
        cameras = load(directory / 'cameras.json', errors)
        run['camera_review'] = {
            'reference': relative(reference / 'cameras.json'),
            'recorded_camera_names': sorted(cameras) if cameras else [],
            'metadata_matches_original_three_cameras': cameras == reference_cameras
                if cameras is not None and reference_cameras is not None else None,
            'different_fields': metadata_differences(reference_cameras, cameras)
                if cameras is not None and reference_cameras is not None else None,
            'note': 'Compare mounts, intrinsics, prim paths and resolution; world camera pose changes with robot motion.',
        }
        audit = load(directory / 'audit.json', errors) or {}
        episode_audit = (audit.get('episodes') or [{}])[0]
        run['saved_clearance_audit'] = {key: episode_audit.get(key) for key in
            ('states_checked', 'minimum_clearance_m', 'minimum_clearance_sample', 'phase_minima',
             'failed_sample_count', 'intersecting_sample_count')}
        try:
            run['trajectory_review'], phases = review_trajectory(directory)
            if role == 'video' and not args.no_images:
                run['visual_review'] = contact_sheet(directory, output / identifier / label / role,
                                                     run, phases, episode_audit, errors)
        except (OSError, ValueError, KeyError, IndexError) as error:
            run['trajectory_review_error'] = f'{type(error).__name__}: {error}'
            errors.append({'path': relative(directory / 'trajectory.h5'), 'error': run['trajectory_review_error']})
        runs.append(run)

    counts = Counter(run['status'] for run in runs)
    current_count = sum(bool(asset['reported_current_accepted']) for asset in assets.values())
    manifest_count = sum(bool(asset['accepted']) for asset in assets.values())
    pending_count = sum(asset['acceptance_state'] == 'withdrawal_pending_geometry_failure'
                        for asset in assets.values())
    superseded_count = sum(len(asset['superseded_acceptances']) for asset in assets.values())
    report = {
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'Read-only descriptive review of every recorded physical probe/video. '
                 'Current manifest acceptance, superseded acceptance and later candidate-specific geometry '
                 'reviews are distinct from original verification passes. A listed acceptance with an explicit '
                 'failed later geometry review is reported as pending withdrawal, not a current success. '
                 'No manifest or original verification files are changed; no simulation or audit reruns.',
        'source_manifest': relative(source / 'manifest.json'),
        'source_manifest_complete_at_read': manifest.get('complete'),
        'source_manifest_accepted_count': manifest.get('accepted_count'),
        'current_accepted_count': current_count,
        'all_current_assets_accepted': current_count == len(assets) if assets else None,
        'manifest_current_accepted_count': manifest_count,
        'pending_manifest_withdrawal_count': pending_count,
        'superseded_acceptance_count': superseded_count,
        'supplemental_geometry_report': relative(geometry_path) if geometry_report else None,
        'assets': list(assets.values()),
        'counts': {'physical_run_directories': len(runs), 'probe_runs': sum(r['role'] == 'probe' for r in runs),
                   'video_runs': sum(r['role'] == 'video' for r in runs), 'by_status': dict(counts)},
        'runs': runs, 'read_errors': errors,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    lines = ['Lower-first 抓放验证：当前接受结果及历史记录', '',
             f'当前可报告接受：{current_count}/{len(assets)}；manifest 列出接受 {manifest_count}；'
             f'几何复查失败、待同步撤回 {pending_count}；已撤回历史接受 {superseded_count}。',
             f'全部资产当前已接受：{current_count == len(assets) if assets else None}；历史失败不改变此项。',
             f'数据目录：{relative(source)}；最近批次完成：{manifest.get("complete")}',
             f'物理运行目录 {len(runs)}；状态：{dict(counts)}',
             '原 verification.passed 仅表示当时验证；不自动计入当前接受。复查只关联同一资产、同一候选的视频。',
             '本报告不重跑审计、不写 manifest、不改原始验证文件；缺失 result 不推断为成功。', '']
    lines.extend(['当前四项结果与视频（固定布局，非成功率统计）', ''])
    for run in runs:
        if not run['current_accepted']:
            continue
        trajectory = run.get('trajectory_review', {})
        final = trajectory.get('last_four_seconds', {})
        lines.append(f'{run["obstacle_id"]} / {run["label"]}')
        lines.append(f'  {trajectory.get("frames")} 帧；实际携瓶最大倾角 '
                     f'{trajectory.get("actual_carried_bottle_tilt", {}).get("maximum_deg")}°；'
                     f'记录人体最小间距 {run["saved_clearance_audit"].get("minimum_clearance_m")} m')
        lines.append(f'  非手指最大净接触力 {run["recorded_max_arm_contact_force_n"]} N；'
                     f'最终瓶倾角 {final.get("final_bottle_tilt_deg")}°；'
                     f'最终在碗上 {run["final_support_evidence"]["recorded_final_on_destination"]}')
        lines.append('  原三相机同步视频：' + str(run['videos'].get('all_cameras')))
    lines.extend(['', '逐项验收与历史尝试', ''])
    for asset in assets.values():
        accepted = asset['reported_current_accepted']
        lines.append(f'{asset["id"]}：当前接受={accepted.get("candidate") if accepted else None}；'
                     f'状态={asset["acceptance_state"]}')
        for old in asset['superseded_acceptances']:
            lines.append(f'  撤回历史接受 {old.get("candidate")}：{old.get("reason")}')
        for review in asset['geometry_revalidation_history']:
            detail = (f'失败状态={review["failed_state_count"]}；'
                      f'最小间距={review.get("minimum_clearance_m")}m；'
                      if review.get('failed_state_count') is not None else
                      f'检查方式={review.get("method", "见原验证记录")}；')
            lines.append(f'  几何复查 {review.get("candidate")}：passed={review.get("passed")}；'
                         f'{detail}报告={review.get("report")}')
        lines.append('')
    for run in runs:
        lines.append(f'{run["obstacle_id"]} / {run["label"]} / {run["role"]}: {run["status"]}')
        lines.append(f'  原验证状态={run["original_verification_status"]}；当前接受={run["current_accepted"]}')
        if run.get('superseded_acceptance'):
            lines.append('  撤回原因：' + str(run['superseded_acceptance'].get('reason')))
        if run.get('geometry_revalidation'):
            geometry = run['geometry_revalidation']
            lines.append(f'  后续几何复查 passed={geometry.get("passed")}；'
                         f'失败状态={geometry.get("failed_state_count")}；报告={geometry.get("report")}')
        review = run.get('trajectory_review', {})
        if review:
            lines.append('  阶段：' + ' → '.join(f'{p["phase"]}({p["actions"]})' for p in review['phase_blocks']))
            speed = review['commanded_joint_speed']['maximum_rad_s']
            tilt = review.get('actual_carried_bottle_tilt', {}).get('maximum_deg')
            lines.append(f'  {review["frames"]} 帧；最大指令速度(含接缝) {speed} rad/s；实际携瓶最大倾角 {tilt}°')
            final = review['last_four_seconds']
            lines.append(f'  最后窗口 {final["duration_s"]:.3f}s；完整末段settle={final["is_final_settle_window"]}；'
                         f'瓶最大漂移={final["bottle_max_position_drift_from_window_start_m"]:.6f}m；'
                         f'最终速度={final["final_bottle_linear_speed_m_s"]:.6f}m/s')
            lines.append(f'  放手后最终瓶倾角 {final["final_bottle_tilt_deg"]:.3f}°；运输的28°限制不作为落碗后竖直门槛。')
        support = run['final_support_evidence']
        lines.append(f'  已记录最终碗支撑判据={support["recorded_final_on_destination"]}；'
                     f'最后样本支撑力Z分量={support.get("sum_force_upward_z_n")} N')
        if run.get('recorded_error'):
            lines.append('  原记录错误：' + str(run['recorded_error']))
        if run.get('trajectory_review_error'):
            lines.append('  H5读取错误：' + run['trajectory_review_error'])
        if run.get('camera_review'):
            lines.append('  原三相机配置一致：' + str(run['camera_review']['metadata_matches_original_three_cameras']))
        sheet = run.get('visual_review', {}).get('contact_sheet')
        if sheet:
            lines.append('  关键帧：' + sheet)
        lines.append('  ' + run['directory'])
    (output / 'summary.txt').write_text('\n'.join(lines) + '\n')
    print(json.dumps({'output': relative(output), 'current_accepted_count': current_count,
                      'manifest_current_accepted_count': manifest_count,
                      'pending_manifest_withdrawal_count': pending_count,
                      'counts': report['counts'], 'read_errors': len(errors)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
