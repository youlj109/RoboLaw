"""Assemble independent left-button gates without upgrading missing evidence."""
import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def probe_videos(directory, frames, dt):
    from kitchen_safety.dataset import CAMERA_NAMES, decode_video
    try:
        if frames <= 0 or dt <= 0:
            raise ValueError('Missing probe frame/time metadata')
        counts = {name: decode_video(directory/(name+'.mp4'), frames, expected_dt=dt)
                  for name in (*CAMERA_NAMES, 'all_cameras')}
        return {'passed': True, 'frames': counts, 'dt': dt}
    except Exception as exc:
        return {'passed': False, 'error': str(exc)}


def paired_gates(report):
    rows = report.get('runs', [])
    expected = {(seed, variant) for seed in (42, 43, 44)
                for variant in ('base', 'human', 'property', 'robot')}
    return {
        'complete_12_unique_pairs': len(rows) == 12 and
            {(row['seed'], row['variant']) for row in rows} == expected,
        'all_dense_plans_valid': bool(rows) and all(row.get('success') and
            row.get('validation', {}).get('valid') for row in rows),
        'all_hazard_off_paths_require_adjustment': bool(rows) and all(
            row.get('true_shape_counterfactual', {}).get('violations') and
            row.get('true_shape_actual', {}).get('samples', 0) > 0 and
            not row.get('true_shape_actual', {}).get('violations')
            for row in rows if row['variant'] != 'base'),
        'human_asset_manifest_verified': report.get('asset_manifest_verification', {})
            .get('human', {}).get('passed') is True,
    }


def episode_report(path, baseline):
    names = ['result', 'independent_validation', 'contact_causality_review',
             'self_collision_detailed_audit', 'mechanism_audit', 'audit']
    reports = {name: read(path / (name + '.json')) for name in names}
    gates = {name: value.get('accepted' if name == 'result' else 'passed') is True
             for name, value in reports.items()}
    result = reports['result']
    if result.get('variant') != 'base':
        strict = read(path/'strict_clearance_audit.json')
        gates['strict_clearance_audit'] = strict.get('executed_augmented_trajectory', {}).get('passed') is True
        for name in ('fixed_base_hazard_audit', 'avoidance_audit'):
            reports[name] = read(path / (name + '.json'))
            gates[name] = reports[name].get('accepted' if name == 'avoidance_audit' else 'passed') is True
        comparison = read(path / 'comparison_validation.json')
        gates['comparison_video_decoded'] = comparison.get('frames', 0) > 0 and comparison.get('video') == comparison.get('frames')
        gates['comparison_uses_matching_baseline'] = bool(comparison.get('baseline_video')) and Path(
            comparison.get('baseline_video', '')).parent.resolve() == baseline.resolve()
    records = read(path / 'records.json')
    frames = len(records)
    gates['initialization_recorded'] = result.get('initial_setup_actions_recorded') == 8 and \
        result.get('physics_audit_includes_setup') is True and \
        reports['independent_validation'].get('initialization_recorded') is True
    causal = reports['contact_causality_review']
    physics_dt = result.get('physics_dt', 0.)
    first = causal.get('first_success_sample')
    first_time = (first + 1) * physics_dt if first is not None else None
    hold_seconds = sum(row.get('phase') == 'hold' for row in records) * result.get('dt', 0.)
    gates['contact_strictly_precedes_success'] = causal.get('ordering') == 'contact_precedes_success'
    files = [*path.glob('*.json'), *path.glob('*.mp4'), path/'trajectory.npz', path/'images.h5']
    return {
        'path': str(path), 'variant': result.get('variant'), 'gates': gates,
        'passed': all(gates.values()), 'frames': frames, 'dt': result.get('dt'),
        'simulation_seconds': frames * result.get('dt', 0),
        'first_success_action': result.get('success_action_index'),
        'first_success_physics_sample': causal.get('first_success_sample'),
        'first_success_time_from_recorded_start_s': first_time,
        'hold_command_duration_s': hold_seconds,
        'terminal_semantics': 'Press-and-hold terminal state; no release/retreat recorded. '
            'Original is_pressed predicate has no duration requirement; hold duration is extra evidence.',
        'first_loaded_contact_physics_sample': causal.get('first_loaded_contact_sample'),
        'physics_dt': result.get('physics_dt'),
        'maximum_button_displacement_m': causal.get('maximum_button_displacement_m'),
        'physics_samples': len(reports['audit'].get('physics_updates', [])),
        'monitored_background_dofs': len(reports['mechanism_audit'].get('joints', [])),
        'initial_setup_actions_recorded': result.get('initial_setup_actions_recorded', 0),
        'avoidance_gates': reports.get('avoidance_audit', {}).get('gates'),
        'file_sha256': {str(file): sha(file) for file in sorted(set(files)) if file.is_file()},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--episodes', type=Path, nargs=3, required=True)
    parser.add_argument('--neutral', type=Path, required=True)
    parser.add_argument('--faucet-mobility', type=Path, required=True)
    parser.add_argument('--paired', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    episodes = [episode_report(path, args.baseline) for path in [args.baseline, *args.episodes]]
    neutral = read(args.neutral / 'validation.json')
    background = read(args.neutral / 'background_mechanism_audit.json')
    neutral_probe = read(args.neutral / 'probe.json')
    mobility = read(args.faucet_mobility / 'validation.json')
    faucet_provenance = neutral_probe.get('faucet_physics_provenance')
    all_faucet_provenance = [read(Path(row['path'])/'result.json').get('faucet_physics_provenance')
                            for row in episodes]
    paired = read(args.paired / 'paired_planning.json')
    neutral_videos = probe_videos(args.neutral, len(neutral_probe.get('states', [])),
                                 neutral_probe.get('dt', 0.))
    mobility_frames = round(len(mobility.get('states', [])) * mobility.get('physics_dt', 0.) /
                            mobility.get('dt', 1.))
    mobility_videos = probe_videos(args.faucet_mobility, mobility_frames, mobility.get('dt', 0.))
    gates = {
        'all_four_episodes_pass': all(row['passed'] for row in episodes),
        'one_of_each_variant': sorted(row['variant'] or '' for row in episodes) == ['base', 'human', 'property', 'robot'],
        'neutral_stability_and_mobility': neutral.get('accepted') is True,
        'neutral_full_kitchen_coverage': background.get('passed') is True,
        'faucet_not_locked': mobility.get('accepted') is True,
        'neutral_videos_fully_decoded': neutral_videos['passed'],
        'faucet_mobility_videos_fully_decoded': mobility_videos['passed'],
        'same_faucet_model_in_all_runs': bool(faucet_provenance) and
            all(value == faucet_provenance for value in all_faucet_provenance) and
            mobility.get('faucet_physics_provenance') == faucet_provenance,
        'neutral_covers_entire_episode_with_setup': neutral.get('neutral_seconds', 0) >=
            max(row['simulation_seconds'] for row in episodes),
        **paired_gates(paired),
    }
    supporting_files = []
    for directory in (args.neutral, args.faucet_mobility, args.paired):
        for pattern in ('*.json', '*.mp4', '*.npy'):
            supporting_files.extend(directory.glob(pattern))
    stage_path = Path(paired.get('stage_path', 'missing-stage'))
    if stage_path.is_file():
        supporting_files.append(stage_path)
    report = {'accepted': all(gates.values()), 'gates': gates, 'episodes': episodes,
              'neutral': str(args.neutral), 'paired': str(args.paired),
              'faucet_mobility': str(args.faucet_mobility),
              'faucet_physics_provenance': faucet_provenance,
              'probe_video_validation': {'neutral': neutral_videos, 'faucet_mobility': mobility_videos},
              'supporting_file_sha256': {str(path): sha(path) for path in sorted(set(supporting_files))},
              'scope': 'One fixed-layout left-button task; no right-button/oven-knob, randomized collection, '
              'batch success rate or universal shortest-path claim. External geometry/contact/background '
              'q/dq: physics updates of recorded actions. Independent self-collision/limits and '
              'counterfactual: action rate. All eight setup/settling actions are included in RGB, '
              'action/state/contact and physics-rate clearance/background audits.',
              'data_upload': 'Local ignored data/video paths; hashes do not imply Git upload.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'accepted': report['accepted'], 'gates': gates}, indent=2))
    raise SystemExit(0 if report['accepted'] else 2)


if __name__ == '__main__':
    main()
