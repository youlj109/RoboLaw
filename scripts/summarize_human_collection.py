"""Count complete, recorded and independently safe default-MPFB episodes.

This selection is separate from benchmark obstruction eligibility. Original
verification files and strict benchmark acceptance are never changed.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import h5py

ROOT = Path(__file__).resolve().parents[1]
SCENE = 'human_default_mpfb'
CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera')


def read(path):
    return json.loads(path.read_text())


def summarize(output, historical, target, write=False):
    selected, duplicates, seen, failures = [], [], set(), []
    for batch in (historical, output / 'episodes'):
        for episode in sorted((batch / SCENE).glob('attempt_*')):
            result_file, verification_file = episode / 'result.json', episode / 'verification.json'
            if not result_file.exists() or not verification_file.exists():
                continue
            result, verification = read(result_file), read(verification_file)
            checks = ('execution_success', 'data_success', 'audit_pass')
            if result.get('success') is not True or not all(verification.get(k) is True for k in checks):
                failures.append(dict(source_episode=str(episode.resolve()),
                    failure_kind=verification.get('failure_kind'),
                    failure_phase=verification.get('failure_phase')))
                continue
            spec = read(episode / 'episode_spec.json')
            identity = tuple(spec['seeds'][k] for k in ('geometry', 'appearance', 'planner'))
            if identity in seen:
                duplicates.append(str(episode.resolve()))
                continue
            image_root = episode / 'rendered' if (episode / 'rendered/images.h5').exists() else episode
            with h5py.File(episode / 'trajectory.h5', 'r') as tr, h5py.File(image_root / 'images.h5', 'r') as images:
                frames = len(tr['action'])
                counts = {name: len(images[name]['jpeg']) for name in CAMERAS}
                if not frames or any(count != frames for count in counts.values()):
                    raise ValueError(f'Previously validated RGB/action counts differ: {episode}')
            seen.add(identity)
            selected.append(dict(episode_index=len(selected), obstacle_id=SCENE,
                source_episode=str(episode.resolve()), rgb_directory=str(image_root.resolve()),
                seeds=spec['seeds'], frames=frames, camera_frames=counts,
                execution_success=True, data_success=True, audit_pass=True,
                benchmark_passed=verification.get('passed') is True,
                augmentation_eligible=verification.get('augmentation_eligible') is True,
                eligibility_note=verification.get('failure_kind'),
                historical=batch == historical))
    available = len(selected)
    selected = selected[:target]
    report = dict(obstacle_id=SCENE, target_count=target, success_count=len(selected),
        deficit=max(0, target-len(selected)), complete=len(selected) == target,
        counting_rule='Complete physical grasp/place, all three recorded RGB streams, and independent safety audit; benchmark obstruction eligibility is reported separately',
        historical_success_count=sum(row['historical'] for row in selected),
        new_success_count=sum(not row['historical'] for row in selected),
        benchmark_passed_count=sum(row['benchmark_passed'] for row in selected),
        available_success_count=available, duplicate_sources_excluded=duplicates,
        failed_attempts=failures, episodes=selected,
        updated_at=datetime.now(timezone.utc).isoformat())
    if write:
        output.mkdir(parents=True, exist_ok=True)
        selection = output / 'completed_episodes'
        selection.mkdir(exist_ok=True)
        for row in selected:
            link = selection / f'episode_{row["episode_index"]:04d}'
            source = Path(row['source_episode'])
            if link.is_symlink():
                if link.resolve() != source:
                    raise ValueError(f'Existing episode selection points elsewhere: {link}')
            elif link.exists():
                raise ValueError(f'Existing episode selection is not a link: {link}')
            else:
                link.symlink_to(source, target_is_directory=True)
        temporary = output / 'successful_episodes.json.tmp'
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(output / 'successful_episodes.json')
        with (output / 'successful_summary.csv').open('w', newline='') as f:
            fields = ('obstacle_id', 'target_count', 'success_count', 'deficit', 'complete',
                      'historical_success_count', 'new_success_count', 'benchmark_passed_count')
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerow({name: report[name] for name in fields})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'datasets/augmentation/human_default20_n4_20261019')
    parser.add_argument('--historical', type=Path, default=ROOT / 'datasets/augmentation/human7_n4_20261019/episodes')
    parser.add_argument('--target', type=int, default=20)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    if args.target < 1:
        parser.error('--target must be positive')
    report = summarize(args.output.resolve(), args.historical.resolve(), args.target, args.write)
    print(json.dumps({k:v for k,v in report.items() if k not in ('episodes', 'failed_attempts')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
