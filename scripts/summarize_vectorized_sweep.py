"""Summarize native-environment measurements without importing Kit or CUDA.

Replay-loop rates, complete collector rates, and validated RGB success counts
have different scopes and stay separate. A 39 s replay equivalent is 585 action
samples; it is not a newly collected successful trajectory.
"""
import argparse
import json
from pathlib import Path


STANDARD_EPISODE_SAMPLES = 585


def _positive(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and value > 0


def _rate(count, seconds, multiplier=1.):
    return count * multiplier / seconds if count is not None and _positive(seconds) else None


def _load(path):
    return json.loads(path.read_text())


def summarize_directory(directory):
    root = Path(directory)
    path = root / 'benchmark.json'
    kind = 'action_replay'
    if not path.exists():
        path, kind = root / 'batch.json', 'independently_planned_collection'
    data = _load(path)
    timing = data.get('timings_seconds', {})
    camera = data.get('camera_timings_seconds', {})
    renderer = data.get('renderer_settings', {})
    n = int(data['num_envs'])
    episodes = []
    for file in sorted(root.glob('env_*/result.json')):
        result = _load(file)
        episodes.append({'directory': file.parent.name, 'result': result,
                         'rgb_file_present': (file.parent / 'images.h5').exists()})
    validation_path = root / 'validation.json'
    validations = _load(validation_path) if validation_path.exists() else []
    validation_by_env = {Path(row.get('path', '')).name: row for row in validations}
    validated_rgb_successes = 0
    validated_rgb_episodes = 0
    for episode in episodes:
        checked = validation_by_env.get(episode['directory'], {})
        rgb_valid = (checked.get('passed') is True and len(checked.get('cameras', {})) == 3)
        validated_rgb_episodes += int(rgb_valid)
        validated_rgb_successes += int(rgb_valid and episode['result'].get('success') is True)

    is_replay = kind == 'action_replay'
    completed = data.get('completed') is True if is_replay else data.get('status') == 'completed'
    record_rgb = (data.get('record_rgb') is True or bool(camera.get('capture_ticks'))
                  or _positive(timing.get('camera_s', 0)) or any(e['rgb_file_present'] for e in episodes))
    full_replay = data.get('full_source_replay') is True if is_replay else None
    frames = ([int(data.get('executed_control_steps', 0))] * n if is_replay else
              [int(e['result'].get('steps', 0)) for e in episodes])
    action_samples = sum(frames)
    # Camera children are parts of camera_s / replay_loop_s, never added to
    # those inclusive timers. Deferred pool time likewise stays pool wall.
    physics = timing.get('physics_step_call_s') if is_replay else timing.get('physics_execution_s')
    camera_write = (camera.get('render_capture_s', 0.) + camera.get('jpeg_write_s', 0.)
                    if is_replay else timing.get('camera_s', 0.))
    execution_rgb = physics + camera_write if physics is not None else None
    loop = timing.get('replay_loop_s') if is_replay else timing.get('collection_loop_s')
    total = timing.get('total_before_close_s')
    successes = (sum(e['result'].get('success') is True for e in episodes) if not is_replay else None)
    diagnostic_successes = data.get('diagnostic_pick_place_success_count') if is_replay and full_replay else None
    capture_ticks = camera.get('capture_ticks') or (max(frames) if frames else 0)
    row = {
        'directory': str(root), 'report': str(path), 'kind': kind, 'num_envs': n,
        'completed': completed, 'record_rgb': record_rgb, 'full_source_replay': full_replay,
        'source_episode': data.get('source_episode'),
        'render_spp': renderer.get('/rtx/directLighting/sampledLighting/samplesPerPixel',
                                   data.get('render_spp_requested', data.get('render_spp'))),
        'renderer_settings': renderer,
        'requested_control_steps_per_env': data.get('requested_steps'),
        'action_samples_total': action_samples, 'action_samples_by_env': frames,
        'loop_wall_s': loop, 'action_samples_per_loop_second': _rate(action_samples, loop),
        'standard_39s_replay_equivalents_per_loop_hour':
            _rate(action_samples, loop, 3600. / STANDARD_EPISODE_SAMPLES),
        'physics_plus_rgb_write_wall_s': execution_rgb,
        'physics_plus_rgb_write_wall_s_per_recorded_episode':
            execution_rgb / n if execution_rgb is not None and n else None,
        'action_samples_per_physics_plus_rgb_write_second': _rate(action_samples, execution_rgb),
        'active_slot_fraction': _rate(action_samples, capture_ticks * n),
        'collector_successes': successes,
        'collector_success_fraction_of_attempts': successes / n if successes is not None and n else None,
        'full_replay_diagnostic_successes': diagnostic_successes,
        'full_replay_diagnostic_success_fraction': diagnostic_successes / n if diagnostic_successes is not None and n else None,
        'validated_rgb_episodes': validated_rgb_episodes,
        'validated_rgb_collector_successes': validated_rgb_successes if not is_replay and record_rgb else None,
        'collector_successes_per_hour_before_close': _rate(successes, total, 3600.),
        'validated_rgb_collector_successes_per_hour_before_close':
            _rate(validated_rgb_successes, total, 3600.) if not is_replay and record_rgb else None,
        'total_before_close_s': total,
        'timings_seconds': timing, 'camera_timings_seconds': camera,
        'validation_report': str(validation_path) if validation_path.exists() else None,
        'validation_elapsed_seconds_sum': sum(r.get('elapsed_seconds', 0.) for r in validations),
        'timing_scope': {
            'loop': (data.get('timing_semantics') or
                     'Replay loop includes physics, GPU state capture and, when enabled, three-camera RGB/JPEG writes.')
                    if is_replay else
                    ('collection_loop_s is the explicit controller loop wall if provided; absent stays null, '
                     'rather than estimating it from partially overlapping component timers.'),
            'physics_plus_rgb_write': 'Sum of physics call wall and camera capture/JPEG write wall; excludes planning, appearance warm-up, safety audit, initialization and final data verification.',
            'before_close': 'Includes reported collector initialization and execution; excludes final app/env shutdown. Separate validation report time is not silently added.',
            'deferred_validation': 'Keep reported wall time; with validation-workers this is pool wall, not cumulative worker CPU time.',
            'rates': '585 action samples form one standard 39 s equivalent at 15 Hz. Equivalent rates are not complete task successes or a policy success-rate estimate.',
        },
    }
    return row


def summarize(directories):
    rows = []
    for directory in directories:
        try:
            rows.append(summarize_directory(directory))
        except (OSError, ValueError, TypeError, KeyError) as error:
            rows.append({'directory': str(directory), 'error': f'{type(error).__name__}: {error}'})
    # Rank like-for-like replay measurements only. A short RGB replay can
    # measure throughput but cannot provide a complete-task success fraction.
    groups = {}
    for row in rows:
        if (row.get('kind') != 'action_replay' or not row.get('completed') or
                not row.get('record_rgb') or row.get('action_samples_per_loop_second') is None):
            continue
        key = (row['source_episode'], row['requested_control_steps_per_env'], row['render_spp'],
               json.dumps(row['renderer_settings'], sort_keys=True))
        groups.setdefault(key, []).append(row)
    ranking = []
    for key, candidates in groups.items():
        ordered = sorted(candidates, key=lambda r: r['action_samples_per_loop_second'], reverse=True)
        ranking.append({'source_episode': key[0], 'requested_steps': key[1], 'render_spp': key[2],
                        'renderer_settings': json.loads(key[3]),
                        'best_measured_num_envs': ordered[0]['num_envs'],
                        'ranked_directories': [r['directory'] for r in ordered],
                        'scope': 'Measured full-RGB replay loop only, not independently planned successful collection throughput.'})
    return {'standard_episode_action_samples': STANDARD_EPISODE_SAMPLES,
            'standard_episode_simulated_seconds': 39., 'rows': rows,
            'rgb_replay_comparisons': ranking}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.directories)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    for row in result['rows']:
        if 'error' in row:
            print(row['directory'], row['error'])
        else:
            rate = row['action_samples_per_loop_second']
            print(f"{row['directory']}: N={row['num_envs']} RGB={row['record_rgb']} "
                  f"samples={row['action_samples_total']} loop_samples/s={rate} "
                  f"collector_success={row['collector_successes']} "
                  f"validated_RGB_success={row['validated_rgb_collector_successes']}")
    return int(any('error' in row for row in result['rows']))


if __name__ == '__main__':
    raise SystemExit(main())
