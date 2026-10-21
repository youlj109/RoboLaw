#!/usr/bin/env python3
"""Compare completed deferred-rendering runs using existing JSON evidence only.

Does not load H5/NPZ, decode images, run validators, or modify source results.
Queue put duration is not instrumented by the benchmark; it remains unknown.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[1]


def project_path(value):
    path = Path(value)
    if path.is_absolute() and str(path).startswith('/workspace/'):
        return ROOT / path.relative_to('/workspace')
    return path if path.is_absolute() else ROOT / path


def read_json(path, default=None):
    path = Path(path)
    return json.loads(path.read_text()) if path.exists() else default


def interval(timing):
    if not timing or timing.get('finish_relative_s') is None:
        return None
    return [timing['start_relative_s'], timing['finish_relative_s']]


def merged(intervals):
    result = []
    for start, finish in sorted(item for item in intervals if item is not None):
        if result and start <= result[-1][1]:
            result[-1][1] = max(finish, result[-1][1])
        else:
            result.append([start, finish])
    return result


def duration(intervals):
    return sum(finish - start for start, finish in merged(intervals))


def overlap(left, right):
    return sum(max(0., min(a1, b1) - max(a0, b0))
               for a0, a1 in merged(left) for b0, b1 in merged(right))


def hourly(count, wall):
    return count * 3600 / wall if wall and wall > 0 else None


def audit_run(directory):
    directory = project_path(directory).resolve()
    summary_path = directory / 'summary.json'
    data = read_json(summary_path)
    if not isinstance(data, dict):
        raise ValueError('Missing summary: {}'.format(summary_path))
    specs = read_json(directory / 'episode_specs.json', [])
    batches = data.get('batches', [])
    batch_by_index = {row['index']: row for row in batches}
    issues, episodes, children = [], [], []
    counts = dict(requested=data['requested_episodes'], source_results=0,
                  raw_task_complete=0, strict_source_success=0, qualified_rgb=0,
                  previously_decoded_images=0, source_action_frames=0)
    physics, render, validation, consumer_rows, qualified_finishes = [], [], [], [], []
    for batch in batches:
        timing = interval(batch.get('physics_timing'))
        if timing:
            physics.append(timing)
        if batch.get('execution'):
            children.append(dict(role='physics', batch_index=batch['index'], **batch['execution']))
    for row in data['episodes']:
        index = row['index']
        source = project_path(row['source']) if row.get('source') else None
        result = read_json(source / 'result.json', {}) if source else {}
        source_spec = read_json(source / 'episode_spec.json') if source else None
        expected_spec = specs[index] if index < len(specs) else None
        batch = batch_by_index.get(row.get('batch_index'), {})
        bt = interval(batch.get('physics_timing'))
        rt = interval(row.get('render_request_timing') or row.get('render_timing'))
        vt = interval(row.get('validation_timing'))
        raw = result.get('arena_success') is True and result.get('final_on_destination') is True
        strict = result.get('success') is True
        validator = read_json(project_path(row['validation_report']), []) if row.get('validation_report') else []
        valid = (row.get('validation_execution', {}).get('exit_code') == 0 and
                 isinstance(validator, list) and len(validator) == 1 and validator[0].get('passed') is True)
        complete = row.get('render_complete') is True and row.get('rendered_frames') == result.get('steps')
        qualified = strict and complete and valid
        record = dict(index=index, source=row.get('source'), steps=result.get('steps'),
                      raw_task_complete=raw, strict_source_success=strict, render_complete=complete,
                      full_validator_passed=valid, qualified_rgb=qualified,
                      source_spec_matches_input_and_result=(source_spec == expected_spec == result.get('episode_spec'))
                      if result else None)
        if result:
            counts['source_results'] += 1
            counts['source_action_frames'] += result.get('steps', 0)
        for key, passed in [('raw_task_complete', raw), ('strict_source_success', strict), ('qualified_rgb', qualified)]:
            counts[key] += int(passed)
        if validator:
            cameras = validator[0].get('cameras', {})
            decoded = sum(item.get('fully_decoded_frames', 0) for item in cameras.values())
            counts['previously_decoded_images'] += decoded
            record.update(previously_decoded_images=decoded,
                all_three_cameras_fully_decoded=(len(cameras) == 3 and all(
                    item.get('frames') == result.get('steps') == item.get('fully_decoded_frames')
                    for item in cameras.values())),
                state_chain_continuous=validator[0].get('state_chain_continuous'),
                appearance_warmup=validator[0].get('appearance_warmup'),
                rgb_action_alignment=validator[0].get('rgb_action_alignment'))
        if bool(row.get('qualified')) != qualified:
            issues.append({'index': index, 'issue': 'summary qualified differs from source and validator evidence'})
        if record['source_spec_matches_input_and_result'] is False:
            issues.append({'index': index, 'issue': 'episode specification differs'})
        if not qualified:
            safety = result.get('deferred_safety', {})
            record['failure'] = dict(summary=row.get('failure'), source_failure_kind=result.get('failure_kind'),
                phase=result.get('failure_phase'), action_index=result.get('failure_action_index'),
                error=result.get('error'), physical_failure_reasons=result.get('physical_failure_reasons'),
                deferred_safety_passed=safety.get('passed'), deferred_safety_failure=safety.get('failure'),
                carried_contact_report=result.get('body_grasp_carried_region'))
        if rt:
            render.append(rt)
            consumer_rows.append(dict(index=index, start=rt[0], finish=vt[1] if vt else rt[1],
                                      source_ready_s=bt[1] if bt else None))
        if vt:
            validation.append(vt)
            if qualified:
                qualified_finishes.append({'index': index, 'finish_relative_s': vt[1]})
        for name, role in [('render_execution', 'standalone_renderer'), ('validation_execution', 'rgb_validator')]:
            if row.get(name):
                children.append(dict(role=role, index=index, **row[name]))
        episodes.append(record)

    lifecycle = data.get('renderer_worker')
    server = {}
    if lifecycle and lifecycle.get('started'):
        children.append(dict(role='shared_renderer', **lifecycle))
        if lifecycle.get('server_report'):
            server = read_json(project_path(lifecycle['server_report']), {})
    shutdown = interval(data.get('renderer_shutdown_timing'))
    consumer = render + validation
    consumer_with_shutdown = consumer + ([shutdown] if shutdown else [])
    consumer_rows.sort(key=lambda row: row['start'])
    gaps = []
    for previous, current in zip(consumer_rows, consumer_rows[1:]):
        ready = current['source_ready_s']
        unavailable = max(0., min(current['start'], ready) - previous['finish']) if ready is not None else None
        gaps.append(dict(after_index=previous['index'], before_index=current['index'],
            observed_gap_s=max(0., current['start'] - previous['finish']),
            source_unavailable_s=unavailable,
            remainder_s=max(0., current['start'] - max(previous['finish'], ready)) if ready is not None else None))
    queue_capacity = data['queue_capacity']
    render_starts = sorted(row['start'] for row in consumer_rows)
    producer_gaps, cumulative_ready, backlog_events = [], 0, []
    for position, batch in enumerate(batches):
        bt = interval(batch.get('physics_timing'))
        if bt is None:
            continue
        ready_count = sum(episodes[i]['strict_source_success'] for i in batch['episode_indices'])
        cumulative_ready += ready_count
        backlog_events.append((bt[1], ready_count))
        next_timing = interval(batches[position+1].get('physics_timing')) if position+1 < len(batches) else None
        if next_timing is None:
            continue
        required_starts = max(0, cumulative_ready - queue_capacity)
        required_start = render_starts[required_starts-1] if required_starts and required_starts <= len(render_starts) else None
        capacity_proxy = (max(0., required_start - bt[1]) if required_start is not None else
                          0. if not required_starts else None)
        producer_gaps.append(dict(after_batch=batch['index'], before_batch=batches[position+1]['index'],
            observed_between_batch_gap_s=max(0., next_timing[0] - bt[1]),
            queue_put_blocked_s=None,
            queue_capacity_wait_proxy_s=capacity_proxy,
            cumulative_successful_sources=cumulative_ready,
            consumer_request_starts_needed_to_enqueue_all=required_starts,
            source_ready_not_yet_requested_at_batch_finish=cumulative_ready-sum(t <= bt[1] for t in render_starts)))
    backlog_events.extend((start, -1) for start in render_starts)
    backlog, peak_backlog = 0, 0
    for _, change in sorted(backlog_events):
        backlog += change
        peak_backlog = max(peak_backlog, backlog)
    first_render = min((t[0] for t in render), default=None)
    last_physics = max((t[1] for t in physics), default=None)
    last_consumer = max((t[1] for t in consumer_with_shutdown), default=None)
    qualified_finishes.sort(key=lambda row: row['finish_relative_s'])
    completion_intervals = [dict(after_index=a['index'], before_index=b['index'],
        seconds=b['finish_relative_s']-a['finish_relative_s'])
        for a, b in zip(qualified_finishes, qualified_finishes[1:])]
    recent = [row['seconds'] for row in completion_intervals[-4:]]
    wall = data.get('total_wall_s')
    final = data.get('status') != 'running' and wall is not None
    all_children_reaped = (all(child.get('process_reaped') is True for child in children) if children else None)
    output = dict(source_summary=str(summary_path), status=data['status'], completed_report=final,
        batch_size=data['batch_size'], validation_workers=data['validation_workers'], queue_capacity=queue_capacity,
        configuration=data['configuration'], counts=counts, total_wall_s=wall,
        raw_task_complete_per_hour=hourly(counts['raw_task_complete'], wall),
        strict_source_success_per_hour=hourly(counts['strict_source_success'], wall),
        qualified_rgb_per_hour=hourly(counts['qualified_rgb'], wall),
        original_reported_counts=data.get('counts'), episodes=episodes, failures=[row for row in episodes if not row['qualified_rgb']],
        timing=dict(physics_command_active_s=duration(physics), render_request_active_s=duration(render),
            rgb_validator_active_s=duration(validation), consumer_command_active_s=duration(consumer),
            renderer_shutdown_s=duration([shutdown]) if shutdown else None,
            physics_consumer_command_overlap_s=overlap(physics, consumer),
            physics_consumer_including_shutdown_overlap_s=overlap(physics, consumer_with_shutdown),
            first_render_fill_s=first_render,
            first_qualified_completion_s=qualified_finishes[0]['finish_relative_s'] if qualified_finishes else None,
            consumer_source_wait_after_first_render_s=sum(row['source_unavailable_s'] or 0 for row in gaps),
            consumer_gaps=gaps, producer_between_batch_gaps=producer_gaps,
            source_ready_not_yet_requested_peak=peak_backlog,
            final_consumer_drain_after_last_physics_s=max(0., last_consumer-last_physics)
                if last_consumer is not None and last_physics is not None else None,
            final_producer_tail_after_last_consumer_s=max(0., last_physics-last_consumer)
                if last_consumer is not None and last_physics is not None else None,
            host_tail_after_last_timed_operation_s=wall-max(last_consumer or 0, last_physics or 0) if wall is not None else None,
            qualified_completions=qualified_finishes, qualified_completion_intervals=completion_intervals,
            last_up_to_four_completion_intervals_mean_s=statistics.mean(recent) if recent else None,
            queue_timing_scope='jobs.put is not timed: queue_put_blocked_s is unknown. Between-batch gaps include queue wait plus bookkeeping. Capacity proxy uses request start as approximate dequeue time and cannot isolate exact put blocking. Source-ready backlog includes not-yet-enqueued sources, not just actual queue contents.',
            interval_scope='Recent completion intervals describe this finite trial, not stable sustained throughput. First lazy renderer startup belongs to its request; shared process lifetime also includes idle and validation and is not additive.'),
        children=children, recorded_children_reaped=all_children_reaped,
        recorded_child_exit_codes=[child.get('exit_code') for child in children],
        summary_all_owned_subprocesses_finished=data.get('all_owned_subprocesses_finished'),
        renderer_lifecycle=lifecycle,
        renderer_server={key: server.get(key) for key in ('completed', 'scene_closed', 'scene_builds',
            'scene_reuses', 'appearance_applications', 'rebuild_history', 'total_before_app_exit_s')},
        host_errors=data.get('host_errors'), issues=issues)
    return output, specs


def compare_runs(directories):
    pairs = [audit_run(directory) for directory in directories]
    runs = [item[0] for item in pairs]
    differences = []
    if runs:
        first = runs[0]
        for other in runs[1:]:
            fields = ['batch_size', 'validation_workers', 'queue_capacity', 'configuration']
            differences.append(dict(left=first['source_summary'], right=other['source_summary'],
                differences={key: [first[key], other[key]] for key in fields if first[key] != other[key]}))
    eligible = [run for run in runs if run['completed_report'] and not run['host_errors'] and
                run['recorded_children_reaped'] is True and run['qualified_rgb_per_hour'] is not None]
    best = max(eligible, key=lambda run: run['qualified_rgb_per_hour']) if eligible else None
    return dict(created_at=datetime.now(timezone.utc).isoformat(), production_quota_eligible=False,
        scope='Read-only JSON evidence; no H5/NPZ access, JPEG decoding, simulation or rerun validation',
        inputs_equal_in_order=all(item[1] == pairs[0][1] for item in pairs) if pairs else None,
        configuration_differences=differences, runs=runs,
        observed_highest_qualified_throughput_source=best['source_summary'] if best else None,
        limits=['Finite input windows include initial fill, failures, drain and actual process shutdown.',
                'Different environment counts can produce different physical outcomes and planning paths.',
                'If queue capacity or other settings differ, compare practical configurations, not a single-factor causal effect.',
                'Two N8 batches do not establish stable throughput for 200 episodes or a global optimum.',
                'Decoded-frame and state-chain claims reuse existing full validator reports; appearance pixels are not reassessed here.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = compare_runs(args.runs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'report': str(args.output), 'inputs_equal_in_order': report['inputs_equal_in_order'],
                     'observed_highest_qualified_throughput_source': report['observed_highest_qualified_throughput_source']}))


if __name__ == '__main__':
    main()
