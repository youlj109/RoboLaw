#!/usr/bin/env python3
"""Replay every immutable online feature snapshot with its frozen profile."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety_risk.evaluator_factory import create_evaluator as Evaluator
from safety_risk.iso_traceable import load_json
from safety_risk.iso_input import raw_frame
from scripts.evaluate_iso_risk import atomic_write


def verify(raw):
    stream = raw['online_iso']
    evaluator = Evaluator(stream['profile'])
    errors = []
    records = stream['records']
    n = raw['metadata']['num_steps']
    sparse = stream.get('schema_version') == 'online-iso-observation-stream-2'
    if sparse:
        from safety_risk.observation_cadence import observation_indices, physics_stride
        stride = physics_stride(stream['physics_dt_s'], stream['decision_hz'])
        decisions = observation_indices(n, stream['physics_dt_s'], stream['decision_hz'])
        sources = [step-1 if step else None for step in decisions]
        expected_references = [{'physics_step_index': step, 'observation_frame_id': step//stride,
                                'classification_fresh': step%stride==0} for step in range(n)]
        if stream.get('execution_references') != expected_references:
            errors.append('physical_execution_reference_mismatch')
        if stream.get('physics_steps_per_observation') != stride:
            errors.append('observation_stride_mismatch')
        if raw.get('episode_meta', {}).get('physics_config', {}).get('physics_dt_s', stream['physics_dt_s']) != stream['physics_dt_s']:
            errors.append('physical_dt_mismatch')
    else:
        sources = list(range(n))
    if len(records) != len(sources):
        errors.append('online_record_count_mismatch')
    if stream['profile_sha256'] != evaluator.profile_hash:
        errors.append('frozen_profile_hash_mismatch')
    raw_checked = 'robot_state' in raw
    raw_differences = []
    injections = {e['frame']:e for e in raw.get('controlled_ir_observations', [])}
    def with_injection(snapshot, decision_step):
        if decision_step in injections:
            from safety_risk.controlled_ir import project_injection
            return project_injection(snapshot, injections[decision_step])
        return snapshot
    for index, record in enumerate(records):
        snapshot = record['input']
        source = sources[index] if index < len(sources) else -999
        if snapshot['frame_index'] != source:
            errors.append(f'frame_index_mismatch:{index}')
        if sparse:
            context = record.get('observation', {})
            if (context.get('observation_frame_id') != index or index >= len(decisions) or
                context.get('decision_physics_step_index') != decisions[index] or
                context.get('risk_source_physics_step_index') != source):
                errors.append(f'observation_context_mismatch:{index}')
            if source is None:
                initial = raw.get('initial_pre_action_state')
                if initial is None:
                    if snapshot.get('features') != {}:
                        errors.append('initial_state_must_remain_unknown')
                else:
                    from safety_risk.online_iso import initial_snapshot
                    try:
                        projected = initial_snapshot(initial, context['decision_sim_time_s'])
                        projected = with_injection(projected, decisions[index])
                        if projected != snapshot:
                            errors.append('initial_state_projection_mismatch')
                    except (ValueError, KeyError, TypeError) as exc:
                        errors.append('initial_state_projection_error:' + str(exc))
            clocks = raw.get('metadata', {}).get('physics_frame_clocks')
            if clocks is not None and len(clocks) != n and 'physical_clock_coverage_mismatch' not in errors:
                errors.append('physical_clock_coverage_mismatch')
            if clocks and index < len(decisions) and decisions[index] < len(clocks):
                decision_time = clocks[decisions[index]]['before_sim_time_s']
                if context.get('decision_sim_time_s') != decision_time:
                    errors.append(f'decision_clock_mismatch:{index}')
                if source is not None and abs(clocks[source]['after_sim_time_s']-decision_time)>1e-6:
                    errors.append(f'pre_action_source_clock_mismatch:{index}')
        if record.get('error'):

            errors.append(f'online_projection_error:{index}:{record["error"]}')
        replay = evaluator.evaluate(snapshot)
        replay['frame_index'] = snapshot['frame_index']
        replay['time_s'] = snapshot['time_s']
        if replay != record['report']:
            errors.append(f'report_diff:{index}')
        if raw_checked and source is not None and 0 <= source < n:
            offline = evaluator.evaluate(with_injection(raw_frame(raw, source), decisions[index]) if sparse else raw_frame(raw, source))
            for key in ('rules', 'risk_levels', 'derived'):
                if offline[key] != record['report'][key]:
                    raw_differences.append(f'{key}:{index}')
    if raw_differences:
        errors.append('full_raw_projection_mismatch')
    times = sorted(record['latency_ms'] for record in records)
    return {'verified': not errors, 'errors': errors, 'expected_frames': len(sources), 'physical_frames': n,
            'online_frames': len(records), 'profile_sha256': evaluator.profile_hash,
            'full_raw_checked': raw_checked, 'full_raw_differences': raw_differences,
            'latency_ms': {'max': max(times, default=None),
                           'p95': times[min(len(times)-1, int(len(times)*0.95))] if times else None},
            'comparison': 'entire report including every rule, evidence, derived value and hash',
            'scope': 'replay of immutable online inputs; does not prove all sensors available or risk-free operation'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('raw', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.raw.resolve() == args.output.resolve():
        p.error('output must not overwrite raw input')
    result = verify(load_json(args.raw))
    atomic_write(args.output, result)
    print(json.dumps(result))
    return 0 if result['verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
