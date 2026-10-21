"""Canonical offline views of RECORDED online decisions; never re-evaluate risk.

Legacy feature/label reports remain byte-for-byte available. Readers select the
explicit index, not a glob/mtime. A null level remains null, including at t=0.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path

CATEGORIES = ('HS', 'PT', 'RS', 'IR')
LEVELS = ('L0', 'L1', 'L2', 'L3')
EPISODE_PATH = 'safety_reports/online_risk_episode705.json'
TIMELINE_PATH = 'safety_reports/online_risk_timeline705.json'
INDEX_PATH = 'risk_report_index705.json'


def digest(value):
    # Exactly the profile/input digest convention used by iso_traceable.
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False).encode('utf-8')).hexdigest()


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, why):
    if not condition:
        raise ValueError(why)


def observed_max(values):
    known = [v for v in values if v in LEVELS]
    return max(known, key=LEVELS.index) if known else None


def build_reports(raw, raw_sha256):
    stream = raw.get('online_iso') or {}
    require(stream.get('schema_version') == 'online-iso-observation-stream-2',
            'Recorded observation stream required; legacy rules are not a fallback')
    records = stream.get('records')
    refs = stream.get('execution_references')
    require(isinstance(records, list) and records, 'No recorded online decisions')
    require(isinstance(refs, list) and refs, 'No physical execution references')
    stride = stream.get('physics_steps_per_observation')
    require(type(stride) is int and stride > 0, 'Invalid observation stride')
    profile_hash = digest(stream['profile'])
    require(profile_hash == stream.get('profile_sha256'), 'Embedded profile digest mismatch')
    categories = {category: [] for category in CATEGORIES}
    for i, record in enumerate(records):
        observation = record['observation']
        require(observation.get('observation_frame_id') == i and
                observation.get('decision_physics_step_index') == i * stride,
                'Observation identity/cadence mismatch')
        require(observation.get('risk_source_physics_step_index') == (i * stride - 1 if i else None),
                'Risk source must precede action; do not synthesize initial evidence')
        report = record['report']
        require(report.get('profile_sha256') == profile_hash, 'Per-decision profile mismatch')
        require(report.get('input_sha256') == digest(record['input']), 'Per-decision input digest mismatch')
        require(report.get('frame_index') == observation['risk_source_physics_step_index'],
                'Report source step differs from recorded observation')
        for category in CATEGORIES:
            item = report['categories'][category]
            level = item.get('level')
            require(level is None or level in LEVELS, 'Invalid recorded risk level')
            require(report['risk_levels'].get(category) == level, 'Inconsistent recorded category level')
            categories[category].append(item)
    for i, reference in enumerate(refs):
        slot = reference.get('observation_frame_id')
        require(reference.get('physics_step_index') == i and slot == i // stride
                and type(slot) is int and 0 <= slot < len(records),
                'Missing, duplicated or invalid physical execution reference')
        require(reference.get('classification_fresh') is (i % stride == 0),
                'Cached execution must not become a fresh risk observation')
    require(len(records) == (len(refs) - 1) // stride + 1, 'Unexecuted/missing decision')
    clocks = raw.get('metadata', {}).get('physics_frame_clocks', [])
    if clocks:
        require(len(clocks) == len(refs) and all(c.get('frame_index') == i for i,c in enumerate(clocks)),
                'Recorded world clocks do not align with execution references')
    common = {'authority': 'recorded_online_iso', 'episode_id': raw.get('episode_meta', {}).get('episode_id'),
              'raw_file': 'sim_raw_gt.json', 'raw_sha256': raw_sha256,
              'profile_sha256': profile_hash, 'profile_digest_convention': 'iso_traceable.digest',
              'evaluator_versions': sorted({r['report']['evaluator_version'] for r in records}),
              'iso_certified': False, 'risk_recomputed': False}
    aggregates = {}
    for category, items in categories.items():
        values = [item.get('level') for item in items]
        aggregates[category] = {
            'observed_decision_max_level': observed_max(values),
            'level_counts': dict(Counter('unknown' if v is None else v for v in values)),
            'unknown_count': values.count(None),
            'incomplete_count': sum(item.get('data_complete') is not True for item in items),
            'data_complete': all(item.get('data_complete') is True and item.get('level') in LEVELS for item in items)}
    errors = sum(r.get('error') is not None for r in records)
    complete = errors == 0 and all(x['data_complete'] for x in aggregates.values())
    maximum = observed_max([x['observed_decision_max_level'] for x in aggregates.values()])
    task = raw.get('outcome_gt', {}).get('task_semantic_success')
    episode = {**common, 'schema': 'canonical-online-risk-episode-705',
        'aggregation': 'maximum of known recorded decisions; not a new frame or all-physics safety verdict',
        'categories': aggregates, 'overall_observed_decision_max_level': maximum,
        'data_complete': complete, 'decision_error_count': errors,
        'decision_count': len(records), 'execution_reference_count': len(refs),
        'initial_partial': records[0]['report'].get('data_complete') is not True,
        'incomplete_after_initial': sum(r['report'].get('data_complete') is not True or r.get('error') is not None for r in records[1:]),
        'task_result': {'task_semantic_success': task, 'source': '/outcome_gt/task_semantic_success'},
        'safety_result': {'L3_observed_in_decisions': maximum == 'L3', 'decision_evidence_complete': complete,
                          'scope': 'recorded 30Hz decisions; retain 120Hz raw contact/outcome evidence separately'},
        'stimulus_result': {'status': 'unknown', 'reason': 'not inferred from risk maxima; use existing independent stimulus audit'},
        'limitations': deepcopy(stream.get('limitations', []))}
    timeline = {**common, 'schema': 'canonical-online-risk-timeline-705',
        'decision_hz': stream['decision_hz'], 'physics_dt_s': stream['physics_dt_s'],
        'physics_steps_per_observation': stride,
        # Keep the exact reports/explanations. Inputs are bound by digest and JSON pointer.
        'records': [{**deepcopy({k:v for k,v in r.items() if k != 'input'}),
                     'input_pointer': '/online_iso/records/%d/input' % i} for i,r in enumerate(records)],
        'execution_references': deepcopy(refs), 'physics_frame_clocks': deepcopy(clocks),
        'camera_timeline_pointer': '/camera_timeline',
        'intervention_pointer': '/applied_intervention_decisions',
        'notes': ['Cached references are not additional classifications.',
                  'Intervention and L3 historical latch are independent of current decision level.',
                  'Use recorded observation/world clocks; do not substitute index times dt.']}
    return episode, timeline


def write_once(path, document):
    data = (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')
    if path.exists():
        require(path.read_bytes() == data, 'Existing derivative differs; create an explicit new revision: ' + str(path))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp705')
    with temporary.open('xb') as stream:
        stream.write(data)
    temporary.replace(path)


def write_episode_reports(directory):
    directory = Path(directory).resolve()
    raw_path = directory / 'sim_raw_gt.json'
    raw_hash = file_sha(raw_path)
    raw = json.loads(raw_path.read_text(encoding='utf-8'))
    episode, timeline = build_reports(raw, raw_hash)
    legacy = [p for p in (directory/'safety_reports').glob('*.json')
              if p.name not in {Path(EPISODE_PATH).name, Path(TIMELINE_PATH).name}]
    legacy += [directory/name for name in ('sim_labels.json', 'sim_labels_timeline.json') if (directory/name).is_file()]
    identities = {str(p.relative_to(directory)).replace('\\','/'): file_sha(p) for p in sorted(legacy)}
    write_once(directory/EPISODE_PATH, episode)
    write_once(directory/TIMELINE_PATH, timeline)
    require(file_sha(raw_path) == raw_hash and all(file_sha(directory/p) == h for p,h in identities.items()),
            'Source changed while deriving report')
    index = {'schema': 'canonical-online-risk-index-705', 'authority': 'recorded_online_iso',
             'canonical_episode_report': EPISODE_PATH, 'canonical_timeline': TIMELINE_PATH,
             'canonical_episode_report_sha256': file_sha(directory/EPISODE_PATH),
             'canonical_timeline_sha256': file_sha(directory/TIMELINE_PATH),
             'raw_file': 'sim_raw_gt.json', 'raw_sha256': raw_hash, 'profile_sha256': episode['profile_sha256'],
             'legacy_noncanonical_files_sha256': identities,
             'legacy_status': 'historical standalone rules, not actual online control labels; retained unchanged',
             'reader_contract': 'Select these paths explicitly. Never choose *_risk.json by modification time.'}
    # Publish authority last, after both derivative files have been verified.
    write_once(directory/INDEX_PATH, index)
    return {'index': str(directory/INDEX_PATH), 'index_sha256': file_sha(directory/INDEX_PATH),
            'decision_count': episode['decision_count'], 'execution_reference_count': episode['execution_reference_count'],
            'profile_sha256': episode['profile_sha256'], 'source_unchanged': True,
            'observed_maxima': {c:v['observed_decision_max_level'] for c,v in episode['categories'].items()},
            'unknown_counts': {c:v['unknown_count'] for c,v in episode['categories'].items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+', type=Path)
    args = parser.parse_args()
    for directory in args.episodes:
        print(json.dumps(write_episode_reports(directory), ensure_ascii=False))


if __name__ == '__main__':
    main()
