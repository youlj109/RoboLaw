"""Audit measured phase probes against exact frozen operation-surface scope.

This checks scope/rule routing, not successful manipulation or material safety.
It reads completed offline results; it does not run geometry or physics.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path


def verify(episode, risk):
    def read(root, name):
        return json.loads((root / name).read_text())
    raw_summary = read(episode, 'raw_capture_summary.json')
    summary = read(risk, 'evaluation_summary.json')
    scope = read(episode, 'relation_scope.json')
    rs = read(episode, 'rs_pair_scope.json')
    assert summary['complete'] and summary['evaluation_mode'] == 'offline_replay'
    assert summary['geometry_backend']['id'] == 'gdist'
    assert summary['raw_sha256'] == raw_summary['raw_writer']['sha256']
    assert summary['scope_hash'] == scope['scope_sha256']
    assert raw_summary['online_geometry_queries'] == raw_summary['online_risk_evaluations'] == 0
    declarations = scope['bindings'].get('operation_contacts', [])
    assert declarations, 'No exact operation surface declarations'
    operations = {x['operation_id'] for x in declarations}
    relations = {r['relation_id']: r for r in scope['relations']}
    expected_rs = {tuple(sorted(p)) for p in rs['evaluated_pairs']}
    contexts = []
    raw_hash = hashlib.sha256()
    with (episode / 'raw_evidence.jsonl').open('rb') as source:
        for line in source:
            raw_hash.update(line)
            row = json.loads(line)
            if row['kind'] in ('observation', 'terminal_observation'):
                contexts.append({k: row.get(k) for k in ('action_index', 'phase', 'operation_id', 'source_physics_step', 'time_s')})
    assert raw_hash.hexdigest() == summary['raw_sha256']
    modes = defaultdict(set)
    frames = []
    with (risk / 'safety_decisions.jsonl').open() as source:
        for index, line in enumerate(source):
            row = json.loads(line)
            assert index < len(contexts)
            assert {k: row.get(k) for k in contexts[index]} == contexts[index]
            geometry, features, report = row['geometry'], row['features'], row['report']
            assert geometry['scope_sha256'] == scope['scope_sha256'] == report['scope_hash']
            assert len(geometry['self_collision']) == len(expected_rs) and {tuple(sorted(p['pair'])) for p in geometry['self_collision']} == expected_rs, 'rs_scope_changed'
            assert report['categories']['IR']['status'] == 'disabled'
            by_feature = {p['relation_id']: p for p in features['pt']['protected_pairs']}
            assert len(by_feature) == len(features['pt']['protected_pairs'])
            assert set(by_feature) == {rid for rid, r in relations.items() if r['category'] == 'PT'}
            by_rule = defaultdict(list)
            for rule in report['rules']:
                by_rule[rule.get('relation_id')].append(rule)
            expected_omissions, expected_queries = {}, set()
            for rid, relation in relations.items():
                if not relation['query_distance']:
                    continue
                removed = set()
                for declaration in relation.get('operation_distance_omissions', []):
                    if row['operation_id'] == declaration['operation_id'] and row['phase'] in declaration['allowed_phases']:
                        removed.update(declaration['target_colliders'])
                retained = set(relation['target_colliders']) - removed
                if removed:
                    expected_omissions[rid] = removed
                    for actual in (geometry['distance_omissions'][rid], by_feature[rid]['distance_omission'], report['distance_omissions'][rid]):
                        assert set(actual['target_colliders']) == removed, 'operation_removed_surfaces_mismatch'
                        assert actual['fully_omitted'] == (not retained)
                        assert actual['force_and_impulse_retained'] is True
                    assert {'force_n', 'impulse_ns', 'physics_window'} <= set(by_feature[rid]), 'operation_contact_features_missing'
                    rules = by_rule[rid]
                    assert {'PT-FORCE-002', 'PT-IMP-002'} <= {r['rule_id'] for r in rules}, 'operation_contact_rules_missing'
                    if not retained:
                        assert rid not in geometry['relations']
                        assert not {'distance_m', 'distance_geometry_evidence', 'distance_coverage_complete', 'source_surface_speed_mps', 'target_surface_speed_mps'} & set(by_feature[rid]), 'fully_omitted_distance_persisted'
                        assert not any(r['rule_id'] == 'PT-DIST-001' for r in rules)
                if retained:
                    expected_queries.add(rid)
                    actual = geometry['relations'][rid]
                    assert set(actual['effective_target_colliders']) == retained, 'unexpected_queried_surfaces'
                    assert actual['complete'] is True, (rid, actual)
                    assert actual['distance_m'] is not None
                    if relation['category'] == 'PT':
                        assert any(r['rule_id'] == 'PT-DIST-001' for r in by_rule[rid])
            assert set(geometry['relations']) == expected_queries
            assert set(geometry['distance_omissions']) == set(expected_omissions) == set(report['distance_omissions'])
            for op in operations:
                mode = 'active' if row['operation_id'] == op and row['phase'] == 'operate' else 'wrong_phase' if row['operation_id'] == op else 'missing_operation' if row['phase'] == 'operate' and row['operation_id'] is None else 'inactive'
                modes[op].add(mode)
            frames.append(dict(action_index=row['action_index'], phase=row['phase'], operation_id=row['operation_id'],
                               omitted_relations=len(expected_omissions), fully_omitted=sum(rid not in expected_queries for rid in expected_omissions),
                               retained_queries=len(expected_queries), rs_pairs=len(expected_rs)))
    assert len(frames) == summary['frames'] == len(contexts)
    for op in operations:
        assert {'active', 'wrong_phase', 'missing_operation', 'inactive'} <= modes[op], (op, modes[op])
    assert any(x['omitted_relations'] for x in frames), 'Operation context never removed a bound surface'
    return dict(passed=True, task_id=scope['task_id'], source_raw_sha256=summary['raw_sha256'],
                scope_sha256=scope['scope_sha256'], frames=frames, operation_contexts={k: sorted(v) for k, v in modes.items()},
                checks=['raw and label source alignment', 'exact per-surface removal', 'unselected distances retained',
                        'no fully omitted distance fields/rules', 'force and impulse rules retained', 'independent RS pair set',
                        'IR disabled', 'active and inactive operation contexts'],
                full_task_trajectory=False, task_success=raw_summary['task_success'], counts_toward_200_quota=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', required=True, type=Path)
    p.add_argument('--risk', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    assert not args.output.exists(), 'Keep prior verification receipts'
    result = verify(args.episode, args.risk)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(passed=True, task_id=result['task_id'], samples=len(result['frames']))))


if __name__ == '__main__':
    main()
