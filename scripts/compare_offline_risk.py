"""Compare offline reconstructed features and grades to an online reference."""
import argparse
from collections import Counter
import itertools
import json
import math
from pathlib import Path
import time


def main():
    start = time.perf_counter()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--offline', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--limit', type=int)
    a = p.parse_args()
    stats = Counter(); differences = Counter(); examples = []
    max_abs = 0.; numeric_count = 0

    def different(path, x, y):
        nonlocal max_abs, numeric_count
        if isinstance(x, bool) or isinstance(y, bool) or x is None or y is None:
            bad = type(x) is not type(y) or x != y
        elif isinstance(x, (float, int)) and isinstance(y, (float, int)):
            numeric_count += 1
            delta = abs(x-y); max_abs = max(max_abs, delta)
            bad = not math.isclose(x, y, abs_tol=1e-7, rel_tol=1e-7)
        elif isinstance(x, dict) and isinstance(y, dict):
            bad = False
            for key in sorted(set(x)|set(y)):
                if key not in x or key not in y:
                    record(path+'/'+key, x.get(key, '<missing>'), y.get(key, '<missing>')); bad = True
                else: bad = different(path+'/'+key, x[key], y[key]) or bad
            return bad
        elif isinstance(x, list) and isinstance(y, list):
            if path == 'features/pt/support_contact_evidence/pairs':
                # This is an unordered contact-pair collection. Preserve
                # multiplicity; canonicalize only this documented field.
                stats['support_pair_list_order_differences'] += x != y
                x, y = sorted(x), sorted(y)
            if len(x) != len(y): record(path+'/length', len(x), len(y)); return True
            bad = False
            for i, (u, v) in enumerate(zip(x, y)): bad = different(path+'/'+str(i), u, v) or bad
            return bad
        else: bad = x != y
        if bad: record(path, x, y)
        return bad

    def record(path, x, y):
        key = '/'.join(path.split('/')[:3])
        differences[key] += 1
        if len(examples) < 20: examples.append(dict(action_index=stats['records'], path=path, offline=x, online=y))

    with a.offline.open() as f, a.reference.open() as g:
        for x, y in itertools.islice(itertools.zip_longest(f, g), a.limit):
            if x is None or y is None: raise ValueError('Record count mismatch')
            x, y = json.loads(x), json.loads(y)
            for key in ('action_index', 'source_physics_step', 'decision_physics_step', 'decision_time_s'):
                if x[key] != y[key]: raise ValueError('Source alignment mismatch: '+key)
            for cat in ('HS', 'PT', 'RS'):
                stats[cat+'_grade_equal'] += x['report']['risk_levels'][cat] == y['report']['risk_levels'][cat]
            stats['features_equal'] += not different('features', x['input']['features'], y['input']['features'])
            # Rules are deterministic functions of the source profile/features.
            stats['rules_equal'] += not different('rules', x['report']['rules'], y['report']['rules'])
            stats['coverage_equal'] += not different('categories', x['report']['categories'], y['report']['categories'])
            stats['scope_reports_equal'] += not different('hs_scopes', x['report'].get('hs_scopes'), y['report'].get('hs_scopes'))
            stats['records'] += 1
    report = dict(passed=not differences, offline=str(a.offline), reference=str(a.reference),
        counts=dict(stats), numeric_values_compared=numeric_count, max_numeric_absolute_difference=max_abs,
        absolute_tolerance=1e-7, relative_tolerance=1e-7,
        differences=dict(differences), examples=examples, comparison_wall_s=time.perf_counter()-start,
        unordered_fields=['features.pt.support_contact_evidence.pairs'],
        scope='All HS/PT/RS feature leaves, all rule fields, category coverage and HS scoped reports at identical causal source steps; support contact pair collections compared as multisets, identity/profile metadata/processing clocks are deliberately separate')
    a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if not report['passed']: raise SystemExit(1)

if __name__ == '__main__': main()
