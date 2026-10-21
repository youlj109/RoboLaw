#!/usr/bin/env python3
"""CPU preflight for the risk branch; does not start Isaac Sim or collect data."""
import importlib.metadata
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    import fcl
    import numpy as np
    from kitchen_safety.fcl_distance import pair_distance
    from kitchen_safety.risk_profile_loader import load_config
    from kitchen_safety.risk_observer import aggregate_known_results, unknown_report
    settings = load_config('configs/safety_observer_arena.json')
    left = {'path': 'preflight/left', 'object': fcl.CollisionObject(fcl.Box(1., 1., 1.))}
    right = {'path': 'preflight/right', 'object': fcl.CollisionObject(
        fcl.Box(1., 1., 1.), fcl.Transform(np.eye(3), np.array([1.25, 0., 0.])))}
    gap = pair_distance(fcl, left, right)
    if abs(gap['distance_m'] - 0.25) > 1e-8 or not gap['nearest_points_valid']:
        raise RuntimeError('FCL analytical clearance preflight failed')
    report = unknown_report('preflight missing channel')
    report['rules'] = [
        {'rule_id': 'PREFLIGHT-KNOWN', 'category': 'HS', 'level': 'L1', 'complete': True, 'missing': []},
        {'rule_id': 'PREFLIGHT-UNKNOWN', 'category': 'HS', 'level': None, 'complete': False, 'missing': ['fixture']},
    ]
    graded = aggregate_known_results(report)
    if graded['categories']['HS']['level'] != 'L1' or graded['categories']['HS']['data_complete']:
        raise RuntimeError('Known-result grading/coverage preflight failed')
    if graded['categories']['IR']['level'] is not None:
        raise RuntimeError('Missing IR evidence was filled with a grade')
    print(json.dumps(dict(
        passed=True, scope='CPU imports, current profile, analytical FCL clearance and known-result aggregation',
        adapter_version=settings['adapter_version'], evaluator_version=settings['evaluator_version'],
        profile_file_sha256=settings['profile_file_sha256'],
        packages={name: importlib.metadata.version(name) for name in ('python-fcl', 'numpy', 'PyYAML')},
        analytical_distance_m=gap['distance_m'],
        simulation_started=False,
    ), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
