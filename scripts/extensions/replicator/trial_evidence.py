"""Acceptance requires physical results and complete independent RGB evidence."""
import json
from pathlib import Path


def load_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def report_passed(value):
    if isinstance(value, dict):
        return value.get('passed') is True
    return isinstance(value, list) and bool(value) and all(report_passed(item) for item in value)


def acceptance(out, reports, statuses, *, require_rgb):
    out, reports = Path(out), Path(reports)
    result = load_json(out / 'result.json') or {}
    gates = {
        'physical_success': result.get('success') is True,
        'original_arena_predicate': result.get('arena_success') is True,
        'final_destination_support': result.get('final_on_destination') is True,
        'physics_rate_safety': result.get('deferred_safety', {}).get('passed') is True,
        'bilateral_body_contacts': result.get('body_grasp_carried_region', {}).get('passed') is True,
        'trajectory_present': (out / 'trajectory.h5').is_file(),
        'collection_exit_zero': statuses.get('collection') == 0,
        'source_relations_passed': report_passed(load_json(out / 'source_relation_audit.json')),
        'source_relations_exit_zero': statuses.get('source_relations') == 0,
    }
    if require_rgb:
        if isinstance(result.get('overview_camera'), dict):
            gates['overview_passed'] = report_passed(load_json(out / 'overview_validation.json'))
            gates['overview_exit_zero'] = statuses.get('overview') == 0
        gates.update({
            'export_exit_zero': statuses.get('export') == 0,
            'validation_exit_zero': statuses.get('validation') == 0,
            'videos_passed': report_passed(load_json(reports / 'videos.json')),
            'validation_passed': report_passed(load_json(reports / 'validation.json')),
        })
        gates['self_collision_passed'] = report_passed(load_json(out / 'self_collision_audit.json'))
        gates['self_collision_exit_zero'] = statuses.get('self_collision') == 0
        if out.parent.name != 'base':
            gates['fixed_base_hazard_passed'] = report_passed(load_json(out / 'fixed_base_hazard_audit.json'))
            gates['fixed_base_hazard_exit_zero'] = statuses.get('fixed_base') == 0
            gates['audit_exit_zero'] = statuses.get('audit') == 0
            gates['audit_passed'] = report_passed(load_json(reports / 'audit.json'))
    if 'avoidance_trials' in out.parts and out.parent.name != 'base':
        gates['background_consistency_passed'] = report_passed(load_json(out / 'background_consistency_audit.json'))
        gates['background_consistency_exit_zero'] = statuses.get('background_consistency') == 0
        gates['causal_avoidance_passed'] = (load_json(out / 'avoidance_audit.json') or {}).get('accepted') is True
        gates['avoidance_exit_zero'] = statuses.get('avoidance') == 0
    return dict(passed=all(gates.values()), acceptance_gates=gates,
                acceptance_scope='Physical source task + substep safety' + (' + complete RGB/video/action validation + hazard audit' if require_rgb else '; no RGB claim'))


def correct_existing(reports, *, require_rgb, publish=False):
    reports = Path(reports)
    root = Path(__file__).resolve().parents[3]
    summary = load_json(reports / 'summary.json') or {}
    out = root / 'datasets/skill_trials/replicator' / reports.parent.name / reports.name
    corrected = {**summary, **acceptance(out, reports, summary.get('exit_codes', {}), require_rgb=require_rgb),
                 'supersedes_acceptance_only': str(reports / 'summary.json'),
                 'correction_reason': 'Original runner checked only process return codes; physical result and complete independent evidence are now required.'}
    (reports / 'summary_corrected.json').write_text(json.dumps(corrected, indent=2) + '\n')
    if publish:
        original = reports / 'summary_legacy_exit_codes_only.json'
        if not original.exists():
            original.write_text(json.dumps(summary, indent=2) + '\n')
        (reports / 'summary.json').write_text(json.dumps(corrected, indent=2) + '\n')
    return corrected

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(); p.add_argument('reports'); p.add_argument('--no-rgb', action='store_true'); p.add_argument('--publish', action='store_true'); a = p.parse_args()
    result = correct_existing(a.reports, require_rgb=not a.no_rgb, publish=a.publish)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
