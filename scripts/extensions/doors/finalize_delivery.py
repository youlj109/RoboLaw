"""Build a small tracked C01 report from existing independent evidence, without rerunning physics."""
import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text())


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, required=True)
    p.add_argument('--report-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refusing to overwrite a delivery report')
    asset = read(a.report_root/'asset_provenance_02.json')
    visual = read(a.report_root/'visual_review.json')
    tests = (a.report_root/'unittest_all.exit').read_text().strip() == '0'
    summary = {'student': 'lwx', 'git_author': 'WinshareLi <22632139@zju.edu.cn>',
        'task': 'kitchen_bench_lightwheel_open_fridge', 'task_id': 'C01',
        'base_branch': 'origin/ylj', 'base_commit': 'a5ab6fe5d45d6eb75ca62c51754aba07bc437e64',
        'host_repository': '/home/youlj/projects/wenxuanli/RoboLaw',
        'container_repository': '/workspace', 'container': 'robolaw-wenxuanli',
        'scope': 'One fixed-layout source task; one real success per risk class. Planning seeds are not additional physical successes or batch success rates.',
        'physics_hz': 120, 'rgb_action_hz': 15, 'episodes': {}, 'neutral_controls': {},
        'paired_planning': {}, 'retained_candidates': {}, 'failures_or_unverified': [],
        'unit_tests_passed': tests, 'resolved_asset_dependencies': asset['resolved_dependencies_present'],
        'visual_review': visual}
    common = [('result.json', 'accepted'), ('validation_full.json', 'accepted'),
        ('self_collision_audit.json', 'passed'), ('review_audit.json', 'passed'),
        ('contact_geometry_audit.json', 'passed'), ('stand_audit.json', 'passed'),
        ('source_layout_audit.json', 'passed')]
    required = [tests, asset['resolved_dependencies_present'], visual['passed']]
    selected = {'base': 'base_03', 'human': 'human_01', 'property': 'property_01', 'robot': 'robot_01'}
    for variant, name in selected.items():
        ep = a.data_root/name
        metadata, result = read(ep/'metadata.json'), read(ep/'result.json')
        gates = {file: {'passed': bool(read(ep/file).get(key)), 'sha256': sha(ep/file)}
                 for file, key in common}
        for file, key in ([] if variant == 'base' else [('avoidance_audit.json', 'accepted'), ('comparison_audit.json', 'passed')]):
            gates[file] = {'passed': bool(read(ep/file).get(key)), 'sha256': sha(ep/file)}
        review = read(ep/'review_audit.json')
        info = {'path': str(ep), 'gates': gates, 'frames': result['frames'],
            'physics_samples': result['physics_samples'], 'safety_checks': result['safety_checks'],
            'simulation_duration_s': result['frames']*metadata['dt'],
            'final_door_rad': result['final_door_rad'], 'strict_source_threshold_rad': .2*metadata['door_joint_limits_rad'][1],
            'selected_handle_offset_m': metadata['selected_handle_offset_z_m'],
            'handle_candidate_failures': metadata['handle_candidate_failures'],
            'controller_parameters': metadata['controller_parameters'],
            'implementation_sha256': metadata['implementation_sha256'],
            'contact_causality': review['contact_causality'],
            'background_mechanisms': review['mechanisms'],
            'source_contact_geometry': read(ep/'contact_geometry_audit.json'),
            'stand': read(ep/'stand_audit.json'),
            'source_layout': read(ep/'source_layout_audit.json'),
            'combined_video': str(ep/'all_cameras.mp4')}
        required.extend(x['passed'] for x in gates.values())
        if variant != 'base':
            av = read(ep/'avoidance_audit.json')
            info['avoidance'] = {key: av[key] for key in ('accepted', 'same_source_layout',
                'same_controller_policy', 'initial_and_goal_feasible', 'hazard_environment_legal',
                'property_stability', 'entity_support', 'handle_target_check', 'path_comparison')}
            info['avoidance']['baseline_counterfactual'] = {key: av['baseline_counterfactual'][key]
                for key in ('violates_safety_margin', 'violates_free_pregrasp', 'min_distance_m', 'required_m', 'minimum', 'geometry')}
            info['avoidance']['augmented_actual'] = av['augmented_actual']
            info['comparison_video'] = str(ep/'baseline_comparison.mp4')
            plan_path = a.report_root/('paired_'+variant+'_01')/'report.json'
            plan = read(plan_path)
            compact = {key: value for key, value in plan.items() if key != 'pairs'}
            compact['pairs'] = []
            for pair in plan['pairs']:
                entry = {key: value for key, value in pair.items() if key not in ('on', 'off', 'fixed_target_control')}
                def compact_query(query):
                    value = {k: v for k, v in query.items() if k != 'positions'}
                    value['positions_sha256'] = hashlib.sha256(json.dumps(query['positions']).encode()).hexdigest()
                    # Keep risk minima/counts, with full violations in the hashed raw report.
                    risk = value.get('risk_with_added_geometry', {})
                    for k in ('violations', 'conservative_violations'):
                        if k in risk:
                            risk[k+'_count'] = len(risk.pop(k))
                    return value
                entry.update({k: compact_query(pair[k]) for k in ('off', 'on')})
                if 'fixed_target_control' in pair:
                    entry['fixed_target_control'] = {k: compact_query(v) for k, v in pair['fixed_target_control'].items()}
                compact['pairs'].append(entry)
            compact['raw_report'] = str(plan_path)
            compact['raw_report_sha256'] = sha(plan_path)
            summary['paired_planning'][variant] = compact
            required.append(plan['stable_policy_necessity_in_tested_seeds'])
            if not plan['stable_policy_necessity_in_tested_seeds']:
                summary['failures_or_unverified'].append(variant+': multi-seed stable necessity failed')
        summary['episodes'][variant] = info
        np = a.data_root/('neutral_'+variant+'_01')
        nr = read(np/'review_audit.json')
        summary['neutral_controls'][variant] = {'path': str(np), 'review': nr,
            'result': read(np/'result.json'), 'review_sha256': sha(np/'review_audit.json')}
        required.append(nr['passed'])
    for directory in sorted(a.data_root.iterdir()):
        if not directory.is_dir():
            continue
        rp = directory/'result.json'
        item = {'selected': directory.name in selected.values(), 'result': read(rp) if rp.exists() else None}
        if (directory/'review_audit.json').exists():
            review = read(directory/'review_audit.json')
            item['review'] = {'passed': review['passed'], 'errors': review['errors']}
        log = a.report_root/(directory.name+'.log')
        if log.exists():
            item['log'] = str(log); item['log_sha256'] = sha(log)
        summary['retained_candidates'][directory.name] = item
    summary['failures_or_unverified'].extend([
        'C02-C08 not implemented or physically verified in this delivery.',
        'Randomized layouts, 11-class expansion, 200-episode collection and physical success rates not tested.',
        'Human regenerated USDC is not byte-identical to historical export; source blend, pose report, wrapper, textures and foot geometry verified. See artifact_manifest_lwx.json. Byte difference cause unestablished.',
        'Historical baseline metadata referenced by frozen layout is unavailable on this clone; its historical byte identity is unverified. Current pinned source placement/relations/composed asset and fresh physical baseline passed independently.',
        'Contact tangential friction and substep-internal temporal order are not measured. No claim of PhysX cooked-shape equivalence.',
        'Human/property legal handle-point adaptation is not fixed-endpoint rerouting. Fixed-target controls remain separately reported.',
        'Local Windows lacks scipy for the collision-scope tests; the actual remote container passed all 11 tests.'])
    summary['c01_current_gates_passed'] = all(required)
    artifacts = {}
    for root in (a.data_root, a.report_root):
        for path in sorted(root.rglob('*')):
            if path.is_file():
                artifacts[str(path)] = {'sha256': sha(path), 'bytes': path.stat().st_size}
    a.output.mkdir(parents=True)
    (a.output/'gate_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (a.output/'artifact_hashes.json').write_text(json.dumps(artifacts, indent=2)+'\n')
    (a.output/'asset_provenance.json').write_text(json.dumps(asset, indent=2)+'\n')
    print('DELIVERY_REPORT', summary['c01_current_gates_passed'], len(artifacts), 'artifacts', flush=True)
    return 0 if summary['c01_current_gates_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
