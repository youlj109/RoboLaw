"""Build a small tracked C01 report from existing independent evidence, without rerunning physics."""
import argparse
import hashlib
import json
import subprocess
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
    p.add_argument('--old-data-root',type=Path,default=Path('datasets/doors/lwx_20261023'))
    p.add_argument('--old-report-root',type=Path,default=Path('reports/doors/lwx_20261023'))
    p.add_argument('--human-attempt',default='human_02')
    p.add_argument('--property-attempt',required=True)
    p.add_argument('--human-paired',default='paired_human_04')
    p.add_argument('--property-paired',required=True)
    p.add_argument('--canonical-config',type=Path,required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refusing to overwrite a delivery report')
    asset = read(a.report_root/'asset_provenance_01.json')
    visual = read(a.report_root/'visual_review.json')
    tests = (a.report_root/'unittest_all.exit').read_text().strip() == '0'
    summary = {'student': 'lwx', 'git_author': 'WinshareLi <22632139@zju.edu.cn>',
        'task': 'kitchen_bench_lightwheel_open_fridge', 'task_id': 'C01',
        'base_branch': 'origin/ylj', 'base_commit': '035fc7c',
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
    preflight = (a.report_root/'preflight.exit').read_text().strip() == '0'
    summary['preflight_passed'] = preflight
    required = [tests, preflight, asset['resolved_dependencies_present'], visual['passed']]
    decoded = {}
    summary['video_decode_reports'] = {}
    for item in visual['decode_reports']:
        path=Path(item);record=read(path)
        summary['video_decode_reports'][item]={'sha256':sha(path),'passed':record['complete_decode_passed']}
        required.append(record['complete_decode_passed']);decoded.update(record['episodes'])
    selected = {'base': 'base_01', 'human': a.human_attempt, 'property': a.property_attempt, 'robot': 'robot_01'}
    for variant, name in selected.items():
        ep = (a.old_data_root if variant=='robot' else a.data_root)/name
        matching_base = a.old_data_root/'base_03' if variant=='robot' else a.data_root/'base_01'
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
        video_checks={}
        episode_decode=decoded[str(ep)]
        for filename in ['external_camera.mp4','external_camera_2.mp4','wrist_camera.mp4','all_cameras.mp4']+([] if variant=='base' else ['baseline_comparison.mp4']):
            proof=episode_decode['videos'][filename]
            video_checks[filename]=bool(proof['passed'] and proof['sha256']==sha(ep/filename))
        info['full_video_decode_checks']=video_checks
        required.extend(video_checks.values())
        info['matching_baseline']=str(matching_base)
        info['retained_from_first_delivery']=variant=='robot'
        source_checks={}
        for label,expected in metadata['implementation_sha256'].items():
            relative=label.removeprefix('/workspace/')
            if variant=='robot':
                blob=subprocess.check_output(['git','show','7f1ce4c47dfe4223c1774a6d909ab40e4c598250:'+relative])
                actual=hashlib.sha256(blob).hexdigest()
                scope='Historical first-delivery commit; robot not recollected'
            else:
                actual=sha(Path(relative));scope='Current collection source bytes'
            source_checks[label]={'expected_sha256':expected,'actual_sha256':actual,'passed':actual==expected,'scope':scope}
        info['source_byte_checks']=source_checks
        required.extend(x['passed'] for x in source_checks.values())
        if variant!='base':
            bm=read(matching_base/'metadata.json')
            match={key:metadata[key]==bm[key] for key in ['source_yaml_sha256','robot_base','robot_yaw_deg','controller_parameters']}
            info['matching_baseline_checks']=match
            required.extend(match.values())
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
            plan_path = (a.old_report_root/'paired_robot_01' if variant=='robot' else a.report_root/(a.human_paired if variant=='human' else a.property_paired))/'report.json'
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
            pair_match={'seeds':plan['seeds_predeclared']==[42,43,44],
                'baseline_hash':plan['baseline_sha256']==sha(matching_base/'trajectory.json'),
                'policy':plan['controller_parameters']==metadata['controller_parameters']}
            info['paired_matching_checks']=pair_match
            required.extend(pair_match.values())
            if variant!='robot':
                config=Path(metadata['variant_config'])
                required.append(plan['config_sha256']==sha(config))
            required.append(plan['stable_policy_necessity_in_tested_seeds'])
            if not plan['stable_policy_necessity_in_tested_seeds']:
                summary['failures_or_unverified'].append(variant+': multi-seed stable necessity failed')
        summary['episodes'][variant] = info
        np = (a.old_data_root if variant=='robot' else a.data_root)/('neutral_'+name)
        nr = read(np/'review_audit.json')
        summary['neutral_controls'][variant] = {'path': str(np), 'review': nr,
            'result': read(np/'result.json'), 'review_sha256': sha(np/'review_audit.json')}
        required.append(nr['passed'])
    retained_base=a.old_data_root/'base_03'
    summary['retained_robot_baseline']={'path':str(retained_base),'gates':{file:{'passed':bool(read(retained_base/file).get(key)),'sha256':sha(retained_base/file)} for file,key in common}}
    required.extend(x['passed'] for x in summary['retained_robot_baseline']['gates'].values())
    old_neutral=a.old_data_root/'neutral_base_01'
    summary['retained_robot_baseline']['neutral_control']={'path':str(old_neutral),'passed':read(old_neutral/'review_audit.json')['passed'],'sha256':sha(old_neutral/'review_audit.json')}
    required.append(summary['retained_robot_baseline']['neutral_control']['passed'])
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
        'Handle-point adaptation and fixed-target planning are distinguished per variant in paired_planning; no claim about all feasible paths.',
        'Local Windows lacks scipy for the collision-scope tests; the actual remote container passed all 15 tests, including dense-pregrasp rejection coverage.'])
    canonical_path=a.canonical_config
    canonical=read(canonical_path)
    summary['canonical_configuration']={'path':str(canonical_path),'sha256':sha(canonical_path),'active_spec_matches':{}}
    for variant in ['human','property','robot']:
        ep=Path(summary['episodes'][variant]['path'])
        def physical_spec(spec):
            return {k:v for k,v in spec.items() if k not in ('semantic_anchor','candidate_evidence')}
        matches=physical_spec(read(ep/'metadata.json')['variant_spec'])==physical_spec(canonical[variant])
        summary['canonical_configuration']['active_spec_matches'][variant]=matches
        required.append(matches)
    summary['configuration_annotation_corrections']=['Historical config06/config07 inherited outdated _revision translation annotations. Explicit position/spec and capture metadata are authoritative. The canonical configuration records corrected translations; physical active specs excluding descriptive semantic_anchor/candidate_evidence annotations are checked against each selected episode. Config06 human capture and its paired report retain their original whole-file hashes.']
    summary['offline_screening'] = {str(path): read(path) for path in sorted(a.report_root.glob('supervisor_*.steps.json'))}
    summary['notes']=['Human and property use fresh dense_pregrasp_v4 baseline; robot retains original baseline and original collection source. Global config hashes differ because inactive risk variants differ; active source layout and policy are checked separately.','Earlier coarse CPU sweep screening cached mutable door collision objects; those candidate passes are not final sweep evidence. Fresh source_layout/avoidance/120Hz audits and search_swept_front_lwx.py are required.']
    summary['artifact_hash_exclusions']=['Finalizer stdout logs: written after manifest generation; excluded to avoid self-referential changing hashes. Raw collection/audit/planning/video evidence is covered.']
    summary['c01_current_gates_passed'] = all(required)
    artifacts = {}
    for root in (a.data_root, a.report_root, a.old_data_root/'robot_01', a.old_data_root/'base_03',a.old_data_root/'neutral_robot_01',a.old_data_root/'neutral_base_01',a.old_report_root/'paired_robot_01'):
        for path in sorted(root.rglob('*')):
            if path.is_file():
                if path.name.startswith('finalize_front_') and path.suffix=='.log':
                    continue
                artifacts[str(path)] = {'sha256': sha(path), 'bytes': path.stat().st_size}
    a.output.mkdir(parents=True)
    (a.output/'gate_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (a.output/'artifact_hashes.json').write_text(json.dumps(artifacts, indent=2)+'\n')
    (a.output/'asset_provenance.json').write_text(json.dumps(asset, indent=2)+'\n')
    print('DELIVERY_REPORT', summary['c01_current_gates_passed'], len(artifacts), 'artifacts', flush=True)
    return 0 if summary['c01_current_gates_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
