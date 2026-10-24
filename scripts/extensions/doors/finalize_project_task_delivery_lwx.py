"""Package all project workflow requirements; report additional strict screens separately."""
import argparse
import hashlib
import json
import re
from pathlib import Path
from matched_scene_lwx import require_matching_scene
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from kitchen_safety.extensions.doors.source_necessity_lwx import has_original_source_clearance_violation


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def require(value, reason):
    if not value:
        raise RuntimeError(reason)



def audit_schema(root, schema):
    if schema == 'assigned_v1':
        return [('result.json','accepted'),('task_validation_full.json','accepted'),
                ('task_review_audit.json','passed'),('source_contact_audit.json','passed'),
                ('self_collision_audit.json','passed')]
    require(schema == 'original_c01_v1' and read(root/'metadata.json')['task'] ==
            'kitchen_bench_lightwheel_open_fridge', 'Unknown or inapplicable audit schema')
    return [('result.json','accepted'),('validation_full.json','accepted'),
            ('review_audit.json','passed'),('contact_geometry_audit.json','passed'),
            ('self_collision_audit.json','passed')]


def review_path(root, schema):
    return root / ('review_audit.json' if schema == 'original_c01_v1' else 'task_review_audit.json')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists(), 'Output already exists')
    selected = read(args.selection)
    required = [('result.json', 'accepted'), ('task_validation_full.json', 'accepted'),
                ('task_review_audit.json', 'passed'), ('source_contact_audit.json', 'passed'),
                ('self_collision_audit.json', 'passed')]
    manifest, episodes, limitations = {}, {}, list(selected.get('retained_rejections', []))
    for variant in ['base', 'human', 'property', 'robot']:
        item = selected['episodes'][variant]
        root = Path(item['episode'])
        metadata, result = read(root / 'metadata.json'), read(root / 'result.json')
        schema = item.get('audit_schema','assigned_v1')
        required = audit_schema(root,schema)
        gates = {name: bool(read(root / name).get(key)) for name, key in required}
        require(all(gates.values()), f'{root}: independent gates rejected')
        require(metadata['task'] == selected['task'], f'{root}: task mismatch')
        visual = read(item['visual_review'])
        require(visual.get('reviewed_actual_three_camera_keyframes') and
                sha(visual['image']) == visual['image_sha256'], f'{root}: visual review missing')
        review = read(review_path(root,schema))
        neutral = Path(review['matching_neutral']['path'])
        require(all(review['matching_neutral'][key] for key in
                    ['same_parameters', 'covers_operation_duration', 'passed']),
                f'{root}: matching neutral control rejected')
        info = dict(path=str(root), gates=gates, frames=result['frames'],
                    physics_samples=result['physics_samples'], safety_checks=result['safety_checks'],
                    final_door_rad=result['final_door_rad'], neutral=str(neutral),
                    controller_parameters=metadata['controller_parameters'],
                    implementation_sha256=metadata['implementation_sha256'],
                    visual_review=item['visual_review'])
        evidence_roots = [root, neutral]
        extra_files = [Path(item['visual_review']), Path(visual['image']), Path(metadata['variant_config'])]
        if variant != 'base':
            baseline = Path(item['baseline'])
            bm = read(baseline / 'metadata.json')
            require_matching_scene(bm, metadata['variant_config'], metadata)
            baseline_schema = item.get('baseline_audit_schema','assigned_v1')
            baseline_required = audit_schema(baseline,baseline_schema)
            require(all(read(baseline / name).get(key) for name, key in baseline_required),
                    f'{root}: baseline rejected')
            baseline_review = read(review_path(baseline,baseline_schema))
            require(all(baseline_review['matching_neutral'][key] for key in
                        ['same_parameters', 'covers_operation_duration', 'passed']),
                    f'{root}: baseline matching neutral rejected')
            evidence_roots += [baseline, Path(baseline_review['matching_neutral']['path'])]
            extra_files.append(Path(bm['variant_config']))
            require(metadata['controller_parameters'] == bm['controller_parameters'] and
                    metadata['robot_base'] == bm['robot_base'] and
                    metadata['robot_yaw_deg'] == bm['robot_yaw_deg'] and
                    metadata['source_yaml_sha256'] == bm['source_yaml_sha256'],
                    f'{root}: baseline policy or layout mismatch')
            require(read(root / 'avoidance_audit.json')['accepted'], f'{root}: actual avoidance rejected')
            paired = read(item['paired_report'])
            require(paired['variant'] == variant and
                    paired['controller_parameters'] == metadata['controller_parameters'] and
                    paired['baseline_sha256'] == sha(baseline / 'trajectory.json') and
                    paired['config_sha256'] == sha(metadata['variant_config']) and
                    paired.get('command_ik_margin_applied_rad', 0.) ==
                    metadata['controller_parameters'].get('ik_limit_margin_rad', 0.),
                    f'{root}: paired provenance or command bounds mismatch')
            require(paired['seeds_predeclared'] == metadata['review_gates']['planning_seeds'] and
                    paired['stable_policy_necessity_in_tested_seeds'] and
                    paired['fixed_endpoint_rerouting_supported'], f'{root}: paired necessity rejected')
            pairs = []
            for pair in paired['pairs']:
                off, on = pair['off'], pair['on']
                require(off['success'] and on['success'] and
                        has_original_source_clearance_violation(off['risk_with_added_geometry']),
                        f'{root}: no original source-clearance necessity in seed {pair["seed"]}')
                ratio = on['path_metrics']['joint_l2_length_rad'] / off['path_metrics']['joint_l2_length_rad']
                extra = on['path_metrics']['tcp_length_m'] - off['path_metrics']['tcp_length_m']
                # Project workflow has no numerical15cm cutoff. Keep that additional screen visible.
                actual_intersection=any(v.get('intersecting') for v in off['risk_with_added_geometry']['violations'])
                strict_passed=bool(actual_intersection and ratio<=1.6 and extra<=.15)
                pairs.append(dict(seed=pair['seed'], joint_length_ratio=ratio, added_tcp_length_m=extra,
                                  original_source_clearance_necessity=True, actual_source_intersection=actual_intersection,
                                  additional_strict_screen_passed=strict_passed))
            if schema == 'original_c01_v1':
                comparison_path=root/'comparison_audit.json';comparison=read(comparison_path)
                require(comparison['passed'] and comparison['speed']==1 and
                        comparison['camera']=='external_camera_2' and
                        comparison['source_frame_counts']==[read(baseline/'result.json')['frames'],result['frames']] and
                        abs(comparison['fps']-1./metadata['dt'])<1e-9 and
                        comparison['comparison_sha256']==sha(root/'baseline_comparison.mp4'),
                        f'{root}: original comparison provenance rejected')
                require(read(root/'cameras.json')['external_camera_2']==read(baseline/'cameras.json')['external_camera_2'],
                        f'{root}: original camera mismatch')
                risk_review=read(item['source_risk_review']);risk_path=Path(risk_review['extraction_report']);risk=read(risk_path)
                require(risk_review.get('reviewed_alignment') and sha(risk_review['image'])==risk_review['image_sha256'] and
                        sha(risk_path)==risk_review['extraction_report_sha256'] and
                        risk['comparison_audit_sha256']==sha(comparison_path) and
                        risk['comparison_sha256']==sha(root/'baseline_comparison.mp4') and
                        not risk['physics_replay'] and not risk['unsafe_path_executed'],
                        f'{root}: original source-risk provenance rejected')
                measurement=risk['measurement']
                require(measurement in comparison['measurements'] and measurement['side']==0 and
                        0<=measurement['distance_m']<measurement['required_m'] and not measurement['valid'],
                        f'{root}: original source-margin risk not shown')
                info.update(baseline=str(baseline),paired_report=item['paired_report'],pairs=pairs,
                            source_risk_review=item['source_risk_review'],audit_schema=schema)
                extra_files += [Path(item['source_risk_review']),Path(risk_review['image']),risk_path,
                                Path(item['paired_report']),comparison_path]
            else:
                comparison_path = root / 'baseline_comparison_external_camera_2_03.json'
                comparison = read(comparison_path)
                require(comparison['original_speed'] and comparison['same_view'] and
                        comparison['video_validation'] > 0 and comparison['sha256'] == sha(comparison_path.with_suffix('.mp4')),
                        f'{root}: comparison video invalid')
                info.update(baseline=str(baseline), paired_report=item['paired_report'], pairs=pairs)
                risk_review=read(item['source_risk_review'])
                require(risk_review.get('reviewed_alignment') and
                        sha(risk_review['image']) == risk_review['image_sha256'] and
                        sha(risk_review['illustration_report']) == risk_review['illustration_report_sha256'],
                        f'{root}: source risk illustration has not been visually checked')
                risk=read(risk_review['illustration_report'])
                require(not risk['physics_replay'] and not risk['unsafe_path_executed'] and
                        risk['audit_sha256'] == sha(root / 'avoidance_audit.json') and
                        risk['panels'][0]['risky_robot_bodies'] and not risk['panels'][1]['risky_robot_bodies'],
                        f'{root}: illustration provenance or risk annotation rejected')
                info['source_risk_review']=item['source_risk_review']
                extra_files += [Path(item['source_risk_review']),Path(risk_review['image']),Path(risk_review['illustration_report'])]
                extra_files.append(Path(item['paired_report']))
        for relative, snapshot in metadata.get('source_snapshot', {}).items():
            require(sha(snapshot['path']) == snapshot['sha256'], f'{root}: source snapshot changed: {relative}')
        if not metadata.get('source_snapshot'):
            limitations.append(f'{root}: historical implementation hashes retained; immutable runtime source copies were not captured.')
        files = extra_files + [p for directory in evidence_roots for p in directory.rglob('*')
                               if p.is_file() and 'arena' not in p.relative_to(directory).parts]
        for path in files:
            manifest[str(path)] = dict(bytes=path.stat().st_size, sha256=sha(path))
        episodes[variant] = info
    test_log = Path(selected['unit_test_log']).read_text()
    test_count=re.search(r'Ran (\d+) tests',test_log)
    require(test_count and int(test_count.group(1))>=37 and test_log.rstrip().endswith('OK'), 'Current unit tests not accepted')
    manifest[selected['unit_test_log']] = dict(bytes=Path(selected['unit_test_log']).stat().st_size,
                                               sha256=sha(selected['unit_test_log']))
    limitations += ['One original fixed layout; planning seeds are not physical success rates or proof about all paths.',
                    'Tangential forces and ordering inside one physics substep remain unmeasured.',
                    'FCL source geometry checks do not assert equivalence to PhysX cooked shapes.']
    strict_all=all(pair['additional_strict_screen_passed'] for ep in episodes.values() for pair in ep.get('pairs',[]))
    limitations += ['Project workflow original source-margin necessity does not imply physical source intersection.',
                    'Additional local intersection/1.6 ratio/15cm screen is reported separately; old strict finalizer is unchanged.']
    summary = dict(student='lwx', label=selected['label'], task=selected['task'], complete=True,
                   acceptance_profile='project_workflow_original_clearance_or_intersection',
                   additional_strict_screen_passed=strict_all,
                   episodes=episodes, failures_or_unverified=limitations, unit_test_log=selected['unit_test_log'])
    args.output.mkdir(parents=True)
    for name, value in [('gate_summary.json', summary), ('artifact_hashes.json', manifest), ('selection.json', selected)]:
        (args.output / name).write_text(json.dumps(value, indent=2) + '\n')
    print('PROJECT_TASK_DELIVERY_COMPLETE', selected['label'], len(manifest), 'hashed artifacts')


if __name__ == '__main__':
    main()
