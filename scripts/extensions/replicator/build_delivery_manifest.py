"""Bind explicit G-mustard delivery evidence; missing/failed gates stay visible.

This CPU assembler verifies supplied reports and their artifact identities. It
does not run simulation, invent visual approval, or certify other source tasks.
"""
import argparse
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np

from kitchen_safety.extensions.replicator.startup_recorder import audit_startup
from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence
from scripts.extensions.replicator.render_comparison import resolve_recorded_path
from scripts.extensions.replicator.review_episode import ROOT, read, require, sha
from scripts.extensions.replicator.review_neutral_evidence import load_neutral
from scripts.extensions.replicator.trial_evidence import acceptance, report_passed

TASK = 'kitchen_bench_replicator_g_shape_mustard_bowl'
VARIANTS = ('human', 'property', 'robot')
RAW_EPISODE_FILES = ('result.json', 'trajectory.h5', 'images.h5', 'physics_states.npz',
    'background_physics_states.npz', 'grasp_contacts.json', 'cameras.json', 'clearances.json',
    'external_camera.mp4', 'external_camera_2.mp4', 'wrist_camera.mp4', 'all_cameras.mp4',
    'overview_camera.mp4', 'overview/images.h5', 'overview/cameras.json', 'overview/frame_alignment.json')


def verify_source_geometry_coverage(manifest, geometry, episode, expected):
    """Bind reported source distances to every real prefix and motion row.

    This checks coverage, thresholds and arithmetic, not a new FCL replay.
    Source surfaces/cooking limitations remain those of the archived audit.
    """
    from scripts.extensions.replicator.variant_geometry import expectation_passed, summarize_samples
    require(geometry['full_reset_to_end_source_geometry'] is True, 'Source audit omits reset/startup geometry')
    fresh = audit_saved_collection_evidence(episode)
    require(fresh['passed'] and fresh == geometry['continuous_collection_evidence'],
            'Source audit continuous physical timeline differs')
    names = {'startup_physics_states.npz', 'startup_audit.json', 'motion_physics_index.npz',
             'motion_physics_alignment.json', 'collection_provenance.json'}
    require(set(geometry['startup_input_sha256']) == names, 'Source audit startup artifact binding incomplete')
    manifest.hashes(geometry['startup_input_sha256'], relative_to=episode)
    with np.load(episode / 'startup_physics_states.npz', allow_pickle=False) as prefix, \
         np.load(episode / 'physics_states.npz', allow_pickle=False) as motion:
        require(prefix['joint_names'].tolist() == motion['joint_names'].tolist(), 'Source audit joint order differs')
        for rows, data, stream in [(geometry['startup_rows'], prefix, 'startup'), (geometry['rows'], motion, 'motion')]:
            require(len(rows) == len(data['q']), 'Source audit physical samples missing')
            for i, row in enumerate(rows):
                require(row['stream'] == stream and row['physics_index'] == i and
                        row['action_index'] == int(data['action_index'][i]) and row['phase'] == str(data['phase'][i]),
                        'Source audit sample identity differs')
                if stream == 'startup':
                    require(row['absolute_physics_step'] == int(data['physics_step'][i]) and
                            row['startup_stage'] == str(data['startup_stage'][i]), 'Source audit prefix timeline differs')
                for mode in ['source_mesh', 'source_hull', 'conservative_execution']:
                    measured = row[mode]
                    distance, margin = measured['distance_m'], measured['required_m']
                    require(np.isfinite([distance, margin, measured['slack_m']]).all() and distance >= 0 and
                            margin == geometry['required_m'] and abs(measured['slack_m']-(distance-margin)) <= 1e-10 and
                            measured['valid'] is (not measured['intersecting'] and distance >= margin),
                            'Source audit per-sample margin/arithmetic differs')
    startup, motion = geometry['startup_rows'], geometry['rows']
    require(geometry['startup_samples'] == len(startup) and geometry['samples'] == len(motion) and
            geometry['total_physics_samples'] == len(startup)+len(motion), 'Source audit total coverage differs')
    summaries = dict(startup_summary=summarize_samples(startup), summary=summarize_samples(motion),
                     whole_episode_summary=summarize_samples(startup+motion))
    require(all(geometry[key] == value for key, value in summaries.items()), 'Source audit claimed summaries differ')
    require(expectation_passed(summaries['whole_episode_summary'] if expected == 'safe' else summaries['summary'], expected),
            'Source audit safe/risk expectation failed for required interval')


def verify_startup(manifest, episode, neutral):
    """Recompute prefix stability and bind its real samples to recorded actions.

    Prefix FCL is the collector's immutable report, not a new geometry replay.
    Its sample coverage and source snapshots must agree with the raw sidecar.
    """
    report = manifest.document(episode / 'startup_audit.json', require_pass=True)
    result = manifest.document(episode / 'result.json')
    integrity = manifest.document(episode / 'startup_code_integrity.json', require_pass=True)
    require(report['schema_version'] == 1 and report == result['startup_recording'] and
            integrity == result['collection_code_integrity'], 'Startup/result/integrity binding differs')
    raw_path = manifest.file(episode / 'startup_physics_states.npz')
    require(sha(raw_path) == report['input_sha256']['startup_physics_states.npz'], 'Startup NPZ hash changed')
    code_keys = {'collector', 'startup_recorder', 'deferred_safety'}
    require(set(report['code_snapshots']) == code_keys and set(report['input_sha256']) == code_keys | {'startup_physics_states.npz'},
            'Incomplete startup source snapshots')
    codes = {key: hashlib.sha256(report['code_snapshots'][key].encode()).hexdigest() for key in code_keys}
    require(codes == {key: report['input_sha256'][key] for key in code_keys} == integrity['initial_sha256'] ==
            integrity['checked_sha256'], 'Startup source snapshot/initial/final hashes differ')
    with np.load(raw_path, allow_pickle=False) as stored:
        data = {name: stored[name] for name in stored.files}
    meta = report['metadata']
    for key in ['joint_names', 'robot_body_names', 'background_paths', 'static_anchor_paths']:
        require(data[key].tolist() == meta[key] and len(set(meta[key])) == len(meta[key]), 'Startup native map mismatch: ' + key)
    require(bool(meta['joint_names']) and bool(meta['robot_body_names']) and bool(meta['background_paths']),
            'Missing native robot/background identities')
    manifest.document(neutral / 'neutral_probe.json', require_pass=True)
    manifest.file(neutral / 'neutral_states.npz')
    hold, _ = load_neutral(neutral, require_full_physics=True)
    topology = meta['background_topology']
    expected_aliases = {key: row['native_path'] for key, row in hold['articulation_root_aliases'].items()}
    aliases = topology['articulation_root_aliases']
    require({key: row['native_path'] for key, row in aliases.items()} == expected_aliases,
            'Startup/neutral articulation alias mapping differs')
    for authored, row in aliases.items():
        require(row['unique_joint_graph_root'] == [authored] and row['native_path'] in meta['background_paths'] and
                np.isfinite([row[k] for k in ['source_mass_kg', 'native_mass_kg', 'translation_error_m', 'rotation_error_deg']]).all() and
                row['source_mass_kg'] > 0 and abs(row['source_mass_kg']-row['native_mass_kg']) <= 1e-4 * max(1., row['source_mass_kg']) and
                0 <= row['translation_error_m'] <= .002 and 0 <= row['rotation_error_deg'] <= 1.,
                'Unproven startup articulation root alias')
    authored = set(hold['joint_connected_bodies'])
    linked = {expected_aliases.get(path, path) for path in authored}
    endpoints = {(row['joint'], row['role'], row['rigid_body']) for row in topology['joint_endpoints']}
    expected_endpoints = {(row['joint'], row['role'], row['rigid_body']) for row in hold['joint_targets']
                          if row['kind'] == 'dynamic_rigid_body'}
    require(set(meta['background_paths']) == set(hold['background_paths']) == set(result['background_rigid_paths']) and
            set(topology['authored_joint_connected_paths']) == authored and endpoints == expected_endpoints and
            set(topology['joint_connected_native_paths']) == linked and linked <= set(meta['background_paths']),
            'Missing startup joint endpoint/background coverage')
    require(set(meta['static_anchor_paths']) == set(hold['static_anchor_paths']) == set(topology['static_anchor_paths']),
            'Startup static anchors differ from validated source anchors')
    for i, path in enumerate(meta['static_anchor_paths']):
        anchor = hold['static_anchors'][path]
        require(anchor['no_rigid_body_ancestor'] is True and not anchor['animated_source_ancestors'] and
                not anchor['animated_runtime_ancestors'] and
                np.max(np.abs(data['static_anchor_transforms'][0, i]-np.asarray(anchor['source_world_transform']))) <= 1e-6,
                'Startup static anchor initial pose differs from source')
    require(meta['background_physics'] == result['background_physics'] == hold['background_physics'],
            'Startup physical profile differs')
    expected_kind = 'dynamic' if result.get('obstacle') and not result['obstacle']['static'] else 'static' if result.get('obstacle') else 'none'
    require(meta['hazard_kind'] == expected_kind and meta['dynamic_hazard'] is (expected_kind == 'dynamic'),
            'Startup hazard identity/kind differs')
    require(np.all(data['has_hazard_state'] == (expected_kind == 'dynamic')) and
            np.all(data['has_hazard_pose'] == (expected_kind != 'none')), 'Startup hazard presence mask differs')
    recomputed = audit_startup(data, meta)
    require(recomputed['passed'], 'Recomputed startup coverage/stability failed: ' + str(recomputed['checks']))
    for key, value in recomputed.items():
        if key == 'checks':
            require(all(report['checks'].get(name) == passed for name, passed in value.items()), 'Startup claimed gates differ')
        else:
            require(report.get(key) == value, 'Startup claimed statistics differ: ' + key)
    n = len(data['physics_step'])
    prefix = report['prefix_safety']
    require(prefix['passed'] is True and prefix['failure'] is None and prefix['samples'] == prefix['checks'] == n and
            prefix['valid_calls'] == 2*n, 'Incomplete/failed prefix FCL coverage')
    dt, action_dt = float(meta['physics_dt_s']), float(meta['action_dt_s'])
    decimation = round(action_dt/dt)
    require(decimation > 0 and abs(decimation*dt-action_dt) < 1e-9, 'Invalid startup sampling ratio')
    settle_actions = result['effective_controls']['initial_settling_actions']
    require(settle_actions * action_dt >= 5.-1e-8 and n == 1+(8+settle_actions)*decimation,
            'Missing original eight warmups or complete initial hold')
    warmup_end = 1+8*decimation
    require(np.array_equal(data['startup_stage'], ['reset_initial'] + ['warmup']*(8*decimation) + ['initial_settle']*(settle_actions*decimation)) and
            np.array_equal(data['warmup_action_index'], [-1] + np.repeat(np.arange(8), decimation).tolist() + [-1]*(settle_actions*decimation)) and
            np.array_equal(data['action_index'], [-1]*warmup_end + np.repeat(np.arange(settle_actions), decimation).tolist()) and
            np.all(data['phase'] == 'settle'), 'Startup stage/action mapping differs from executed prefix')
    with h5py.File(manifest.file(episode / 'trajectory.h5'), 'r') as trajectory:
        require(abs(float(trajectory.attrs['dt'])-action_dt) < 1e-9 and
                json.loads(trajectory.attrs['joint_names']) == meta['joint_names'] and
                json.loads(trajectory.attrs['robot_body_names']) == meta['robot_body_names'], 'Startup/trajectory native map or dt differs')
        require(len(trajectory['time']) > settle_actions, 'Missing actual execution after startup')
        indices = (8+np.arange(settle_actions+1))*decimation
        for name in ['q', 'dq', 'mustard', 'bowl', 'robot_body_pose_w', 'background_rigid_poses']:
            require(np.array_equal(trajectory['obs/'+name][:settle_actions], data[name][indices[:-1]]) and
                    np.array_equal(trajectory['next_obs/'+name][:settle_actions], data[name][indices[1:]]),
                    'Startup/trajectory actual state mismatch: ' + name)
        total_actions = len(trajectory['time'])
    alignment = manifest.document(episode / 'overview/frame_alignment.json')['measurements']
    first_camera_step = meta['reset_physics_step']+8*decimation
    require(len(alignment) == total_actions and all(row['action_index'] == i and
            row['physics_step_before'] == row['physics_step_after'] == first_camera_step+i*decimation
            for i, row in enumerate(alignment)), 'Startup/recorded camera physics-step handoff differs')
    require(integrity['checked_physics_step'] == first_camera_step+total_actions*decimation,
            'Source integrity check does not cover the complete actual execution')
    with np.load(manifest.file(episode / 'physics_states.npz'), allow_pickle=False) as suffix:
        require(np.array_equal(suffix['action_index'], np.repeat(np.arange(settle_actions, total_actions), decimation)),
                'Prefix/full-execution physics coverage has a missing or duplicate action')
    if expected_kind == 'dynamic':
        from scripts.extensions.replicator.review_physical_controls import audit_complete_dynamic_hazard
        dynamic = audit_complete_dynamic_hazard(episode)
        require(dynamic['passed'], 'Dynamic hazard failed complete reset-to-end original stability gates: ' + str(dynamic['checks']))
        manifest.hashes(dynamic['input_sha256'], relative_to=episode)
        recomputed['complete_dynamic_hazard'] = dynamic
    return recomputed


class Manifest:
    def __init__(self):
        self.checks = {}
        self.artifacts = {}

    def file(self, path):
        path = resolve_recorded_path(path).resolve()
        require(path.is_file(), 'Missing evidence: ' + str(path))
        self.artifacts[str(path)] = dict(sha256=sha(path), bytes=path.stat().st_size)
        return path

    def document(self, path, require_pass=False):
        data = read(self.file(path))
        if require_pass:
            require(report_passed(data), 'Report did not pass: ' + str(path))
            if isinstance(data, dict) and 'checks' in data:
                checks = data['checks']
                if isinstance(checks, dict):
                    passed = bool(checks) and all(value is True for value in checks.values())
                elif isinstance(checks, list):
                    passed = bool(checks) and all(isinstance(row, dict) and
                        (row.get('passed') is True if 'passed' in row else row.get('intersecting_pair_count') == 0)
                        for row in checks)
                else:
                    passed = False
                require(passed, 'Report contains failed or missing checks: ' + str(path))
        return data

    def check(self, name, operation):
        try:
            operation()
            self.checks[name] = dict(passed=True)
        except Exception as error:
            self.checks[name] = dict(passed=False, error=f'{type(error).__name__}: {error}')

    def hashes(self, mapping, relative_to=None):
        require(isinstance(mapping, dict) and bool(mapping), 'Missing artifact hash mapping')
        for name, digest in mapping.items():
            path = Path(name)
            if relative_to is not None and not path.is_absolute():
                path = Path(relative_to) / path
            actual = self.file(path)
            require(sha(actual) == digest, 'Changed reviewed artifact: ' + str(actual))

    def visual(self, filename, episode, required_files):
        document = self.document(filename, require_pass=True)
        require(resolve_recorded_path(document['episode']).resolve() == episode, 'Visual review refers to a different episode')
        require(isinstance(document.get('observations'), list) and bool(document['observations']) and
                all(isinstance(item, str) and item.strip() for item in document['observations']),
                'Explicit visual observations are required; no automatic approval')
        require(bool(document.get('scope')), 'Visual review must declare its sampled/full scope')
        self.hashes(document['sha256'])
        bound = {resolve_recorded_path(path).resolve() for path in document['sha256']}
        require(set(required_files) <= bound, 'Visual review does not bind the required images/video')
        return document


def episode_checks(manifest, label, spec, task_id, neutral):
    episode, reports = resolve_recorded_path(spec['episode']).resolve(), resolve_recorded_path(spec['reports']).resolve()

    def physical():
        for name in RAW_EPISODE_FILES:
            manifest.file(episode / name)
        result = manifest.document(episode / 'result.json')
        require(result['task_id'] == task_id, 'Wrong source task')
        summary = manifest.document(reports / 'summary.json', require_pass=True)
        evidence = acceptance(episode, reports, summary['exit_codes'], require_rgb=True)
        require(evidence['passed'], 'Current runner evidence failed: ' + str(evidence['acceptance_gates']))
        for name in ['validation.json', 'videos.json']:
            manifest.document(reports / name, require_pass=True)
        for name in ['source_relation_audit.json', 'self_collision_audit.json', 'overview_validation.json']:
            manifest.document(episode / name, require_pass=True)
        manifest.file(reports / 'collection.log')

    def actual():
        data = manifest.document(spec['actual_review'], require_pass=True)
        require(resolve_recorded_path(data['episode']).resolve() == episode, 'Actual review episode mismatch')
        require(resolve_recorded_path(data['neutral']).resolve() == neutral, 'Actual review neutral mismatch')
        require(data['neutral_raw_schema'] == 'full_post_reset_physics' and
                data['background_reference'] == 'reset_initial_full_physics',
                'Final actual review requires complete reset-based background and neutral physics')
        manifest.hashes(data['input_sha256'])
        manifest.hashes(data['review_code_sha256'])
        manifest.hashes(data['predicate_source_sha256'])
        bottle = manifest.file(data['source_bottle']['path'])
        require(sha(bottle) == data['source_bottle']['sha256'], 'Reviewed bottle asset changed')

    def visual():
        raw = manifest.document(spec['raw_review'])
        require(resolve_recorded_path(raw['episode']).resolve() == episode and
                raw['source_predicate'] is True and raw['current_final_on_destination'] is True,
                'Wrong or unsuccessful raw montage source')
        manifest.hashes(raw['sha256'], relative_to=episode)
        montage = manifest.file(resolve_recorded_path(spec['raw_review']).parent / raw['raw_montage'])
        inspected = manifest.visual(spec['visual_review'], episode, [episode / 'images.h5', episode / 'trajectory.h5', montage])
        require(inspected['indices'] == raw['montage_indices'] and
                {'external_camera_2', 'wrist_camera'} <= set(inspected['cameras']), 'Visual sampled frames/cameras mismatch')

    def collection():
        recomputed = audit_saved_collection_evidence(episode)
        require(recomputed['passed'], 'Absolute timeline/source provenance failed: ' +
                str(recomputed.get('error', recomputed['checks'])))
        manifest.hashes(recomputed['input_sha256'], relative_to=episode)
        alignment = manifest.document(episode / 'motion_physics_alignment.json', require_pass=True)
        provenance = manifest.document(episode / 'collection_provenance.json', require_pass=True)
        result = manifest.document(episode / 'result.json')
        require(result['motion_physics_alignment'] == alignment and result['collection_provenance'] == provenance,
                'Absolute timeline/provenance differs from actual result')
        require(provenance['background_physics'] == result['background_physics'] and
                provenance['checked_physics_step'] == result['collection_code_integrity']['checked_physics_step'],
                'Source profile or final actual physics step differs')

    manifest.check(label + '.physical_and_rgb', physical)
    manifest.check(label + '.independent_actual', actual)
    manifest.check(label + '.sampled_visual', visual)
    manifest.check(label + '.complete_startup_physics', lambda: verify_startup(manifest, episode, neutral))
    manifest.check(label + '.absolute_motion_and_provenance', collection)
    return episode


def build(spec, spec_path=None):
    manifest = Manifest()
    if spec_path is not None:
        manifest.file(spec_path)
    baselines, variants = spec.get('baselines', {}), spec.get('variants', {})
    neutral = resolve_recorded_path(spec['neutral']).resolve()

    def structure():
        require(spec['task_id'] == TASK, 'This first-task checker only handles the G-mustard source task')
        require(set(baselines) == {'base', 'human'} and set(variants) == set(VARIANTS),
                'Require both matched baselines and all three variants')
        require(spec['planning_seeds'] == [42, 43, 44], 'Predeclared planning seed set differs')
        episodes = [row['episode'] for row in [*baselines.values(), *variants.values()]]
        require(len(set(episodes)) == 5, 'Each physical episode must be unique')

    def source():
        assignment = manifest.document(spec['assignment'])
        task = next(row for row in assignment['tasks'] if row['id'] == spec['task_id'])
        require(sha(manifest.file(task['source_yaml'])) == task['source_sha256'], 'Source YAML changed')
        assets = manifest.document(spec['asset_manifest'])
        manifest.document(spec['asset_verification'], require_pass=True)
        for row in assets['files']:
            path = manifest.file(row['path'])
            require(path.stat().st_size == row['bytes'] and sha(path) == row['sha256'], 'Source asset changed: ' + str(path))
        require(bool(spec['provenance_files']), 'Explicit implementation/source provenance is required')
        for path in spec['provenance_files']:
            manifest.file(path)

    def physics():
        manifest.document(neutral / 'neutral_probe.json', require_pass=True)
        manifest.file(neutral / 'neutral_states.npz')
        hold, _ = load_neutral(neutral, require_full_physics=True)
        require(hold['duration_seconds'] >= 45., 'Final neutral evidence must cover at least 45 seconds')
        decoded = hold['video_validation']
        expected = {neutral / name for name in ['all_cameras.mp4', 'external_camera.mp4', 'external_camera_2.mp4',
                    'wrist_camera.mp4', 'overview/all_cameras.mp4', 'overview/overview_camera.mp4']}
        require(isinstance(decoded, dict) and {neutral / name for name in decoded} == expected,
                'Missing neutral video decode checks')
        for name, row in decoded.items():
            require(row['frames'] == hold['video_frames'] and sha(manifest.file(neutral / name)) == row['sha256'],
                    'Neutral video decode coverage or hash changed')
        visual = manifest.document(neutral / 'visual_review.json')
        require(visual['visual_sample_passed'] is True and bool(visual['observations']) and bool(visual['scope']),
                'Explicit neutral visual inspection missing or failed')
        require({'external_camera_2', 'overview_camera'} <= set(visual['cameras']) and
                bool(visual['frames']) and all(0 <= index < hold['video_frames'] for index in visual['frames']),
                'Neutral sampled camera/frame coverage invalid')
        require({'images.h5', 'overview/images.h5', 'neutral_probe.json', 'neutral_states.npz', 'neutral_review_montage.jpg'} <=
                set(visual['sha256']), 'Neutral visual review must bind original imagery and state')
        manifest.hashes(visual['sha256'], relative_to=neutral)
        mobility_dir = resolve_recorded_path(spec['mobility'])
        mobility = manifest.document(mobility_dir / 'mobility_probe.json', require_pass=True)
        require(mobility['background_physics_provenance'] == hold['background_physics'], 'Mobility/neutral physics profile mismatch')
        require(all(mobility['execution_configuration'][key] == hold['execution_configuration'][key]
                    for key in ['simulation_device', 'physics_threads', 'torch_threads']),
                'Mobility/neutral solver execution configuration differs')
        from scripts.extensions.replicator.review_physical_controls import audit_mobility
        raw_mobility = audit_mobility(mobility_dir)
        require(raw_mobility['passed'], 'Mobility raw physical/wrench recomputation failed')
        manifest.hashes(mobility['input_sha256'], relative_to=mobility_dir)
        independent = manifest.document(spec['mobility_review'], require_pass=True)
        require(resolve_recorded_path(independent['mobility']).resolve() == mobility_dir.resolve() and
                independent['raw_physical_review'] == raw_mobility, 'Independent mobility review identifies different raw evidence')
        manifest.hashes(independent['input_sha256'], relative_to=mobility_dir)
        manifest.hashes(independent['review_code_sha256'])
        for name in ['external_camera.mp4', 'external_camera_2.mp4', 'wrist_camera.mp4', 'all_cameras.mp4', 'overview/overview_camera.mp4']:
            row = independent['videos'][name]
            require(mobility['decoded_frames'][Path(name).stem] == row['frames'] == 90 and row['dt'] == mobility['dt'] and
                    sha(manifest.file(mobility_dir / name)) == row['sha256'],
                    'Mobility video decode coverage or hash changed')
        sampled = manifest.document(mobility_dir / 'visual_review.json')
        require(sampled['visual_sample_passed'] is True and bool(sampled['observations']) and bool(sampled['scope']) and
                'overview_camera' in sampled['cameras'] and bool(sampled['frames']) and
                all(0 <= i < 90 for i in sampled['frames']), 'Mobility explicit sampled visual review missing')
        require({'overview/images.h5', 'overview/overview_camera.mp4', 'mobility_probe.json', 'recorded_states.json',
                 'wrench_calls.json', 'mobility_review_montage.jpg'} <= set(sampled['sha256']),
                'Mobility visual scope does not bind actual raw evidence')
        manifest.hashes(sampled['sha256'], relative_to=mobility_dir)

    manifest.check('complete_specification', structure)
    manifest.check('source_assets_and_implementation', source)
    manifest.check('neutral_and_mobility', physics)
    for label, entry in baselines.items():
        episode_checks(manifest, 'baseline.' + label, entry, spec['task_id'], neutral)
    for variant, entry in variants.items():
        episode = episode_checks(manifest, variant, entry, spec['task_id'], neutral)

        def causal(entry=entry, variant=variant, episode=episode):
            baseline_name = 'human' if variant == 'human' else 'base'
            require(entry['baseline'] == baseline_name, 'Wrong matching control profile')
            baseline = resolve_recorded_path(baselines[baseline_name]['episode']).resolve()
            layout = manifest.file(entry['layout'])
            audit = manifest.document(episode / 'avoidance_audit.json')
            require(audit['accepted'] is True and bool(audit['gates']) and all(v is True for v in audit['gates'].values()),
                    'Avoidance audit failed')
            require(resolve_recorded_path(audit['baseline_episode']).resolve() == baseline, 'Avoidance baseline mismatch')
            for key, name in [('result', 'result.json'), ('trajectory', 'trajectory.h5'), ('physics_states', 'physics_states.npz')]:
                require(sha(manifest.file(baseline / name)) == audit['baseline_sha256'][key], 'Avoidance baseline artifact changed')
            for name in ['fixed_base_hazard_audit.json', 'background_consistency_audit.json']:
                manifest.document(episode / name, require_pass=True)
            manifest.document(resolve_recorded_path(entry['reports']) / 'audit.json', require_pass=True)
            manifest.file(episode / 'path_comparison.png')
            for key, expected, subject in [('source_risk', 'risk', baseline), ('source_safe', 'safe', episode)]:
                geometry = manifest.document(entry[key], require_pass=True)
                require(geometry['expected'] == expected and resolve_recorded_path(geometry['episode']).resolve() == subject,
                        'Wrong source geometry expectation/episode')
                require(geometry['layout_sha256'] == sha(layout) and geometry['physics_sha256'] == sha(subject / 'physics_states.npz') and
                        geometry['result_sha256'] == sha(subject / 'result.json'), 'Source geometry inputs changed')
                manifest.hashes(geometry['code_sha256'])
                from scripts.extensions.replicator.variant_geometry import variant_settings
                _, required, planning, _ = variant_settings(read(layout), variant)
                require(geometry['required_m'] == required and geometry['planning_required_m'] == planning,
                        'Source geometry changed the original variant margins')
                verify_source_geometry_coverage(manifest, geometry, subject, expected)
                source = geometry['source']
                for item in [source['bottle_mesh'], source['bottle_hull'], source['robot_source_mesh']]:
                    manifest.hashes({item['source']: item['sha256']})
                manifest.hashes({source['robot_source']: source['robot_sha256']})
                manifest.hashes({item['path']: item['sha256'] for item in source['hazard_layers']})

        def paired(entry=entry, variant=variant):
            baseline = resolve_recorded_path(baselines[entry['baseline']]['episode']).resolve()
            pair_path = manifest.file(entry['paired_report'])
            pairs = manifest.document(pair_path, require_pass=True)
            require(pairs['status'] == 'completed' and pairs['variant'] == variant and pairs['seeds'] == spec['planning_seeds'],
                    'Incomplete or incorrect planning pairs')
            require(resolve_recorded_path(pairs['baseline']).resolve() == baseline, 'Planning baseline mismatch')
            require(pairs['baseline_result_sha256'] == sha(baseline / 'result.json') and
                    pairs['baseline_trajectory_sha256'] == sha(baseline / 'trajectory.h5') and
                    pairs['layout_sha256'] == sha(resolve_recorded_path(entry['layout'])), 'Planning inputs changed')
            require(len(pairs['rows']) == 6 and len(pairs['pairs']) == 3 and
                    all(row['passed'] is True and all(v is True for v in row['gates'].values()) for row in pairs['pairs']),
                    'Missing or failed seed pair')
            manifest.hashes(pairs['code_sha256'])
            for row in pairs['rows']:
                require(row['success'] is True, 'Failed planner proposal retained')
                require(sha(manifest.file(pair_path.parent / row['path_file'])) == row['path_sha256'], 'Paired NPZ changed')
                geometry_file = pair_path.parent / (Path(row['path_file']).stem + '_geometry.json')
                require(sha(manifest.file(geometry_file)) == row['geometry_sha256'], 'Paired geometry trace changed')

        def comparison(entry=entry, episode=episode):
            video = manifest.file(entry['comparison'])
            report = manifest.document(video.with_suffix('.validation.json'), require_pass=True)
            require(resolve_recorded_path(report['episode']).resolve() == episode and report['output_sha256'] == sha(video),
                    'Comparison video identity mismatch')
            manifest.hashes(report['input_sha256'])
            manifest.visual(entry['comparison_visual_review'], episode, [video])

        def explanation(entry=entry, episode=episode):
            baseline = resolve_recorded_path(baselines[entry['baseline']]['episode']).resolve()
            video = manifest.file(entry['geometry_explanation'])
            validation = video.with_suffix('.validation.json')
            report = manifest.document(validation, require_pass=True)
            require(report['diagnostic_only'] is False and report['complete_source_physics_bound'] is True,
                    'A diagnostic or incomplete source explanation cannot satisfy final delivery')
            require(resolve_recorded_path(report['episode']).resolve() == episode and
                    resolve_recorded_path(report['baseline']).resolve() == baseline and
                    resolve_recorded_path(report['layout']).resolve() == resolve_recorded_path(entry['layout']).resolve() and
                    report['output_sha256'] == sha(video), 'Source explanation identities differ')
            sources = [resolve_recorded_path(entry[key]).resolve() for key in ['source_risk', 'source_safe']]
            require([resolve_recorded_path(path).resolve() for path in report['source_reports']] == sources,
                    'Explanation source audits differ from accepted real evidence')
            manifest.hashes(report['input_sha256'])
            bound = {resolve_recorded_path(path).resolve() for path in report['input_sha256']}
            for subject in [baseline, episode]:
                require({subject / name for name in ['result.json', 'trajectory.h5', 'physics_states.npz',
                    'startup_physics_states.npz', 'startup_audit.json', 'motion_physics_index.npz',
                    'motion_physics_alignment.json', 'collection_provenance.json']} <= bound,
                    'Explanation misses exact full-physics inputs')
            require(set(sources) <= bound, 'Explanation does not hash its source audits')
            counts, rates = [], []
            for subject in [baseline, episode]:
                with h5py.File(subject / 'trajectory.h5') as f:
                    counts.append(len(f['action'])); rates.append(float(f.attrs['dt']))
            require(rates[0] == rates[1] == report['dt'] and
                    len(report['frames']) == report['decoded_frames'] == max(counts), 'Explanation frame rate/count differs')
            for i, row in enumerate(report['frames']):
                require(row['output_frame'] == i and len(row['panels']) == 2 and all(
                    panel['image_obs_index'] == min(i, count-1) and panel['held'] is (i >= count)
                    for panel, count in zip(row['panels'], counts)), 'Explanation warps or mislabels original frames')
            inspected = manifest.visual(entry['geometry_explanation_visual_review'], episode, [video, validation])
            require(inspected['camera'] == report['camera'] and bool(inspected['indices']) and
                    all(0 <= i < max(counts) for i in inspected['indices']), 'Explanation sampled camera/frames mismatch')

        manifest.check(variant + '.causal_and_source_geometry', causal)
        manifest.check(variant + '.three_seed_free_transfer', paired)
        manifest.check(variant + '.comparison_and_explicit_visual', comparison)
        manifest.check(variant + '.source_explanation_and_explicit_visual', explanation)
    for path in spec.get('retained_failed_attempts', []):
        manifest.check('retained_attempt.' + path, lambda path=path: manifest.file(path))
    failures = {name: row['error'] for name, row in manifest.checks.items() if not row['passed']}
    passed = bool(manifest.checks) and not failures
    coverage = dict(group_original_tasks=14, source_task=spec.get('task_id'), current_task_complete=passed,
                    other_original_tasks_certified=0, variants={variant: {gate: manifest.checks.get(variant + '.' + gate, {'passed': False})['passed']
                    for gate in ['physical_and_rgb', 'independent_actual', 'sampled_visual', 'complete_startup_physics', 'absolute_motion_and_provenance',
                                 'causal_and_source_geometry', 'three_seed_free_transfer', 'comparison_and_explicit_visual',
                                 'source_explanation_and_explicit_visual']}
                    for variant in VARIANTS})
    return dict(schema_version=1, task_id=spec.get('task_id'), passed=passed, coverage=coverage,
                checks=manifest.checks, failed_or_missing=failures, artifacts=manifest.artifacts,
                evidence_scope='One fixed-layout G-mustard task with two matching control profiles, three physical variants and three-seed free-transfer planning. Mechanical evidence/identity assembly only; no new physical or automatic visual certification.',
                visual_policy='Requires separately authored explicit observations bound to reviewed media; this tool never creates a positive visual review.',
                other_source_tasks_not_certified=13, specification=spec)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    try:
        report = build(read(args.spec), args.spec)
    except Exception as error:
        report = dict(passed=False, failed_or_missing={'specification': f'{type(error).__name__}: {error}'})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps({key: report[key] for key in ['passed', 'failed_or_missing']}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
