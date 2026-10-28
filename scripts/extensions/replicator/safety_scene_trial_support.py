"""CPU-only preparation for fixed G-mustard safety scene trials.

This adapter preserves the legacy collector's fixed-layout branch. It does not
certify native physics, successful task execution, or a safety benchmark.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

TASK = 'kitchen_bench_replicator_g_shape_mustard_bowl'
OPTIONS = ('grasp_tilt_deg', 'grasp_yaw_deg', 'grasp_offset_z_m', 'grasp_offset_y_m',
           'transport_bottle_height_m', 'supported_release', 'transport_tilt_cap_deg',
           'placement_strategy', 'placement_tcp_height_m')


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def local(root: Path, value: str | Path) -> Path:
    value = str(value)
    if value.startswith('/workspace/'):
        value = value[len('/workspace/'):]
    result = (root / value).resolve()
    result.relative_to(root.resolve())
    return result


def read(path: Path):
    return json.loads(path.read_text())


def convert_scene(scene: dict, reference: dict) -> dict:
    if scene.get('task_id') != TASK:
        raise ValueError('unsupported_task: only G-shape mustard-to-bowl has this collector adapter')
    for key in ('task_id', 'kitchen_usd', 'stand_height', 'robot_position', 'robot_yaw_deg'):
        if scene.get(key) != reference.get(key):
            raise ValueError('Original task geometry differs: ' + key)
    objects = scene.get('objects', {})
    if set(objects) != {'mustard_bottle', 'bowl'}:
        raise ValueError('Original task object identities differ')
    for name, prefix, asset in (
            ('mustard_bottle', 'mustard', 'assets/robolab/objects/hope/mustard_bottle.usd'),
            ('bowl', 'bowl', 'assets/robolab/objects/ycb/bowl.usd')):
        expected = dict(usd_path=asset, position=reference[prefix + '_position'],
                        yaw_deg=reference.get(prefix + '_yaw_deg', 0))
        if objects[name] != expected:
            raise ValueError('Original object geometry differs: ' + name)
    obstacle = scene.get('obstacle', {})
    original = reference.get('obstacle', {})
    for key in ('id', 'category', 'usd_path', 'static'):
        if obstacle.get(key) != original.get(key):
            raise ValueError('Obstacle asset identity/physics differs: ' + key)
    for key, value in obstacle.items():
        if key in ('position', 'yaw_deg', 'label', 'provenance'):
            continue
        if key not in original or value != original[key]:
            raise ValueError('Obstacle geometry/physics cannot be overridden: ' + key)
    position, yaw = obstacle.get('position'), obstacle.get('yaw_deg')
    if (not isinstance(position, list) or len(position) != 3
            or not all(type(x) in (int, float) and math.isfinite(x) for x in position)
            or type(yaw) not in (int, float) or not math.isfinite(yaw)):
        raise ValueError('Obstacle position and yaw must be finite')
    converted = copy.deepcopy(reference)
    converted['obstacle']['position'] = list(position)
    converted['obstacle']['yaw_deg'] = yaw
    assert {k: v for k, v in converted.items() if k != 'obstacle'} == {
        k: v for k, v in reference.items() if k != 'obstacle'}
    return converted


def select_baseline(root: Path, inventory: dict, reference: dict, category: str) -> dict:
    matches = [row for row in inventory.get('tasks', []) if row.get('task_id') == TASK]
    if len(matches) != 1 or matches[0].get('status') != 'verified_baseline':
        raise ValueError('No verified G-mustard baseline inventory entry')
    profile = 'human' if category == 'human' else 'base'
    candidates = [row for row in matches[0].get('baselines', []) if row.get('profile') == profile]
    if len(candidates) != 1:
        raise ValueError('Missing or ambiguous matching baseline profile: ' + profile)
    baseline = candidates[0]
    if (baseline.get('status') != 'verified_baseline' or baseline.get('errors')
            or not baseline.get('checks') or not all(v is True for v in baseline['checks'].values())):
        raise ValueError('Baseline independent acceptance is incomplete')
    hashes = baseline.get('files')
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError('Baseline has no bound files')
    for name, expected in hashes.items():
        if sha(local(root, name)) != expected:
            raise ValueError('Baseline bound input changed: ' + name)
    original = {k: v for k, v in reference.items() if k != 'obstacle'}
    if baseline.get('source_layout') != original:
        raise ValueError('Baseline original layout differs from current source geometry/planning')
    controls = baseline['controls']
    if set(controls['collector_options']) != set(OPTIONS):
        raise ValueError('Unrecognized baseline collector options; inspect instead of silently defaulting')
    planning = reference['planning']
    for name, current in (
            ('transport_object_padding_m', planning.get('transport_object_padding_m_by_variant', {}).get(category, 0.)),
            ('transfer_mode', planning.get('transfer_mode_by_variant', {}).get(category, 'front_waypoints'))):
        if controls.get(name) != current:
            raise ValueError('Baseline control mismatch: ' + name)
    if controls.get('seed') != controls.get('planner_seed'):
        raise ValueError('Fixed-layout collector cannot represent different simulation and planner seeds')
    return copy.deepcopy(baseline)


def collector_arguments(layout_path: str, output: str, layout: dict, baseline: dict) -> list[str]:
    controls, config = baseline['controls'], baseline['execution_configuration']
    result = ['scripts/extensions/replicator/collect.py', '--variant', layout['obstacle']['category'],
              '--obstacle_id', layout['obstacle']['id'], '--layout-json', layout_path, '--output', output,
              '--planner', 'curobo', '--seed', str(controls['seed']),
              '--initial-settling-seconds', str(controls['initial_settling_seconds'])]
    for field, flag in (('simulation_device', '--device'), ('physics_threads', '--physics-threads'),
                        ('torch_threads', '--torch-threads'), ('execution_checks', '--execution-checks'),
                        ('camera_owner', '--camera-owner'), ('camera_device', '--camera-device'),
                        ('episode_length_s', '--episode-length-s'), ('max_steps', '--max-steps')):
        if field not in config:
            raise ValueError('Missing baseline execution configuration: ' + field)
        if config[field] is not None:
            result += [flag, str(config[field])]
    for key in ('background_rgb', 'cpu_rgb_copy', 'reuse_planner'):
        if config.get(key) is True:
            result.append('--' + key.replace('_', '-'))
    result += ['--record-rgb', '--overview-camera']
    for key in OPTIONS:
        value = controls['collector_options'][key]
        if key == 'supported_release':
            if type(value) is not bool:
                raise ValueError('supported_release is not boolean')
            result.append('--supported-release' if value else '--no-supported-release')
        elif value is not None:
            result += ['--' + key.replace('_', '-'), str(value)]
    return result


def review_jobs(episode: str, report: str, layout: str, baseline: str) -> list[dict]:
    """Independent checks remain separate; no aggregate acceptance flag is invented."""
    prefix = 'scripts/extensions/replicator/'
    return [
        dict(name='source_relations', args=[prefix + 'audit_source_relation.py', '--episode', episode,
                                           '--report', report + '/source_relations.json']),
        dict(name='fixed_base', args=[prefix + 'audit_fixed_base.py', '--episode', episode,
                                     '--report', report + '/fixed_base.json']),
        dict(name='self_collision', uses_gpu=True, args=[prefix + 'audit_self_collision.py', episode]),
        dict(name='native_recording_integrity', args=[prefix + 'safety_scene_native_recording_review.py',
                                                    '--episode', episode, '--report', report + '/native_recording.json']),
        dict(name='source_geometry_safe', args=[prefix + 'audit_variant_source_geometry.py', '--episode', episode,
                                               '--layout', layout, '--expect', 'safe', '--report', report + '/source_geometry_safe.json']),
        dict(name='baseline_source_geometry_risk', args=[prefix + 'audit_variant_source_geometry.py', '--episode', baseline,
                                                       '--layout', layout, '--expect', 'risk', '--hazard-episode', episode,
                                                       '--report', report + '/baseline_source_geometry_risk.json']),
        dict(name='video_export', args=['scripts/export_collection_previews.py', episode, '--report', report + '/videos.json']),
        dict(name='data_validation', args=['scripts/validate_collection.py', episode, '--require-rgb',
                                          '--require-previews', '--report', report + '/validation.json']),
        dict(name='overview_validation', args=[prefix + 'validate_overview.py', episode]),
    ]
