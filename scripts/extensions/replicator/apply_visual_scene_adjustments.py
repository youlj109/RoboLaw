#!/usr/bin/env python3
"""Publish a new, immutable visual scene version from bound correction evidence.

Only two explicitly named task-object positions may change for a repaired base.
That exception requires a separately bound source-geometry/relation certificate.
Images and reviews are reused only for unchanged config hashes. This publishes
visual candidates, never native-physics or trajectory acceptance.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator.finalize_visual_scenes import bound, digest, index_rows, read
from kitchen_safety.extensions.replicator.scene_build_runtime import atomic_json, stamp


def same_task_except_repaired_positions(old, new):
    old, new = copy.deepcopy(old), copy.deepcopy(new)
    old.pop('obstacle', None)
    new.pop('obstacle', None)
    for name in ('mustard_bottle', 'bowl'):
        pos = new['objects'][name]['position']
        if len(pos) != 3 or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in pos):
            raise ValueError('Invalid corrected position: ' + name)
        old['objects'][name]['position'] = pos
    return old == new


def publish(base_path, overlays, certificate_paths, fresh_views, config_dir, report_dir, previous_folder):
    if config_dir.exists() or report_dir.exists():
        raise FileExistsError('Use a new output version; earlier evidence is immutable')
    base = read(base_path)
    rows = copy.deepcopy(index_rows(base))
    if len(rows) != 154 or len({r['task_id'] for r in rows.values()}) != 14:
        raise ValueError('Expected the complete 14 by 11 source manifest')
    certificates, repaired_bases = {}, {}
    for certificate_path in certificate_paths:
        certificate = read(certificate_path)
        task_id = certificate['task_id']
        if task_id in certificates:
            raise ValueError('Duplicate task repair certificate: ' + task_id)
        if not all(certificate.get(k) is True for k in ('passed', 'relations_passed', 'source_geometry_passed')):
            raise ValueError('Task-layout repair has not passed its independent source checks')
        repaired_base = read(bound(ROOT, certificate['repaired_layout_path'], certificate['repaired_layout_sha256']))
        if repaired_base['task_id'] != task_id or len(certificate.get('source_reports', [])) < 2:
            raise ValueError('Repair certificate must bind its task and independent source reports')
        for item in certificate['source_reports']:
            bound(ROOT, item['path'], item['sha256'])
        certificates[task_id] = certificate_path
        repaired_bases[task_id] = repaired_base
    changed, repaired = set(), []
    for overlay_path in overlays:
        for scene_id, update in index_rows(read(overlay_path)).items():
            if scene_id in changed or scene_id not in rows:
                raise ValueError('Duplicate or unknown correction: ' + scene_id)
            old_row = rows[scene_id]
            old = read(bound(ROOT, old_row['config_path'], old_row['config_sha256']))
            new = read(bound(ROOT, update['config_path'], update['config_sha256']))
            evidence = read(bound(ROOT, update['evidence_path'], update['evidence_sha256']))
            if (update.get('semantic_geometry_passed') is not True
                    or evidence.get('semantic_geometry_passed') is not True
                    or evidence.get('config_sha256') != update['config_sha256']):
                raise ValueError('Unbound or failing correction geometry: ' + scene_id)
            old_task = {k:v for k,v in old.items() if k != 'obstacle'}
            new_task = {k:v for k,v in new.items() if k != 'obstacle'}
            if new_task != old_task:
                repaired_base = repaired_bases.get(update['task_id'])
                if (repaired_base is None or not same_task_except_repaired_positions(old, new)
                        or new_task != {k:v for k,v in repaired_base.items() if k != 'obstacle'}):
                    raise ValueError('Uncertified source-task modification: ' + scene_id)
                repaired.append(scene_id)
            row = {**old_row, **update, 'status':'adjusted', 'risk_verified':False,
                   'native_stability_verified':False, 'safe_execution_verified':False, 'accepted':False,
                   'previous_config_path':old_row['config_path'], 'previous_config_sha256':old_row['config_sha256']}
            for key in ('risk_evidence_path', 'risk_evidence_sha256', 'new_visual_review', 'preview_path'):
                row.pop(key, None)
            if scene_id in repaired:
                certificate_path = certificates[update['task_id']]
                row.update(source_layout_repaired=True, source_layout_repair_evidence_path=certificate_path.relative_to(ROOT).as_posix(),
                           source_layout_repair_evidence_sha256=digest(certificate_path), original_task_geometry_preserved=False)
            rows[scene_id] = row
            changed.add(scene_id)
    for repaired_task in certificates:
        task_scenes = {r['scene_id'] for r in rows.values() if r['task_id'] == repaired_task}
        if len(task_scenes) != 11 or task_scenes - set(repaired):
            raise ValueError('A corrected original task must be shared by all its 11 variants: ' + repaired_task)
    if set(fresh_views) - rows.keys():
        raise ValueError('Unknown requested fresh-view scene')
    need_render = changed | set(fresh_views)
    copies, values = [], []
    for scene_id, row in rows.items():
        source = bound(ROOT, row['config_path'], row['config_sha256'])
        target = config_dir / row['task_id'].removeprefix('kitchen_bench_replicator_') / (row['variant_id'] + '.json')
        copies.append((source, target))
        row.update(merged_source_config_path=row['config_path'], config_path=target.relative_to(ROOT).as_posix())
        if scene_id in need_render:
            row['new_visual_review_status'] = 'pending'
        values.append(row)
    for source, target in copies:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    common = dict(schema_version=1, stage='visual_adjustments_published', updated_at=stamp(), total=154,
        scope='Visual/source-geometry correction only; native stability, successful execution and new trajectory risk remain unverified.',
        previous_manifest=base_path.relative_to(ROOT).as_posix(), previous_manifest_sha256=digest(base_path),
        changed_scenes=sorted(changed), fresh_view_scenes=sorted(fresh_views), repaired_source_tasks=sorted(certificates))
    atomic_json(config_dir/'manifest.json', dict(common, tasks=sorted({r['task_id'] for r in values}), scenes=values))
    atomic_json(report_dir/'progress.json', dict(common, entries=values))
    atomic_json(report_dir/'render_manifest.json', dict(common, total=len(need_render), scenes=[r for r in values if r['scene_id'] in need_render]))
    for name in ('preview_reuse.json', 'render_reuse.json'):
        data = read(previous_folder/name)
        retained = [r for r in data['entries'] if r['scene_id'] not in need_render]
        for r in retained:
            r['current_config_path'] = rows[r['scene_id']]['config_path']
        atomic_json(report_dir/name, dict(common, entries=retained))
    for source in previous_folder.glob('review_*.json'):
        data = read(source)
        data['entries'] = [r for r in data['entries'] if r['scene_id'] not in need_render]
        data['scope'] = 'Reused unchanged-config visual reviews; earlier report retained at ' + str(source.relative_to(ROOT))
        data['counts'] = dict(Counter(r['visual_status'] for r in data['entries']))
        data['expected_entries'] = len(data['entries'])
        data['reviewed_image_count'] = sum(len(r.get('reviewed_views', [])) for r in data['entries'])
        atomic_json(report_dir/source.name, data)
    atomic_json(report_dir/'config_integrity.json', dict(common, all_config_hashes_match=all(digest(ROOT/r['config_path'])==r['config_sha256'] for r in values),
        preserved_risk_configs=sum(r.get('risk_verified') is True for r in values), changed_source_task_scenes=repaired,
        source_task_fields_preserved_except_certified_positions=True, render_required=len(need_render)))
    return dict(configured=154, changed=len(changed), source_task_corrected=len(repaired), render_required=len(need_render))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-manifest', type=Path, default=ROOT/'configs/extensions/replicator/visual_scene_variants_final01/manifest.json')
    parser.add_argument('--base-reports', type=Path, default=ROOT/'reports/replicator_safety_upgrade/visual_placement_final01')
    parser.add_argument('--overlay-manifest', type=Path, action='append', required=True)
    parser.add_argument('--repair-certificate', type=Path, action='append', default=[])
    parser.add_argument('--fresh-view', action='append', default=[])
    parser.add_argument('--config-output', type=Path, required=True)
    parser.add_argument('--report-output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(publish(args.base_manifest.resolve(), [p.resolve() for p in args.overlay_manifest],
        [p.resolve() for p in args.repair_certificate], args.fresh_view,
        args.config_output.resolve(), args.report_output.resolve(), args.base_reports.resolve()), ensure_ascii=False))


if __name__ == '__main__':
    main()
