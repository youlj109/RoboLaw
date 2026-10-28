#!/usr/bin/env python3
"""CPU-only, immutable merge of frozen scene placements and reviewed preview reuse.

Source manifests/configs/evidence remain read-only. Risk-verified source configs
take final precedence and are copied byte-for-byte. This is not safety acceptance.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import atomic_json, stamp

PREFIX = 'kitchen_bench_replicator_'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Expected JSON object: ' + str(path))
    return data


def local(root, value):
    path = (root / value).resolve()
    path.relative_to(root)
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def bound(root, value, expected=None):
    path = local(root, value)
    if expected is not None and digest(path) != expected:
        raise ValueError('Bound input changed: ' + str(path))
    return path


def index_rows(data):
    rows = data.get('scenes', data.get('entries', []))
    result = {}
    for row in rows:
        if row['scene_id'] in result:
            raise ValueError('Duplicate scene: ' + row['scene_id'])
        expected = row['task_id'].removeprefix(PREFIX) + '__' + row['variant_id']
        if row['scene_id'] != expected:
            raise ValueError('Scene identity mismatch: ' + row['scene_id'])
        result[row['scene_id']] = row
    return result


def merged_rows(root, base, overlays, risk, expected_total=154):
    rows = copy.deepcopy(index_rows(base))
    if len(rows) != expected_total:
        raise ValueError(f'Expected {expected_total} base scenes, got {len(rows)}')
    for overlay in overlays:
        for scene_id, candidate in index_rows(overlay).items():
            if scene_id not in rows:
                raise ValueError('Unknown overlay scene: ' + scene_id)
            previous = rows[scene_id]
            if candidate.get('status') == 'visual_candidate' and candidate.get('semantic_geometry_passed') is True:
                bound(root, candidate['config_path'], candidate['config_sha256'])
                rows[scene_id] = {**previous, **copy.deepcopy(candidate), 'status':'adjusted', 'merge_origin':'tabletop_visual_candidate'}
            else:
                rows[scene_id] = {**previous, 'status':'needs_repair', 'semantic_geometry_passed':False,
                    'reason':candidate.get('reason') or 'Visual adjustment search did not produce a feasible candidate; previous geometry retained.',
                    'evidence_path':candidate['evidence_path'], 'evidence_sha256':candidate['evidence_sha256'],
                    'merge_origin':'tabletop_failed_preserve_previous_config'}
    risk_ids = []
    for scene_id, source in index_rows(risk).items():
        if source.get('risk_verified') is not True:
            continue
        if scene_id not in rows:
            raise ValueError('Unknown risk scene: ' + scene_id)
        evidence = bound(root, source['evidence_path'], source['evidence_sha256'])
        report = read(evidence)
        config = bound(root, source['config_path'], report['config_sha256'])
        if report.get('risk_verified') is not True or report.get('geometry_passed') is not True:
            raise ValueError('Risk source lacks bound passing geometry/risk: ' + scene_id)
        selected_before_risk = rows[scene_id]['config_sha256']
        rows[scene_id].update(config_path=source['config_path'], config_sha256=digest(config),
            risk_verified=True, risk_evidence_path=source['evidence_path'], risk_evidence_sha256=source['evidence_sha256'],
            merge_origin='preserved_risk_verified_bytes')
        # Keep a semantic claim only if its checked config remains the one selected.
        old = index_rows(base)[scene_id]
        if selected_before_risk != digest(config) and old.get('config_sha256') == digest(config):
            rows[scene_id].update(**{k:old.get(k) for k in ('status','semantic_geometry_passed','evidence_path','evidence_sha256','reason')})
        if old.get('config_sha256') != digest(config):
            rows[scene_id].update(semantic_geometry_passed=False, status='retained',
                reason='Preserved risk-verified config bytes; visual semantic placement needs fresh review.')
        risk_ids.append(scene_id)
    for scene_id, row in rows.items():
        config = bound(root, row['config_path'], row['config_sha256'])
        original = bound(root, row['original_config_path'])
        current, old = read(config), read(original)
        if current.get('task_id') != row['task_id'] or {k:v for k,v in current.items() if k != 'obstacle'} != {k:v for k,v in old.items() if k != 'obstacle'}:
            raise ValueError('Original task geometry changed: ' + scene_id)
        if row.get('semantic_geometry_passed') is True and not row.get('evidence_sha256'):
            raise ValueError('Passing geometry lacks an evidence hash: '+scene_id)
        if row.get('evidence_path'):
            evidence = bound(root, row['evidence_path'], row.get('evidence_sha256'))
            row['evidence_sha256'] = digest(evidence)
        row.update(native_stability_verified=False, safe_execution_verified=False, accepted=False,
                   original_config_sha256=digest(original))
    return rows, sorted(risk_ids)


def review_index(root, folder):
    result = {}
    for path in sorted(folder.glob('review_*.json'), key=lambda p:p.stat().st_mtime_ns):
        for row in read(path).get('entries', []):
            result[row['scene_id']] = dict(row, review_path=path.relative_to(root).as_posix(), review_sha256=digest(path))
    return result


def reusable_preview(root, row, review, scene):
    """Reuse only a current candidate review with unchanged scene and loaded-image provenance."""
    if review.get('visual_status') != 'candidate' or review.get('preview_path') != scene.get('preview_path'):
        return None
    try:
        original = bound(root, row['original_config_path'], row['original_config_sha256'])
        current = bound(root, row['config_path'], row['config_sha256'])
        if read(current) != read(original):
            return None
        image = bound(root, review['preview_path'], review['preview_sha256'])
        evidence = bound(root, scene['evidence_path'])
        loaded = read(evidence)
        if (loaded.get('layout_sha256') != digest(original) or loaded.get('preview_sha256') != digest(image)
                or loaded.get('checks', {}).get('simulation_loaded') is not True):
            return None
        bound(root, review['review_path'], review['review_sha256'])
    except (ValueError, KeyError, OSError):
        return None
    return dict(scene_id=row['scene_id'], task_id=row['task_id'], variant_id=row['variant_id'],
        current_config_path=row['config_path'], current_config_sha256=row['config_sha256'],
        original_config_path=row['original_config_path'], original_config_sha256=digest(original),
        preview_path=review['preview_path'], preview_sha256=digest(image),
        visual_review_path=review['review_path'], visual_review_sha256=review['review_sha256'],
        visual_status='candidate', source_render_evidence_path=scene['evidence_path'], source_render_evidence_sha256=digest(evidence),
        reason='沿用原图，摆位未变；原图初筛为可保留待验证。未补计新多视角像素条件或安全验证。')


def completed_renders(root):
    """Current-config render evidence may be reused across equivalent config paths."""
    result = {}
    folder = root/'reports/replicator_safety_upgrade/visual_placement'
    for path in sorted(folder.glob('render*progress.json'),key=lambda p:p.stat().st_mtime_ns):
        for row in read(path).get('entries',[]):
            if row.get('render_status') not in ('ready','needs_view_review'):
                continue
            try:
                evidence=bound(root,row['evidence_path'],row.get('evidence_sha256'))
                report=read(evidence)
                if (report.get('config_sha256') != row['config_sha256']
                        or report.get('readability',{}).get('readability_passed') != row.get('readability_passed')):
                    continue
                if not {'overview','operation_side','operation_top'} <= {v['kind'] for v in row['views']}:
                    continue
                for view in row['views']:
                    bound(root,view['preview_path'],view['preview_sha256'])
            except (ValueError,KeyError,OSError):
                continue
            result[(row['scene_id'],row['config_sha256'])]=dict(row,source_progress_path=path.relative_to(root).as_posix(),
                evidence_sha256=digest(evidence))
    return result


def finalize(root, base_path, overlay_paths, risk_path, config_dir, report_dir, check_only=False):
    inputs = [base_path, *overlay_paths, risk_path, root/'reports/replicator_scene_build/progress.json']
    snapshots = {p.relative_to(root).as_posix():digest(p) for p in inputs}
    base, overlays, risk = read(base_path), [read(p) for p in overlay_paths], read(risk_path)
    if any('running' in str(d.get('stage','')) for d in [base,*overlays]):
        raise ValueError('Input placement still running; wait for freeze')
    rows, risk_ids = merged_rows(root, base, overlays, risk)
    if len(risk_ids) != 21:
        raise ValueError(f'Expected all 21 preserved risk scenes, got {len(risk_ids)}')
    reviews = review_index(root, root/'reports/replicator_safety_upgrade/visual_triage')
    original_scenes = index_rows(read(inputs[-1]))
    reused, render_reused, render, values = [], [], [], []
    previous_renders=completed_renders(root)
    copy_plan = []
    for scene_id, source in rows.items():
        row = copy.deepcopy(source)
        source_config = local(root,row['config_path'])
        target_config = config_dir / row['task_id'].removeprefix(PREFIX) / (row['variant_id']+'.json')
        target_evidence = report_dir / 'evidence' / row['task_id'].removeprefix(PREFIX) / (row['variant_id']+'.json')
        evidence = bound(root,row['evidence_path'],row['evidence_sha256']) if row.get('evidence_path') else None
        copy_plan.append((source_config,target_config))
        if evidence:
            copy_plan.append((evidence,target_evidence))
        reuse = reusable_preview(root,row,reviews.get(scene_id,{}),original_scenes.get(scene_id,{}))
        row.update(merged_source_config_path=row['config_path'], merged_source_evidence_path=row['evidence_path'],
            config_path=target_config.relative_to(root).as_posix(), evidence_path=target_evidence.relative_to(root).as_posix() if evidence else None,
            visual_review=reviews.get(scene_id,{}), new_visual_review_status='pending')
        prior_render=previous_renders.get((scene_id,row['config_sha256']))
        if prior_render:
            render_reused.append(dict(prior_render,current_config_path=row['config_path'],current_config_sha256=row['config_sha256'],
                reason='Reused completed multi-view renderer evidence with identical config bytes; source config path may differ.'))
        elif reuse:
            reuse['current_config_path'] = row['config_path']
            reused.append(reuse)
        else:
            row['render_reason'] = '配置变化或原图非candidate/缺完整绑定证据，需新多视角渲染。'
            render.append(row)
        values.append(row)
    summary = dict(total=len(values), render_required=len(render), preview_reused=len(reused), completed_render_reused=len(render_reused),
        reused_render_needs_view_review=sum(r['render_status']=='needs_view_review' for r in render_reused),
        risk_byte_exact_preserved=len(risk_ids), statuses={s:sum(r['status']==s for r in values) for s in ('adjusted','retained','needs_repair','pending')})
    if check_only:
        return summary
    if config_dir.exists() or report_dir.exists():
        raise FileExistsError('Final output exists; use new --config-output and --report-output directories')
    for source,target in copy_plan:
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(source.read_bytes())
        if digest(target) != digest(source):
            raise ValueError('Byte copy verification failed: '+str(target))
    # Source manifests, config/evidence inputs and source task geometry remain bound.
    if any(digest(root/p)!=sha for p,sha in snapshots.items()):
        raise ValueError('Input manifest changed during merge; final progress not published')
    for row in values:
        if digest(root/row['config_path']) != row['config_sha256'] or digest(root/row['merged_source_config_path']) != row['config_sha256']:
            raise ValueError('Config changed during merge: '+row['scene_id'])
    common=dict(schema_version=1,stage='final_visual_placement_merged',updated_at=stamp(),total=len(values),
        scope='Final CPU visual-placement merge; originals retained, no new native physics, task completion or safety acceptance.',
        source_manifests_sha256=snapshots,counts=summary)
    atomic_json(config_dir/'manifest.json',dict(common,tasks=sorted({r['task_id'] for r in values}),scenes=values))
    atomic_json(report_dir/'preview_reuse.json',dict(common,entries=reused))
    atomic_json(report_dir/'render_reuse.json',dict(common,entries=render_reused))
    atomic_json(report_dir/'render_manifest.json',dict(common,total=len(render),scenes=render))
    atomic_json(report_dir/'config_integrity.json',dict(common,risk_scene_ids=risk_ids,
        all_config_hashes_match=True,original_task_geometry_preserved=True,
        copied_files=[dict(source=s.relative_to(root).as_posix(),destination=t.relative_to(root).as_posix(),sha256=digest(t)) for s,t in copy_plan]))
    atomic_json(report_dir/'progress.json',dict(common,entries=values))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-manifest',type=Path,default=ROOT/'configs/extensions/replicator/visual_scene_variants_v3/manifest.json')
    parser.add_argument('--overlay-manifest',type=Path,action='append')
    parser.add_argument('--risk-progress',type=Path,default=ROOT/'reports/replicator_safety_upgrade/progress.json')
    parser.add_argument('--config-output',type=Path,default=ROOT/'configs/extensions/replicator/visual_scene_variants_final01')
    parser.add_argument('--report-output',type=Path,default=ROOT/'reports/replicator_safety_upgrade/visual_placement_final01')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    overlays=args.overlay_manifest or [ROOT/'configs/extensions/replicator/visual_tabletop_repairs/manifest.json']
    result=finalize(ROOT,args.base_manifest.resolve(),[p.resolve() for p in overlays],args.risk_progress.resolve(),
        args.config_output.resolve(),args.report_output.resolve(),args.check_only)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
