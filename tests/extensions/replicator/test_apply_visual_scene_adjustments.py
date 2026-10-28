"""Scene publication must preserve prior evidence and isolate task repairs."""
import copy
import json
from pathlib import Path

import pytest

from scripts.extensions.replicator import apply_visual_scene_adjustments as publish_module


def fixture_data(tmp_path, monkeypatch):
    monkeypatch.setattr(publish_module, 'ROOT', tmp_path)

    def write(path, value):
        target = tmp_path / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(value))
        return target

    rows, overlays, certificates = [], [], []
    for task_index in range(14):
        task = f'task{task_index}'
        layout = dict(task_id=task, robot_position=[0, 0, 0], objects={
            'mustard_bottle': dict(position=[0, 0, 1], usd_path='mustard.usd'),
            'bowl': dict(position=[1, 0, 1], usd_path='bowl.usd')})
        new_base = copy.deepcopy(layout)
        new_base['objects']['bowl']['position'][1] = .05
        for variant_index in range(11):
            variant = f'variant{variant_index}'
            scene_id = f'{task}__{variant}'
            old = write(f'old/{scene_id}.json', dict(layout, obstacle=dict(id=variant)))
            row = dict(scene_id=scene_id, task_id=task, variant_id=variant,
                       config_path=str(old.relative_to(tmp_path)), config_sha256=publish_module.digest(old),
                       risk_verified=len(rows) < 21)
            rows.append(row)
            if task_index >= 12:
                new = write(f'new/{scene_id}.json', dict(new_base, obstacle=dict(id=variant)))
                evidence = write(f'evidence/{scene_id}.json', dict(
                    semantic_geometry_passed=True, config_sha256=publish_module.digest(new)))
                overlays.append(dict(row, config_path=str(new.relative_to(tmp_path)),
                    config_sha256=publish_module.digest(new), semantic_geometry_passed=True,
                    evidence_path=str(evidence.relative_to(tmp_path)), evidence_sha256=publish_module.digest(evidence)))
        if task_index >= 12:
            base = write(f'bases/{task}.json', new_base)
            reports = [write(f'evidence/{task}_{kind}.json', dict(passed=True)) for kind in ('relations', 'geometry')]
            certificates.append(write(f'certificates/{task}.json', dict(task_id=task, passed=True,
                relations_passed=True, source_geometry_passed=True,
                repaired_layout_path=str(base.relative_to(tmp_path)), repaired_layout_sha256=publish_module.digest(base),
                source_reports=[dict(path=str(p.relative_to(tmp_path)), sha256=publish_module.digest(p)) for p in reports])))
    old_manifest = write('old/manifest.json', dict(scenes=rows))
    overlay = write('new/manifest.json', dict(scenes=overlays))
    for name in ('preview_reuse', 'render_reuse'):
        write(f'previous_reports/{name}.json', dict(entries=[]))
    write('previous_reports/review_previous.json', dict(entries=[dict(scene_id=r['scene_id'],
        config_sha256=r['config_sha256'], visual_status='candidate', reviewed_views=[]) for r in rows]))
    return old_manifest, overlay, certificates


def test_two_task_repairs_preserve_unchanged_reviews_and_risk(tmp_path, monkeypatch):
    manifest, overlay, certificates = fixture_data(tmp_path, monkeypatch)
    result = publish_module.publish(manifest, [overlay], certificates, [], tmp_path/'published',
                                    tmp_path/'reports', tmp_path/'previous_reports')
    assert result == dict(configured=154, changed=22, source_task_corrected=22, render_required=22)
    published = json.loads((tmp_path/'published/manifest.json').read_text())['scenes']
    assert sum(r.get('source_layout_repaired', False) for r in published) == 22
    assert sum(r.get('risk_verified', False) for r in published) == 21
    reviews = json.loads((tmp_path/'reports/review_previous.json').read_text())['entries']
    assert len(reviews) == 132
    assert not {r['scene_id'] for r in reviews} & {r['scene_id'] for r in published if r.get('source_layout_repaired')}


def test_missing_task_certificate_does_not_publish_partial_output(tmp_path, monkeypatch):
    manifest, overlay, certificates = fixture_data(tmp_path, monkeypatch)
    with pytest.raises(ValueError, match='Uncertified source-task modification'):
        publish_module.publish(manifest, [overlay], certificates[:1], [], tmp_path/'published',
                               tmp_path/'reports', tmp_path/'previous_reports')
    assert not (tmp_path/'published').exists()
    assert not (tmp_path/'reports').exists()
