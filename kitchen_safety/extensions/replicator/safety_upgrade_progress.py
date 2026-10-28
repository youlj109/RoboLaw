"""Aggregate independent safety-stage evidence without promoting scene loading.

This module uses only the standard library, so the dashboard and supervisor can
run outside Isaac Sim. Per-task workers never write the shared progress file.
"""
from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
REPORT = Path('reports/replicator_safety_upgrade')
FLAGS = ('geometry_passed', 'risk_verified', 'native_stability_verified',
         'safe_execution_verified', 'video_saved')
STATES = {'pending', 'waiting_baseline', 'screening', 'geometry_passed',
          'risk_verified', 'executing', 'accepted', 'needs_repair'}


def stamp():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n')
    temporary.replace(path)


def aggregate(root=ROOT, *, active_work=(), stage='risk_screening'):
    root = Path(root)
    original = json.loads((root/'reports/replicator_scene_build/progress.json').read_text())
    rows = {}
    errors = []
    for old in original['entries']:
        row = {key: old[key] for key in ('scene_id', 'task_id', 'variant_id',
               'category', 'config_path', 'preview_path') if key in old}
        row.update({name: False for name in FLAGS})
        row.update(status='waiting_baseline', baseline_status='waiting_baseline',
                   reason='等待同任务真实成功基线及新摆位检查。')
        if row['scene_id'] in rows:
            raise ValueError('Duplicate original scene: ' + row['scene_id'])
        rows[row['scene_id']] = row
    for path in sorted((root/REPORT/'tasks').glob('*/result.json')):
        try:
            report = json.loads(path.read_text())
            seen = set()
            for update in report['entries']:
                update = dict(update)
                update['status'] = {'pending_geometry': 'pending', 'screening_geometry': 'screening',
                                    'needs_placement': 'needs_repair'}.get(update.get('status'), update.get('status'))
                key = update['scene_id']
                if key in seen or key not in rows:
                    raise ValueError('Unknown/duplicate scene ID: ' + key)
                seen.add(key)
                old = rows[key]
                if any(update.get(name) != old[name] for name in
                       ('task_id', 'variant_id', 'category')):
                    raise ValueError('Canonical scene identity changed: ' + key)
                if update.get('status') not in STATES:
                    raise ValueError('Unknown safety-stage status: ' + key)
                new = dict(old, **update)
                if new.get('config_path') != old.get('config_path') and 'preview_path' not in update:
                    # A picture of the old pose must not illustrate a new pose.
                    new['previous_preview_path'] = old.get('preview_path')
                    new['preview_path'] = None
                for name in FLAGS:
                    new[name] = update.get(name) is True
                if new['baseline_status'] not in ('verified', 'accepted'):
                    new['risk_verified'] = False
                    new['safe_execution_verified'] = False
                if new['safe_execution_verified'] and not all(new[name] for name in
                        ('geometry_passed', 'risk_verified', 'native_stability_verified')):
                    new['safe_execution_verified'] = False
                    new.update(status='needs_repair', reason='安全执行缺少配套几何、风险或原生稳定证据。')
                if new['status'] == 'accepted' and not all(new[name] for name in FLAGS):
                    new.update(status='needs_repair', reason='完整验收标记缺少逐项证据。')
                new['task_report_path'] = str(path.relative_to(root))
                rows[key] = new
        except (ValueError, KeyError, TypeError, OSError) as error:
            errors.append(dict(path=str(path.relative_to(root)), error=str(error)))
    # New renders have an independent writer. They add presentation evidence
    # only, and can never promote a geometry or safety flag.
    import hashlib
    for path in sorted((root/REPORT).glob('render_progress*.json')):
        try:
            for view in json.loads(path.read_text()).get('entries', []):
                row = rows.get(view.get('scene_id'))
                if not row or view.get('status') != 'ready' or view.get('config_path') != row.get('config_path'):
                    continue
                evidence_path = root/view['evidence_path']
                evidence = json.loads(evidence_path.read_text())
                config_path = root/row['config_path']
                preview_path = root/view['preview_path']
                if (not preview_path.is_file() or evidence.get('layout_sha256') !=
                        hashlib.sha256(config_path.read_bytes()).hexdigest() or
                        evidence.get('preview_sha256') != hashlib.sha256(preview_path.read_bytes()).hexdigest()):
                    continue
                row.update(preview_path=view['preview_path'], scene_path=view.get('scene_path'),
                           render_evidence_path=view['evidence_path'])
        except (ValueError, KeyError, TypeError, OSError) as error:
            errors.append(dict(path=str(path.relative_to(root)), error=str(error)))
    entries = list(rows.values())
    data = dict(schema_version=1, total=len(entries), stage=stage,
                updated_at=stamp(), entries=entries, active_work=list(active_work),
                counts=dict(Counter(row['status'] for row in entries)),
                checks={name: sum(row[name] for row in entries) for name in FLAGS},
                aggregation_errors=errors,
                scope='原场景加载、单条真实基线离线风险、原生稳定、安全执行和视频分别统计；单次通过不代表多种子稳健性。')
    atomic_json(root/REPORT/'progress.json', data)
    return data
