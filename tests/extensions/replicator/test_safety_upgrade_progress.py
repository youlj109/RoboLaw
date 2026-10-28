import json
from pathlib import Path

from kitchen_safety.extensions.replicator.safety_upgrade_progress import aggregate, FLAGS


def setup_tree(root):
    path = root/'reports/replicator_scene_build/progress.json'
    path.parent.mkdir(parents=True)
    row = dict(scene_id='task__hazard', task_id='task', variant_id='hazard', category='human',
               status='ready', checks=dict(simulation_loaded=True))
    path.write_text(json.dumps(dict(entries=[row])))
    return row


def task_report(root, row):
    path = root/'reports/replicator_safety_upgrade/tasks/task/result.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(dict(entries=[row])))


def test_loaded_scene_cannot_become_verified_safety_scene(tmp_path):
    setup_tree(tmp_path)
    result = aggregate(tmp_path)
    assert result['total'] == 1
    assert result['counts'] == {'waiting_baseline': 1}
    assert not any(result['checks'].values())


def test_missing_successful_baseline_rejects_risk_and_safe_flags(tmp_path):
    row = setup_tree(tmp_path)
    row.update({name: True for name in FLAGS})
    row.update(status='accepted', baseline_status='waiting_baseline')
    task_report(tmp_path, row)
    result = aggregate(tmp_path)
    assert result['checks']['risk_verified'] == 0
    assert result['checks']['safe_execution_verified'] == 0
    assert result['counts'] == {'needs_repair': 1}


def test_missing_native_stability_cannot_be_accepted(tmp_path):
    row = setup_tree(tmp_path)
    row.update({name: True for name in FLAGS})
    row.update(status='accepted', baseline_status='verified', native_stability_verified=False)
    task_report(tmp_path, row)
    result = aggregate(tmp_path)
    assert result['checks']['safe_execution_verified'] == 0
    assert result['counts'] == {'needs_repair': 1}


def test_mismatched_asset_identity_does_not_replace_original(tmp_path):
    row = setup_tree(tmp_path)
    row.update(status='risk_verified', baseline_status='verified', risk_verified=True,
               variant_id='different_asset')
    task_report(tmp_path, row)
    result = aggregate(tmp_path)
    assert len(result['aggregation_errors']) == 1
    assert result['checks']['risk_verified'] == 0
    assert result['entries'][0]['variant_id'] == 'hazard'
