import json
from scripts.extensions.controls.summarize_delivery import task_evidence


def test_missing_task_report_is_not_completion(tmp_path):
    report=task_evidence('temperature',tmp_path/'missing.json')
    assert not report['accepted']
    assert not report['gates']['four_unique_physical_roles']


def test_accepted_flag_alone_is_not_completion(tmp_path):
    path=tmp_path/'report.json'
    path.write_text(json.dumps({'accepted':True,'episodes':[]}))
    report=task_evidence('left',path)
    assert not report['accepted']
    assert report['gates']['accepted_independent_report']
    assert not report['gates']['correct_task']
