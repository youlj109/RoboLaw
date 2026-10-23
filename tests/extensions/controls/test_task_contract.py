import hashlib
import json

import pytest

from kitchen_safety.extensions.controls.task_contract import ROOT, task_contract


def test_assigned_tasks_have_distinct_original_source_bindings():
    bindings=[task_contract(key) for key in ('left','right','temperature')]
    assert len({b['task_id'] for b in bindings})==3
    for binding in bindings:
        text=binding['source'].read_text()
        assert binding['task_id'] in text
        assert binding['joint'] in text


def test_right_candidate_is_bound_to_measured_right_task():
    config=json.loads((ROOT/'configs/extensions/controls/right_button_trial.json').read_text())
    contract=task_contract('right')
    assert config['task_id']==contract['task_id']
    assert config['button_joint']==contract['joint']
    assert config['source_sha256']==hashlib.sha256(contract['source'].read_bytes()).hexdigest()
    assert config['button_center_world_m'][0]>2.14
    assert config['variants']=={}  # No untested hazard layout is advertised.


def test_unknown_task_is_not_silently_left():
    with pytest.raises(ValueError):
        task_contract('unknown')


def test_right_prepress_pose_is_not_left_target():
    from kitchen_safety.extensions.controls.planning import button_prepress_pose
    config=json.loads((ROOT/'configs/extensions/controls/right_button_trial.json').read_text())
    pose=button_prepress_pose(config)
    assert abs(pose[0,3]-2.1455796052410947)<1e-12
    assert abs(pose[2,3]-1.0271396496470036)<1e-12
    config.pop('button_center_world_m')
    with pytest.raises(ValueError):
        button_prepress_pose(config)
