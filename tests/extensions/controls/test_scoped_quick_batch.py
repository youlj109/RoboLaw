"""Frozen-layout reference and scene-selection contracts; not planning evidence."""
import importlib.util
import json
from pathlib import Path
import pytest

path=Path(__file__).resolve().parents[3]/'scripts/extensions/controls/quick_plan_batch.py'
spec=importlib.util.spec_from_file_location('scoped_quick_batch',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_explicit_subset_retains_order_and_does_not_rerun_other_scenes():
    rows=[{'id':'a'},{'id':'b'},{'id':'c'}]
    assert module.select_scenes(rows,['c','a'])==[rows[0],rows[2]]


def test_unknown_scene_cannot_silently_skip_work():
    with pytest.raises(ValueError,match='Unknown scenes'):
        module.select_scenes([{'id':'a'}],['typo'])


def test_failed_planning_hints_do_not_suggest_moving_frozen_hazards():
    assert '不移动实体' in module.failure_hint('START_STATE_IN_COLLISION')
    assert '不移动灯' in module.scene_hint('08_oven_temperature_property')


@pytest.mark.parametrize('move_layout',[False,True])
def test_controller_reference_cannot_move_frozen_entities(tmp_path,monkeypatch,move_layout):
    monkeypatch.setattr(module,'ROOT',tmp_path)
    source={'task_id':'right','variants':{'robot':{'position':[1.,2.,3.]}}}
    candidate=json.loads(json.dumps(source));candidate['prepress_planning']={'ik_branch_seed':[0.]*7}
    if move_layout:candidate['variants']['robot']['position'][0]=9.
    (tmp_path/'source.json').write_text(json.dumps(source));(tmp_path/'candidate.json').write_text(json.dumps(candidate))
    row={'id':'scene','visual_config':'source.json','task':'right_cancel'}
    refs={'scene':{'config':'candidate.json','baseline':'baseline','stage':'stage'}}
    if move_layout:
        with pytest.raises(ValueError,match='frozen'):module.planning_inputs(row,refs)
    else:assert module.planning_inputs(row,refs)==('candidate.json','baseline','stage')
