"""Aggregation tests only; fixtures do not represent physical experiments."""
import importlib.util
import json
from pathlib import Path

import pytest

SOURCE=Path(__file__).resolve().parents[3]/'scripts/extensions/controls/summarize_reviewed_completion.py'
spec=importlib.util.spec_from_file_location('summarize_reviewed_completion',SOURCE)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def case(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'ROOT',tmp_path)
    task_source=tmp_path/'task.yaml'
    task_source.write_text('original task')
    monkeypatch.setattr(module,'task_contract',lambda task:dict(task_id='left-task',source=task_source))
    source=tmp_path/'source.json'
    config=dict(task_id='left-task',source_sha256=module.sha(task_source),robot_pose_xyzw=[0.]*7,
                variants={'human':{'position':[1.,2.,3.]}},prepress_planning={'method':'curobo_joint_goal','ik_branch_seed':[0.]*7})
    source.write_text(json.dumps(config))

    def write(path,data):
        path.write_text(json.dumps(data))

    for name in ('episode','base'):
        directory=tmp_path/name
        directory.mkdir()
        write(directory/'config.json',config)
        write(directory/'result.json',dict(task_id='left-task',source_sha256=module.sha(task_source),
            config_sha256=module.sha(directory/'config.json'),success=True,accepted=True,
            variant='base' if name=='base' else 'human',
            physical_object_setters_after_reset=False,initial_robot_pose=[0.]*7))
        write(directory/'records.json',[])
        write(directory/'contacts.json',[])
        write(directory/'audit.json',{'passed':True,'physics_updates':[{'valid':True}]})
        for filename in ('mechanism_audit.json','fixed_base_hazard_audit.json',
                         'self_collision_detailed_audit.json','independent_validation.json'):
            write(directory/filename,{'passed':True})
        write(directory/'validation.json',{'passed':True,'record_count':1})
        write(directory/'contact_causality_review.json',{'passed':True,'input_sha256':{
            key:module.sha(directory/(key+'.json')) for key in ('result','records','contacts','audit')}})
        for camera in module.CAMERAS:
            (directory/(camera+'.mp4')).write_bytes(b'fixture only')
    return dict(scene='01_left_cancel_human',task='left',variant='human',episode='episode',baseline='base',
                source_config='source.json',source_sha256=module.sha(source))


def test_all_recording_gates_are_required(case):
    report=module.review(case)
    assert report['physical_task_accepted']
    assert not report['failed_gates']
    (module.ROOT/'base/self_collision_detailed_audit.json').unlink()
    assert not module.review(case)['gates']['baseline_self_collision']


def test_layout_changes_are_not_accepted_from_old_pass_flags(case):
    path=module.ROOT/'episode/config.json'
    config=json.loads(path.read_text())
    config['variants']['human']['position'][0]=9.
    path.write_text(json.dumps(config))
    report=module.review(case)
    assert not report['physical_task_accepted']
    assert not report['gates']['scene_and_task_frozen']


def test_baseline_requires_same_actual_controller(case):
    path=module.ROOT/'base/config.json'
    config=json.loads(path.read_text())
    config['prepress_planning']['ik_branch_seed'][0]=1.
    path.write_text(json.dumps(config))
    assert not module.review(case)['gates']['matched_physical_baseline']


def test_stale_causal_review_is_rejected(case):
    (module.ROOT/'episode/records.json').write_text('[{"changed":true}]')
    assert not module.review(case)['gates']['contact_review_bound_to_current_inputs']


def test_wrong_scene_role_is_not_coverage(case):
    case['scene']='03_left_cancel_obstacle'
    assert not module.review(case)['gates']['expected_scene_and_role']


def test_exact_nine_scene_coverage_is_required():
    wrong=[dict(scene='other'+str(i),physical_task_accepted=True) for i in range(9)]
    assert not module.complete_coverage(wrong)
    correct=[dict(scene=sid,physical_task_accepted=True) for sid in module.EXPECTED_SCENES]
    assert module.complete_coverage(correct)
    correct[-1]['physical_task_accepted']=False
    assert not module.complete_coverage(correct)
