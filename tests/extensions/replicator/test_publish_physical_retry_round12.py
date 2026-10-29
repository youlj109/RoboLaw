"""One-scene round12 cannot erase or rerun the native mustard success."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.extensions.replicator import publish_physical_retry_round12 as pub
ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def fixture(monkeypatch,tmp_path):
    spec=importlib.util.spec_from_file_location('r11_publication_fixture',ROOT/'tests/extensions/replicator/test_publish_physical_retry_round11.py');r11=importlib.util.module_from_spec(spec);spec.loader.exec_module(r11)
    f=r11.fixture.__wrapped__(monkeypatch,tmp_path);write=f.write
    successful=next(e for e in f.manifest['entries'] if e['executed']);raw=r11.pub.read(successful['execution_result_path']);raw['status']='complete'
    result=write(successful['execution_result_path'],raw);successful['execution_result_sha256']=result['sha256']
    next(e for e in f.audit['entries'] if e['scene_id']==successful['scene_id'])['result']=result;f.flush()
    previous=r11.pub.build();assert previous['current_revision_counts']['success']==153
    monkeypatch.setattr(pub,'ROOT',tmp_path);write(pub.PREVIOUS,previous)
    prior=previous['source_retry_manifest'];source=f.audit['auditor_source'];evidence=write('r12/preparation.json',dict(new_lower_waypoint=True))
    plan=dict(retry_round=12,layout_changed=False,layout_changed_from_original=True,success_predicate_changed=False,source_previous_retry_manifest=prior,source_first_pass_manifest=prior,source_camera_retake_manifest=prior,execution_settings={},entries=[])
    manifest=dict(kind='physical_failure_retry',retry_round=12,stage='complete',rendering_enabled=False,source_previous_retry_manifest=prior,entries=[])
    audit=deepcopy(f.audit);audit.update(kind='physical_retry_round12_independent_audit',all1_audited=True,audited_scenes=1,entries=[],intermediate_retention_checker_source=source,source_previous_retry_audit=previous['source_execution_audit']);audit.pop('all2_audited')
    diag=deepcopy(f.diagnostics);diag.update(retry_round=12,total=0,diagnosed=0,entries=[])
    oldrow=next(e for e in f.manifest['entries'] if not e['executed']);row=deepcopy(oldrow);sid=row['scene_id'];assert sid=='u_shape_bowl_sink__human_default_mpfb'
    old=next(e for e in previous['entries'] if e['scene_id']==sid)
    for key in ('execution_status','executed','attempts','task_success'):row.pop(key)
    row.update(physical_attempt_number=12,planning_status='generated',previous_retry_result=old.get('current_revision_result') or old['latest_physical_result'],adjustment=dict(description='Explicit lower carry waypoint',evidence_path=evidence['path'],evidence_sha256=evidence['sha256']))
    pose=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    recipe=dict(retry_round=12,scene_id=sid,payload_name='bowl',retained_scene_revision=True,scene_settings_changed_this_round=False,
        joint_space_target_q_by_phase={'lower':[[0.]*7,[0.]*7],'retreat':[[0.]*7]},phase_waypoints_world={'lower':[pose],'retreat':[]},
        planning_options_by_phase={'lower':{'enable_graph':True},'retreat':{'enable_graph':True}},hold_orientation_phases=[],linear_motion_phases=[],segment_deviation_limits_m={},
        planning_path_policy=dict(kind='collision_checked_graph_joint_goals_with_carry_waypoints',previous_straight_line_gate_removed=True,reason_zh='Add only an explicit carry waypoint.'))
    candidate=pub.read(row['candidate_planning_result']['path']);candidate.update(recipe=recipe,status='generated',failure_reason=None)
    planning=write('r12/planning/'+sid+'.json',candidate);row['candidate_planning_result']=planning
    trajectory=write('r12/trajectory/'+sid+'.npz',{'fixture':'trajectory'});row.update(trajectory_path=trajectory['path'],trajectory_sha256=trajectory['sha256'],failure_reason=None)
    plan['entries'].append(deepcopy(row));row.update(execution_status='success',attempts=1,executed=True,task_success=True)
    checked={k:row[k] for k in ('scene_id','planning_status','execution_status','attempts','executed','task_success','candidate_planning_result')};checked.update(status='passed',scene_revision_audit=dict(scene_revision=row['scene_revision'],layout_changed=True,retained_scene_revision=True,scene_settings_changed_this_round=False,retained_from_manifest=prior))
    raw=dict(row,kind='physical_failure_retry_execution',status='complete',planning_result_path=planning['path'],planning_result_sha256=planning['sha256'],original_planning_result=dict(path=row['result_path'],sha256=row['result_sha256']),source_previous_retry_manifest=prior,prior_physical_attempts=[])
    result=write(Path('exports')/pub.RUN/'executions'/sid/'result.json',raw);row.update(execution_result_path=result['path'],execution_result_sha256=result['sha256']);checked.update(result=result,no_render_gate_passed=True)
    manifest['entries'].append(row);audit['entries'].append(checked)
    planref=write('r12/plan.json',plan);manifest.update(retry_plan_path=planref['path'],retry_plan_sha256=planref['sha256']);diag['source_plan']=planref
    def flush():
        audit['manifest_snapshot']=write(pub.MANIFEST,manifest);write(pub.AUDIT,audit);write(pub.PLANNING_DIAGNOSTICS,diag)
    flush();return SimpleNamespace(previous=previous,plan=plan,manifest=manifest,audit=audit,diagnostics=diag,write=write,flush=flush)


def test_single_remaining_success_adds_one_and_retains_previous_mustard(fixture):
    data=pub.build();assert len(data['entries'])==154
    assert data['counts']['previous_success']==153 and data['counts']['current_revision_success']==154
    assert data['counts']['latest_success']==151 and data['counts']['retry_cohort']==1 and data['counts']['physical_attempts']==1
    target=fixture.manifest['entries'][0]['scene_id']
    for old,new in zip(fixture.previous['entries'],data['entries']):
        for key in old:
            if key.startswith('current_revision') and new['scene_id']==target:continue
            assert new[key]==old[key]
        if old['current_revision_status']=='retry_round11_passed':
            assert new['current_revision_status']=='retry_round11_passed' and new['retry_round12'] is None
    assert data['source_previous_retry_audit']==fixture.previous['source_execution_audit']
    text=pub.document(data);assert '第十二轮' in text and '当前采用版本154/154' in text and '通过1/1' in text


def test_ancestral_helper10_and_prior_success_audit_are_bound(fixture):
    fixture.audit['intermediate_retention_checker_source']=dict(path='missing.py',sha256='0'*64);fixture.flush()
    with pytest.raises((ValueError,FileNotFoundError)):pub.build()
    fixture.audit['intermediate_retention_checker_source']=fixture.audit['auditor_source']
    fixture.audit['source_previous_retry_audit']=dict(path='missing-audit.json',sha256='0'*64);fixture.flush()
    with pytest.raises((ValueError,FileNotFoundError)):pub.build()


def test_second_scene_cannot_enter_single_scene_cohort(fixture):
    extra=deepcopy(fixture.manifest['entries'][0]);extra['scene_id']='previously_successful';fixture.manifest['entries'].append(extra);fixture.flush()
    with pytest.raises(ValueError,match='exactly 1 terminal'):pub.build()


def test_single_planning_failure_has_zero_episode_and_keeps_current153(fixture):
    row=fixture.manifest['entries'][0];candidate=pub.read(row['candidate_planning_result']['path'])
    segment=dict(phase='lower',segment_index=0,solver_status='TRAJOPT_FAIL',solver_success=False,samples=0,status='failed')
    candidate.update(status='failed',failed_phase='lower',failure_reason='TRAJOPT_FAIL',phases=[dict(segment,segments=[segment])])
    reference=fixture.write(row['candidate_planning_result']['path'],candidate)
    for target in (row,fixture.plan['entries'][0]):
        target.update(planning_status='failed',trajectory_path=None,trajectory_sha256=None,failure_reason='TRAJOPT_FAIL',candidate_planning_result=reference)
    row.update(execution_status='planning_failed',executed=False,attempts=0,task_success=False)
    row.pop('execution_result_path');row.pop('execution_result_sha256')
    audit=fixture.audit['entries'][0]
    for key in ('planning_status','execution_status','executed','attempts','task_success','candidate_planning_result'):audit[key]=row[key]
    audit.pop('result');audit.pop('no_render_gate_passed')
    diagnostic=dict(scene_id=row['scene_id'],task_id=row['task_id'],variant_id=row['variant_id'],scene_revision=row['scene_revision'],candidate_planning_result=reference,
        config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        status='diagnosed',failed_phase='lower',solver_status='TRAJOPT_FAIL',root_cause_confirmed=False,scene_infeasibility_established=False,
        observed_cpu_facts_zh=['失败段没有返回轨迹，未执行物理。'],interpretation_zh='未确认场景不可行。',next_adjustment_zh='检查失败求解的记录约束。',
        diagnostic_method='solver_failure_without_failed_segment_path',failed_segment_trajectory_available=False,
        failed_solver_record=dict(phase='lower',segment_index=0,solver_status='TRAJOPT_FAIL',solver_success=False,samples=0,trajectory_saved=False))
    planref=fixture.write('r12/plan.json',fixture.plan);fixture.manifest['retry_plan_sha256']=planref['sha256']
    fixture.diagnostics.update(total=1,diagnosed=1,entries=[diagnostic],source_plan=planref);fixture.flush()
    data=pub.build();assert data['counts']['current_revision_success']==153 and data['counts']['physical_attempts']==0
    entry=next(e for e in data['entries'] if e['scene_id']==row['scene_id'])
    assert entry['retry_round12']['result'] is None and entry['retry_round12']['attempts']==0
