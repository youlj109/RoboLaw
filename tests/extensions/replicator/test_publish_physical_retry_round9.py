"""Retained-layout round9 keeps original151/current152 and all eight retry layers."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.extensions.replicator import publish_physical_retry_round9 as pub

ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def fixture(monkeypatch,tmp_path):
    spec=importlib.util.spec_from_file_location('r8_publication_fixture',ROOT/'tests/extensions/replicator/test_publish_physical_retry_round8.py');r8=importlib.util.module_from_spec(spec);spec.loader.exec_module(r8)
    historical=r8.fixture.__wrapped__(monkeypatch,tmp_path);previous=r8.pub.build();monkeypatch.setattr(pub,'ROOT',tmp_path)
    def write(path,value):
        path=pub.local(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False));return pub.ref(path)
    # A source-bound round8 actual failure plus a zero-episode planning failure.
    oldrow=next(r for r in historical.manifest['entries'] if r['executed'])
    raw=pub.read(oldrow['execution_result_path']);raw.update(execution_status='failed',task_success=False)
    old_result=write(oldrow['execution_result_path'],raw);oldrow.update(execution_status='failed',task_success=False,execution_result_sha256=old_result['sha256'])
    prior=write(r8.pub.MANIFEST,historical.manifest);previous['source_retry_manifest']=prior
    oldentry=next(e for e in previous['entries'] if e['scene_id']==oldrow['scene_id'])
    oldentry.update(current_revision_success=False,current_revision_status='failed',current_revision_result=old_result)
    oldentry['retry_round8'].update(execution_status='failed',task_success=False,result=old_result,diagnostic_visual_review=dict(status='complete',scene_id=oldentry['scene_id'],execution_result_sha256=old_result['sha256']))
    previous['current_revision_counts'].update(success=152,failed=2)
    previous['counts'].update(current_revision_success=152,current_revision_failed=2,new_successes=0)
    for task in previous['tasks']:
        group=[e for e in previous['entries'] if e['scene_id'] in task['scene_ids']]
        task.update(round8_success=0,current_revision_success=sum(e['current_revision_success'] for e in group),current_revision_remaining_failed=sum(not e['current_revision_success'] for e in group))
    write(pub.PREVIOUS,previous)
    evidence=write('r9/cpu/evidence.json',{'synthetic':True});source=write('r9/code/source.py',{'source':'synthetic audit implementation bytes'})
    plan=dict(retry_round=9,layout_changed=False,layout_changed_from_original=True,success_predicate_changed=False,source_previous_retry_manifest=prior,source_first_pass_manifest=prior,source_camera_retake_manifest=prior,execution_settings={},entries=[])
    manifest=dict(kind='physical_failure_retry',retry_round=9,stage='complete',rendering_enabled=False,source_previous_retry_manifest=prior,entries=[])
    audit=dict(kind='physical_retry_round9_independent_audit',passed=True,errors=[],all2_audited=True,audited_scenes=2,require_complete=True,selected_scene=None,entries=[],**{k:source for k in ('auditor_source','numerical_auditor_source','scene_revision_checker_source','previous_revision_checker_source','composed_usd_auditor_source','retained_scene_revision_checker_source','retained_revision_auditor_source')})
    diagnostics=dict(kind='physical_retry_planning_failure_cpu_diagnostics',retry_round=9,total=1,diagnosed=1,complete=True,source_plan_unchanged=True,errors=0,physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=0,layout_changed=False,layout_changed_from_original=True,retained_scene_revision=True,diagnostic_sources={},method='Retained failed solver record',limitations=['No new IK or physics'],entries=[])
    prior_rows={r['scene_id']:r for r in historical.manifest['entries']}
    failed=[e for e in previous['entries'] if not e['current_revision_success']];assert len(failed)==2
    for old in failed:
        sid=old['scene_id'];original=prior_rows[sid];revision=old['current_revision'];cfg=old['current_revision_config'];stage=old['retry_round8']['source_scene'];composition=pub.read(revision['path'])['new_composition']
        identity={k:original[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256','scene_revision','layout_changed')};identity.update(retained_scene_revision=True,scene_settings_changed_this_round=False,scene_settings_changed_once=True)
        generated=not original['executed'];status='MotionGenStatus.FINETUNE_TRAJOPT_FAIL';segment=dict(phase='lift',segment_index=0,solver_status=status,solver_success=False,samples=0,status='failed')
        policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='仅改变轨迹，保留同一布局。')
        recipe=dict(retry_round=9,retained_scene_revision=True,scene_settings_changed_this_round=False,planning_path_policy=policy,segment_deviation_limits_m={})
        candidate=dict(identity,status='generated' if generated else 'failed',attempts=1,executed=False,planning_only=True,composition=composition,recipe=recipe,failure_reason=None if generated else status,artifacts={},failed_phase=None if generated else 'lift',phases=[] if generated else [dict(segment,segments=[deepcopy(segment)])])
        planning=write('r9/planning/'+sid+'.json',candidate);row=dict(identity,result_path=original['result_path'],result_sha256=original['result_sha256'],source_scene_path=stage['path'],source_scene_sha256=stage['sha256'],candidate_planning_result=planning,previous_retry_result=old.get('current_revision_result') or old['latest_physical_result'],physical_attempt_number=9,planning_status='generated' if generated else 'failed',trajectory_path=None,trajectory_sha256=None,failure_reason=None if generated else status,adjustment=dict(description='Retained-layout trajectory refinement',evidence_path=evidence['path'],evidence_sha256=evidence['sha256']))
        if generated:
            trajectory=write('r9/trajectory/'+sid+'.npz',{'fixture':'trajectory'});row.update(trajectory_path=trajectory['path'],trajectory_sha256=trajectory['sha256'])
        plan['entries'].append(deepcopy(row));row.update(execution_status='success' if generated else 'planning_failed',attempts=int(generated),executed=generated,task_success=generated)
        checked={k:row[k] for k in ('scene_id','planning_status','execution_status','attempts','executed','task_success','candidate_planning_result')};checked.update(status='passed',scene_revision_audit=dict(scene_revision=revision,layout_changed=True,retained_scene_revision=True,scene_settings_changed_this_round=False,retained_from_manifest=prior))
        if generated:
            raw=dict(row,kind='physical_failure_retry_execution',planning_result_path=planning['path'],planning_result_sha256=planning['sha256'],original_planning_result=dict(path=original['result_path'],sha256=original['result_sha256']),source_previous_retry_manifest=prior,prior_physical_attempts=[])
            result=write(Path('exports')/pub.RUN/'executions'/sid/'result.json',raw);row.update(execution_result_path=result['path'],execution_result_sha256=result['sha256']);checked.update(result=result,no_render_gate_passed=True)
        else:
            diagnostics['entries'].append(dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],scene_revision=revision,candidate_planning_result=planning,config=cfg,source_scene=stage,status='diagnosed',failed_phase='lift',solver_status=status,root_cause_confirmed=False,scene_infeasibility_established=False,observed_cpu_facts_zh=['失败段没有返回路径；物理未执行。'],interpretation_zh='单次求解失败不证明无解。',next_adjustment_zh='下一轮保留硬FK门槛并检查软约束求解参数。',diagnostic_method='solver_failure_without_failed_segment_path',failed_segment_trajectory_available=False,failed_solver_record=dict(phase='lift',segment_index=0,solver_status=status,solver_success=False,samples=0,trajectory_saved=False)))
        manifest['entries'].append(row);audit['entries'].append(checked)
    plan_ref=write('r9/plan.json',plan);manifest.update(retry_plan_path=plan_ref['path'],retry_plan_sha256=plan_ref['sha256']);diagnostics['source_plan']=plan_ref
    def flush():
        audit['manifest_snapshot']=write(pub.MANIFEST,manifest);write(pub.AUDIT,audit);write(pub.PLANNING_DIAGNOSTICS,diagnostics)
    flush();return SimpleNamespace(previous=previous,manifest=manifest,plan=plan,audit=audit,diagnostics=diagnostics,write=write,flush=flush)


def test_retains_layout_history_original151_current152_plus_one(fixture):
    data=pub.build();assert len(data['entries'])==154 and len(data['tasks'])==14
    assert data['original_layout_counts']['success']==151 and data['current_revision_counts']['success']==153
    assert data['current_revision_counts']['revised_scenes_this_round']==0
    assert data['counts']['physical_attempts']==1 and data['counts']['planning_failed']==1
    for old,new in zip(fixture.previous['entries'],data['entries']):
        for key in old:
            if key.startswith('current_revision') and new['retry_round9']:continue
            assert new[key]==old[key]
        retry=new['retry_round9']
        if retry and retry['execution_status']=='planning_failed':
            assert retry['result'] is None and retry['attempts']==0 and retry['executed'] is False
            assert new['current_revision_result']==old['current_revision_result']
            assert new['current_revision_result_retry_round']==8
            assert new['current_revision_result_scope']=='latest_physical_attempt_on_retained_layout'
            assert retry['diagnostic_visual_review']['prior_review']==old['retry_round8']['diagnostic_visual_review']
        elif retry:
            assert new['current_revision_result_retry_round']==9
        if retry:assert retry['changed_fields']==[] and retry['previous_config']==retry['config']
    assert data['layout_changed'] is False and data['layout_changed_from_original'] is True
    text=pub.document(data);assert '第九轮保留布局' in text and '当前采用版本153/154' in text
    assert 'retry_round9.result 为null' in text and '新版七✓' in text and '九✓' in text
    assert data['markdown_report_path']=='docs/extensions/replicator/PHYSICAL_RETRY_ROUND9_20261029.md'


@pytest.mark.parametrize('key',['retained_scene_revision_checker_source','retained_revision_auditor_source'])
def test_retained_auditor_sources_are_hash_bound(fixture,key):
    fixture.audit[key]=dict(path='stale.py',sha256='0'*64);fixture.flush()
    with pytest.raises((ValueError,FileNotFoundError)):pub.build()


def test_same_revision_cannot_change_sources_or_layout_scope(fixture):
    row=fixture.manifest['entries'][0];row['scene_settings_changed_this_round']=True;fixture.flush()
    with pytest.raises(ValueError,match='Retained layout scope differs'):pub.build()


def test_prior_actual_must_be_latest_on_adopted_layout_not_original(fixture):
    row=next(r for r in fixture.plan['entries'] if r['planning_status']=='failed')
    old=next(e for e in fixture.previous['entries'] if e['scene_id']==row['scene_id'])
    row['previous_retry_result']=old['latest_physical_result']
    actual=next(r for r in fixture.manifest['entries'] if r['scene_id']==row['scene_id']);actual['previous_retry_result']=row['previous_retry_result']
    ref=fixture.write('r9/plan.json',fixture.plan);fixture.manifest['retry_plan_sha256']=ref['sha256'];fixture.diagnostics['source_plan']=ref;fixture.flush()
    with pytest.raises(ValueError,match='prior actual result differs'):pub.build()


def test_zero_episode_no_visual_and_solver_failure_does_not_fabricate_path(fixture):
    data=pub.build();assert data['diagnostic_visual_review']['review_required'] is False
    assert '本轮没有实际失败，因此未生成新失败诊断图' in pub.document(data)
    fixture.diagnostics['entries'][0]['failed_solver_record']['trajectory_saved']=True;fixture.flush()
    with pytest.raises(ValueError,match='fabricate a path'):pub.build()


def test_unknown_diagnostic_method_is_rejected(fixture):
    fixture.diagnostics['entries'][0]['diagnostic_method']='assumed_ik_failure';fixture.flush()
    with pytest.raises(ValueError,match='Unknown planning diagnostic method'):pub.build()
