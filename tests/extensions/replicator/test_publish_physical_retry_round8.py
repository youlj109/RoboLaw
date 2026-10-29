"""Round8 keeps original151 and prior current152, with a real revision7→8 chain."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.extensions.replicator import publish_physical_retry_round8 as pub

ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def fixture(monkeypatch,tmp_path):
    spec=importlib.util.spec_from_file_location('r7_publication_fixture',ROOT/'tests/extensions/replicator/test_publish_physical_retry_round7.py');r7=importlib.util.module_from_spec(spec);spec.loader.exec_module(r7)
    historical=r7.fixture.__wrapped__(monkeypatch,tmp_path);previous=r7.pub.build();monkeypatch.setattr(pub,'ROOT',tmp_path)
    def write(path,value):
        path=pub.local(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False));return pub.ref(path)
    write(pub.PREVIOUS,previous);prior=previous['source_retry_manifest'];prior_rows={r['scene_id']:r for r in historical.manifest['entries']}
    evidence=write('r8/cpu/evidence.json',{'synthetic':True});source=write('r8/code/source.py',{'source':'synthetic audit implementation bytes'})
    plan=dict(retry_round=8,layout_changed=True,success_predicate_changed=False,source_previous_retry_manifest=prior,source_first_pass_manifest=prior,source_camera_retake_manifest=prior,execution_settings={},entries=[])
    manifest=dict(kind='physical_failure_retry',retry_round=8,stage='complete',rendering_enabled=False,source_previous_retry_manifest=prior,entries=[])
    audit=dict(kind='physical_retry_round8_independent_audit',passed=True,errors=[],all2_audited=True,audited_scenes=2,require_complete=True,selected_scene=None,entries=[],auditor_source=source,numerical_auditor_source=source,scene_revision_checker_source=source,previous_revision_checker_source=source,composed_usd_auditor_source=source)
    diagnostics=dict(kind='physical_retry_planning_failure_cpu_diagnostics',retry_round=8,total=1,diagnosed=1,complete=True,source_plan_unchanged=True,errors=0,physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=0,layout_changed=True,diagnostic_sources={},method='Retained failed solver record',limitations=['No new IK or physics'],entries=[])
    failed=[e for e in previous['entries'] if not e['current_revision_success']];assert len(failed)==2
    for index,old in enumerate(failed):
        sid=old['scene_id'];original=prior_rows[sid];oldcfg=old['current_revision_config'];oldstage=old['retry_round7']['source_scene'];cfg=pub.read(oldcfg['path']);cfg['obstacle']['position'][1]+=.01;newcfg=write('r8/new/'+sid+'/layout.json',cfg);newstage=write('r8/new/'+sid+'/scene.usda',{'usd':'new round8 source'})
        composition=dict(source_paths_sha256={},layout_sha256=newcfg['sha256'],scene_sha256=newstage['sha256'])
        revision=write('r8/revision/'+sid+'.json',dict(kind='hazard_pose_revision',retry_round=8,scene_id=sid,prior_manifest=prior,previous_revision=old['current_revision'],prior_row_source={k:original[k] for k in pub.validate_scene_revision.__globals__['SOURCE_KEYS']},old_config=oldcfg,old_source_scene=oldstage,new_config=newcfg,new_source_scene=newstage,changed_fields=['obstacle.position'],new_composition=composition))
        identity=dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],config_path=newcfg['path'],config_sha256=newcfg['sha256'],scene_revision=revision,layout_changed=True)
        generated=index==0;status='MotionGenStatus.FINETUNE_TRAJOPT_FAIL';segment=dict(phase='lift',segment_index=0,solver_status=status,solver_success=False,samples=0,status='failed')
        policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='默认碰撞规划。')
        candidate=dict(identity,status='generated' if generated else 'failed',attempts=1,executed=False,planning_only=True,composition=composition,recipe=dict(planning_path_policy=policy,segment_deviation_limits_m={}),failure_reason=None if generated else status,artifacts={},failed_phase=None if generated else 'lift',phases=[] if generated else [dict(segment,segments=[deepcopy(segment)])])
        planning=write('r8/planning/'+sid+'.json',candidate);row=dict(identity,result_path=original['result_path'],result_sha256=original['result_sha256'],source_scene_path=newstage['path'],source_scene_sha256=newstage['sha256'],candidate_planning_result=planning,previous_retry_result=old['latest_physical_result'],physical_attempt_number=8,planning_status='generated' if generated else 'failed',trajectory_path=None,trajectory_sha256=None,failure_reason=None if generated else status,adjustment=dict(description='New short approach and hazard pose',evidence_path=evidence['path'],evidence_sha256=evidence['sha256']))
        if generated:
            trajectory=write('r8/trajectory/'+sid+'.npz',{'fixture':'trajectory'});row.update(trajectory_path=trajectory['path'],trajectory_sha256=trajectory['sha256'])
        plan['entries'].append(deepcopy(row));row.update(execution_status='success' if generated else 'planning_failed',attempts=int(generated),executed=generated,task_success=generated)
        checked={k:row[k] for k in ('scene_id','planning_status','execution_status','attempts','executed','task_success','candidate_planning_result')};checked.update(status='passed',scene_revision_audit=dict(scene_revision=revision,layout_changed=True))
        if generated:
            raw=dict(row,kind='physical_failure_retry_execution',planning_result_path=planning['path'],planning_result_sha256=planning['sha256'],prior_physical_attempts=[])
            result=write(Path('exports')/pub.RUN/'executions'/sid/'result.json',raw);row.update(execution_result_path=result['path'],execution_result_sha256=result['sha256']);checked.update(result=result,no_render_gate_passed=True)
        else:
            diagnostics['entries'].append(dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],scene_revision=revision,candidate_planning_result=planning,config=newcfg,source_scene=newstage,status='diagnosed',failed_phase='lift',solver_status=status,root_cause_confirmed=False,scene_infeasibility_established=False,observed_cpu_facts_zh=['失败段没有返回路径；物理未执行。'],interpretation_zh='单次求解失败不证明无解。',next_adjustment_zh='下一轮检查端点与源几何。',diagnostic_method='solver_failure_without_failed_segment_path',failed_segment_trajectory_available=False,failed_solver_record=dict(phase='lift',segment_index=0,solver_status=status,solver_success=False,samples=0,trajectory_saved=False)))
        manifest['entries'].append(row);audit['entries'].append(checked)
    plan_ref=write('r8/plan.json',plan);manifest.update(retry_plan_path=plan_ref['path'],retry_plan_sha256=plan_ref['sha256']);diagnostics['source_plan']=plan_ref
    def flush():
        audit['manifest_snapshot']=write(pub.MANIFEST,manifest);write(pub.AUDIT,audit);write(pub.PLANNING_DIAGNOSTICS,diagnostics)
    flush();return SimpleNamespace(previous=previous,manifest=manifest,plan=plan,audit=audit,diagnostics=diagnostics,write=write,flush=flush)


def test_preserves_round7_success_original151_current152_plus_one(fixture):
    data=pub.build();assert len(data['entries'])==154 and len(data['tasks'])==14
    assert data['original_layout_counts']['success']==data['counts']['latest_success']==151
    assert data['counts']['previous_success']==152 and data['current_revision_counts']['success']==153
    assert data['counts']['physical_attempts']==1 and data['counts']['planning_failed']==1
    targets={e['scene_id'] for e in fixture.manifest['entries']}
    for old,new in zip(fixture.previous['entries'],data['entries']):
        for key in old:
            if old['scene_id'] in targets and key.startswith('current_revision'):continue
            assert new[key]==old[key]
        if new['retry_round8'] and new['retry_round8']['execution_status']=='planning_failed':
            assert new['current_revision_result'] is None and 'result' not in new['retry_round8']
        if old['current_revision_status']=='retry_round7_passed':
            assert new['current_revision_status']=='retry_round7_passed' and new['current_revision_success'] is True
    assert data['markdown_report_path']==pub.DOC_PATH=='docs/extensions/replicator/PHYSICAL_RETRY_ROUND8_20261029.md'
    text=pub.document(data);assert '原布局成绩保持151/154' in text and '当前采用版本153/154' in text
    assert '新版七✓' in text and '新版八✓' in text and '14 × 11' in text
    assert '](../../../reports/' not in text


@pytest.mark.parametrize('key',['auditor_source','numerical_auditor_source','scene_revision_checker_source','previous_revision_checker_source','composed_usd_auditor_source'])
def test_auditor_and_revision_source_sha_are_bound(fixture,key):
    fixture.audit[key]=dict(path='stale.py',sha256='0'*64);fixture.flush()
    with pytest.raises((ValueError,FileNotFoundError)):pub.build()


def test_prior_revision_chain_cannot_be_replaced(fixture):
    row=fixture.manifest['entries'][0];rev=pub.read(row['scene_revision']['path']);rev['previous_revision']=dict(path='wrong.json',sha256='0'*64)
    newref=fixture.write(row['scene_revision']['path'],rev);row['scene_revision']=newref;fixture.audit['entries'][0]['scene_revision_audit']['scene_revision']=newref;fixture.flush()
    with pytest.raises(ValueError,match='previous selected layout revision'):pub.build()


def test_no_actual_fail_requires_no_visual_and_no_stale_review(fixture):
    data=pub.build();assert data['diagnostic_visual_review']['review_required'] is False
    assert '本轮没有实际失败，因此未生成新失败诊断图' in pub.document(data)
    fixture.write(pub.VISUAL,dict(kind='physical_failure_visual_review',entries=[dict(scene_id='old')]))
    with pytest.raises(ValueError,match='exactly one source-bound'):pub.build()


def test_planning_failure_zero_episode_and_diagnostic_no_invented_path(fixture):
    row=next(r for r in fixture.manifest['entries'] if r['execution_status']=='planning_failed');row.update(attempts=1,executed=True);next(r for r in fixture.audit['entries'] if r['scene_id']==row['scene_id']).update(attempts=1,executed=True);fixture.flush()
    with pytest.raises(ValueError,match='zero physical attempts'):pub.build()
    row.update(attempts=0,executed=False);next(r for r in fixture.audit['entries'] if r['scene_id']==row['scene_id']).update(attempts=0,executed=False);fixture.diagnostics['entries'][0]['failed_solver_record']['trajectory_saved']=True;fixture.flush()
    with pytest.raises(ValueError,match='fabricate a path'):pub.build()


def test_historical_camera_config_is_original_not_intermediate_r7(fixture):
    row=fixture.manifest['entries'][0];revision=pub.validate_scene_revision(pub.ROOT,row['scene_revision'],expected_scene_id=row['scene_id'],current=row)
    old=next(e for e in fixture.previous['entries'] if e['scene_id']==row['scene_id'])
    assert revision['historical_camera_config']==dict(path=old['config_path'],sha256=old['config_sha256'])
    assert revision['historical_camera_config']!=old['current_revision_config']


def test_document_actual_failure_wording_is_dynamic(fixture):
    # Template-only branch: no fabricated actual result is accepted by build.
    data=pub.build();data['diagnostic_visual_review']['review_required']=True
    data['counts']['physical_failed']=1
    text=pub.document(data)
    assert '失败诊断图仅由本轮实际状态重建，零新增物理' in text
    assert '本轮没有实际失败，因此未生成新失败诊断图' not in text
    assert '本轮实际执行失败为0景' not in text


def test_execution_revision_uses_real_raw_original_plan_binding(fixture):
    raw=deepcopy(fixture.manifest['entries'][0]);original=dict(path=raw.pop('result_path'),sha256=raw.pop('result_sha256'))
    raw['original_planning_result']=original
    checked=pub.validate_execution_revision(raw)
    assert checked['document']['new_config']==dict(path=raw['config_path'],sha256=raw['config_sha256'])
    raw['original_planning_result']=dict(path='unbound-history.json',sha256='0'*64)
    with pytest.raises((ValueError,FileNotFoundError)):pub.validate_execution_revision(raw)
