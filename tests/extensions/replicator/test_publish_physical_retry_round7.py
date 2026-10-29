"""Round7 dual-version publication/source gates; synthetic data, no GPU/physics."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.extensions.replicator import publish_physical_retry_round7 as pub

ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def fixture(monkeypatch,tmp_path):
    previous=json.loads((ROOT/pub.PREVIOUS).read_text());monkeypatch.setattr(pub,'ROOT',tmp_path)
    def write(path,value):
        path=pub.local(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False));return pub.ref(path)
    evidence=write('cpu/evidence.json',{'synthetic':True});prior_manifest=dict(kind='physical_failure_retry',retry_round=6,stage='complete',entries=[])
    old_by_id={}; failed=[e for e in previous['entries'] if not e['latest_success']];assert len(failed)==3
    for old in failed:
        sid=old['scene_id'];cfg=write('old/'+sid+'/layout.json',dict(task_id=old['task_id'],robot_position=[0,0,0],obstacle=dict(position=[1,0,0],yaw_deg=0,usd_path='asset.usda')))
        stage=write('old/'+sid+'/scene.usda',{'usd':'old source fixture'});initial=write('old/'+sid+'/planning.json',dict(composition={'source_paths_sha256':{}}));last=write('old/'+sid+'/physical.json',{'scene_id':sid})
        old.update(config_path=cfg['path'],config_sha256=cfg['sha256'],latest_physical_result=last)
        original=dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],config_path=cfg['path'],config_sha256=cfg['sha256'],source_scene_path=stage['path'],source_scene_sha256=stage['sha256'],result_path=initial['path'],result_sha256=initial['sha256'],execution_status='planning_failed',executed=False,attempts=0)
        prior_manifest['entries'].append(original);old_by_id[sid]=(old,original,cfg,stage)
    prior=write('prior/manifest.json',prior_manifest);previous['source_retry_manifest']=prior;write(pub.PREVIOUS,previous)
    plan=dict(retry_round=7,layout_changed=True,success_predicate_changed=False,source_previous_retry_manifest=prior,source_first_pass_manifest=prior,source_camera_retake_manifest=prior,execution_settings={},entries=[])
    manifest=dict(kind='physical_failure_retry',retry_round=7,stage='complete',rendering_enabled=False,source_previous_retry_manifest=prior,entries=[])
    audit=dict(kind='physical_retry_round7_independent_audit',passed=True,errors=[],all3_audited=True,audited_scenes=3,require_complete=True,selected_scene=None,entries=[])
    diagnostics=dict(kind='physical_retry_planning_failure_cpu_diagnostics',retry_round=7,total=2,diagnosed=2,complete=True,source_plan_unchanged=True,errors=0,physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=0,layout_changed=True,diagnostic_sources={},method='Retained solver failures',limitations=['No new IK or physics'],entries=[])
    for index,(sid,(old,original,oldcfg,oldstage)) in enumerate(old_by_id.items()):
        cfg=pub.read(oldcfg['path']);cfg['obstacle']['position']=[1,.02,0];newcfg=write('new/'+sid+'/layout.json',cfg);newstage=write('new/'+sid+'/scene.usda',{'usd':'new source fixture'})
        composition=dict(source_paths_sha256={},layout_sha256=newcfg['sha256'],scene_sha256=newstage['sha256'])
        revision=write('revision/'+sid+'.json',dict(kind='hazard_pose_revision',retry_round=7,scene_id=sid,prior_manifest=prior,prior_row_source={k:original[k] for k in pub.validate_scene_revision.__globals__['SOURCE_KEYS']},old_config=oldcfg,old_source_scene=oldstage,new_config=newcfg,new_source_scene=newstage,changed_fields=['obstacle.position'],new_composition=composition))
        generated=index==0;identity=dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],config_path=newcfg['path'],config_sha256=newcfg['sha256'],scene_revision=revision,layout_changed=True)
        status='MotionGenStatus.FINETUNE_TRAJOPT_FAIL';segment=dict(phase='lift',segment_index=0,solver_status=status,solver_success=False,samples=0,status='failed')
        policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='保留碰撞规划，去掉额外直线门槛。')
        candidate=dict(identity,status='generated' if generated else 'failed',attempts=1,executed=False,planning_only=True,composition=composition,recipe=dict(planning_path_policy=policy,segment_deviation_limits_m={}),failure_reason=None if generated else status,artifacts={},failed_phase=None if generated else 'lift',phases=[] if generated else [dict(segment,segments=[deepcopy(segment)])])
        planning=write('planning/'+sid+'.json',candidate)
        row=dict(identity,result_path=original['result_path'],result_sha256=original['result_sha256'],source_scene_path=newstage['path'],source_scene_sha256=newstage['sha256'],candidate_planning_result=planning,previous_retry_result=old['latest_physical_result'],physical_attempt_number=7,planning_status='generated' if generated else 'failed',trajectory_path=None,trajectory_sha256=None,failure_reason=None if generated else status,adjustment=dict(description='Synthetic hazard translation',evidence_path=evidence['path'],evidence_sha256=evidence['sha256']))
        if generated:
            trajectory=write('trajectory/'+sid+'.npz',{'fixture':'trajectory bytes'});row.update(trajectory_path=trajectory['path'],trajectory_sha256=trajectory['sha256'])
        plan['entries'].append(deepcopy(row));row.update(execution_status='success' if generated else 'planning_failed',attempts=int(generated),executed=generated,task_success=generated)
        checked={k:row[k] for k in ('scene_id','planning_status','execution_status','attempts','executed','task_success','candidate_planning_result')};checked.update(status='passed',scene_revision_audit=dict(scene_revision=revision,layout_changed=True))
        if generated:
            raw=dict(row,kind='physical_failure_retry_execution',planning_result_path=planning['path'],planning_result_sha256=planning['sha256'],prior_physical_attempts=[])
            result=write(Path('exports')/pub.RUN/'executions'/sid/'result.json',raw);row.update(execution_result_path=result['path'],execution_result_sha256=result['sha256']);checked.update(result=result,no_render_gate_passed=True)
        else:
            diagnostics['entries'].append(dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],scene_revision=revision,candidate_planning_result=planning,config=newcfg,source_scene=newstage,status='diagnosed',failed_phase='lift',solver_status=status,root_cause_confirmed=False,scene_infeasibility_established=False,observed_cpu_facts_zh=['求解未返回失败段路径，无新增物理。'],interpretation_zh='一次求解失败不说明无解。',next_adjustment_zh='检查目标端点与源几何。',diagnostic_method='solver_failure_without_failed_segment_path',failed_segment_trajectory_available=False,failed_solver_record=dict(phase='lift',segment_index=0,solver_status=status,solver_success=False,samples=0,trajectory_saved=False)))
        manifest['entries'].append(row);audit['entries'].append(checked)
    plan_ref=write('plan.json',plan);manifest.update(retry_plan_path=plan_ref['path'],retry_plan_sha256=plan_ref['sha256']);diagnostics['source_plan']=plan_ref
    def flush():
        audit['manifest_snapshot']=write(pub.MANIFEST,manifest);write(pub.AUDIT,audit);write(pub.PLANNING_DIAGNOSTICS,diagnostics)
    flush();return SimpleNamespace(previous=previous,plan=plan,manifest=manifest,audit=audit,diagnostics=diagnostics,write=write,flush=flush,root=tmp_path)


def test_separate_original_151_from_current_152_and_preserve_all_history(fixture):
    data=pub.build();assert len(data['entries'])==154 and len(data['tasks'])==14
    assert data['original_layout_counts']['success']==data['counts']['latest_success']==151
    assert data['current_revision_counts']['success']==data['counts']['current_revision_success']==152
    assert data['counts']['physical_attempts']==1 and data['counts']['planning_failed']==2
    for old,new in zip(fixture.previous['entries'],data['entries']):
        for key in old:assert new[key]==old[key]
        retry=new['retry_round7']
        if retry and retry['execution_status']=='planning_failed':
            assert retry['attempts']==0 and retry['executed'] is False and 'result' not in retry
            assert new['current_revision_result'] is None and new['latest_physical_result']==old['latest_physical_result']
        if retry and retry['task_success']:
            assert new['latest_success'] is False and new['current_revision_success'] is True
            assert new['current_revision_result']!=new['latest_physical_result']
    assert data['diagnostic_visual_review']['review_required'] is False
    assert data['safety_accepted'] is False
    assert pub.DOC_PATH.endswith('PHYSICAL_RETRY_ROUND7_20261029.md')
    assert data['markdown_report_path']==pub.DOC_PATH
    text=pub.document(data);assert '原布局成绩保持151/154' in text and '当前采用版本152/154' in text and '14 × 11' in text
    assert '](../../../reports/' not in text


def test_planning_failure_cannot_become_new_episode(fixture):
    row=next(r for r in fixture.manifest['entries'] if r['execution_status']=='planning_failed');row.update(attempts=1,executed=True)
    next(r for r in fixture.audit['entries'] if r['scene_id']==row['scene_id']).update(attempts=1,executed=True);fixture.flush()
    with pytest.raises(ValueError,match='zero physical attempts'):pub.build()


def test_revision_rejects_original_task_change_even_with_rebound_sha(fixture):
    row=fixture.manifest['entries'][0];revision=pub.read(row['scene_revision']['path']);cfg=pub.read(revision['new_config']['path']);cfg['robot_position']=[.1,0,0]
    revision['new_config']=fixture.write(revision['new_config']['path'],cfg);newref=fixture.write(row['scene_revision']['path'],revision)
    row['scene_revision']=newref;row['config_sha256']=revision['new_config']['sha256'];fixture.audit['entries'][0]['scene_revision_audit']['scene_revision']=newref;fixture.flush()
    with pytest.raises(ValueError,match='beyond added-hazard'):pub.build()


def test_revision_stale_source_rejected(fixture):
    row=fixture.manifest['entries'][0];revision=pub.read(row['scene_revision']['path']);fixture.write(revision['new_source_scene']['path'],{'changed':'source'})
    with pytest.raises(ValueError,match='source SHA differs'):pub.build()


def test_diag_must_bind_new_revision_and_cannot_invent_failed_path(fixture):
    item=fixture.diagnostics['entries'][0];original=deepcopy(item['scene_revision']);item['scene_revision']=dict(path='old.json',sha256='0'*64);fixture.flush()
    with pytest.raises(ValueError,match='scene_revision binding differs'):pub.build()
    item['scene_revision']=original;item['failed_solver_record']['trajectory_saved']=True;fixture.flush()
    with pytest.raises(ValueError,match='fabricate a path'):pub.build()


def test_old_line_gate_method_not_allowed_in_new_free_path_round(fixture):
    fixture.diagnostics['entries'][0]['diagnostic_method']='independent_source_fk_segment_gate';fixture.flush()
    with pytest.raises(ValueError,match='removed the additional straight-line gate'):pub.build()


def test_empty_visual_allowed_but_stale_observation_rejected(fixture):
    fixture.write(pub.VISUAL,dict(kind='physical_failure_visual_review',entries=[]));assert pub.build()['diagnostic_visual_review']['source_visual_review']
    fixture.write(pub.VISUAL,dict(kind='physical_failure_visual_review',entries=[dict(scene_id='unrelated')]))
    with pytest.raises(ValueError,match='exactly one source-bound'):pub.build()
