"""Round10 preserves real retained-layout history and the previous zero episodes."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.extensions.replicator import publish_physical_retry_round10 as pub
ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def fixture(monkeypatch,tmp_path):
    spec=importlib.util.spec_from_file_location('r9_publication_fixture',ROOT/'tests/extensions/replicator/test_publish_physical_retry_round9.py');r9=importlib.util.module_from_spec(spec);spec.loader.exec_module(r9)
    f=r9.fixture.__wrapped__(monkeypatch,tmp_path);write=f.write
    # Round9 had two planning failures, no new physical episodes.
    for row in f.manifest['entries']:
        if not row['executed']:continue
        sid=row['scene_id'];candidate=r9.pub.read(row['candidate_planning_result']['path'])
        segment=dict(phase='lower',segment_index=0,solver_status='IK_FAIL',solver_success=False,samples=0,status='failed')
        candidate.update(status='failed',failed_phase='lower',failure_reason='IK_FAIL',phases=[dict(segment,segments=[segment])])
        reference=write(row['candidate_planning_result']['path'],candidate)
        planrow=next(e for e in f.plan['entries'] if e['scene_id']==sid)
        for item in (row,planrow):item.update(planning_status='failed',trajectory_path=None,trajectory_sha256=None,failure_reason='IK_FAIL',candidate_planning_result=reference)
        row.update(execution_status='planning_failed',executed=False,attempts=0,task_success=False)
        row.pop('execution_result_path');row.pop('execution_result_sha256')
        audit=next(e for e in f.audit['entries'] if e['scene_id']==sid)
        for key in ('planning_status','execution_status','executed','attempts','task_success','candidate_planning_result'):audit[key]=row[key]
        audit.pop('result');audit.pop('no_render_gate_passed')
        diag=deepcopy(f.diagnostics['entries'][0]);diag.update(scene_id=sid,task_id=row['task_id'],variant_id=row['variant_id'],scene_revision=row['scene_revision'],candidate_planning_result=reference,config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),failed_phase='lower',solver_status='IK_FAIL',failed_solver_record=dict(phase='lower',segment_index=0,solver_status='IK_FAIL',solver_success=False,samples=0,trajectory_saved=False))
        f.diagnostics['entries'].append(diag)
    reference=write('r9/plan.json',f.plan);f.manifest['retry_plan_sha256']=reference['sha256'];f.diagnostics.update(source_plan=reference,total=2,diagnosed=2);f.flush()
    previous=r9.pub.build();assert previous['current_revision_counts']['success']==152
    monkeypatch.setattr(pub,'ROOT',tmp_path);write(pub.PREVIOUS,previous)
    prior=previous['source_retry_manifest'];source=f.audit['auditor_source'];evidence=write('r10/preparation.json',dict(reused_joint_goals=True))
    plan=dict(retry_round=10,layout_changed=False,layout_changed_from_original=True,success_predicate_changed=False,source_previous_retry_manifest=prior,source_first_pass_manifest=prior,source_camera_retake_manifest=prior,execution_settings={},entries=[])
    manifest=dict(kind='physical_failure_retry',retry_round=10,stage='complete',rendering_enabled=False,source_previous_retry_manifest=prior,entries=[])
    audit=deepcopy(f.audit);audit.update(kind='physical_retry_round10_independent_audit',entries=[],previous_retention_checker_source=source)
    diag=deepcopy(f.diagnostics);diag.update(retry_round=10,total=1,diagnosed=1,entries=[])
    for index,oldrow in enumerate(f.manifest['entries']):
        row=deepcopy(oldrow);sid=row['scene_id'];old=next(e for e in previous['entries'] if e['scene_id']==sid);generated=index==1
        for key in ('execution_status','executed','attempts','task_success'):row.pop(key)
        row.update(physical_attempt_number=10,planning_status='generated' if generated else 'failed',previous_retry_result=old.get('current_revision_result') or old['latest_physical_result'],adjustment=dict(description='Use retained checked joint goals',evidence_path=evidence['path'],evidence_sha256=evidence['sha256']))
        # The fixture uses bowl endpoint repair; numerical FK is independently tested by audit10.
        recipe=dict(retry_round=10,payload_name='bowl',retained_scene_revision=True,scene_settings_changed_this_round=False,
            joint_space_target_q_by_phase={'lower':[[0.]*7],'retreat':[[0.]*7]},planning_options_by_phase={'lower':{'enable_graph':False},'retreat':{'enable_graph':False}},hold_orientation_phases=[],linear_motion_phases=[],segment_deviation_limits_m={},
            planning_path_policy=dict(kind='collision_checked_joint_goal_endpoint',previous_straight_line_gate_removed=True,reason_zh='Use checked endpoint branch, retain collision checks.'))
        candidate=pub.read(row['candidate_planning_result']['path']);candidate.update(recipe=recipe,status='generated' if generated else 'failed')
        planning=write('r10/planning/'+sid+'.json',candidate);row['candidate_planning_result']=planning
        if generated:
            trajectory=write('r10/trajectory/'+sid+'.npz',{'fixture':'trajectory'});row.update(trajectory_path=trajectory['path'],trajectory_sha256=trajectory['sha256'],failure_reason=None)
        plan['entries'].append(deepcopy(row));row.update(execution_status='success' if generated else 'planning_failed',attempts=int(generated),executed=generated,task_success=generated)
        checked={k:row[k] for k in ('scene_id','planning_status','execution_status','attempts','executed','task_success','candidate_planning_result')};checked.update(status='passed',scene_revision_audit=dict(scene_revision=row['scene_revision'],layout_changed=True,retained_scene_revision=True,scene_settings_changed_this_round=False,retained_from_manifest=prior))
        if generated:
            raw=dict(row,kind='physical_failure_retry_execution',planning_result_path=planning['path'],planning_result_sha256=planning['sha256'],original_planning_result=dict(path=row['result_path'],sha256=row['result_sha256']),source_previous_retry_manifest=prior,prior_physical_attempts=[])
            result=write(Path('exports')/pub.RUN/'executions'/sid/'result.json',raw);row.update(execution_result_path=result['path'],execution_result_sha256=result['sha256']);checked.update(result=result,no_render_gate_passed=True)
        else:
            item=deepcopy(next(d for d in f.diagnostics['entries'] if d['scene_id']==sid));item['candidate_planning_result']=planning;diag['entries'].append(item)
        manifest['entries'].append(row);audit['entries'].append(checked)
    planref=write('r10/plan.json',plan);manifest.update(retry_plan_path=planref['path'],retry_plan_sha256=planref['sha256']);diag['source_plan']=planref
    def flush():
        audit['manifest_snapshot']=write(pub.MANIFEST,manifest);write(pub.AUDIT,audit);write(pub.PLANNING_DIAGNOSTICS,diag)
    flush();return SimpleNamespace(previous=previous,plan=plan,manifest=manifest,audit=audit,diagnostics=diag,write=write,flush=flush)


def test_round10_retains_nine_layers_and_old_actual_result(fixture):
    data=pub.build();assert data['counts']['current_revision_success']==153 and data['counts']['latest_success']==151
    assert data['counts']['physical_attempts']==1 and data['counts']['planning_failed']==1
    for old,new in zip(fixture.previous['entries'],data['entries']):
        for key in old:
            if key.startswith('current_revision') and new['retry_round10']:continue
            assert new[key]==old[key]
        r=new['retry_round10']
        if r and not r['executed']:
            assert r['result'] is None and r['attempts']==0
            assert new['current_revision_result']==old['current_revision_result']
            assert new['current_revision_result_retry_round']==8
        if r and r['executed']:assert new['current_revision_result_retry_round']==10
    assert '第十轮保留布局' in pub.document(data) and 'MotionGen.plan_single_js' in pub.document(data)
    assert data['markdown_report_path'].endswith('PHYSICAL_RETRY_ROUND10_20261029.md')


def test_helper9_source_cannot_drift(fixture):
    fixture.audit['previous_retention_checker_source']=dict(path='missing.py',sha256='0'*64);fixture.flush()
    with pytest.raises((ValueError,FileNotFoundError)):pub.build()


def test_retained_round9_cannot_invent_episode(fixture):
    prior=fixture.previous['source_retry_manifest'];m=pub.read(prior['path']);m['entries'][0]['attempts']=1
    fixture.write(prior['path'],m)
    with pytest.raises(ValueError,match='Bound source changed'):pub.build()


@pytest.mark.parametrize('method',['independent_source_fk_lift_corridor_gate','independent_source_fk_joint_goal_endpoint_gate'])
def test_saved_path_rejection_requires_matching_independent_metrics(monkeypatch,tmp_path,method):
    monkeypatch.setattr(pub,'ROOT',tmp_path)
    path=tmp_path/'saved.npz'
    import numpy as np
    np.savez(path,q=np.zeros((2,7)))
    ref=pub.ref(path);lift=method.endswith('lift_corridor_gate')
    limits=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002)
    gate=dict(reference_world=np.eye(4).tolist(),thresholds=limits,source_samples=2,max_xy_deviation_m=.004,
        max_orientation_deviation_deg=21.,min_z_offset_m=0.,final_z_offset_m=.03,passed=False)
    endpoint=dict(joint_space_endpoint_error_rad=.005,endpoint_fk_position_error_m=.003,endpoint_fk_orientation_error_rad=.03,
        joint_goal_endpoint_gate_passed=False,independent_fk_verified=True)
    seg=dict(phase='lift' if lift else 'lower',segment_index=0,solver_status='LIFT_PATH_CONSTRAINT_LIMIT' if lift else 'JOINT_GOAL_ENDPOINT_LIMIT',
        solver_success=True,status='failed',samples=2,trajectory=ref,lift_path_validation=gate,**endpoint)
    audit=dict(phase=seg['phase'],segment_index=0,trajectory=ref,independent_fk_verified=True,lift_path_validation=gate,joint_goal_endpoint_audit=endpoint)
    item=dict(diagnostic_method=method,failed_solver_record={k:seg[k] for k in ('phase','segment_index','solver_status','solver_success','samples')},
        failed_segment_trajectory_available=True,failed_path_fk_available=True,failed_segment_trajectory=ref,segment_audit=dict(segments=[audit]),
        independent_lift_gate=deepcopy(gate),independent_joint_endpoint_gate=deepcopy(endpoint))
    item['failed_solver_record']['trajectory_saved']=True
    candidate=dict(recipe=dict(lift_path_validation=limits));report=dict(cpu_ik_seed_attempts_per_scene=0)
    pub.validate_saved_path_diagnostic(item,seg,candidate,report)
    altered=deepcopy(item)
    altered['independent_lift_gate' if lift else 'independent_joint_endpoint_gate']['max_orientation_deviation_deg' if lift else 'endpoint_fk_orientation_error_rad']=0.
    with pytest.raises(ValueError):pub.validate_saved_path_diagnostic(altered,seg,candidate,report)
    altered=deepcopy(item);altered['failed_segment_trajectory_available']=False
    with pytest.raises(ValueError,match='actual returned failed segment'):pub.validate_saved_path_diagnostic(altered,seg,candidate,report)
    path.write_bytes(b'changed')
    with pytest.raises(ValueError,match='Bound source changed'):pub.validate_saved_path_diagnostic(item,seg,candidate,report)
