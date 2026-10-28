"""Verify merge precedence and asset non-expansion without physics/GPU queries."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.reviewed_scope_lock import load_lock,apply_lock
from kitchen_safety.ylj_risk_capture import input_preflight
from kitchen_safety.task_contract import digest


def audit_validation_run(run):
    """Read-only comparison with original NPZ and already completed risk/RGB."""
    import numpy as np
    try:
        from orjson import loads
    except ImportError:loads=json.loads
    def read(path):return json.loads(path.read_text())
    episode=run/'capture/native_000/raw_episode';risk=run/'risk/native_000';rgb=run/'rendered/native_000'
    capture=read(episode/'ylj_integration.json');raw=read(episode/'raw_capture_summary.json')
    evaluated=read(risk/'evaluation_summary.json');rendered=read(rgb/'render_report.json')
    scope=read(episode/'relation_scope.json');lock=scope['bindings']['reviewed_scope_lock']
    assert capture['sequence_complete'] and capture['reset_gate']['passed']
    assert evaluated['complete'] and evaluated['raw_preflight']['passed']
    assert rendered['complete'] and rendered['physics_steps_advanced']==0
    assert raw['online_geometry_queries']==raw['online_risk_evaluations']==0
    assert not capture['replanned'] and not capture['trajectory_retimed']
    assert scope['reviewed_scope_ceiling']['added_evaluated_assets']==[]
    assert scope['reviewed_scope_ceiling']['actual_PT_assets']<=len(lock['pt_targets'])
    trajectory=episode/'controller_inputs/trajectory.npz'
    assert hashlib.sha256(trajectory.read_bytes()).hexdigest()==capture['entry']['trajectory_sha256']
    with np.load(trajectory,allow_pickle=False) as archive:data=dict(archive)
    indices=[];holds={};observations=0;h=hashlib.sha256()
    for line in (episode/'raw_evidence.jsonl').open('rb'):
        h.update(line);packet=loads(line)
        if packet['kind']!='observation':continue
        observations+=1;evidence=packet['controller_evidence'];index=evidence['source_trajectory_index']
        if index<0:
            phase=evidence['source_phase'];holds[phase]=holds.get(phase,0)+1;continue
        indices.append(index)
        actual=np.asarray(packet['nominal_action'],dtype=np.float32)
        expected=np.r_[data['q'][index],data['gripper'][index]].astype(np.float32)[None]
        assert np.array_equal(actual,expected)
        assert evidence['source_phase']==str(data['phase'][index])
    assert indices==list(range(len(data['q'])))
    assert holds==dict(initial_settle=150,close=60,release=60,final_settle=90)
    assert observations==len(indices)+sum(holds.values()) and raw['physics_steps']==4*observations
    assert h.hexdigest()==raw['raw_writer']['sha256']==evaluated['raw_sha256']==rendered['source_raw_sha256']
    assert evaluated['frames']==rendered['frames']==observations+1
    allowed={r['relation_id'] for r in scope['relations']};frames=0
    for line in (risk/'safety_decisions.jsonl').open('rb'):
        row=loads(line);frames+=1
        assert set(row['geometry']['relations'])<=allowed
        assert row['report']['categories']['HS']['status']=='not_applicable'
        assert row['report']['disabled_categories']==['IR']
    assert frames==evaluated['frames']
    return dict(passed=True,scene_id=capture['scene_id'],source_raw_sha256=h.hexdigest(),
        actual_original_NPZ_commands=len(indices),physical_holds=holds,recorded_actions=observations,
        physics_steps=raw['physics_steps'],risk_samples=frames,PT_assets=scope['reviewed_scope_ceiling']['actual_PT_assets'],
        added_evaluated_assets=0,source_commands_exact=True,trajectory_retimed=False,
        online_risk_queries=0,online_risk_grades=0,HS='not_applicable',IR='disabled',
        original_task_success=capture['task_success'],risk_level_counts=evaluated['risk_level_counts'],
        collection_wall_s=raw['collection_wall_s'],offline_evaluation_wall_s=evaluated['wall_s'],
        geometry_wall_s=evaluated['timings']['geometry_s'],render_wall_s=rendered['wall_s'],
        offline_render_physics_steps_advanced=0,video_sha256=hashlib.sha256((rgb/'risk_review.mp4').read_bytes()).hexdigest(),
        scope='One real source-bound trajectory and its current reviewed asset scope, not all 154 physical scenes')


def verify(repo,runtime,deployed,output,snapshot,validation_run=None):
    locks=json.loads((repo/'configs/risk_integration/replicator_scope_lock.json').read_text())
    index=json.loads((repo/'configs/extensions/replicator/final_154_scenes_20261028.json').read_text())
    suite=json.loads((repo/'configs/risk_task_suite/suite.json').read_text())
    protected=['kitchen_safety/task_scope.py','kitchen_safety/task_features.py','kitchen_safety/task_rules.py',
        'kitchen_safety/task_mesh.py','kitchen_safety/task_workspace.py','kitchen_safety/task_contact_policy.py',
        'kitchen_safety/task_target_policy.py','kitchen_safety/gdist_mesh_backend.py',
        'kitchen_safety/hs_dynamic.py','scripts/evaluate_native_task.py','scripts/render_native_task.py',
        'configs/safety_observer_arena.json','configs/risk_capture_workflow.json']
    for folder in ['configs/risk_task_suite','configs/risk_scope_policies','configs/risk_workspace_policies']:
        protected.extend(str(p.relative_to(repo)) for p in (repo/folder).rglob('*') if p.is_file())
    profile=json.loads((repo/'configs/safety_observer_arena.json').read_text())['profile_file']
    protected.append(profile)
    checked=[]
    for relative in sorted(set(protected)):
        path=repo/relative
        if not path.exists():
            if relative=='kitchen_safety/hs_dynamic.py':continue
            raise ValueError('Missing protected source '+relative)
        original=subprocess.check_output(['git','show',snapshot+':'+relative],cwd=repo)
        if path.read_bytes()!=original:raise ValueError('Risk-source precedence violation: '+relative)
        if (deployed/relative).read_bytes()!=path.read_bytes():raise ValueError('Runtime differs: '+relative)
        checked.append(relative)
    integration_sources=['kitchen_safety/reviewed_scope_lock.py','kitchen_safety/task_risk_setup.py',
        'kitchen_safety/ylj_risk_capture.py','kitchen_safety/task_capture.py','kitchen_safety/native_workflow.py',
        'kitchen_safety/extensions/replicator/planned_execution.py','scripts/collect_ylj_risk_task.py',
        'scripts/risk_capture_workflow.py','configs/risk_integration/replicator_scope_lock.json']
    for relative in integration_sources:
        if (deployed/relative).read_bytes()!=(repo/relative).read_bytes():
            raise ValueError('Integrated runtime source differs: '+relative)
    planner='kitchen_safety/extensions/replicator/quick_trajectory.py'
    if (repo/planner).read_bytes()!=subprocess.check_output(['git','show','origin/ylj:'+planner],cwd=repo):
        raise ValueError('ylj planner was unexpectedly modified')
    candidates=[]
    # Read only the current review and explicit user exclusion migrations.
    for parent in runtime.iterdir():
        if parent.is_dir() and (parent.name=='latest31_review_20261028' or parent.name.endswith('_exclusion_20261028')):
            candidates.extend(parent.glob('*/capture/native_000/raw_episode/relation_scope.json'))
    wanted={v['source_scope_sha256'] for v in locks['tasks'].values()};sources={}
    for path in candidates:
        scope=json.loads(path.read_text())
        if scope['scope_sha256'] in wanted:sources[scope['scope_sha256']]=(path,scope)
    results=[]
    for task_id,row in locks['tasks'].items():
        path,scope=sources[row['source_scope_sha256']]
        inventory=json.loads((path.parent/'mesh_inventory.json').read_text())
        contract=json.loads((repo/'configs/risk_task_suite'/(task_id+'.json')).read_text())
        after=apply_lock(scope,inventory,load_lock(repo,contract))
        before_relations={r['relation_id']:r for r in scope['relations']}
        after_relations={r['relation_id']:r for r in after['relations']}
        if not after_relations.keys()<=before_relations.keys():
            raise ValueError('New relation introduced: '+task_id)
        for rid,relation in after_relations.items():
            before=before_relations[rid]
            if relation['category']!='PT':
                if relation!=before:raise ValueError('HS relation changed')
            else:
                if not set(relation['contact_target_colliders'])<=set(before['contact_target_colliders']):
                    raise ValueError('New PT contact surface')
                if not set(relation['target_colliders'])<=set(before['target_colliders']):
                    raise ValueError('New PT distance surface')
                for key in ('grade_force','grade_impulse','source_colliders','applicability'):
                    if relation[key]!=before[key]:raise ValueError('Retained relation semantics changed')
        if after['rs']!=scope['rs']:raise ValueError('Independent RS scope changed')
        results.append(dict(task_id=task_id,source_scope_sha256=row['source_scope_sha256'],
            approved_PT_assets=len(row['pt_targets']),actual_PT_assets=after['reviewed_scope_ceiling']['actual_PT_assets'],
            retained_relation_semantics_preserved=True,
            unused_candidate_relations_removed=len(before_relations)-len(after_relations),
            new_evaluated_assets=0,HS_RS_unchanged=True))
    preflight=[]
    for entry in index['entries']:
        tid=entry['task_id'].removeprefix('kitchen_bench_')
        contract=json.loads((repo/'configs/risk_task_suite'/(tid+'.json')).read_text())
        preflight.append(input_preflight(contract,index,entry['scene_id'],load_lock(repo,contract)))
    report=dict(passed=True,physics_steps_advanced=0,risk_GPU_queries=0,
        risk_snapshot_commit=snapshot,ylj_source_commit='e8c8fedfba335e3a1c7bcc979f380eb63138689f',
        protected_risk_files_unchanged=checked,task_contracts_preserved=len(suite['tasks']),
        integration_runtime_sources_match=integration_sources,ylj_quick_planner_byte_identical=True,
        reviewed_Replicator_tasks=len(results),asset_scope_checks=results,
        ylj_scenes_indexed=len(preflight),
        scenes_missing_inputs=sum(bool(r['missing_inputs']) for r in preflight),
        scenes_with_changed_inputs=sum(bool(r['changed_inputs']) for r in preflight),
        source_gate_failures=[r for r in preflight if r['original_source_gate_error']],
        human_variants_require_separate_HS_scope_review=sum(r['unreviewed_human_would_expand_scope'] for r in preflight),
        real_ylj_capture_verified=False)
    report['remaining_requirement']='Review new human scope separately; other scenes have not received physical end-to-end validation'
    if report['scenes_missing_inputs'] or report['scenes_with_changed_inputs'] or report['source_gate_failures']:
        report['remaining_requirement']='Restore original source-bound input files and exact external assets before replay'
    if validation_run:
        report['end_to_end_validation']=audit_validation_run(validation_run)
        report['real_ylj_capture_verified']=True
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('protected_risk_files_unchanged','asset_scope_checks')},ensure_ascii=False))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-evidence',type=Path,required=True)
    p.add_argument('--deployed-source',type=Path,required=True)
    p.add_argument('--risk-snapshot',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--validation-run',type=Path)
    args=p.parse_args();verify(ROOT,args.runtime_evidence,args.deployed_source,args.output,args.risk_snapshot,args.validation_run)
