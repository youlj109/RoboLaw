"""Verify merge precedence and asset non-expansion without physics/GPU queries."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.reviewed_scope_lock import load_lock,apply_lock
from kitchen_safety.ylj_risk_capture import input_preflight
from kitchen_safety.task_contract import digest


def verify(repo,runtime,deployed,output,snapshot):
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
        human_variants_require_separate_HS_scope_review=sum(r['unreviewed_human_would_expand_scope'] for r in preflight),
        real_ylj_capture_verified=False,
        remaining_requirement='Restore the original SHA-bound ylj planning input bundle and matching external assets before real trajectory replay')
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('protected_risk_files_unchanged','asset_scope_checks')},ensure_ascii=False))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-evidence',type=Path,required=True)
    p.add_argument('--deployed-source',type=Path,required=True)
    p.add_argument('--risk-snapshot',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();verify(ROOT,args.runtime_evidence,args.deployed_source,args.output,args.risk_snapshot)
