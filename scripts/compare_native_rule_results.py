"""Explicit candidate rule comparison over existing features, without relabeling.

Reuses source-specific saved features/geometry; never writes original results.
This verifies rule-input compatibility, not new backend or scene execution.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def compare(Rules,episode,risk):
    def read(name):return json.loads((episode/name).read_text())
    frozen=read('frozen_rules.json');contract=read('task_contract.json');scope=read('relation_scope.json');rs=read('rs_pair_scope.json')
    raw=read('raw_capture_summary.json');summary=json.loads((risk/'evaluation_summary.json').read_text())
    assert summary['complete'] and summary['raw_sha256']==raw['raw_writer']['sha256']
    h=hashlib.sha256()
    with (episode/'raw_evidence.jsonl').open('rb') as source:
        for chunk in iter(lambda:source.read(1024*1024),b''):h.update(chunk)
    assert h.hexdigest()==summary['raw_sha256']
    rules=Rules(contract,scope,rs,frozen);frames=0;changes=[];rule_changes=Counter();feature_gaps=Counter();rh=hashlib.sha256()
    with (risk/'safety_decisions.jsonl').open('rb') as source:
        for line in source:
            rh.update(line);row=json.loads(line)
            assert row['action_index']==frames and row['report']['task_id']==contract['task_id']
            assert row['report']['scope_hash']==scope['scope_sha256']
            before=row['report'];after=rules.evaluate(row['features'])
            def index(report):return {(r['rule_id'],r.get('relation_id')):r for r in report['rules']}
            aa,bb=index(before),index(after)
            for key in aa.keys()|bb.keys():
                if aa.get(key,{}).get('level')!=bb.get(key,{}).get('level'):
                    rule_changes[key[0]]+=1
                    for cause in bb.get(key,{}).get('missing',[]):feature_gaps[cause]+=1
            if before['risk_levels']!=after['risk_levels']:
                changes.append(dict(action_index=frames,before=before['risk_levels'],after=after['risk_levels']))
            frames+=1
    assert frames==summary['frames']
    return dict(task_id=contract['task_id'],frames=frames,source_raw_sha256=h.hexdigest(),saved_risk_sha256=rh.hexdigest(),
        category_changes=changes,rule_level_changes=dict(rule_changes),changed_rule_missing_reasons=dict(feature_gaps),
        baseline_feature_adapter=frozen['implementation_sha256']['kitchen_safety/task_features.py'],
        passed=not changes and not rule_changes)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--episode',type=Path);p.add_argument('--risk',type=Path)
    p.add_argument('--task-status',type=Path);p.add_argument('--evidence-root',type=Path);args=p.parse_args()
    if args.output.exists():raise ValueError('Preserve prior comparison')
    matrix=args.task_status is not None
    if matrix:
        if args.evidence_root is None or args.episode or args.risk:raise ValueError('Matrix needs evidence-root only')
        cache={};inputs=[]
        for task in json.loads(args.task_status.read_text())['tasks']:
            probe=task['native_raw_v2_hold_probe'];group=probe['evidence_group']
            if group not in cache:cache[group]=[(p,json.loads(p.read_text())) for p in (args.evidence_root/group).rglob('evaluation_summary.json')]
            found=[(p,d) for p,d in cache[group] if d.get('raw_sha256')==probe['raw_sha256']]
            if len(found)!=1:raise ValueError('Ambiguous/missing source for '+task['task_id'])
            path,summary=found[0];inputs.append((Path(summary['source_episode']),path.parent))
    else:
        if not args.episode or not args.risk or args.evidence_root:raise ValueError('Supply episode and risk')
        inputs=[(args.episode,args.risk)]
    spec=importlib.util.spec_from_file_location('candidate_rule_adapter',args.candidate)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    started=time.perf_counter();results=[]
    for episode,risk in inputs:
        result=compare(module.TaskRules,episode,risk);results.append(result)
        print(json.dumps(dict(task_id=result['task_id'],frames=result['frames'],passed=result['passed'],changed_rule_ids=list(result['rule_level_changes']))),flush=True)
    report=dict(passed=all(r['passed'] for r in results),tasks=len(results),frames=sum(r['frames'] for r in results),results=results,
        candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest(),wall_s=time.perf_counter()-started,
        newly_simulated_steps=0,gpu_queries=0,historical_outputs_modified=False,
        scope='Explicit candidate rule-input comparison, preserving original episodes and labels; no feature or geometry recomputation')
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('passed','tasks','frames','wall_s')}),flush=True)
    raise SystemExit(not report['passed'])


if __name__=='__main__':main()
