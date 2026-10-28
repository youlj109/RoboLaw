#!/usr/bin/env python3
"""Summarize quick candidate artifacts without collision or physics validation."""
from collections import Counter, defaultdict
from datetime import datetime
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import atomic_json, stamp


def summarize(output):
    progress=json.loads((output/'progress.json').read_text())
    expected=['pregrasp','approach','lift','transfer','lower','retreat']
    hash_cache={}
    def digest(path):
        path=Path(path)
        if path not in hash_cache:
            hash_cache[path]=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        return hash_cache[path]
    tasks=defaultdict(Counter); stages=Counter(); codes=Counter()
    config_errors=[];asset_errors=[];artifact_errors=[];results=[];partial=0
    for row in progress['entries']:
        tasks[row['task_id']][row['status']]+=1
        if digest(ROOT/row['config_path'])!=row['config_sha256']:
            config_errors.append(row['scene_id'])
        result_path=output/'scenes'/row['scene_id']/'result.json'
        if not result_path.is_file():
            artifact_errors.append(dict(scene_id=row['scene_id'],error='missing_result'))
            continue
        result=json.loads(result_path.read_text());results.append(result)
        if result['config_sha256']!=row['config_sha256'] or result['status']!=row['status']:
            artifact_errors.append(dict(scene_id=row['scene_id'],error='result_progress_binding_mismatch'))
        for source,wanted in result.get('composition',{}).get('source_paths_sha256',{}).items():
            if digest(ROOT/source)!=wanted:
                asset_errors.append(dict(scene_id=row['scene_id'],source=source))
        if row['status']=='failed':
            stages[result['stage']]+=1;codes[result.get('failure_code','unknown')]+=1
            if result.get('trajectory_complete') is True:
                artifact_errors.append(dict(scene_id=row['scene_id'],error='failed_result_marked_complete'))
        if 'partial_trajectory' in result.get('artifacts',{}):partial+=1
        for name,artifact in result.get('artifacts',{}).items():
            path=ROOT/artifact['path']
            if digest(path)!=artifact.get('sha256'):
                artifact_errors.append(dict(scene_id=row['scene_id'],error='artifact_hash_mismatch',artifact=name))
        if row['status']!='generated':continue
        try:
            assert result['trajectory_complete'] is True
            assert [phase['phase'] for phase in result['phases']]==expected
            assert all(phase['status']=='generated' for phase in result['phases'])
            assert result.get('executed') is False and result.get('task_success_verified') is False
            archive=ROOT/result['artifacts']['trajectory']['path']
            with np.load(archive,allow_pickle=False) as arrays:
                q=arrays['q'];tcp=arrays['tcp_world'];phase=arrays['phase'];grip=arrays['gripper']
                assert q.ndim==2 and q.shape[1]==7 and len(q)>1 and np.isfinite(q).all()
                assert tcp.shape==(len(q),4,4) and np.isfinite(tcp).all()
                assert grip.shape==(len(q),) and np.isfinite(grip).all()
                assert phase.shape==(len(q),) and list(dict.fromkeys(phase.tolist()))==expected
                assert float(arrays['dt'])>0
        except Exception as error:
            artifact_errors.append(dict(scene_id=row['scene_id'],error='generated_archive_integrity',detail=repr(error)))
    retry_round = progress.get('retry_round', 0)
    timed_results = [result for result in results if result.get('retry_round') == retry_round] if retry_round else results
    starts=[datetime.fromisoformat(result['started_at']) for result in timed_results if result.get('started_at')]
    ends=[datetime.fromisoformat(result['updated_at']) for result in timed_results if result.get('updated_at')]
    terminal=all(row['status'] in ('generated','failed','skipped') for row in progress['entries'])
    summary=dict(schema_version=1,created_at=stamp(),scope='Artifact/config integrity only; no physical or avoidance verification.',
        total=len(progress['entries']),terminal=terminal,unique_scenes=len({row['scene_id'] for row in progress['entries']}),
        counts=dict(Counter(row['status'] for row in progress['entries'])),
        task_counts={task:dict(counts) for task,counts in tasks.items()},failure_stage_counts=dict(stages),failure_code_counts=dict(codes),
        partial_trajectory_count=partial,
        first_scene_started_at=min(starts).isoformat() if starts else None,
        last_scene_finished_at=max(ends).isoformat() if ends else None,
        first_start_to_last_finish_seconds=(max(ends)-min(starts)).total_seconds() if starts and ends else None,
        elapsed_scope=('This retry round only; retained first-round successful results are excluded from runtime.' if retry_round else
            'First smoke scene start through final scene finish, including the short code/preflight gap between smoke and batch.'),
        summed_scene_runtime_seconds=sum(result.get('elapsed_seconds',0.) for result in timed_results),
        config_binding_errors=config_errors,source_asset_binding_errors=asset_errors,artifact_integrity_errors=artifact_errors,
        unique_config_source_and_artifact_files_hashed=len(hash_cache),
        artifact_integrity_passed=terminal and len(progress['entries'])==154 and len({row['scene_id'] for row in progress['entries']})==154 and not(config_errors or asset_errors or artifact_errors),
        physical_execution_verified=False,long_term_stability_verified=False,avoidance_verified=False,task_success_verified=False)
    if retry_round:
        summary.update(retry_round=retry_round, retry_counts=progress['retry_counts'], retry_total=progress['retry_total'],
                       carried_forward=progress['carried_forward'], round_actions=progress.get('round_actions', {}),
                       fixed_layout_integrity=progress.get('fixed_layout_integrity'))
    atomic_json(output/'summary.json',summary)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'reports/replicator_trajectory_generation_20261026')
    args=parser.parse_args();summary=summarize(args.output.resolve())
    print(json.dumps(summary,indent=2,ensure_ascii=False))
    raise SystemExit(0 if summary['artifact_integrity_passed'] else 1)
