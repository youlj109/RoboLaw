#!/usr/bin/env python3
"""Read-only CPU application of a declared local grasp to another original layout."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from scripts.extensions.replicator.search_mustard_grasp_cpu import Search,save
from scripts.extensions.replicator.variant_geometry import code_evidence


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--search-dir',type=Path,required=True)
    p.add_argument('--preparation',type=Path,action='append',required=True);p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args()
    if args.output_dir.exists():raise FileExistsError(args.output_dir)
    original=json.loads((args.search_dir/'report.json').read_text());decl=json.loads((args.search_dir/'declaration.json').read_text())
    if not original['passed'] or not original['code_unchanged'] or original['declaration_sha256']!=sha(args.search_dir/'declaration.json'):
        raise ValueError('Require preserved successful nominal CPU candidate evidence')
    for path,digest in decl['code_sha256'].items():
        if sha(ROOT/path)!=digest:raise ValueError('Changed candidate implementation '+path)
    if sha(decl['preparation'])!=decl['preparation_sha256']:raise ValueError('Changed original preparation')
    source=load_preparation(decl['preparation']);grasp=original['selected']['grasp']
    evidence=code_evidence([Path(__file__),*[ROOT/path for path in decl['code_sha256']]])
    args.output_dir.mkdir(parents=True)
    save(args.output_dir/'declaration.json',dict(source_search=str(args.search_dir),source_search_sha256=sha(args.search_dir/'report.json'),
        grasp=grasp,preparations={str(path):sha(path) for path in args.preparation},layout_changes=False,
        selection='Test the exact chosen bottle-local grasp on each named untouched original prepared layout. Failures remain failures; no new orientation or object-pose fallback.',
        execution_ready=False,**evidence))
    rows=[]
    for path in args.preparation:
        prepared=load_preparation(path)
        if prepared['layout']['objects']['mustard_bottle']['usd_path']!=source['layout']['objects']['mustard_bottle']['usd_path']:
            raise ValueError('Different source bottle is not an eligible rigid-pose transfer')
        search=Search(prepared);initial=search.scene.evaluate('mustard_bottle',[0.,0.,0.])
        target=search.bp@np.asarray(grasp['bottle_to_tcp_local'])
        envelope=search.envelope_scan(grasp);endpoint=search.gripper_environment(target)
        trajectory=search.trajectory(target,search.bp) if initial['passed'] and envelope['passed'] and endpoint['passed'] else None
        row=dict(task_id=prepared['task_id'],preparation=str(path),preparation_sha256=sha(path),
            source_initial_geometry=initial,gripper_bottle_approach=envelope,gripper_environment_endpoint=endpoint,
            sampled_trajectory=trajectory,passed=bool(trajectory and trajectory['passed']),execution_ready=False)
        rows.append(row);save(args.output_dir/(prepared['task_id']+'.json'),row)
        print(json.dumps(dict(task=row['task_id'],passed=row['passed'],stage=trajectory.get('stage') if trajectory else 'gripper_environment_or_envelope',execution_ready=False)),flush=True)
    unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    output=dict(results=rows,passed=bool(unchanged and all(row['passed'] for row in rows)),code_unchanged=unchanged,
        execution_ready=False,physics_launched=False,curobo_validated=False,self_collision_validated=False,
        declaration_sha256=sha(args.output_dir/'declaration.json'),
        scope='Nominal source and conservative all-pair checks at sampled original-q joint line and CPU vertical IK. Body closure evidence in source layout is not a physical or per-layout contact certificate; actual neutral, source relations after settling, self-collision, cuRobo, bilateral force, lift/place and videos remain required.')
    save(args.output_dir/'report.json',output)
    return int(not output['passed'])


if __name__=='__main__':raise SystemExit(main())
