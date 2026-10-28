#!/usr/bin/env python3
"""Prepare an unexecuted source/evidence-bound independent mustard controller."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--preparation',type=Path,required=True)
    p.add_argument('--search-dir',type=Path,required=True);p.add_argument('--branch-dir',type=Path);p.add_argument('--transfer-dir',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    prepared=load_preparation(args.preparation);report=json.loads((args.search_dir/'report.json').read_text())
    declaration=json.loads((args.search_dir/'declaration.json').read_text())
    if not report['passed'] or not report['code_unchanged'] or sha(args.search_dir/'declaration.json')!=report['declaration_sha256']:
        raise ValueError('Require a completed unchanged CPU candidate')
    selected=report['selected'];inputs={'preparation':args.preparation,'search_report':args.search_dir/'report.json','search_declaration':args.search_dir/'declaration.json'}
    if args.transfer_dir:
        transfer=json.loads((args.transfer_dir/'report.json').read_text())
        if not transfer['code_unchanged']:raise ValueError('Transfer dependencies changed')
        row=next(r for r in transfer['results'] if r['task_id']==prepared['task_id'])
        if not row['passed'] or row['preparation_sha256']!=sha(args.preparation):raise ValueError('This task transfer did not pass')
        trajectory=row['sampled_trajectory'];inputs['transfer_report']=args.transfer_dir/'report.json'
    else:
        if declaration['preparation_sha256']!=sha(args.preparation):raise ValueError('Original search preparation differs')
        trajectory=next(g['trajectory'] for a in report['attempts'] if a['grid_index']==selected['grid_index'] for g in a['grasps'] if g['candidate_index']==selected['candidate_index'])
    if args.branch_dir:
        if args.transfer_dir:raise ValueError('A branch cannot be copied to another layout')
        branch=json.loads((args.branch_dir/'report.json').read_text());bd=json.loads((args.branch_dir/'declaration.json').read_text())
        if not branch['passed'] or not branch['code_unchanged'] or bd['source_search_sha256']!=sha(args.search_dir/'report.json'):
            raise ValueError('Invalid independent branch evidence')
        trajectory=next(r for r in branch['results'] if r['seed_index']==branch['selected_seed_index'])
        inputs.update(branch_report=args.branch_dir/'report.json',branch_declaration=args.branch_dir/'declaration.json')
    if any(selected['delta_xyz_m']):raise ValueError('This initial controller supports unchanged prepared bottle placement only')
    layout=prepared['layout'];result=dict(schema_version=1,task_id=prepared['task_id'],
        inputs={key:dict(path=str(path.resolve().relative_to(ROOT)),sha256=sha(path)) for key,path in inputs.items()},
        source_layout_sha256=prepared['evidence']['input_hashes']['layout']['sha256'],
        source_kitchen_sha256=sha(ROOT/layout['kitchen_usd']),source_object_sha256={n:sha(ROOT/c['usd_path']) for n,c in layout['objects'].items()},
        bottle_to_tcp_local=selected['grasp']['bottle_to_tcp_local'],pregrasp_ik_seed_rad=trajectory['pregrasp_q_rad'],
        motion=dict(pregrasp_height_m=.125,lift_height_m=.145,release_gap_m=.003,transport_guard_above_rim_m=.05),
        clearances=prepared['profile']['clearances'],baseline_and_variants_share_profile=True,
        selection='Fixed source-local CPU grasp candidate; recompute world target from actual settled bottle pose and IK using the bound branch only as a numerical seed. Reset q and source layout never change.',
        execution_ready=False,physical_success_verified=False)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),task=result['task_id'],execution_ready=False)))


if __name__=='__main__':main()
