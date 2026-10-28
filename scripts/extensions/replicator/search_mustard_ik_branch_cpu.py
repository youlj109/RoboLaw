#!/usr/bin/env python3
"""Predeclared CPU seed search for the same source-constrained grasp target."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.search_mustard_grasp_cpu import Search,save
from scripts.extensions.replicator.variant_geometry import code_evidence


def evaluate(search,target,seed):
    pregrasp=target.copy();pregrasp[2,3]+=.125
    try:q=search.kin.ik(pregrasp,seed,attempts=1)
    except RuntimeError as error:return dict(passed=False,stage='pregrasp_ik',reason=str(error))
    actual=search.kin.fk(q);pos=float(np.linalg.norm(actual[:3,3]-pregrasp[:3,3]))
    angle=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@pregrasp[:3,:3].T).magnitude()))
    margin=float(np.min(np.minimum(q-search.kin.limits[:,0],search.kin.limits[:,1]-q)))
    output=dict(passed=False,pregrasp_q_rad=q.tolist(),pregrasp_target_world=pregrasp.tolist(),approach_target_world=target.tolist(),
        pregrasp_position_error_m=pos,pregrasp_rotation_error_deg=angle,pregrasp_minimum_joint_bound_distance_rad=margin,rows=[])
    if pos>.002 or angle>2 or margin<0:return dict(output,stage='pregrasp_ik_error')
    count=max(2,int(np.ceil(np.max(np.abs(q-search.q))/.02))+1)
    for index,fraction in enumerate(np.linspace(0,1,count)):
        value=search.q+fraction*(q-search.q)
        row=dict(phase='pregrasp',sample=index,q_rad=value.tolist(),**search.check_q(value,search.bp,'pregrasp'));output['rows'].append(row)
        if not row['passed']:return dict(output,stage='initial_to_pregrasp_joint_line',declared_samples=count)
    try:plan=local_cartesian_candidate(search.kin,q,target,'approach',1/15,.4)
    except RuntimeError as error:return dict(output,stage='approach_ik',reason=str(error))
    dense=[plan['positions'][0]]
    for a,b in zip(plan['positions'][:-1],plan['positions'][1:]):
        count=max(1,int(np.ceil(np.max(np.abs(b-a))/.02)))
        dense.extend(a+(b-a)*t for t in np.linspace(0,1,count+1)[1:])
    for index,value in enumerate(dense):
        row=dict(phase='approach',sample=index,q_rad=value.tolist(),**search.check_q(value,search.bp,'approach'));output['rows'].append(row)
        if not row['passed']:return dict(output,stage='approach_geometry',approach_declared_samples=len(dense))
    positions=np.asarray([row['q_rad'] for row in output['rows']])
    return dict(output,passed=True,approach_q_rad=dense[-1].tolist(),approach_metadata=plan['metadata'],
        all_samples_minimum_joint_bound_distance_rad=float(np.min(np.minimum(positions-search.kin.limits[:,0],search.kin.limits[:,1]-positions))))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--search-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args()
    if args.output_dir.exists():raise FileExistsError(args.output_dir)
    original=json.loads((args.search_dir/'report.json').read_text());declaration=json.loads((args.search_dir/'declaration.json').read_text())
    if not original['passed'] or original['declaration_sha256']!=sha(args.search_dir/'declaration.json'):raise ValueError('Require original successful CPU candidate')
    if np.any(original['selected']['delta_xyz_m']):raise ValueError('This branch check is scoped to unchanged prepared nominal bottle pose')
    for path,digest in declaration['code_sha256'].items():
        if sha(ROOT/path)!=digest:raise ValueError('Changed original CPU checker '+path)
    prepared=load_preparation(declaration['preparation']);search=Search(prepared)
    selected=original['selected'];prior=next(g['trajectory'] for a in original['attempts'] if a['grid_index']==selected['grid_index'] for g in a['grasps'] if g['candidate_index']==selected['candidate_index'])
    nominal=np.asarray(prior['pregrasp_q_rad']);seeds=[dict(name='original_initial_q',q_rad=search.q.tolist()),dict(name='prior_pregrasp',q_rad=nominal.tolist())]
    for joint in (0,2,4,6):
        for offset in (-.5,.5):
            value=nominal.copy();value[joint]+=offset;value=np.clip(value,search.kin.limits[:,0]+1e-4,search.kin.limits[:,1]-1e-4)
            seeds.append(dict(name=f'prior_joint{joint+1}_{offset:+g}',q_rad=value.tolist()))
    for offset in (-1.5,1.5):
        value=search.q.copy();value[6]+=offset
        seeds.append(dict(name=f'initial_joint7_{offset:+g}',q_rad=value.tolist()))
    evidence=code_evidence([Path(__file__),*[ROOT/path for path in declaration['code_sha256']]])
    args.output_dir.mkdir(parents=True)
    save(args.output_dir/'declaration.json',dict(source_search=str(args.search_dir),source_search_sha256=sha(args.search_dir/'report.json'),
        preparation=declaration['preparation'],preparation_sha256=sha(declaration['preparation']),selected_grasp=selected,seeds=seeds,
        fixed='Exact same source layout, grasp TCP target, original reset q and collision gates. These are numerical IK seeds only.',
        selection='Maximum minimum distance to original joint bounds across the fully passing sampled path; ties keep declared seed order. No original joint limit or collision threshold changes.',
        execution_ready=False,**evidence))
    target=search.bp@np.asarray(selected['grasp']['bottle_to_tcp_local']);rows=[]
    for index,seed in enumerate(seeds):
        row=dict(seed_index=index,name=seed['name'],**evaluate(search,target,np.asarray(seed['q_rad'])));rows.append(row)
        print(json.dumps(dict(seed_index=index,passed=row['passed'],minimum_joint_bound_rad=row.get('all_samples_minimum_joint_bound_distance_rad'),stage=row.get('stage'))),flush=True)
    eligible=[row for row in rows if row['passed']]
    chosen=max(eligible,key=lambda r:r['all_samples_minimum_joint_bound_distance_rad']) if eligible else None
    unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    report=dict(passed=bool(chosen and unchanged),selected_seed_index=chosen['seed_index'] if chosen else None,
        results=rows,code_unchanged=unchanged,execution_ready=False,curobo_validated=False,physics_launched=False,self_collision_validated=False,
        declaration_sha256=sha(args.output_dir/'declaration.json'),
        limitation='Joint-bound distance is a conditioning diagnostic, not a cuRobo check_valid result. Source/conservative collision tests are sampled and exclude only declared inner-finger/payload contact; no physics or task-success proof.')
    save(args.output_dir/'report.json',report)
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
