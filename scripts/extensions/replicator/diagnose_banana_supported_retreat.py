#!/usr/bin/env python3
"""Read-only attribution of the actual rejected banana retreat, no physics."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.diagnose_task_plan import reconstruct, cpu_sphere_audit
from scripts.extensions.replicator.task_audit_sidecars import sha, ROOT
from scripts.extensions.replicator.task_collect import json_write


class RetreatMasks:
    """Mirror the strict supported planner's banana-only candidate mask."""
    def __init__(self, original): self.original=original
    def __getattr__(self, name): return getattr(self.original,name)
    def cuda_candidate_masks(self, phase, attached):
        if phase!='retreat' or attached: raise ValueError('Expected free-object retreat')
        return ('banana',)


def source_pairs(world, positions):
    rows=[]
    for i,q in enumerate(positions):
        failures=[];support=[]
        for pair in world._pairs(q,(), 'planning',world.gripper_joint_positions,None,None,max_slack=0.):
            if pair[0]>0:break
            value=world._measurement(pair)
            if not value['valid']:
                (support if value['robot_part']=='payload/banana' and '/bowl/' in value['obstacle'] else failures).append(value)
        rows.append(dict(sample=i,robot_and_other_pairs_passed=not failures,failures=failures,
                         free_banana_bowl_source_intersections=support))
    return dict(all_robot_and_other_pairs_passed=all(r['robot_and_other_pairs_passed'] for r in rows),rows=rows,
        scope='Exact source FCL pair attribution; static free-banana/bowl contacts are retained separately, never used here to authorize execution.')


def run(args):
    world,kin,start,target,meta,info=reconstruct(args.episode,'retreat')
    plan=info['original_plan_record'];archive=args.episode/plan['rejected_candidate_file']
    if sha(archive)!=plan['rejected_candidate_sha256']:raise ValueError('Rejected path hash mismatch')
    with np.load(archive,allow_pickle=False) as data: positions=data['positions'].copy()
    if not np.array_equal(start,positions[0]):raise ValueError('Start mismatch')
    world.contact_policy=RetreatMasks(world.contact_policy)
    report=dict(accepted=False,physical_execution=False,task_success=False,input_sha256=info['input_sha256'],
                reconstruction=info,rejected_path_sha256=sha(archive),source_pairs=source_pairs(world,positions))
    report['cpu_spheres'],spheres=cpu_sphere_audit(world,kin,positions)
    np.savez_compressed(args.output/'cpu_spheres.npz',spheres=spheres,positions=positions)
    candidates=[]
    if args.search:
        from kitchen_safety.extensions.replicator.banana_retreat_motion import retreat_candidates,cartesian_retreat_candidate
        for description in retreat_candidates(kin,start,target):
            row=dict(id=description['id'],cpu_passed=False)
            try:
                candidate=cartesian_retreat_candidate(kin,start,description,meta['action_dt_s'],.4)
                row['source_pairs']=source_pairs(world,candidate['positions'])
                row['cpu_spheres'],_=cpu_sphere_audit(world,kin,candidate['positions'])
                row['cpu_passed']=row['source_pairs']['all_robot_and_other_pairs_passed'] and not row['cpu_spheres']['world_intersections'] and not row['cpu_spheres']['joint_urdf_violations']
                row['metadata']=candidate['metadata']
                np.savez_compressed(args.output/(row['id']+'.npz'),positions=candidate['positions'],poses=candidate['poses'],effective_goal_pose=candidate['effective_goal_pose'],start_q=start)
                row['path_sha256']=sha(args.output/(row['id']+'.npz'))
            except Exception as exc:row['error']=repr(exc)
            candidates.append(row)
            print(json.dumps(dict(candidate=row['id'],cpu_passed=row['cpu_passed'],error=row.get('error'))),flush=True)
        report['candidates']=candidates
    if args.gpu:
        import fcntl
        lock=(ROOT/'reports/skill_trials/gpu.lock').open('a+')
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        from kitchen_safety.extensions.replicator.banana_supported_planner import SupportedBananaPlanner
        from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components,sphere_pair_diagnostics
        # Exact previously observed certificate, solely for reproducing the query.
        # No source-contact or execution permission is inferred from this copy.
        world.planning_certificate=deepcopy(plan['observed_target_support_certificate'])
        planner=SupportedBananaPlanner(world,kin,dt=meta['action_dt_s'],seed=42,max_joint_speed=.4)
        planner._prepare_path_query((),0,'retreat')
        report['cuda'],state=constraint_components(planner,positions)
        report['cuda_attribution'],_=sphere_pair_diagnostics(planner,state,report['cuda'])
        for row in candidates:
            if not row['cpu_passed']:continue
            with np.load(args.output/(row['id']+'.npz')) as raw:candidate_positions=raw['positions']
            row['cuda'],_=constraint_components(planner,candidate_positions)
            row['cuda_passed']=row['cuda']['valid']
            print(json.dumps(dict(candidate=row['id'],cuda_passed=row['cuda_passed'])),flush=True)
        lock.close()
    json_write(args.output/'diagnostic.json',report)
    print(json.dumps(dict(output=str(args.output),source_robot_failures=sum(not r['robot_and_other_pairs_passed'] for r in report['source_pairs']['rows']),
        cpu_sphere_pairs=len(report['cpu_spheres']['world_intersections']),cuda=report.get('cuda',{}).get('valid'))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--gpu',action='store_true')
    parser.add_argument('--search',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False);run(args)


if __name__=='__main__':main()
