#!/usr/bin/env python3
"""Read-only fixed-goal CPU diagnosis of frozen round3 planning failures.

One deterministic IK seed and two endpoint checks per failure. No new candidate,
cuRobo solve, Kit, GPU, rendering or physics is started; no layout is modified.
Requires arena Python with pxr/python-fcl and CUDA_VISIBLE_DEVICES=''.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import planning_failure_cpu_geometry as geometry
from scripts.extensions.replicator.planning_failure_cpu_geometry import ASSET,Probe,sha,local,bound,binding,summarize


def parser():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--plan',default='configs/extensions/replicator/physical_retry_round3_20261029.json')
 p.add_argument('--output',default='reports/replicator_physical_retry_round3_20261029')
 return p

def main():
 if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('This diagnostic must hide CUDA devices')
 args=parser().parse_args();PLAN=local(args.plan);OUT=local(args.output)
 OUT.mkdir(parents=True,exist_ok=True)
 if (OUT/'planning_failure_diagnostics.json').exists() or (OUT/'planning_failure_diagnostics.partial.json').exists():raise FileExistsError('Retain previous diagnosis; no automatic repeated CPU IK')
 plan=json.loads(PLAN.read_text());source=sha(PLAN)
 if plan.get('retry_round')!=3 or plan.get('kind')!='physical_failure_retry_plan' or plan.get('expected_scene_count')!=29:raise ValueError('Expected frozen round3 plan')
 selected=[r for r in plan['entries'] if r['planning_status']=='failed']
 if len(plan['entries'])!=29 or len({r['scene_id'] for r in plan['entries']})!=29:raise ValueError('Expected unique 29-scene cohort')
 report=dict(kind='physical_retry_planning_failure_cpu_diagnostics',schema_version=1,retry_round=3,
  source_plan=binding(PLAN),total=len(selected),complete=False,entries=[],physical_execution_started=False,kit_started=False,
  curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=1,layout_changed=False,
  method='Source URDF robot spheres + 5mm source planner buffer; source USD triangles/primitive AABBs; original phase masks; one fixed-goal CPU IK seed.',
  limitations=['Endpoint diagnostics only; no continuous path or physical-force verification.','Conservative robot/payload sphere overlap is not proof of exact physical mesh collision.',
   'FCL distance values for intersecting geometry are not interpreted as penetration depth.','A failed single IK seed or one colliding IK branch does not establish scene infeasibility.'],
  diagnostic_sources={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),Path(geometry.__file__),ASSET/'franka_robotiq_2f_85_zero_curobo.yml',ASSET/'urdf/franka_robotiq_2f_85_zero.urdf',ASSET/'tcp_calibration.json',ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/extensions/replicator/quick_trajectory.py']})
 for row in selected:
  begin=time.monotonic();result_path=bound(row['candidate_planning_result']);result=json.loads(result_path.read_text())
  phase=result.get('failed_phase') or result['stage'];detail=next(p for p in result['phases'] if p['phase']==phase)
  entry=dict(scene_id=row['scene_id'],task_id=row['task_id'],variant_id=row['variant_id'],candidate_planning_result=row['candidate_planning_result'],
   config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
   failed_phase=phase,solver_status=detail['solver_status'],commanded_gripper_joint_rad=result['nominal_gripper_joint_rad'],
   planning_gripper_joint_rad=result.get('planning_gripper_joint_rad',result['nominal_gripper_joint_rad']),root_cause_confirmed=False,scene_infeasibility_established=False)
  try:
   probe=Probe(row,result);q=np.asarray(result['start_q']);partial=result.get('artifacts',{}).get('partial_trajectory');samples=0
   if partial:
    path=bound(partial)
    with np.load(path,allow_pickle=False) as data:q=data['q'][-1].copy();samples=len(data['q'])
    entry['partial_trajectory']=partial
   entry['failed_phase_start']=dict(source='last successful partial-trajectory command' if partial else 'source-authored initial q',
    completed_samples=samples,q=q.tolist(),tcp_world=probe.kin.fk(q).tolist(),collision_diagnostic=probe.check(q))
   target=np.asarray(result['targets'][phase]);goal=dict(solved=False,target_world=target.tolist(),seed_q=q.tolist(),seed_attempts=1)
   try:
    candidate_q=probe.kin.ik(target,q,attempts=1);actual=probe.kin.fk(candidate_q)
    goal.update(solved=True,q=candidate_q.tolist(),position_error_m=float(np.linalg.norm(actual[:3,3]-target[:3,3])),
     orientation_error_rad=float(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude()),collision_diagnostic=probe.check(candidate_q))
   except RuntimeError as error:goal['error']=str(error)
   entry['fixed_goal_one_seed_ik']=goal
   facts,interpretation,recommendation=summarize(entry)
   entry.update(observed_cpu_facts_zh=facts,interpretation_zh=interpretation,next_adjustment_zh=recommendation,status='diagnosed')
  except Exception as error:entry.update(status='diagnostic_error',error=repr(error),next_adjustment_zh='先排查CPU诊断所需源资产或几何读取，不能据此判断任务无解。')
  entry['elapsed_seconds']=time.monotonic()-begin
  if sha(result_path)!=row['candidate_planning_result']['sha256']:raise RuntimeError('Candidate changed during diagnostic')
  report['entries'].append(entry)
  (OUT/'planning_failure_diagnostics.partial.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
  print('CPU_DIAG',row['scene_id'],entry['status'],round(entry['elapsed_seconds'],2),flush=True)
 if sha(PLAN)!=source:raise RuntimeError('Frozen plan changed')
 report.update(complete=True,source_plan_unchanged=True,diagnosed=sum(e['status']=='diagnosed' for e in report['entries']),
  errors=sum(e['status']!='diagnosed' for e in report['entries']))
 with (OUT/'planning_failure_diagnostics.json').open('x') as stream:json.dump(report,stream,ensure_ascii=False,indent=2);stream.write('\n')
 print('CPU_DIAG_COMPLETE',report['diagnosed'],report['errors'],flush=True)

if __name__=='__main__':main()
