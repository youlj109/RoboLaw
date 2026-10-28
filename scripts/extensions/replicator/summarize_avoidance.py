"""Summarize specified causal-avoidance episodes without treating old scenes as evidence."""
import json
from pathlib import Path
import h5py
from trial_evidence import acceptance,load_json
ROOT=Path(__file__).resolve().parents[3]
TRIALS=[('human','attempt_04'),('property','attempt_02'),('robot','attempt_01')]
def main():
 rows=[]
 for variant,attempt in TRIALS:
  out=ROOT/'datasets/avoidance_trials/replicator'/variant/attempt;rep=ROOT/'reports/avoidance_trials/replicator'/variant/attempt;r=load_json(out/'result.json') or {};s=load_json(rep/'summary.json') or {};a=load_json(out/'avoidance_audit.json') or {};accepted=acceptance(out,rep,s.get('exit_codes',{}),require_rgb=True)['passed']
  row=dict(variant=variant,attempt=attempt,accepted=accepted,physical_success=r.get('success',False),avoidance_audit_accepted=a.get('accepted',False),dataset=str(out.relative_to(ROOT)),reports=str(rep.relative_to(ROOT)),baseline=a.get('baseline_episode'),source_yaml_sha256=r.get('layout',{}).get('source_yaml_sha256'),same_source_layout=a.get('same_source_layout',False),actual_initial_states_match=a.get('initial_state_comparison',{}).get('passed',False),background_consistency_passed=(load_json(out/'background_consistency_audit.json') or {}).get('passed',False),same_effective_controls=a.get('same_effective_controls',False),baseline_counterfactual_minimum_m=a.get('baseline_counterfactual',{}).get('min_distance_m'),baseline_violating_physics_samples=len(a.get('baseline_counterfactual',{}).get('violations',[])),actual_minimum_m=a.get('augmented_actual',{}).get('min_distance_m'),required_execution_margin_m=a.get('augmented_actual',{}).get('required_m'),videos=[str((out/(name+'.mp4')).relative_to(ROOT)) for name in ['external_camera','external_camera_2','wrist_camera','all_cameras','overview_camera','comparison'] if (out/(name+'.mp4')).exists()])
  if (out/'trajectory.h5').exists():
   with h5py.File(out/'trajectory.h5') as f:row['frames']=len(f['action'])
  if r.get('error'):row['failure_reason']=r['error']
  rows.append(row)
 report=dict(task_id='kitchen_bench_replicator_g_shape_mustard_bowl',accepted_episodes=sum(x['accepted'] for x in rows),required_episodes=3,complete=all(x['accepted'] for x in rows),fixed_only=True,variants=rows,scope='Causal avoidance is separate from the prior 3 physical-only skill-trial scenes. Same valid source layout and effective controls; baseline hazard violations + safe replanned physics + all independent sidecars/RGB required. No unsafe counterfactual executed; no 200-episode or randomized claim.',other_source_tasks_not_verified=13)
 target=ROOT/'reports/avoidance_trials/replicator/summary.json';target.write_text(json.dumps(report,indent=2)+'\n');(ROOT/'docs/extensions/replicator/avoidance_coverage.json').write_text(json.dumps(report,indent=2)+'\n')
 coverage=load_json(ROOT/'docs/extensions/replicator/coverage.json') or {};coverage['scope']='Prior successful_episodes counts physical scene trials only; causal avoidance evidence is separate below.';coverage['causal_avoidance']=dict(accepted_episodes=report['accepted_episodes'],required_episodes=3,complete=report['complete'],report='docs/extensions/replicator/avoidance_coverage.json');(ROOT/'docs/extensions/replicator/coverage.json').write_text(json.dumps(coverage,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
