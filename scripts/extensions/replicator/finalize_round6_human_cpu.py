#!/usr/bin/env python3
"""Freeze one already-screened round6 target and source-check joint goals.
No new Cartesian target, CUDA, Kit, physics or collision policy changes.
"""
from pathlib import Path
from copy import deepcopy
import os,sys,json
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,sampled_segments,pose_check
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe,PHASES
OUT=ROOT/'reports/replicator_physical_retry_round6_20261029'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round6_human_recipes_20261029.json'
SID='u_shape_bowl_sink__human_default_mpfb'

def main():
 assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
 if CONFIG.exists():raise FileExistsError('Frozen output exists')
 source=OUT/'human_cpu/candidates.json';candidate_report=read(source);selected=deepcopy(candidate_report['trials'][2]);r=selected['recipe'];manifest=read(geo.bound(candidate_report['source_manifest']));row=next(x for x in manifest['entries'] if x['scene_id']==SID);previous=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(candidate_report['source_original_planning']));probe=geo.Probe(row,previous);masks=masks_for(original)
 # These are the same already-declared lift waypoints. Screening was stopped
 # by the later lower endpoint, so complete independent lift/retreat checks.
 end=selected['endpoints'];segments=sampled_segments(probe,r,end,masks,('lift',));assert segments['passed'],'Continuous ideal lift screening failed'
 checks={x['phase']:x for x in end['checks']};retreat=pose_check(probe,np.array(r['target_poses_world']['retreat']),np.array(checks['lower']['q_rad']),'retreat',r,masks)
 goals={'pregrasp':[checks['pregrasp']['q_rad']],'approach':[checks['approach']['q_rad']],'lift':[x['q_rad'] for x in segments['samples'] if x['fraction']==1.]}
 fkchecks=[]
 for phase,values in goals.items():
  targets=[*r.get('phase_waypoints_world',{}).get(phase,[]),r['target_poses_world'][phase]];assert len(values)==len(targets)
  for index,(q,target) in enumerate(zip(values,targets)):
   q=np.asarray(q);target=np.asarray(target);fk=probe.kin.fk(q);pe=float(np.linalg.norm(fk[:3,3]-target[:3,3]));re=float(Rotation.from_matrix(fk[:3,:3]@target[:3,:3].T).magnitude());assert pe<=.001 and re<=.01 and np.all(q>=probe.kin.limits[:,0]) and np.all(q<=probe.kin.limits[:,1])
   fkchecks.append(dict(phase=phase,segment_index=index,q_rad=q.tolist(),position_error_m=pe,rotation_error_rad=re,passed=True))
 r['joint_space_target_q_by_phase']=goals;r['planning_options_by_phase']={p:dict(enable_graph=False) for p in goals};r['hold_orientation_phases']=[p for p in r.get('hold_orientation_phases',[]) if p not in goals];r['linear_motion_phases']=[p for p in r.get('linear_motion_phases',[]) if p not in goals]
 region=selected['nominal_destination'];assert region['passed'] and region['historical_measured_attachment_reused'] is False and region['derived_from_new_nominal_attachment'] is True
 region_path=OUT/'human_nominal_destination.json';save(region_path,region);r['destination_region_check']=dict(passed=True,evidence_path=str(region_path.relative_to(ROOT)),sha256=geo.sha(region_path));r['destination_nominal_z_limit_m']=.06
 r.update(retry_round=6,frozen_candidate_count=1,one_selected_candidate=True,candidate_index=2,cpu_screen_passed=False,previous_candidate_planning_result=row['candidate_planning_result'],latest_actual_result=row['previous_retry_result'],config_path=row['config_path'],config_sha256=row['config_sha256'],cpu_warning='名义源整碗与排水口无冲突，但lower端点的保守attached_object球包络对Drain_0仍有1对冲突；retreat的单一CPU IK分支另有panda_link5与FaucetSpout/FaucetTip两对冲突。完全张爪和直线退出的夹爪源mesh净空通过，不代表整臂无碰撞。保留原掩码，交唯一一次cuRobo判定，完整CPU未通过。',description='保留round5新碗缘抓姿；pregrasp/approach固定已通过源几何检查的IK分支，lift每个原分段固定连续IK关节目标并保持原5mm实际TCP路径门禁。放置以本轮nominal_attachment重新计算，碗直立偏航-90°，原目标X偏移-50mm、名义源碗底距原槽底3mm、Z变化-15.72mm；全开爪后竖直退出200mm。不继承旧90mm实测补偿；新实际支撑未验证。lower保守携物球仍有排水口冲突、retreat单一IK分支另有link5水龙头冲突警告。',limitations='CPU ideal samples and joint goals are not an injected trajectory seed or an actual cuRobo path. Public plan_single_js will plan each frozen segment once, then actual FK must satisfy unchanged 5mm gate. No physical grasp/support/safety guarantee.')
 targets,a,grip=validate_recipe(r,original,probe.layout)
 report=dict(kind='round6_human_frozen_nominal_and_joint_goal_evidence',scene_id=SID,source_cpu_candidates=ref(source),selected_index=2,source_manifest=candidate_report['source_manifest'],source_previous_planning=candidate_report['source_previous_planning'],source_original_planning=candidate_report['source_original_planning'],source_script=ref(Path(__file__)),new_nominal_destination=ref(region_path),source_contact_passed=candidate_report['source_contact']['passed'],source_nonpayload_closing_clear=candidate_report['source_closing']['all_nonpayload_world_clear'],source_bowl_world=selected['source_bowl_world'],source_gripper_release=selected['source_gripper_release'],source_open_retreat_sweep=selected['source_open_retreat_sweep'],endpoint_checks=end,retreat_endpoint=retreat,continuous_ideal_lift_samples=segments,joint_goals_fk_checks=fkchecks,shared_recipe_validation_passed=True,cpu_screen_passed=False,warning_zh=r['cpu_warning'],candidate_count_frozen=1,maximum_cpu_candidate_count=8,new_cartesian_candidates_after_first_screen=0,new_curobo_calls=0,new_physical_episodes=0,old_measured_attachment_reused=False,scope='Only nominal source pose evidence for current grasp. No successful measured attachment or physical support is inferred; source geometry, masks, layout, physics/controller/predicate and 5mm path gate unchanged.')
 report_path=OUT/'human_frozen_cpu_evidence.json';save(report_path,report);r['evidence']=ref(report_path);save(CONFIG,dict(kind='physical_retry_round6_human_recipes',schema_version=1,source_evidence=ref(report_path),recipes={SID:r}))
 print(json.dumps(dict(config=ref(CONFIG),report=ref(report_path),nominal_goal_delta_xyz_m=region['nominal_goal_delta_xyz_m'],joint_goals=len(fkchecks),max_joint_goal_fk_position_error_m=max(x['position_error_m'] for x in fkchecks),max_joint_goal_fk_rotation_error_rad=max(x['rotation_error_rad'] for x in fkchecks),lift_samples=len(segments['samples']),retreat_endpoint_passed=retreat['passed'],full_cpu_passed=False),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
