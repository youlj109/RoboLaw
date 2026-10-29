#!/usr/bin/env python3
"""Freeze three already checked lift IK goals; no IK, CUDA, Kit or physics calls."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
SID='u_shape_mustard_bowl__robot_sharp_equipment_housing'
RUN='replicator_physical_retry_round10_20261029'

def ref(path):
    path=ROOT/path
    return dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())

def bound(binding):
    assert ref(binding['path'])==binding, 'Evidence changed: '+binding['path']
    return json.loads((ROOT/binding['path']).read_text())

def main():
    previous_recipe_ref=ref('configs/extensions/replicator/physical_retry_round9_mustard_recipes_20261029.json')
    previous=bound(previous_recipe_ref)['recipes'][SID]
    prep=bound(previous['evidence'])
    agreement_ref=ref('reports/replicator_physical_retry_round9_20261029/cpu_curobo_static_agreement.json')
    agreement=bound(agreement_ref)
    checked=next(e for e in agreement['entries'] if e['scene_id']==SID)
    assert checked['source_cpu_report']==previous['evidence'] and checked['sample_count']==78
    assert checked['summaries']['world_constraint_nonzero_samples']==0 and checked['summaries']['self_constraint_nonzero_samples']==0
    assert checked['gpu_api_calls']==dict(plan_single=0,plan_single_js=0,ik_solve=0)
    previous_manifest=bound(agreement['source_manifest'])
    prior=next(r for r in previous_manifest['entries'] if r['scene_id']==SID)
    assert previous_manifest['retry_round']==9 and previous_manifest['stage']=='complete'
    assert prior['attempts']==0 and not prior['executed'] and prior['execution_status']=='planning_failed'
    assert checked['candidate_planning_result']==prior['candidate_planning_result']
    bound(prior['candidate_planning_result'])
    assert checked['config']==prep['retained_config'] and checked['source_scene']==prep['retained_source_scene']
    assert checked['retained_scene_revision']==prep['retained_scene_revision']==prior['scene_revision']
    branch=next(b for b in prep['checks'] if b['seed_kind']=='retained_round8_planned_approach_endpoint')
    assert branch['passed'] and len(branch['samples'])==39
    endpoints=sorted([s for s in branch['samples'] if s['fraction']==1.0],key=lambda s:s['segment_index'])
    goals=previous['phase_waypoints_world']['lift']+[previous['target_poses_world']['lift']]
    assert [s['segment_index'] for s in endpoints]==[0,1,2] and len(goals)==3
    validations=[]
    for s,goal in zip(endpoints,goals):
        assert s['passed'] and s['declared_lift_contract_passed'] and len(s['q_rad'])==7
        same=next(v for v in checked['samples'] if v['q_rad']==s['q_rad'])
        assert same['world_constraint_value']==same['self_constraint_value']==0
        actual=np.asarray(s['actual_tcp_world']);goal=np.asarray(goal)
        position=float(np.linalg.norm(actual[:3,3]-goal[:3,3]))
        angle=float(Rotation.from_matrix(actual[:3,:3]@goal[:3,:3].T).magnitude())
        assert position<=.001 and angle<=.01
        validations.append(dict(segment_index=s['segment_index'],q_rad=s['q_rad'],cpu_position_error_m=position,cpu_rotation_error_rad=angle,
            previously_measured_curobo_static_sample=deepcopy(same)))
    recipe=deepcopy(previous)
    recipe.update(retry_round=10,hold_orientation_phases=[],linear_motion_phases=[],
        planning_options_by_phase={'lift':{'enable_graph':False}},
        joint_space_target_q_by_phase={'lift':[s['q_rad'] for s in endpoints]},
        planning_path_policy=dict(kind='collision_checked_joint_goal_lift_corridor',previous_straight_line_gate_removed=True,
            reason_zh='保持原抓姿与三段抬升终点，使用已核验无碰撞的连续IK关节目标避免重新选取末端分支；真实返回路径仍须满足8mm/4度/-2mm抬升合同。'),
        description='保留第八轮布局和第九轮全部TCP目标、夹爪指令及抬升合同。lift三段改用公开plan_single_js固定已验证的关节终点，graph关闭；不注入CPU理想轨迹，不放宽真实FK门槛。')
    recipe['limitations']=list(recipe.get('limitations',[]))+[
        '已保存的39点CPU连续IK与78点cuRobo静态检查仅证明这些离散姿态通过当时模型检查；实际优化路径可能仍偏离。',
        '公开plan_single_js只固定每段关节终点，不注入CPU轨迹；完整返回路径及重定时路径必须另外通过lift合同，物理抓持尚待实际测试。']
    report=dict(kind='physical_retry_round10_mustard_reused_joint_goal_preparation',scene_id=SID,
        source_previous_recipe=previous_recipe_ref,source_previous_cpu_preparation=previous['evidence'],
        source_curobo_static_agreement=agreement_ref,source_previous_manifest=agreement['source_manifest'],
        previous_candidate_planning_result=prior['candidate_planning_result'],retained_scene_revision=prior['scene_revision'],
        retained_config=prep['retained_config'],retained_source_scene=prep['retained_source_scene'],source_script=ref(Path(__file__).relative_to(ROOT)),
        scene_settings_changed_this_round=False,retained_layout=True,selected_branch=branch['seed_kind'],
        selected_endpoints=validations,candidate_count=1,new_ik_calls=0,new_gpu_calls=0,physics_episodes=0,
        unchanged_fields=['target_poses_world','phase_waypoints_world','nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad','lift_path_validation','segment_deviation_limits_m'],
        scope='Extract existing, source-bound joint goals only. No new IK, GPU query/planning, physical episode or layout edit; future returned path and physical outcome remain unverified.')
    report_path=ROOT/'reports'/RUN/'mustard_joint_goal_preparation.json';recipe_path=ROOT/'configs/extensions/replicator/physical_retry_round10_mustard_recipes_20261029.json'
    if report_path.exists() or recipe_path.exists():raise FileExistsError('Frozen evidence already exists')
    for k in report['unchanged_fields']:assert recipe[k]==previous[k]
    report_path.parent.mkdir(parents=True,exist_ok=True)
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    recipe['evidence']=ref(report_path.relative_to(ROOT))
    recipe_path.write_text(json.dumps(dict(kind='physical_retry_round10_recipes',recipes={SID:recipe},evidence=recipe['evidence'],scene_settings_changed_this_round=False),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(recipe=ref(recipe_path.relative_to(ROOT)),evidence=recipe['evidence'],joint_goals=recipe['joint_space_target_q_by_phase']),ensure_ascii=False))
if __name__=='__main__':main()
