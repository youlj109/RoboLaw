#!/usr/bin/env python3
"""Freeze the next solver policy from recorded rounds9/10; zero new solves."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
SID='u_shape_mustard_bowl__robot_sharp_equipment_housing'
RUN='replicator_physical_retry_round11_20261029'

def ref(path):
    p=ROOT/path
    return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def read(binding):
    assert ref(binding['path'])==binding,'Changed evidence: '+binding['path']
    return json.loads((ROOT/binding['path']).read_text())
def main():
    oldref=ref('configs/extensions/replicator/physical_retry_round9_mustard_recipes_20261029.json');old=read(oldref)['recipes'][SID]
    manifest_ref=ref('exports/replicator_physical_retry_round10_20261029/manifest.json');manifest=read(manifest_ref)
    assert manifest['stage']=='complete' and manifest['retry_round']==10
    row=next(r for r in manifest['entries'] if r['scene_id']==SID)
    assert row['execution_status']=='planning_failed' and row['attempts']==0 and row['executed'] is False
    previous=read(row['candidate_planning_result']);phase=previous['phases'][-1];segment=phase['segments'][-1]
    assert previous['failed_phase']=='lift' and segment['solver_success'] is True and segment['solver_status']=='LIFT_PATH_CONSTRAINT_LIMIT'
    assert segment['joint_goal_endpoint_gate_passed'] is True and segment['lift_path_validation']['passed'] is False
    assert ref(segment['trajectory']['path'])==segment['trajectory']
    diagnostic_ref=ref('reports/replicator_physical_retry_round10_20261029/planning_failure_diagnostics.json')
    diag=next(e for e in read(diagnostic_ref)['entries'] if e['scene_id']==SID)
    assert diag['candidate_planning_result']==row['candidate_planning_result'] and diag['diagnostic_method']=='independent_source_fk_lift_corridor_gate'
    assert diag['failed_segment_trajectory']==segment['trajectory']
    agreement_ref=ref('reports/replicator_physical_retry_round9_20261029/cpu_curobo_static_agreement.json')
    agreement=next(e for e in read(agreement_ref)['entries'] if e['scene_id']==SID)
    assert agreement['source_cpu_report']==old['evidence'] and agreement['sample_count']==78
    assert agreement['summaries']['world_constraint_nonzero_samples']==agreement['summaries']['self_constraint_nonzero_samples']==0
    read(old['evidence'])
    recipe=deepcopy(old);recipe.update(retry_round=11,planning_options_by_phase={'lift':{'constraint_weight':1.0,'enable_graph':True}},
        description='保留第八轮布局、第九轮全部TCP目标与抬升硬合同。lift恢复笛卡尔姿态保持/竖直软代价，权重由20改为1并允许graph初始化；仅一次求解，实际FK仍须通过8mm/4度/-2mm门槛。')
    assert recipe['hold_orientation_phases']==recipe['linear_motion_phases']==['lift']
    assert not recipe.get('joint_space_target_q_by_phase') and recipe['segment_deviation_limits_m']=={}
    recipe['planning_path_policy']=dict(kind='collision_checked_lift_corridor',previous_straight_line_gate_removed=True,
        reason_zh='第十轮JS终点准确但中途旋转约21度，回到有姿态/竖直软代价的笛卡尔路径；降低软权重并启用graph以改善求解初始化，硬抬升合同不变。')
    report=dict(kind='physical_retry_round11_mustard_solver_policy_preparation',scene_id=SID,
        source_round9_recipe=oldref,source_round9_cpu_preparation=old['evidence'],source_previous_manifest=manifest_ref,
        previous_candidate=row['candidate_planning_result'],previous_rejected_trajectory=segment['trajectory'],
        source_planning_diagnostics=diagnostic_ref,previous_independent_lift_gate=diag['independent_lift_gate'],
        source_curobo_static_agreement=agreement_ref,retained_scene_revision=row['scene_revision'],
        config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        source_script=ref(Path(__file__).relative_to(ROOT)),scene_settings_changed_this_round=False,
        candidate_count=1,new_ik_calls=0,new_gpu_calls=0,physical_episodes=0,
        changes_from_round9={'retry_round':[9,11],'lift_constraint_weight':[20.,1.],'lift_enable_graph':[False,True]},
        unchanged_fields=['target_poses_world','phase_waypoints_world','nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad','lift_path_validation','hold_orientation_phases','linear_motion_phases','segment_deviation_limits_m'],
        inference_zh='已有理想采样及静态碰撞检查未发现相应点碰撞，但不证明连接路径一定可求。新策略只改变一次求解的软代价及初始化选项，须以后续真实规划与物理结果判定。')
    for key in report['unchanged_fields']:assert recipe[key]==old[key]
    out=ROOT/'reports'/RUN/'mustard_solver_policy_preparation.json';dest=ROOT/'configs/extensions/replicator/physical_retry_round11_mustard_recipes_20261029.json'
    if out.exists() or dest.exists():raise FileExistsError('Do not overwrite frozen recipe/evidence')
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    recipe['evidence']=ref(out.relative_to(ROOT))
    dest.write_text(json.dumps(dict(kind='physical_retry_round11_recipes',recipes={SID:recipe},evidence=recipe['evidence'],scene_settings_changed_this_round=False),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(recipe=ref(dest.relative_to(ROOT)),evidence=recipe['evidence']),ensure_ascii=False))
if __name__=='__main__':main()
