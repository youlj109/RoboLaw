#!/usr/bin/env python3
"""Diagnose only saved round10 optimizer outputs, using independent source FK."""
import json
from pathlib import Path
import sys
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import audit_physical_retry_round10 as audit


def diagnose(row):
    candidate=audit.bound_json(row['candidate_planning_result']);declared=candidate['recipe']
    phase=next(p for p in candidate['phases'] if p['phase']==candidate['failed_phase']);detail=phase.get('segments',[phase])[-1]
    audit.audit_free_path_policy(candidate,declared)
    checks=audit.audit_declared_segments(candidate,declared,row);prefix=checks['saved_path_source_fk_audit']
    available=detail.get('trajectory') is not None
    item=dict(scene_id=row['scene_id'],task_id=row['task_id'],variant_id=row['variant_id'],status='diagnosed',
        scene_revision=row['scene_revision'],candidate_planning_result=row['candidate_planning_result'],
        config={'path':row['config_path'],'sha256':row['config_sha256']},source_scene={'path':row['source_scene_path'],'sha256':row['source_scene_sha256']},
        failed_phase=phase['phase'],solver_status=phase['solver_status'],failed_segment_trajectory_available=available,
        failed_solver_record=dict(phase=phase['phase'],segment_index=detail.get('segment_index'),solver_status=detail['solver_status'],
            solver_success=detail['solver_success'],samples=detail.get('samples',0),trajectory_saved=available),
        partial_trajectory=candidate.get('artifacts',{}).get('partial_trajectory'),completed_prefix=prefix,segment_audit=checks,
        planning_path_policy=declared['planning_path_policy'],layout_changed=True,retained_scene_revision=True,scene_settings_changed_this_round=False,
        root_cause_confirmed=False,scene_infeasibility_established=False,physical_attempts=0,executed=False,
        frozen_joint_targets=declared['joint_space_target_q_by_phase'],frozen_cpu_screening_evidence=declared['evidence'],cpu_screening_reused_without_new_solver=True)
    facts=[f"沿用 r8 布局和第九轮目标位姿，唯一候选在 {phase['phase']} 返回 {phase['solver_status']}。",
        f"已保存成功前缀 {prefix['independent_fk_samples']} 点，源 USD 独立 FK 复核通过；没有启动新物理测试。",
        '固定关节目标只指定求解终点，实际路径仍来自 cuRobo；未把 CPU 理想插值路径输入为执行轨迹。']
    if not available:
        audit.require(detail['solver_success'] is False and detail.get('samples',0)==0,'No-path failure has fabricated solver samples')
        item.update(diagnostic_method='solver_failure_without_failed_segment_path',failed_path_fk_available=False,
            interpretation_zh='当前固定目标分支已通过源 FK 检查，但轨迹优化阶段没有返回可接受路径。端点可行不证明连接路径可行，不能据单次 TRAJOPT_FAIL 认定场景无解。',
            next_adjustment_zh='下轮先查看已保存优化诊断中约束出现的阶段和组件，区分连接路径与端点问题；只针对已定位因素选择一项初始化或分段修改，保留原场景和成功判据。')
        facts.append('失败段没有返回路径，因此没有为该失败段伪造 FK、碰撞位置或物理接触证据。')
    else:
        checked=next(s for s in checks['segments'] if s['phase']==phase['phase'] and s['segment_index']==detail['segment_index'])
        audit.require(checked.get('independent_fk_verified') is True and checked['trajectory']==detail['trajectory'],'Rejected path lacks independent FK binding')
        item.update(failed_segment_trajectory=detail['trajectory'],failed_path_fk_available=True,
            failed_joint_goal_endpoint_audit=checked.get('joint_goal_endpoint_audit'),planning_rejection_reason_confirmed=True)
        if phase['solver_status']=='LIFT_PATH_CONSTRAINT_LIMIT':
            gate=checked['lift_path_validation'];audit.require(gate['passed'] is False,'Lift rejection has no violated frozen gate')
            item.update(diagnostic_method='independent_source_fk_lift_corridor_gate',independent_lift_gate=gate,
                interpretation_zh='此次求解器成功返回路径且关节终点符合目标，但中途腕部姿态偏离冻结的抬升约束。独立 FK 确认了硬门槛拒绝原因，仍不能由此认定场景或所有路径不可行。',
                next_adjustment_zh='下轮保留8mm/4度/-2mm抬升门槛，针对中间腕部旋转选择显式姿态约束或更密的冻结关节路标；实际返回路径仍须逐样本检查，不以终点相同代替路径保持。')
            facts.append(f"拒绝段 {gate['source_samples']} 点独立 FK：最大横移 {gate['max_xy_deviation_m']*1000:.4f}mm，最大姿态偏差 {gate['max_orientation_deviation_deg']:.6f}度，最低Z偏移 {gate['min_z_offset_m']*1000:.4f}mm；门槛分别8mm、4度、-2mm。")
        elif phase['solver_status']=='JOINT_GOAL_ENDPOINT_LIMIT':
            gate=checked['joint_goal_endpoint_audit'];audit.require(gate['joint_goal_endpoint_gate_passed'] is False,'Endpoint rejection has no violated frozen gate')
            item.update(diagnostic_method='independent_source_fk_joint_goal_endpoint_gate',independent_joint_endpoint_gate=gate,
                interpretation_zh='求解器返回了路径，但实际末关节或末端 FK 未满足冻结终点容差，故未进行物理执行。',
                next_adjustment_zh='下轮围绕实际末关节误差与优化停止条件选择一项修复，保留原目标和终点验收阈值。')
        else:raise ValueError('Unknown saved-path rejection method: '+phase['solver_status'])
    item['observed_cpu_facts_zh']=facts
    audit.bound_json(declared['evidence'])
    return item


def main():
    path=ROOT/'configs/extensions/replicator/physical_retry_round10_20261029.json';binding=audit.file_binding(path);plan=audit.read(path)
    entries=[diagnose(row) for row in plan['entries'] if row['planning_status']=='failed']
    report=dict(kind='physical_retry_planning_failure_cpu_diagnostics',schema_version=2,retry_round=10,
        created_at=datetime.now(timezone.utc).isoformat(),source_plan=binding,source_plan_unchanged=audit.file_binding(path)==binding,
        total=len(entries),diagnosed=len(entries),complete=True,errors=[],entries=entries,layout_changed=False,layout_changed_from_original=True,
        retained_scene_revision=True,scene_settings_changed_this_round=False,physical_execution_started=False,kit_started=False,
        curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=0,
        method='Saved optimizer status and source-USD FK of all returned paths and declared joint goals; no new IK/GPU/physics.',
        limitations=['返回关节终点不约束中间轨迹。','无返回轨迹的失败段不声称已检查其路径碰撞。','本轮没有新布局或物理测试，原布局151/154，当前采用版本152/154。'],
        diagnostic_sources={str(p.relative_to(ROOT)):audit.digest(p) for p in [Path(__file__),Path(audit.__file__),Path(audit.numeric.__file__)]})
    audit.require(report['source_plan_unchanged'],'Frozen plan changed during diagnosis')
    output=ROOT/'reports/replicator_physical_retry_round10_20261029/planning_failure_diagnostics.json'
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(audit.file_binding(output)))

if __name__=='__main__':main()
