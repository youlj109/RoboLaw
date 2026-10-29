#!/usr/bin/env python3
"""Bind saved round9 solver failures and prefix FK; no new IK/planning/physics."""
import json
from pathlib import Path
import sys
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import audit_physical_retry_round9 as audit


def main():
    path=ROOT/'configs/extensions/replicator/physical_retry_round9_20261029.json'
    binding=audit.file_binding(path);plan=audit.read(path);entries=[]
    for row in plan['entries']:
        if row['planning_status']!='failed':continue
        candidate=audit.bound_json(row['candidate_planning_result'])
        declared=candidate['recipe'];phase=next(p for p in candidate['phases'] if p['phase']==candidate['failed_phase'])
        detail=phase.get('segments',[phase])[-1]
        audit.require(detail['solver_success'] is False and detail.get('samples',0)==0 and not detail.get('trajectory'),
                      'A saved rejected path requires a different diagnostic method')
        audit.audit_free_path_policy(candidate,declared)
        prefix=audit.audit_saved_free_path(candidate,row)
        human=row['variant_id']=='human_default_mpfb'
        facts=[f"本轮沿用 r8 布局的唯一候选在 {phase['phase']} 阶段返回 {phase['solver_status']}；失败阶段无保存轨迹。",
               f"此前保存的成功阶段共 {prefix['independent_fk_samples']} 个样本，已用源机器人 USD 独立重算 FK；不是失败阶段的轨迹。",
               ('芥末瓶仅声明 lift 的 8mm 横移/4度旋转/-2mm 下沉门槛，但求解器没有返回该段路径，门槛未有可测路径；不能写成硬门槛拒绝。' if not human else '保留碰撞检查的自由绕行规划，未恢复原先额外的全程 5mm 直线门槛；当前失败不是旧门槛拒绝。'),
               '完整规划未生成，物理尝试为 0；未增加目标候选或启动新的 CPU IK / cuRobo / 物理执行。']
        item=dict(scene_id=row['scene_id'],task_id=row['task_id'],variant_id=row['variant_id'],status='diagnosed',
            scene_revision=row['scene_revision'],candidate_planning_result=row['candidate_planning_result'],
            config={'path':row['config_path'],'sha256':row['config_sha256']},
            source_scene={'path':row['source_scene_path'],'sha256':row['source_scene_sha256']},
            diagnostic_method='solver_failure_without_failed_segment_path',failed_phase=phase['phase'],solver_status=phase['solver_status'],
            failed_solver_record=dict(phase=phase['phase'],segment_index=detail.get('segment_index'),solver_status=detail['solver_status'],
                                     solver_success=False,samples=0,trajectory_saved=False),
            failed_segment_trajectory_available=False,partial_trajectory=candidate.get('artifacts',{}).get('partial_trajectory'),
            completed_prefix=prefix,planning_path_policy=declared['planning_path_policy'],layout_changed=True,
            retained_scene_revision=True,scene_settings_changed_this_round=False,lift_path_validation_declared=declared.get('lift_path_validation'),
            failed_path_fk_available=False,
            observed_cpu_facts_zh=facts,root_cause_confirmed=False,scene_infeasibility_established=False,
            physical_attempts=0,executed=False,
            interpretation_zh=('人体场景在当前记录的目标阶段求解失败。已冻结 CPU 端点筛选仅为旁证，不能把不同关节分支或近似碰撞模型的提示当成失败路径实测，也不能据此证明场景无解。' if human else
                '首个抬升段的末轮轨迹优化失败，现有结果不提供该失败路径，无法从轨迹重算确认具体碰撞或优化原因；不能据此证明场景不可行。'),
            next_adjustment_zh=('下一轮优先围绕失败的放置端姿态、机器人与人体近邻关系选择一个明确候选，保留原任务目标与物理判据；本轮不追加求解或执行。' if human else
                '下一轮先核对实际优化器的碰撞模型、接触角和 lift 姿态代价，考虑降低软代价或允许 graph 绕行，同时保留实际 FK 抬升门槛。当前没有失败段路径，不能确认究竟是碰撞还是优化器局部失败。'))
        item['observed_cpu_facts_zh'].insert(1,'本次已生成的成功阶段：'+
            '、'.join(p['phase'] for p in candidate['phases'] if p['status']=='generated')+'。')
        if human:
            audit.bound_json(declared['evidence'])
            item['frozen_cpu_screening_evidence']=declared['evidence']
            item['cpu_screening_reused_without_new_solver']=True
        if human and (ROOT/'reports/replicator_physical_retry_round9_20261029/human_planning_failure_diagnostic.json').exists():
            side=ROOT/'reports/replicator_physical_retry_round9_20261029/human_planning_failure_diagnostic.json'
            secondary=audit.read(side)
            audit.require(secondary['candidate_planning_result']==row['candidate_planning_result'],'Human secondary evidence differs')
            item['bound_historical_cpu_interpretation']=audit.file_binding(side)
            item['frozen_cpu_screening_evidence']=declared['evidence']
            item['observed_cpu_facts_zh'].append('人体与机器人/名义载荷的碰撞提示来自规划前已冻结的有限 CPU 筛选，未在本次诊断新增 IK 搜索。')
        entries.append(item)
    report=dict(kind='physical_retry_planning_failure_cpu_diagnostics',schema_version=2,retry_round=9,
        created_at=datetime.now(timezone.utc).isoformat(),source_plan=binding,source_plan_unchanged=audit.file_binding(path)==binding,
        total=len(entries),diagnosed=len(entries),complete=True,errors=[],entries=entries,layout_changed=False,layout_changed_from_original=True,
        retained_scene_revision=True,scene_settings_changed_this_round=False,
        physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=0,
        method='Saved solver-state audit and independent source-USD FK of completed prefixes; no failed path, new IK, GPU or physics.',
        limitations=['失败阶段没有保存的轨迹，因此不声称已重算其碰撞或确认唯一根因。','成功前缀的 FK 复核不构成完整规划、物理成功或安全验收。',
                     '本轮没有新布局修订，继续采用 r8 config/source/revision；原布局151/154，当前采用版本152/154。'],
        diagnostic_sources={str(p.relative_to(ROOT)):audit.digest(p) for p in [Path(__file__),Path(audit.__file__),Path(audit.numeric.__file__)]})
    audit.require(report['source_plan_unchanged'],'Frozen plan changed during diagnosis')
    output=ROOT/'reports/replicator_physical_retry_round9_20261029/planning_failure_diagnostics.json'
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(audit.file_binding(output)))

if __name__=='__main__':main()
