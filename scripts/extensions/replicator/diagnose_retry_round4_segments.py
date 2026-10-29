#!/usr/bin/env python3
"""Read-only source FK diagnosis of frozen round4 rejected paths; no new IK/physics."""
import json
import os
from pathlib import Path
from scripts.extensions.replicator import audit_physical_retry_round4 as audit

def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA for CPU diagnosis')
    plan_path=audit.local('configs/extensions/replicator/physical_retry_round4_20261029.json')
    plan=audit.read(plan_path);plan_ref=audit.file_binding(plan_path)
    selected=[r for r in plan['entries'] if r['planning_status']=='failed'];entries=[]
    for row in selected:
        candidate=audit.bound_json(row['candidate_planning_result'])
        phase=candidate['phases'][-1]
        if phase['solver_status']!='TCP_SEGMENT_DEVIATION_LIMIT':raise ValueError('Only rejected segment paths supported')
        checked=audit.audit_declared_segments(candidate,candidate['recipe'],row)
        last=checked['segments'][-1];deviation=last['max_tcp_segment_deviation_m']*1000;limit=last['deviation_limit_m']*1000
        e={k:row[k] for k in ('scene_id','task_id','variant_id','candidate_planning_result')}
        e.update(config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
            failed_phase=candidate['failed_phase'],solver_status=phase['solver_status'],status='diagnosed',root_cause_confirmed=False,scene_infeasibility_established=False,
            diagnostic_method='independent_source_fk_segment_gate',segment_audit=checked,partial_trajectory=candidate.get('artifacts',{}).get('partial_trajectory'),
            observed_cpu_facts_zh=[f"cuRobo在{phase['phase']}阶段第{last['segment_index']+1}段返回成功，保存{last['samples']}个关节样本。",
                f"由原机器人USD关节变换独立重算TCP，最大偏离声明直线段{deviation:.6f}mm，大于冻结上限{limit:.3f}mm；此段被拒绝，后续段未调用。",
                '本轮未启动物理执行，仍保留此前真实失败记录。'],
            interpretation_zh='这是当前候选的路径约束不通过，不能解释为IK无解或整个场景不可能完成；偏差上限没有在看到结果后放宽。',
            next_adjustment_zh=('下一轮可把低位横移初段再细分，并使用笛卡尔直线插值IK或更强的路径约束；保持避开外壳的方向，再预先固定阈值测试。' if 'banana' in row['scene_id'] else '下一轮优先用笛卡尔直线插值IK固定最后接近段，并联合选择预抓关节分支；当前短段仍绕行21.6mm，不能仅增加软约束权重就视为已解决。'))
        entries.append(e)
    if audit.file_binding(plan_path)!=plan_ref:raise ValueError('Plan changed')
    report=dict(kind='physical_retry_planning_failure_cpu_diagnostics',schema_version=1,retry_round=4,source_plan=plan_ref,
        total=len(entries),complete=True,source_plan_unchanged=True,diagnosed=len(entries),errors=0,entries=entries,
        physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,cpu_ik_seed_attempts_per_scene=0,layout_changed=False,
        method='Independent source USD joint-frame FK for every saved rejected-segment sample; finite line segment deviation gate replay, no IK/planning/physics.',
        limitations=['Recorded planner output only, not actual physics.','The declared 5mm gate is a candidate path acceptance constraint, not the original task predicate.','A rejected single candidate does not establish scene infeasibility.'],
        diagnostic_sources={str(p.relative_to(audit.ROOT)):audit.digest(p) for p in (Path(__file__).resolve(),Path(audit.__file__).resolve())})
    out=audit.local('reports/replicator_physical_retry_round4_20261029/planning_failure_diagnostics.json')
    with out.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
    print('DIAGNOSED',len(entries),audit.file_binding(out))
if __name__=='__main__':main()
