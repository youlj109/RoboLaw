#!/usr/bin/env python3
"""Diagnose retained round5 planning evidence with source FK only.

No cuRobo, IK search, Kit or physical execution is started. A solver failure
without a returned path is not treated as a measured rejected trajectory.
"""
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator import audit_physical_retry_round4 as audit

PLAN = 'configs/extensions/replicator/physical_retry_round5_20261029.json'
OUTPUT = 'reports/replicator_physical_retry_round5_20261029/planning_failure_diagnostics.json'


def prefix_evidence(candidate, kin):
    """Independently check only the completed phases actually saved in the NPZ."""
    reference = candidate['artifacts']['partial_trajectory']
    path = audit.bound(reference['path'], reference['sha256'])
    with np.load(path, allow_pickle=False) as data:
        q, saved, labels = data['q'].copy(), data['tcp_world'].copy(), data['phase'].astype(str)
    phases = candidate['phases'][:-1]
    expected = np.concatenate([np.repeat(p['phase'], p['samples']) for p in phases])
    audit.require(all(p['status'] == 'generated' and p['solver_success'] is True for p in phases),
                  'Prefix includes a failed phase')
    audit.require(q.shape == (len(expected), 7) and np.isfinite(q).all()
                  and np.array_equal(labels, expected), 'Partial path phases/samples differ')
    audit.require(candidate['failed_phase'] not in set(labels), 'Failed phase entered the executable prefix')
    computed = np.asarray([kin.fk(value) for value in q])
    audit.require(saved.shape == computed.shape and np.isfinite(saved).all()
                  and np.allclose(saved, computed, atol=2e-6, rtol=0), 'Prefix FK differs from source USD')
    audit.require(np.allclose(q[0], candidate['start_q'], atol=1e-4, rtol=0), 'Prefix initial q changed')
    rows = []
    for phase in phases:
        indices = np.flatnonzero(labels == phase['phase'])
        target = np.asarray(candidate['targets'][phase['phase']])
        rows.append(dict(phase=phase['phase'], samples=len(indices),
                         final_q_rad=q[indices[-1]].tolist(),
                         final_tcp_world=computed[indices[-1]].tolist(),
                         goal_position_error_m=float(np.linalg.norm(computed[indices[-1], :3, 3] - target[:3, 3]))))
    companion = candidate['artifacts'].get('partial_trajectory_json')
    if companion:
        audit.bound(companion['path'], companion['sha256'])
    return dict(trajectory=reference, trajectory_json=companion, samples=len(q), phases=rows,
                independent_source_fk_verified=True,
                maximum_saved_fk_matrix_error=float(np.abs(saved-computed).max()),
                failed_phase_included=False, complete_executable_trajectory=False,
                scope='Saved successful planning prefix only; no collision/IK/physical-success assertion.')


def diagnose(row):
    candidate = audit.bound_json(row['candidate_planning_result'])
    recipe_ref = row['adjustment']['recipe_source']
    recipe = audit.bound_json(recipe_ref)['recipes'][row['scene_id']]
    audit.require(candidate['recipe'] == recipe, 'Candidate differs from frozen recipe')
    config = dict(path=row['config_path'], sha256=row['config_sha256'])
    scene = dict(path=row['source_scene_path'], sha256=row['source_scene_sha256'])
    layout = audit.bound_json(config)
    audit.bound(scene['path'], scene['sha256'])
    preparation = dict(path=row['adjustment']['evidence_path'], sha256=row['adjustment']['evidence_sha256'])
    audit.bound(preparation['path'], preparation['sha256'])
    audit.bound(row['result_path'], row['result_sha256'])
    audit.bound(row['previous_retry_result']['path'], row['previous_retry_result']['sha256'])
    for path, digest in candidate['composition']['source_paths_sha256'].items():
        audit.bound(path, digest)
    for key in ('scene_id', 'task_id', 'variant_id'):
        audit.require(candidate[key] == row[key], 'Candidate identity differs')
    audit.require(candidate['status'] == 'failed' and candidate['trajectory_complete'] is False
                  and candidate['executed'] is False and candidate['attempts'] == 1
                  and row['trajectory_path'] is None and row['trajectory_sha256'] is None,
                  'Planning failure has an executable/physical result')
    phase = candidate['phases'][-1]
    audit.require(phase['phase'] == candidate['failed_phase'] and phase['status'] == 'failed',
                  'Failed phase differs from candidate')
    kin = audit.SourceFK(layout)
    checked = audit.audit_declared_segments(candidate, recipe, row, kin=kin)
    prefix = prefix_evidence(candidate, kin)
    last = phase['segments'][-1]
    options = recipe.get('planning_options_by_phase', {}).get(phase['phase'])
    audit.require(last.get('planning_options') == phase.get('planning_options') == options,
                  'Failed phase planner options differ from frozen recipe')
    audit.require(last.get('planner_seed_reset') == 'all_including_finetune'
                  and last.get('graph_enabled') is False and last.get('graph_fallback_enabled') is False,
                  'Failed phase seed/graph record differs')
    weights = [20., 20., 20., 20., 20., 0.] if phase['vertical_motion_cost_requested'] else [20., 20., 20., 0., 0., 0.]
    audit.require(last['pose_metric_hold_weights'] == weights, 'Failed phase metric weights differ')
    entry = {key: row[key] for key in ('scene_id', 'task_id', 'variant_id', 'candidate_planning_result')}
    entry.update(config=config, source_scene=scene, recipe_source=recipe_ref,
                 cpu_preparation=preparation, previous_physical_result=row['previous_retry_result'],
                 failed_phase=candidate['failed_phase'], solver_status=phase['solver_status'], status='diagnosed',
                 root_cause_confirmed=False, scene_infeasibility_established=False,
                 partial_trajectory=candidate['artifacts']['partial_trajectory'],
                 completed_prefix=prefix, segment_audit=checked,
                 source_robot_model=kin.source, planning_options=options,
                 failed_segment_index=last['segment_index'],
                 failed_segment_trajectory_available=bool(last.get('trajectory')),
                 layout_changed=False, new_ik_attempts=0, physical_execution_started=False)
    complete_names = '、'.join(p['phase'] for p in prefix['phases'])
    common = f"仅保存此前成功的{complete_names}前缀，共{prefix['samples']}个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。"
    if phase['solver_status'] == 'TCP_SEGMENT_DEVIATION_LIMIT':
        rejected = checked['segments'][-1]
        audit.require(last['solver_success'] is True and rejected['deviation_gate_passed'] is False,
                      'Missing independently verified rejected segment')
        deviation, limit = rejected['max_tcp_segment_deviation_m']*1000, rejected['deviation_limit_m']*1000
        entry.update(diagnostic_method='independent_source_fk_segment_gate',
            observed_cpu_facts_zh=[
                f"cuRobo在{phase['phase']}第{last['segment_index']+1}段返回solver_success=true，保存{last['samples']}个关节样本。",
                f"独立原机器人USD FK重算最大TCP有限线段偏差{deviation:.6f}mm，超过冻结{limit:.3f}mm；保留被拒路径，后续段没有调用。",
                common, '本轮没有完整可执行轨迹，也没有该景的新物理结果。'],
            interpretation_zh='这一次候选在首个10mm上抬段未通过路径限制；权重20及禁用graph没有保证TCP直线。该结果不是物理抓持失败，也未证明场景或IK无解。后续下放未规划，旧抓姿的测量补偿参考不能当作新抓姿已经验证的下放证据。',
            next_adjustment_zh='下一轮把首个竖直10mm抬升明确单独设为固定XY的短段约束，后续远离人体的斜向段保留姿态约束；预先冻结连续关节种子及5mm验收界限。新抓姿的下放补偿需重新绑定适用的attachment依据；本轮不追加规划或物理测试。')
    else:
        audit.require(phase['solver_status'] == 'MotionGenStatus.FINETUNE_TRAJOPT_FAIL'
                      and last['solver_success'] is False and last['status'] == 'failed'
                      and last['samples'] == 0 and not last.get('trajectory')
                      and not phase.get('trajectory'), 'Unsupported solver-failure evidence')
        failed_record = dict(phase=phase['phase'], segment_index=last['segment_index'],
                             solver_status=last['solver_status'], solver_success=False,
                             samples=0, trajectory_saved=False)
        entry.update(diagnostic_method='solver_failure_without_failed_segment_path',
            failed_solver_record=failed_record,
            observed_cpu_facts_zh=[
                f"{phase['phase']}第{last['segment_index']+1}段唯一一次调用返回MotionGenStatus.FINETUNE_TRAJOPT_FAIL、solver_success=false、samples=0；没有返回或保存该失败段轨迹。",
                '冻结选项为constraint_weight=20、enable_graph=false；记录完整seed重置且无graph回退。',
                common, '没有对不存在的失败段计算FK偏差、碰撞或IK诊断；本轮没有该景的新物理结果。'],
            interpretation_zh='当前单候选的轨迹优化精调未成功。已有前缀只证明之前阶段生成了路径，不能说明失败段的碰撞位置、反向超调、IK不可达或全局不可行；仅凭状态也不能确定是强约束、关节分支还是数值问题。',
            next_adjustment_zh=(
                '下一轮保留远离外壳的方向和5mm gate，对首个低位斜移段预先冻结从approach末关节状态连续展开的关节种子，再单次测试；把姿态权重作为提前声明的优化参数，不能在失败后加次数或放宽gate。' if 'banana' in row['scene_id'] else
                '下一轮联合调整预抓关节分支与首个10mm下降段的种子，使接近从前一段末关节状态连续开始；预先固定姿态/XY权重及5mm gate后再单次测试，不把本次精调失败解释为接触或抓持失败。'))
    return entry


def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '':
        raise RuntimeError('Hide CUDA for CPU-only recorded-evidence diagnosis')
    output = audit.local(OUTPUT)
    if output.exists():
        raise FileExistsError('Preserve existing frozen diagnosis: '+str(output))
    plan_path = audit.local(PLAN)
    plan_ref = audit.file_binding(plan_path)
    plan = audit.read(plan_path)
    audit.require(plan['retry_round'] == 5 and plan['layout_changed'] is False
                  and plan['success_predicate_changed'] is False, 'Unexpected plan scope')
    selected = [r for r in plan['entries'] if r['planning_status'] == 'failed']
    entries = [diagnose(row) for row in selected]
    audit.require(audit.file_binding(plan_path) == plan_ref, 'Plan changed during diagnosis')
    sources = (Path(__file__).resolve(), Path(audit.__file__).resolve(), Path(audit.numeric.__file__).resolve())
    report = dict(kind='physical_retry_planning_failure_cpu_diagnostics', schema_version=1, retry_round=5,
                  source_plan=plan_ref, total=len(entries), complete=True, source_plan_unchanged=True,
                  diagnosed=len(entries), errors=0, entries=entries,
                  physical_execution_started=False, kit_started=False, curobo_planning_calls=0,
                  new_target_candidates=0, cpu_ik_seed_attempts_per_scene=0, layout_changed=False,
                  method='Retained solver records and independently recomputed source-USD FK for saved prefixes/rejected segments only. No new IK or planning.',
                  limitations=['No failed-segment path exists for FINETUNE_TRAJOPT_FAIL; no path deviation/collision inference is made for those segments.',
                               'Successful planning prefixes are incomplete and were not executed in physics.',
                               'The frozen 5mm gate is a planning acceptance constraint, not a relaxed original task predicate.',
                               'One failed candidate does not establish scene infeasibility.'],
                  diagnostic_sources={str(p.relative_to(audit.ROOT)): audit.digest(p) for p in sources})
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(dict(diagnosed=len(entries), report=audit.file_binding(output)), ensure_ascii=False))


if __name__ == '__main__':
    main()
