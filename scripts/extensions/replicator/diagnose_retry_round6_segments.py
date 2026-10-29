#!/usr/bin/env python3
"""Read-only round6 source-FK diagnostics of retained joint-space solver paths.

No new cuRobo calls, IK, Kit or physical episodes. Public joint-space goals
constrain the endpoint; the prior CPU path was never injected or executed.
"""
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import audit_physical_retry_round6 as audit
from scripts.extensions.replicator import diagnose_retry_round5_segments as prior_diagnostic

PLAN='configs/extensions/replicator/physical_retry_round6_20261029.json'
OUTPUT='reports/replicator_physical_retry_round6_20261029/planning_failure_diagnostics.json'


def measured_curve(kin,segment):
    reference=segment['trajectory']
    with np.load(audit.bound(reference['path'],reference['sha256']),allow_pickle=False) as data:
        q=data['q'].copy();saved=data['tcp_world'].copy()
    poses=np.array([kin.fk(v) for v in q])
    audit.require(np.allclose(poses,saved,atol=2e-6,rtol=0),'Recorded TCP differs from independent USD FK')
    start=np.array(segment['start_world']);target=np.array(segment['target_world'])
    q0=np.array(segment['start_q']);qgoal=np.array(segment['joint_space_goal_q'])
    xyz=poses[:,:3,3];a=start[:3,3];b=target[:3,3];delta=b-a;length2=float(delta@delta)
    projection=(xyz-a)@delta/length2 if length2>1e-20 else np.zeros(len(q))
    closest=a+np.clip(projection,0,1)[:,None]*delta
    residual=xyz-closest;deviations=np.linalg.norm(residual,axis=1);index=int(deviations.argmax())
    lower=np.minimum(q0,qgoal);upper=np.maximum(q0,qgoal)
    # Endpoint-interval excursion is not the same as violating robot limits.
    excursions=np.maximum(np.maximum(lower-q,q-upper),0)
    max_joint_index=int(np.unravel_index(excursions.argmax(),excursions.shape)[1])
    joint_peak_index=int(excursions[:,max_joint_index].argmax())
    limit_excess=np.maximum(np.maximum(kin.limits[:,0]-q,q-kin.limits[:,1]),0)
    joint_rows=[]
    for j in range(7):
        sample=int(excursions[:,j].argmax())
        joint_rows.append(dict(joint_name=f'panda_joint{j+1}',start_rad=float(q0[j]),goal_rad=float(qgoal[j]),
            path_min_rad=float(q[:,j].min()),path_max_rad=float(q[:,j].max()),
            maximum_endpoint_interval_excursion_rad=float(excursions[sample,j]),excursion_sample_index=sample))
    rotation=poses[:,:3,:3]@target[:3,:3].T
    angles=np.arccos(np.clip((np.trace(rotation,axis1=1,axis2=2)-1)/2,-1,1))
    # Reconstruct the nominal joint-linear initialization from this run's exact
    # start/goal, without invoking IK or the optimizer. This is a comparison,
    # not evidence that the returned path stayed on the nominal line.
    fractions=np.linspace(0,1,40);linear_q=q0[None]+fractions[:,None]*(qgoal-q0)[None]
    linear_poses=np.array([kin.fk(v) for v in linear_q])
    linear_deviation=audit.segment_deviation(linear_poses[:,:3,3],a,b)
    same_time_jointline=q0[None]+np.linspace(0,1,len(q))[:,None]*(qgoal-q0)[None]
    result=dict(trajectory=reference,independent_fk_verified=True,samples=len(q),
        start_tcp_xyz_m=a.tolist(),goal_tcp_xyz_m=b.tolist(),target_segment_length_m=float(np.linalg.norm(delta)),
        maximum_tcp_segment_deviation_m=float(deviations[index]),maximum_deviation_sample_index=index,
        maximum_deviation_time_s=float(index*segment['interpolation_dt']),
        tcp_xyz_at_maximum_deviation_m=xyz[index].tolist(),
        closest_declared_segment_point_at_maximum_m=closest[index].tolist(),
        tcp_residual_xyz_at_maximum_m=residual[index].tolist(),
        tcp_displacement_from_start_at_maximum_m=(xyz[index]-a).tolist(),
        joint_q_at_maximum_tcp_deviation_rad=q[index].tolist(),
        joint_endpoint_interval_excursion_at_maximum_tcp_deviation_rad=excursions[index].tolist(),
        minimum_segment_projection_fraction=float(projection.min()),maximum_segment_projection_fraction=float(projection.max()),
        actual_tcp_path_length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
        maximum_orientation_error_from_goal_rad=float(angles.max()),
        maximum_orientation_error_sample_index=int(angles.argmax()),
        final_goal_joint_max_abs_error_rad=float(np.max(np.abs(q[-1]-qgoal))),
        final_goal_tcp_position_error_m=float(np.linalg.norm(xyz[-1]-b)),
        maximum_joint_limit_excess_rad=float(limit_excess.max()),
        joint_excursions_relative_to_endpoint_intervals=joint_rows,
        largest_joint_endpoint_interval_excursion=dict(joint_name=f'panda_joint{max_joint_index+1}',
            value_rad=float(excursions[joint_peak_index,max_joint_index]),sample_index=joint_peak_index),
        maximum_joint_difference_from_same_time_linear_initialization_rad=float(np.max(np.abs(q-same_time_jointline))),
        nominal_joint_linear_comparison=dict(samples=40,source='Current retained start_q and frozen goal_q; independent source USD FK only',
            maximum_tcp_segment_deviation_m=float(linear_deviation),within_frozen_gate=bool(linear_deviation<=segment['deviation_limit_m']),
            injected_into_solver=False,physical_execution=False,
            scope='No new IK, no collision rescreen, no planner call; this nominal comparison is not the actual optimizer path.'))
    audit.require(np.isclose(result['maximum_tcp_segment_deviation_m'],segment['max_tcp_segment_deviation_m'],atol=1e-9,rtol=0),
                  'Measured gate deviation differs')
    return result


def diagnose(row):
    candidate=audit.bound_json(row['candidate_planning_result'])
    recipe_ref=row['adjustment']['recipe_source'];recipe=audit.bound_json(recipe_ref)['recipes'][row['scene_id']]
    audit.require(candidate['recipe']==recipe,'Frozen recipe differs')
    config=dict(path=row['config_path'],sha256=row['config_sha256']);scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256'])
    layout=audit.bound_json(config);audit.bound(scene['path'],scene['sha256'])
    evidence=dict(path=row['adjustment']['evidence_path'],sha256=row['adjustment']['evidence_sha256'])
    audit.bound(evidence['path'],evidence['sha256'])
    for path,digest in candidate['composition']['source_paths_sha256'].items():audit.bound(path,digest)
    for key in ('scene_id','task_id','variant_id'):audit.require(candidate[key]==row[key],'Scene identity differs')
    audit.require(candidate['status']=='failed' and candidate['executed'] is False and candidate['trajectory_complete'] is False
        and candidate['attempts']==1 and row['trajectory_path'] is None and row['trajectory_sha256'] is None,
        'Failed candidate unexpectedly executable')
    phase=candidate['phases'][-1];last=phase['segments'][-1]
    audit.require(candidate['failed_phase']==phase['phase'] and last['solver_status']=='TCP_SEGMENT_DEVIATION_LIMIT'
        and last['solver_success'] is True and last['deviation_gate_passed'] is False,'Expected retained gate-rejected solver path')
    kin=audit.SourceFK(layout);checked=audit.audit_declared_segments(candidate,recipe,row,kin=kin)
    prefix=prior_diagnostic.prefix_evidence(candidate,kin);curve=measured_curve(kin,last)
    extra=curve['largest_joint_endpoint_interval_excursion'];res=np.array(curve['tcp_residual_xyz_at_maximum_m'])*1000
    linear=curve['nominal_joint_linear_comparison']['maximum_tcp_segment_deviation_m']*1000
    facts=[f"公开MotionGen.plan_single_js在{phase['phase']}第{last['segment_index']+1}段单次返回solver_success=true；保存{last['samples']}样本，原5mm有限线段gate拒绝，后续段未调用。",
        f"源USD独立FK确认最大TCP偏差{curve['maximum_tcp_segment_deviation_m']*1000:.6f}mm，位于样本{curve['maximum_deviation_sample_index']}；相对最近声明线段点的世界XYZ偏移为[{res[0]:.3f}, {res[1]:.3f}, {res[2]:.3f}]mm。",
        f"实际末关节与冻结goal_q最大差{curve['final_goal_joint_max_abs_error_rad']:.9g}rad，TCP末点位置误差{curve['final_goal_tcp_position_error_m']*1000:.6f}mm；终点命中不等于中间路径满足限制。",
        f"相对各关节起末值区间，最大额外偏移发生于{extra['joint_name']}，为{extra['value_rad']:.6f}rad；这不是机器人硬关节限位超限，源限位最大超出量另记录为{curve['maximum_joint_limit_excess_rad']:.9g}rad。",
        f"由本轮相同start_q/goal_q重算40点名义关节直线，最大TCP偏差{linear:.6f}mm；该CPU直线没有注入cuRobo、更没有被物理执行，不能代表实际返回曲线。",
        f"独立核验此前成功前缀{prefix['samples']}样本；完整轨迹未生成，本景本轮零物理执行。"]
    next_change=('下一轮保留源几何已通过的避壳方向，用支持显式初始轨迹的公开局部TrajOpt接口输入整段连续IK关节序列，并对该低位斜移段同时加沿声明线段的位移/姿态代价；提前限制信赖域或关节偏离初始轨迹的幅度。只固定goal_q已经不足，仍须保留实际5mm gate。'
        if 'banana' in row['scene_id'] else
        '下一轮对首个10mm竖直下降段使用支持显式初始轨迹的公开局部TrajOpt接口，输入连续IK序列并同时约束姿态、固定XY及沿Z的有界进度；在初始轨迹附近优化，避免再次用仅固定终点的整段关节空间查询。保留实际5mm gate。'
        if 'mustard' in row['scene_id'] else
        '下一轮将最初短竖直上抬作为单独的局部轨迹优化段，使用显式连续IK初始序列并施加固定XY/姿态代价和有界竖直进度；达到净空后再开始远离人体的斜向段。不能仅换goal_q或提高单一软权重，仍保留实际5mm gate。')
    return dict(scene_id=row['scene_id'],task_id=row['task_id'],variant_id=row['variant_id'],
        candidate_planning_result=row['candidate_planning_result'],config=config,source_scene=scene,
        recipe_source=recipe_ref,cpu_preparation=evidence,previous_physical_result=row['previous_retry_result'],
        failed_phase=candidate['failed_phase'],solver_status=phase['solver_status'],status='diagnosed',
        diagnostic_method='independent_source_fk_segment_gate',root_cause_confirmed=False,scene_infeasibility_established=False,
        partial_trajectory=candidate['artifacts']['partial_trajectory'],segment_audit=checked,
        completed_prefix=prefix,retained_path_quantification=curve,source_robot_model=kin.source,
        failed_segment_trajectory_available=True,failed_segment_index=last['segment_index'],
        planning_options=last['planning_options'],planner_api=last['planner_api'],
        observed_cpu_facts_zh=facts,
        interpretation_zh='冻结关节目标与实际末点一致，当前失败表现为优化器中间曲线远离冻结短段，而非目标坐标/API传参错误。公开joint-space查询只固定终点分支，没有注入CPU连续IK路径，也未启用Cartesian姿态/位移代价。具体内部优化因素未确证，本轮未重跑；不能据此宣称场景无解、碰撞已发生或抓持失败。',
        next_adjustment_zh=next_change+' 本轮不实施该建议，不新增GPU/IK/物理尝试。',
        layout_changed=False,new_ik_attempts=0,physical_execution_started=False)


def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA for read-only CPU diagnosis')
    output=audit.local(OUTPUT)
    if output.exists():raise FileExistsError('Do not replace frozen diagnosis')
    plan_path=audit.local(PLAN);plan_ref=audit.file_binding(plan_path);plan=audit.read(plan_path)
    audit.require(plan['retry_round']==6 and plan['layout_changed'] is False and plan['success_predicate_changed'] is False,
        'Unexpected plan scope')
    audit.require(all(r['planning_status'] in ('generated','failed') for r in plan['entries']),'All candidates must be terminal')
    entries=[diagnose(row) for row in plan['entries'] if row['planning_status']=='failed']
    audit.require(audit.file_binding(plan_path)==plan_ref,'Plan changed during diagnosis')
    sources=(Path(__file__).resolve(),Path(audit.__file__).resolve(),Path(audit.numeric.__file__).resolve(),
             Path(audit.numeric.numeric.__file__).resolve(),Path(prior_diagnostic.__file__).resolve())
    report=dict(kind='physical_retry_planning_failure_cpu_diagnostics',schema_version=1,retry_round=6,
        source_plan=plan_ref,total=len(entries),complete=True,source_plan_unchanged=True,diagnosed=len(entries),errors=0,entries=entries,
        physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,
        cpu_ik_seed_attempts_per_scene=0,layout_changed=False,
        method='Independent source-USD FK of retained solver paths and prefixes; endpoint-joint interval excursions and same-start/goal nominal joint-line comparison. No new IK/planning/physics.',
        limitations=['Joint endpoint-interval excursion is not a robot hard-limit violation.',
            'The comparison CPU joint-line was neither injected into cuRobo nor physically executed.',
            'The actual saved cuRobo path, not ideal CPU geometry, decides the unchanged 5mm gate.',
            'No collision or physical outcome is inferred from these FK-only records; one failed candidate does not establish scene infeasibility.'],
        diagnostic_sources={str(p.relative_to(audit.ROOT)):audit.digest(p) for p in sources})
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:json.dump(report,stream,ensure_ascii=False,indent=2);stream.write('\n')
    print(json.dumps(dict(diagnosed=len(entries),report=audit.file_binding(output))))
    for e in entries:
        c=e['retained_path_quantification'];print(e['scene_id'],c['maximum_tcp_segment_deviation_m'],c['largest_joint_endpoint_interval_excursion'],c['final_goal_joint_max_abs_error_rad'])

if __name__=='__main__':main()
