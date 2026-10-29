#!/usr/bin/env python3
"""Freeze existing CPU branch goals after read-only GPU retreat checks.

No new IK candidate or plan call. The round9 layout, poses and evidence remain
unchanged. ESDF raw center distance is explicitly distinguished from sphere
surface clearance; actual collision constraints are evaluated separately.
"""
from copy import deepcopy
from pathlib import Path
import fcntl,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.round10_static_collision_diagnostic import diagnose
from scripts.extensions.replicator.clarify_round9_gpu_esdf_scope import normalize_esdf
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
SID='u_shape_bowl_sink__human_default_mpfb'
OUT=ROOT/'reports/replicator_physical_retry_round10_20261029/human_joint_goal'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round10_human_recipes_20261029.json'
PREVIOUS=ROOT/'configs/extensions/replicator/physical_retry_round9_human_recipes_20261029.json'


def main():
    if CONFIG.exists() or OUT.exists():raise FileExistsError('Round10 human already frozen')
    OUT.mkdir(parents=True)
    prior=read(PREVIOUS)['recipes'][SID]
    manifests=ROOT/'exports/replicator_physical_retry_round9_20261029/manifest.json'
    row=next(x for x in read(manifests)['entries'] if x['scene_id']==SID)
    result=read(geo.bound(row['candidate_planning_result']))
    cpu_path=ROOT/'reports/replicator_physical_retry_round9_20261029/human_cpu/withdraw_direction_correction.json'
    cpu=read(cpu_path);selected=next(t for t in cpu['trials'] if t['declaration']['name']==cpu['selected_name'])
    samples=selected['ideal_samples'];assert len(samples)==22 and all(s['passed'] for s in samples)
    qs=np.array([s['ik_collision']['q_rad'] for s in samples]);targets=[np.array(s['target_world']) for s in samples]
    labels=[f'fixed_retreat_segment{s["segment"]}_fraction{s["fraction"]:g}' for s in samples]
    # Use the same diagnostic machinery, but the retreat has no payload attached
    # and its source gripper is fully open, exactly as the original phase.
    result=deepcopy(result);result['failed_phase']='retreat';result['planning_gripper_joint_rad']=0.
    # R9 planning stopped before retreat, so supply its immutable original mask.
    from scripts.extensions.replicator.prepare_round4_other_cpu import masks_for
    original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
    result['phases'].append(dict(phase='retreat',contact_masks=masks_for(original)['retreat']))
    lock=(ROOT/'reports/skill_trials/gpu.lock').open('a+');print('WAITING_GPU_LOCK',flush=True);fcntl.flock(lock,fcntl.LOCK_EX)
    checked=diagnose(row,result,qs,labels,targets,ref(cpu_path))
    checked=normalize_esdf(checked);checked.update(payload_attached=False,planning_gripper_joint_rad=0.,source_diagnostic_helper=ref(ROOT/'scripts/extensions/replicator/round10_static_collision_diagnostic.py'))
    assert checked['summaries']['world_constraint_nonzero_samples']==0
    assert checked['summaries']['self_constraint_nonzero_samples']==0
    assert checked['summaries']['max_fk_position_error_m']<=.001 and checked['summaries']['max_fk_rotation_error_rad']<=.01
    old_gpu_path=ROOT/'reports/replicator_physical_retry_round9_20261029/cpu_curobo_static_agreement.json'
    old_gpu=read(old_gpu_path);lower=normalize_esdf(next(x for x in old_gpu['entries'] if x['scene_id']==SID))
    lower_q=lower['samples'][0]['q_rad']
    assert lower['summaries']['world_constraint_nonzero_samples']==lower['summaries']['self_constraint_nonzero_samples']==0
    report=dict(kind='round10_human_existing_joint_goal_gpu_validation',scene_id=SID,source_script=ref(Path(__file__)),source_previous_recipe=ref(PREVIOUS),
        source_previous_manifest=ref(manifests),source_previous_planning=row['candidate_planning_result'],source_retreat_cpu=ref(cpu_path),source_lower_gpu_agreement=ref(old_gpu_path),
        nominal_destination=deepcopy(prior['destination_region_check']),retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),retained_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        lower_existing_gpu_check=lower,retreat_existing_cpu_samples_gpu_check=checked,
        selected_q_goals=dict(lower=dict(q_rad=lower_q,cpu_source=ref(ROOT/'reports/replicator_physical_retry_round9_20261029/human_cpu/candidates.json'),trial_index=4,field='lower_endpoint.q_rad'),
            retreat=[dict(q_rad=qs[i].tolist(),cpu_source=ref(cpu_path),selected_trial_name=cpu['selected_name'],sample_index=i) for i in (10,21)]),
        new_candidates_searched=0,new_ik_calls=0,new_planning_calls=0,physical_episodes=0,passed=True,
        scope='Same round9 frozen target poses and existing CPU q only. Explicit public joint-space goal mode for lower/retreat is a round10 planner policy; sampled collision-free q do not establish a trajectory or actual physical success.')
    path=OUT/'gpu_validated_existing_goals.json';save(path,report)
    r=deepcopy(prior);r.update(retry_round=10,evidence=ref(path),previous_frozen_recipe=ref(PREVIOUS),previous_candidate_planning_result=row['candidate_planning_result'],
        joint_space_target_q_by_phase=dict(lower=[lower_q],retreat=[qs[i].tolist() for i in (10,21)]),
        planning_options_by_phase={p:{'enable_graph':False} for p in ('lower','retreat')},hold_orientation_phases=[],linear_motion_phases=[],
        planning_path_policy=dict(kind='collision_checked_joint_goal_endpoint',previous_straight_line_gate_removed=True,
            reason_zh='沿用r8布局和r9全部TCP目标，只对lower/retreat调用公开plan_single_js并固定已由cuRobo FK/实际collision checker验证的源关节目标；不声称注入完整CPU轨迹，实际返回路径仍需碰撞规划与物理执行。'),
        source_joint_goal_evidence=ref(path),
        description='保持第九轮105度直立下放姿态和原全部目标；lower指定已GPU静态验证的无碰关节分支，retreat指定原两段CPU连续分支端点（22点已GPU静态复核）。采用显式joint-space目标接口，不再靠笛卡尔IK重新找到同一分支。布局、原物理判据和collision masks不变。')
    r['remaining_cpu_warnings']=['Static FK/collision checks do not establish the next joint-space optimized curve or physical success.','Lower transfer-to-goal path was not validated here; the planner still must solve and collision-check the full motion.']
    r['finalizer_source']=ref(Path(__file__))
    validate_recipe(r,original,read(geo.bound(dict(path=row['config_path'],sha256=row['config_sha256']))))
    save(CONFIG,dict(kind='physical_retry_round10_human_recipes',recipes={SID:r},evidence=ref(path),layout_changed=False))
    print(json.dumps(dict(recipe=ref(CONFIG),report=ref(path),retreat_samples=22,retreat_summary=checked['summaries']),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
