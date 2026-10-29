#!/usr/bin/env python3
"""Retain round10 goals and select explicit public JS graph initialization.

Only SDK source inspection and JSON preparation occur here. No GPU, IK,
trajectory planning or physical execution is invoked.
"""
from copy import deepcopy
from pathlib import Path
import hashlib,importlib.util,json,os,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref
SID='u_shape_bowl_sink__human_default_mpfb'
OUT=ROOT/'reports/replicator_physical_retry_round11_20261029/human_graph_seed'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round11_human_recipes_20261029.json'
PREVIOUS=ROOT/'configs/extensions/replicator/physical_retry_round10_human_recipes_20261029.json'

def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
    if OUT.exists() or CONFIG.exists():raise FileExistsError('Round11 human already prepared')
    OUT.mkdir(parents=True)
    prior=read(PREVIOUS)['recipes'][SID]
    manifest=ROOT/'exports/replicator_physical_retry_round10_20261029/manifest.json'
    row=next(x for x in read(manifest)['entries'] if x['scene_id']==SID)
    candidate=read(geo.bound(row['candidate_planning_result']))
    sdk=Path(importlib.util.find_spec('curobo').origin).parent/'wrap/reacher/motion_gen.py'
    text=sdk.read_text();lines=text.splitlines()
    ranges=[(2059,2150),(3611,3699)]
    excerpts=[dict(first_line=a,last_line=b,source='\n'.join(lines[a-1:b])) for a,b in ranges]
    assert 'for n in range(plan_config.max_attempts)' in excerpts[0]['source']
    assert 'if plan_config.enable_graph:' in excerpts[1]['source']
    assert 'graph_result = self.graph_search' in excerpts[1]['source']
    assert excerpts[1]['source'].index('graph_result = self.graph_search')<excerpts[1]['source'].index('# do trajopt:')
    api_path=OUT/'installed_js_graph_api.json'
    save(api_path,dict(kind='installed_curobo_public_js_graph_source_review',installed_path=str(sdk),installed_sha256=hashlib.sha256(sdk.read_bytes()).hexdigest(),excerpts=excerpts,
        conclusion='enable_graph=True is tested inside the first _plan_js_from_solve_state call, before trajectory optimization; max_attempts=1 does not defer this search. Successful graph paths seed trajopt. With need_graph_success=False, failed graph search may still be followed by ordinary trajectory optimization.',
        actual_planner_calls=0,source_script=ref(Path(__file__))))
    partial=candidate['artifacts']['partial_trajectory']
    with np.load(geo.bound(partial),allow_pickle=False) as z:
        indices=np.flatnonzero(z['phase']=='transfer');qstart=z['q'][indices[-1]].copy()
    qgoal=np.asarray(prior['joint_space_target_q_by_phase']['lower'][0])
    lower=next(p for p in candidate['phases'] if p['phase']=='lower')
    report=dict(kind='round11_human_retained_joint_goals_graph_seed_preparation',scene_id=SID,
        source_script=ref(Path(__file__)),source_previous_recipe=ref(PREVIOUS),source_previous_manifest=ref(manifest),source_previous_planning=row['candidate_planning_result'],
        source_previous_partial_trajectory=partial,source_verified_q_goals=prior['source_joint_goal_evidence'],source_installed_js_api=ref(api_path),
        retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),retained_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        scene_revision=row['scene_revision'],prior_lower_solver_status=lower['solver_status'],prior_actual_transfer_planned_q_rad=qstart.tolist(),fixed_lower_goal_q_rad=qgoal.tolist(),
        maximum_joint_delta_from_prior_transfer_rad=float(np.max(np.abs(qgoal-qstart))),
        change={'lower.enable_graph':{'old':False,'new':True},'retreat.enable_graph':{'old':False,'new':True}},
        unchanged=['target_poses_world','phase_waypoints_world','joint_space_target_q_by_phase','nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad','config_path','config_sha256','source_scene_path','source_scene_sha256'],
        candidate_count=1,new_cpu_candidates=0,new_ik_calls=0,new_gpu_planning_calls=0,new_physical_episodes=0,
        graph_success_required=False,
        limitations=['Graph search is enabled as a seed source, not asserted to succeed.','Collision-free endpoints do not imply a connected path; the actual returned path and native episode remain untested.','The retained graph-free round10 failure is immutable and is not relabeled successful.'])
    report_path=OUT/'preparation.json';save(report_path,report)
    r=deepcopy(prior);r.update(retry_round=11,evidence=ref(report_path),previous_frozen_recipe=ref(PREVIOUS),previous_candidate_planning_result=row['candidate_planning_result'],
        planning_options_by_phase={p:{'enable_graph':True} for p in ('lower','retreat')},
        planning_path_policy=dict(kind='collision_checked_graph_joint_goals',previous_straight_line_gate_removed=True,
            reason_zh='保持第十轮全部TCP和已验证关节目标，仅在公开plan_single_js调用的第一轮启用图搜索，给轨迹优化提供可行路径种子；不强制图成功，不把图开关等同于实际避障路径或物理成功。'),
        description='沿用第十轮105度直立下放、退出两端点及全部精确关节目标。lower/retreat在单次plan_single_js中先启用图搜索，再进行原碰撞约束下的轨迹优化；布局、抓持参数、原成功判据均不变。',
        finalizer_source=ref(Path(__file__)))
    for key in report['unchanged']:assert r[key]==prior[key],key
    save(CONFIG,dict(kind='physical_retry_round11_human_recipes',recipes={SID:r},evidence=ref(report_path),layout_changed=False))
    print(json.dumps(dict(recipe=ref(CONFIG),report=ref(report_path),api_evidence=ref(api_path)),ensure_ascii=False))

if __name__=='__main__':main()
