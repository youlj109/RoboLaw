#!/usr/bin/env python3
"""Targeted withdrawal correction after fixed-layout yaw screening; CPU only."""
from copy import deepcopy
from pathlib import Path
import json, os, sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
ROOT=Path(__file__).resolve().parents[3]; sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round9_human_cpu import SID, OUT, CONFIG, MANIFEST, branches
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe

def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
    report_path=OUT/'withdraw_direction_correction.json'
    if report_path.exists(): raise FileExistsError(report_path)
    pack=read(CONFIG); recipe=deepcopy(pack['recipes'][SID]); initial_binding=ref(CONFIG)
    save(OUT/'initial_yaw_screen_recipe.json',pack)
    base=read(OUT/'candidates.json'); selected=base['trials'][base['selected_index']]
    row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
    previous=read(geo.bound(row['candidate_planning_result']))
    original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
    probe=geo.Probe(row,previous); opener=OpenGeometry(probe); masks=masks_for(original)
    lower=np.array(recipe['target_poses_world']['lower']); goal=np.array(recipe['final_payload_pose_world'])
    lower_q=np.array(selected['lower_endpoint']['q_rad']); authored=np.array(previous['start_q'])
    lift=lower.copy();lift[2,3]+=.10
    away=lower.copy();away[:3,3]+=[0.,.10,.25]
    reverse=np.array(previous['targets']['transfer'])
    declarations=[dict(name='vertical_then_away_from_human',poses=[lift.tolist(),away.tolist()]),
                  dict(name='vertical_then_reverse_to_prior_transfer',poses=[lift.tolist(),reverse.tolist()])]
    save(OUT/'withdraw_direction_declarations.json',dict(source_script=ref(Path(__file__)),source_yaw_screen=ref(OUT/'candidates.json'),initial_recipe=initial_binding,
        prior_collision='panda_panda_link3 versus human clothing for all four original retreat endpoint seeds',
        fixed_lower_yaw_deg=105.,maximum_directed_withdraw_corrections=2,candidates=declarations))
    trials=[]; chosen=None
    for declaration in declarations:
        q=lower_q.copy();start=lower.copy();samples=[];endpoints=[]
        passed=True
        for segment,end_list in enumerate(declaration['poses']):
            end=np.array(end_list)
            endpoint=branches(probe,recipe,end,'retreat',q,masks,authored); endpoints.append(endpoint)
            rotation=Slerp([0.,1.],Rotation.from_matrix([start[:3,:3],end[:3,:3]]))
            for fraction in np.linspace(0.,1.,11):
                tcp=np.eye(4);tcp[:3,:3]=rotation(float(fraction)).as_matrix();tcp[:3,3]=(1-fraction)*start[:3,3]+fraction*end[:3,3]
                check=pose_check(probe,tcp,q,'retreat',recipe,masks)
                source=opener.check(tcp,goal,angles=(0.,))
                sample_passed=bool(check['passed'] and source['fully_open_fingers_clear'] and source['fully_open_gripper_hulls_clear'] and source['all_sampled_gripper_world_clear'])
                samples.append(dict(segment=segment,fraction=float(fraction),target_world=tcp.tolist(),ik_collision=check,opened_gripper_source=source,passed=sample_passed))
                if 'q_rad' in check:q=np.array(check['q_rad'])
                if not sample_passed: passed=False
            start=end
        passed=passed and all(x['passed'] for x in endpoints)
        trial=dict(declaration=declaration,passed=passed,endpoints=endpoints,ideal_samples=samples)
        trials.append(trial)
        print(json.dumps(dict(name=declaration['name'],passed=passed,endpoint_passed=[e['passed'] for e in endpoints],sample_passed=sum(s['passed'] for s in samples),total_samples=len(samples),failed=[dict(segment=s['segment'],fraction=s['fraction'],collision=s['ik_collision'].get('collision'),source=s['opened_gripper_source']['samples'][0]) for s in samples if not s['passed']]),ensure_ascii=False),flush=True)
        if passed:chosen=trial;break
    report=dict(kind='round9_human_targeted_withdraw_direction_correction',scene_id=SID,source_initial_recipe=initial_binding,initial_recipe_copy=ref(OUT/'initial_yaw_screen_recipe.json'),source_yaw_screen=ref(OUT/'candidates.json'),source_script=ref(Path(__file__)),declarations=ref(OUT/'withdraw_direction_declarations.json'),trials=trials,selected_name=chosen['declaration']['name'] if chosen else None,passed=chosen is not None,
        scope='Finite ideal TCP samples with continuous prior-IK seed and full opened source gripper; no actual cuRobo curve, physics, or continuous collision guarantee.',curobo_calls=0,physical_episodes=0,layout_changed=False)
    save(report_path,report)
    if chosen:
        recipe['target_poses_world']['retreat']=chosen['declaration']['poses'][-1]
        recipe['phase_waypoints_world']['retreat']=chosen['declaration']['poses'][:-1]
        recipe.update(evidence=ref(report_path),cpu_screen_passed=True,remaining_cpu_warnings=[],ideal_segment_samples_passed=True,withdraw_direction_correction=ref(report_path))
        recipe['description']+=' 退出先垂直升100mm再离开，定向CPU复核已逐点检查全开夹爪源几何及机械臂，但不等于实际规划路径或物理成功。'
        validate_recipe(recipe,original,probe.layout)
        save(CONFIG,dict(kind='physical_retry_round9_human_recipes',recipes={SID:recipe},evidence=ref(report_path),layout_changed=False))
        print(json.dumps(dict(final_config=ref(CONFIG),evidence=ref(report_path)),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
