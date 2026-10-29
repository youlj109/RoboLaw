#!/usr/bin/env python3
"""Bounded CPU continuation seed preparation; no cuRobo, Kit or physics."""
import json
import os
from pathlib import Path
from copy import deepcopy
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from kitchen_safety.extensions.replicator.physical_retry_round2 import PHASES

OUT=ROOT/'reports/replicator_physical_retry_round6_20261029'
REPORT=OUT/'other_continuation_probe.json'
MANIFEST=ROOT/'exports/replicator_physical_retry_round5_20261029/manifest.json'
SIDS=('l_island_banana_bowl__robot_sharp_equipment_housing','u_shape_mustard_bowl__robot_sharp_equipment_housing')
HORIZON=32

def continuation(probe,recipe,start_q,phases,masks):
    q=np.array(start_q);segments=[];failed=None
    for phase in phases:
        endpoints=[np.array(p) for p in recipe['phase_waypoints_world'].get(phase,[])]+[np.array(recipe['target_poses_world'][phase])]
        for index,end in enumerate(endpoints):
            start=probe.kin.fk(q);path=[q.copy()];poses=[start.copy()];checks=[]
            rotation=Slerp([0.,1.],Rotation.from_matrix([start[:3,:3],end[:3,:3]]))
            # Include the exact previously returned solver state, then continue
            # from each preceding CPU IK state, without random restart.
            for fraction in np.linspace(0.,1.,HORIZON):
                target=end.copy();target[:3,3]=(1-fraction)*start[:3,3]+fraction*end[:3,3];target[:3,:3]=rotation(fraction).as_matrix()
                if fraction==0:
                    probe.phase=phase;probe.carrying=phase in ('lift','transfer','lower');probe.grip=recipe['planning_gripper_joint_rad'] if probe.carrying else 0.
                    probe.attachment=np.array(recipe['nominal_attachment']);probe.masks=masks[phase]
                    collision=probe.check(q);score=len(collision['unique_world_pairs'])+len(collision['self_sphere_overlaps'])
                    check=dict(passed=score==0,score=score,q_rad=q.tolist(),position_error_m=0.,orientation_error_rad=0.,collision=dict(unique_world_pairs=collision['unique_world_pairs'],self_sphere_overlaps=collision['self_sphere_overlaps'],masked_objects=masks[phase]))
                else:
                    check=pose_check(probe,target,q,phase,recipe,masks)
                    if 'q_rad' in check:
                        new=np.array(check['q_rad']);check['step_max_joint_delta_rad']=float(np.max(np.abs(new-q)));q=new;path.append(q.copy());poses.append(probe.kin.fk(q))
                check['fraction']=float(fraction);checks.append(check)
                if not check['passed'] or check.get('step_max_joint_delta_rad',0)>.15:
                    failed=dict(phase=phase,segment_index=index,fraction=float(fraction));break
            item=dict(phase=phase,segment_index=index,start_world=start.tolist(),target_world=end.tolist(),q=np.asarray(path).tolist(),tcp_world=np.asarray(poses).tolist(),checks=checks,passed=failed is None)
            segments.append(item)
            if failed:return dict(passed=False,failed_at=failed,segments=segments)
    return dict(passed=True,failed_at=None,segments=segments)

def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA')
    if REPORT.exists():raise FileExistsError(REPORT)
    OUT.mkdir(parents=True,exist_ok=True)
    manifest=read(MANIFEST);records={r['scene_id']:r for r in manifest['entries']}
    report=dict(kind='round6_other_continuous_ik_probe',source_manifest=ref(MANIFEST),source_script=ref(Path(__file__)),source_geometry=ref(Path(geo.__file__)),horizon=HORIZON,max_candidates_per_scene=8,physics_launched=False,curobo_called=False,kit_started=False,layout_changed=False,entries=[])
    for sid in SIDS:
        row=records[sid];previous=read(geo.bound(row['candidate_planning_result']));recipe=deepcopy(previous['recipe']);masks=masks_for(read(ROOT/row['result_path']))
        probe=geo.Probe(row,dict(previous,failed_phase=previous['failed_phase']))
        first=previous['phases'][-1]['segments'][0];q=np.array(first['start_q'])
        phases=('lift',) if 'banana' in sid else ('approach','lift')
        result=continuation(probe,recipe,q,phases,masks)
        report['entries'].append(dict(scene_id=sid,previous_candidate=row['candidate_planning_result'],config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),candidate_count=1,source_start_q=q.tolist(),original_masks=masks,recipe=recipe,continuation=result))
        save(REPORT.with_suffix('.partial.json'),report)
        print(sid,'passed',result['passed'],'segments',len(result['segments']),'failure',result['failed_at'],flush=True)
        del probe
    save(REPORT,report);print(json.dumps(ref(REPORT)),flush=True)
def freeze():
    """Freeze one joint-goal recipe; continuous IK is evidence, not solver input."""
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA')
    config=ROOT/'configs/extensions/replicator/physical_retry_round6_other_recipes_20261029.json'
    final_report=OUT/'other_joint_goal_cpu_preparation.json'
    if config.exists() or final_report.exists():raise FileExistsError('Round6 inputs already frozen')
    probe_report=read(REPORT)
    legacy=OUT/'other_continuation_probe_source.py'
    if probe_report['source_script']['sha256']==ref(legacy)['sha256']:
        probe_report['source_script']=ref(legacy)
        probe_report['source_preservation_note']='Exact CPU probe source preserved before adding the freeze adapter; prior numerical results unchanged.'
        save(REPORT,probe_report)
    records={r['scene_id']:r for r in read(MANIFEST)['entries']}
    final=dict(kind='round6_other_joint_goal_cpu_preparation',source_manifest=ref(MANIFEST),source_continuation_probe=ref(REPORT),source_script=ref(Path(__file__)),entries=[],candidate_count_per_scene=1,physics_launched=False,curobo_called=False,kit_started=False,layout_changed=False,
        interface='Public MotionGen.plan_single_js fixed goal joint configuration per declared endpoint; CPU continuous path is not injected into cuRobo.',
        limitations='Finite CPU checks and joint-linear initialization only. Actual cuRobo FK must pass the unchanged 5mm finite-segment gate. No physical success or safety assertion.')
    recipes={}
    for source in probe_report['entries']:
        sid=source['scene_id'];row=records[sid];previous=read(geo.bound(row['candidate_planning_result']));recipe=deepcopy(source['recipe'])
        if source['previous_candidate']!=row['candidate_planning_result'] or not source['continuation']['passed']:raise ValueError('Probe source changed or continuation incomplete')
        probe=geo.Probe(row,dict(previous,failed_phase=previous['failed_phase']));masks=source['original_masks'];joint={};checks=[]
        for seg in source['continuation']['segments']:joint.setdefault(seg['phase'],[]).append(seg['q'][-1])
        prefix_ref=previous['artifacts']['partial_trajectory']
        with np.load(geo.bound(prefix_ref),allow_pickle=False) as saved:
            prefix_q=saved['q'].copy();prefix_phase=saved['phase'].astype(str)
        prefix_phases=[p for p in PHASES if p in set(prefix_phase)]
        for phase in prefix_phases:
            joint[phase]=[prefix_q[np.flatnonzero(prefix_phase==phase)[-1]].tolist()]
        q=np.array(previous['start_q']);all_valid=True
        for phase in PHASES:
            if phase not in joint:continue
            endposes=[np.array(t) for t in recipe.get('phase_waypoints_world',{}).get(phase,[])]+[np.array(recipe['target_poses_world'][phase])]
            for index,(goal,pose) in enumerate(zip(joint[phase],endposes)):
                goal=np.array(goal);actual=probe.kin.fk(goal)
                pe=float(np.linalg.norm(actual[:3,3]-pose[:3,3]));re=float(Rotation.from_matrix(actual[:3,:3]@pose[:3,:3].T).magnitude())
                if pe>.001 or re>.01:raise ValueError('Frozen joint goal differs from TCP goal')
                check=dict(phase=phase,segment_index=index,start_q=q.tolist(),goal_q=goal.tolist(),goal_position_error_m=pe,goal_rotation_error_rad=re)
                if phase not in prefix_phases:
                    # MotionGen initializes joint-space queries from a joint
                    # line. Check that nominal initial line without claiming
                    # the optimizer must return it.
                    samples=[];start=probe.kin.fk(q);delta=pose[:3,3]-start[:3,3];denom=float(delta@delta)
                    probe.phase=phase;probe.carrying=phase in ('lift','transfer','lower');probe.grip=recipe['planning_gripper_joint_rad'] if probe.carrying else 0.
                    probe.attachment=np.array(recipe['nominal_attachment']);probe.masks=masks[phase]
                    for fraction in np.linspace(0,1,40):
                        trial=(1-fraction)*q+fraction*goal;fk=probe.kin.fk(trial);coll=probe.check(trial)
                        t=float(np.clip((fk[:3,3]-start[:3,3])@delta/denom,0,1)) if denom>1e-20 else 0.
                        deviation=float(np.linalg.norm(fk[:3,3]-start[:3,3]-t*delta))
                        hit=bool(coll['unique_world_pairs'] or coll['self_sphere_overlaps'])
                        samples.append(dict(fraction=float(fraction),tcp_segment_deviation_m=deviation,world_pairs=coll['unique_world_pairs'],self_sphere_overlaps=coll['self_sphere_overlaps'],passed=not hit and deviation<=.005))
                    passed=all(x['passed'] for x in samples);all_valid &= passed
                    check.update(joint_linear_seed_check_passed=passed,samples=samples,max_tcp_segment_deviation_m=max(x['tcp_segment_deviation_m'] for x in samples))
                else:check.update(scope='Fixed joint endpoint from prior successful saved planning prefix; unconstrained transit still requires new cuRobo planning.')
                checks.append(check);q=goal.copy()
        for key in ('evidence','cpu_screen_passed','selected_round5_candidate','selected_round4_candidate','cpu_preparation_evidence','tcp_constraint_diagnosis'):
            recipe.pop(key,None)
        constrained=('lift',) if 'banana' in sid else ('approach','lift')
        recipe['joint_space_target_q_by_phase']=joint
        recipe['planning_options_by_phase']={phase:dict(enable_graph=False) for phase in joint}
        recipe['hold_orientation_phases']=[p for p in recipe.get('hold_orientation_phases',[]) if p not in joint]
        recipe['linear_motion_phases']=[p for p in recipe.get('linear_motion_phases',[]) if p not in joint]
        recipe.update(retry_round=6,frozen_candidate_count=1,cpu_preselection_target_set_count=1,
            previous_candidate_planning_result=row['candidate_planning_result'],latest_actual_result=row['previous_retry_result'],
            selected_round6_candidate=dict(name='fixed_continuation_joint_branch_same_source_clear_route',geometry_candidate_count=1,
                public_solver='MotionGen.plan_single_js',cpu_continuation_is_injected_seed=False),
            cpu_screen_passed=bool(all_valid),
            description=('保留原避壳折线与抓姿，固定连续IK的9个lift关节端点，前序pregrasp/approach也固定上轮已成功的关节分支。' if 'banana' in sid else '保留瓶身抓姿及60mm预抓下降，固定连续IK的6个approach和9个lift关节端点，pregrasp固定上轮已成功关节分支。')+' 使用公开plan_single_js，每段一次，不再增加软权重；原5mm实际FK验收限制不变。',
            limitations='CPU continuous IK is preparation evidence, not an injected trajectory. Fixed joint goals guide one public joint-space cuRobo query per segment. CPU sampled joint-line geometry does not certify actual optimizer path, contacts or task success.')
        from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
        validate_recipe(recipe,read(ROOT/row['result_path']),read(ROOT/row['config_path']))
        final['entries'].append(dict(scene_id=sid,previous_candidate=row['candidate_planning_result'],config=source['config'],source_scene=source['source_scene'],previous_completed_prefix=prefix_ref,original_masks=masks,continuous_ik_segments=len(source['continuation']['segments']),joint_goal_checks=checks,joint_linear_seed_check_passed=bool(all_valid),selected=deepcopy(recipe)))
        recipes[sid]=recipe;del probe
        print(sid,'joint_goal_phases',list(joint),'joint_linear_passed',all_valid,flush=True)
    save(final_report,final)
    for recipe in recipes.values():recipe['evidence']=ref(final_report)
    save(config,dict(kind='physical_retry_round6_other_recipes',recipes=recipes,evidence=ref(final_report),layout_changed=False))
    print(json.dumps(dict(config=ref(config),report=ref(final_report),probe=ref(REPORT))),flush=True)

if __name__=='__main__':
    if '--freeze' in sys.argv:freeze()
    else:main()
