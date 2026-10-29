#!/usr/bin/env python3
"""Bounded CPU preparation of two round4 fixed-layout candidates.

This checks declared source geometry/IK samples, never a GPU or physical path.
Existing reports and all earlier round inputs remain immutable.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator import planning_failure_cpu_geometry as geo
from scripts.extensions.replicator.prepare_round2_mustard import Gripper, scene_shapes, read, save
from scripts.extensions.replicator.prepare_round3_mustard import source_screen
from kitchen_safety.extensions.replicator.physical_retry_round2_mustard import make_targets, PHASES
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe

OUT = ROOT/'reports/replicator_physical_retry_round4_20261029'
REPORT = OUT/'other_cpu_preparation.json'
CONFIG = ROOT/'configs/extensions/replicator/physical_retry_round4_other_recipes_20261029.json'
MANIFEST = ROOT/'exports/replicator_physical_retry_round3_20261029/manifest.json'
SNAPSHOT = ROOT/'configs/extensions/replicator/physical_retry_results_round3_20261029.json'
SIDS = ('l_island_banana_bowl__robot_sharp_equipment_housing',
        'u_shape_mustard_bowl__robot_sharp_equipment_housing')


def ref(path):
    return geo.binding(Path(path))


def masks_for(previous):
    masks = {p['phase']:p['contact_masks'] for p in previous['phases']}
    while set(masks) != set(PHASES):
        previous = read(geo.bound(dict(path=previous['prior_result_path'], sha256=previous['prior_result_sha256'])))
        for p in previous['phases']: masks.setdefault(p['phase'], p['contact_masks'])
    return masks


def pose_check(probe, target, seed, phase, recipe, masks):
    try:
        q = probe.kin.ik(target, seed, attempts=1)
    except RuntimeError as exc:
        return dict(passed=False, error=str(exc), score=999)
    actual = probe.kin.fk(q)
    probe.phase = phase; probe.carrying = phase in ('lift','transfer','lower')
    probe.grip = recipe['planning_gripper_joint_rad'] if probe.carrying else 0.
    probe.attachment = np.asarray(recipe['nominal_attachment']); probe.masks = masks[phase]
    collision = probe.check(q)
    pe = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
    re = float(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude())
    score = len(collision['unique_world_pairs']) + len(collision['self_sphere_overlaps'])
    return dict(passed=bool(score == 0 and pe < .002 and re < .035), score=score,
        q_rad=q.tolist(), position_error_m=pe, orientation_error_rad=re,
        collision=dict(unique_world_pairs=collision['unique_world_pairs'],
            self_sphere_overlaps=collision['self_sphere_overlaps'], masked_objects=masks[phase]))


def endpoint_checks(probe, recipe, previous, masks, first):
    checks=[]; q=np.asarray(previous['start_q'])
    seeds=previous.get('recipe',{}).get('cpu_endpoint_ik',{})
    for phase in (first,*PHASES):
        if phase in seeds and seeds[phase].get('solved'): seed=np.asarray(seeds[phase]['q'])
        else: seed=q
        check=pose_check(probe,np.asarray(recipe['target_poses_world'][phase]),seed,phase,recipe,masks)
        check['phase']=phase; checks.append(check)
        if not check['passed']: return dict(passed=False, checks=checks)
        q=np.asarray(check['q_rad'])
    return dict(passed=True,checks=checks)


def sampled_segments(probe, recipe, endpoints, masks, phases):
    """Ideal pose sample checks only; do not validate later cuRobo curves."""
    targets={p:np.asarray(t) for p,t in recipe['target_poses_world'].items()}
    q_by_phase={x['phase']:np.asarray(x['q_rad']) for x in endpoints['checks'] if 'q_rad' in x}
    rows=[]
    for phase in phases:
        start=targets[PHASES[PHASES.index(phase)-1]]
        q=q_by_phase[PHASES[PHASES.index(phase)-1]]
        points=[np.asarray(x) for x in recipe.get('phase_waypoints_world',{}).get(phase,[])]+[targets[phase]]
        for segment,end in enumerate(points):
            for fraction in np.linspace(0.,1.,5):
                target=start.copy(); target[:3,3]=(1-fraction)*start[:3,3]+fraction*end[:3,3]
                if not np.allclose(start[:3,:3],end[:3,:3],atol=1e-7):
                    raise ValueError('Sample only fixed orientation segments')
                check=pose_check(probe,target,q,phase,recipe,masks)
                rows.append(dict(phase=phase,segment=segment,fraction=float(fraction),**check))
                if not check['passed']: return dict(passed=False,samples=rows)
                q=np.asarray(check['q_rad'])
            start=end
    return dict(passed=True,samples=rows,
        limitation='Finite ideal TCP segments, not the future GPU path or real physical contacts.')


def banana_candidates(previous, probe):
    old=previous['recipe']; base=np.asarray(old['target_poses_world']['approach'])
    away=base[:3,3]-np.asarray(probe.layout['obstacle']['position']);away[2]=0.;away/=np.linalg.norm(away)
    # The first CPU screen found the toward-robot diagonal still enters the
    # housing. Move away from the housing at low height, then rise vertically.
    for horizontal,height,first_rise in ((.10,.10,.008),(.14,.10,.008),(.10,.14,.015),(.14,.14,.015)):
        r=deepcopy(old); r['target_poses_world']['lift']=base.copy().tolist()
        t=np.asarray(r['target_poses_world']['lift']); t[:3,3]+=horizontal*away+np.array([0,0,height])
        r['target_poses_world']['lift']=t.tolist()
        r['linear_motion_phases']=[]
        elbow=t.copy();elbow[2,3]=base[2,3]+first_rise
        points=[]
        for start,end in ((base,elbow),(elbow,t)):
            for fraction in (.25,.5,.75,1.):
                point=end.copy();point[:3,3]=(1-fraction)*start[:3,3]+fraction*end[:3,3];points.append(point.tolist())
        r['phase_waypoints_world']={'lift':points[:-1]}
        yield dict(name=f'banana_away_housing{horizontal:g}_then_rise{height:g}',away_from_housing_world=away.tolist(),away_m=horizontal,rise_m=height,initial_rise_m=first_rise),r


def mustard_candidates(previous):
    fixed=np.array([[0.,-1.,0.],[0.,0.,1.],[-1.,0.,0.]])
    # New small offsets avoid the rejected larger azimuth/low-body candidates.
    for tilt,yaw,z in ((35.,260.,.005),(35.,280.,.005),(35.,270.,.012),
                       (35.,260.,.012),(35.,280.,.012),(40.,270.,.012)):
        rz=Rotation.from_euler('z',yaw,degrees=True).as_matrix(); local=np.eye(4)
        local[:3,:3]=rz@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@fixed
        local[:3,3]=rz@np.array([0.,-.010,z])
        candidate=dict(name=f'body_tilt{tilt:g}_yaw{yaw:g}_z{z:g}',bottle_to_tcp_local=local.tolist(),
            tilt_deg=tilt,rotation_about_bottle_z_deg=yaw,body_local_tcp_z_m=z,
            planning_gripper_joint_rad=.30,execution_gripper_joint_rad=.70)
        r=make_targets(previous,candidate)
        a=np.asarray(r['target_poses_world']['approach']); pre=a.copy(); pre[2,3]+=.125
        r['target_poses_world']['pregrasp']=pre.tolist()
        r['linear_motion_phases']=['approach','lift']
        yield candidate,r


def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA for this CPU-only work')
    if REPORT.exists() or CONFIG.exists():raise FileExistsError('Frozen round4 outputs already exist')
    manifest=read(MANIFEST); snapshot=read(SNAPSHOT)
    records={r['scene_id']:r for r in manifest['entries']}
    published={r['scene_id']:r for r in snapshot['entries']}
    report=dict(kind='physical_retry_round4_other_cpu_preparation',source_manifest=ref(MANIFEST),
        source_snapshot=ref(SNAPSHOT),source_script=ref(Path(__file__)),source_geometry_helper=ref(Path(geo.__file__)),
        layout_changed=False,physics_launched=False,kit_started=False,curobo_called=False,
        rejected_first_screen=ref(OUT/'rejected_first_cpu_screen.json'),
        candidate_count_per_scene=1,entries=[])
    recipes={}
    for sid in SIDS:
        begin=time.monotonic(); row=records[sid]
        previous=read(geo.bound(row['candidate_planning_result']))
        probe=geo.Probe(row,dict(previous,failed_phase='approach'))
        masks=masks_for(previous); mustard='mustard' in sid
        if mustard: gripper=Gripper(); shapes=scene_shapes(geo.bound(dict(path=row['source_scene_path'],sha256=row['source_scene_sha256'])))
        trials=[]; selected=None; best=None
        declarations=mustard_candidates(previous) if mustard else banana_candidates(previous,probe)
        for declaration,recipe in declarations:
            trial=dict(candidate=declaration)
            if mustard:
                trial['source']=source_screen(recipe,shapes,gripper)
                if not trial['source']['passed']:
                    trials.append(trial); print(sid,declaration['name'],trial['source']['stage'],flush=True);continue
                recipe['planning_gripper_joint_rad']=trial['source']['estimated_contact_angle_rad']
            endpoint=endpoint_checks(probe,recipe,previous,masks,'approach' if mustard else 'lift');trial['endpoints']=endpoint
            if endpoint['passed']:
                trial['sampled_ideal_segments']=sampled_segments(probe,recipe,endpoint,masks,('approach','lift') if mustard else ('lift','transfer'))
            passed=endpoint['passed'] and trial['sampled_ideal_segments']['passed']
            trials.append(trial)
            progress=sum(x['passed'] for x in endpoint['checks'])
            score=(progress, sum(x['passed'] for x in trial.get('sampled_ideal_segments',{}).get('samples',[])), -sum(x['score'] for x in endpoint['checks']))
            if best is None or score>best[0]:best=(score,deepcopy(recipe),len(trials)-1)
            print(sid,declaration['name'],'endpoints',endpoint['passed'],'ideal_segment',trial.get('sampled_ideal_segments',{}).get('passed'),flush=True)
            if passed:selected=recipe;selected_index=len(trials)-1;break
        if selected is None:
            if best is None:raise RuntimeError('No source-passing candidate; retain log and do not freeze '+sid)
            selected=best[1];selected_index=best[2]
        final_trial=trials[selected_index]
        passed=final_trial['endpoints']['passed'] and final_trial.get('sampled_ideal_segments',{}).get('passed',False)
        targets={p:np.asarray(t) for p,t in selected['target_poses_world'].items()}
        selected['hold_orientation_phases']=[p for p in PHASES[1:] if np.allclose(targets[p][:3,:3],targets[PHASES[PHASES.index(p)-1]][:3,:3],atol=1e-7)]
        for key in ('cpu_endpoint_ik','cpu_endpoint_clear_of_detected_pairs','remaining_cpu_warnings','cpu_preparation_evidence','superseded_preparation_recipe','retry_round','evidence'):
            selected.pop(key,None)
        selected.update(scene_id=sid,task_id=row['task_id'],payload_name=previous['payload_name'],layout_fixed=True,
            cpu_screen_passed=passed,previous_candidate_planning_result=row['candidate_planning_result'],
            latest_actual_result=published[sid]['latest_physical_result'],retry_round=4,frozen_candidate_count=1,
            selected_round4_candidate=final_trial['candidate'],
            limitations='CPU finite endpoint and ideal-segment source checks only; actual cuRobo path must pass its own gate. No physical success or safety claim.')
        # These are intermediate goals before the unchanged phase endpoint.
        # The shared planner independently gates actual FK to each 3D segment.
        segmented=('approach','lift') if mustard else ('lift',)
        selected.setdefault('phase_waypoints_world',{})
        selected['segment_deviation_limits_m']={p:.005 for p in segmented}
        for phase in segmented:
            if phase in selected['phase_waypoints_world']:continue
            start=targets[PHASES[PHASES.index(phase)-1]];end=targets[phase]
            count=min(9,max(2,int(np.ceil(np.linalg.norm(end[:3,3]-start[:3,3])/.025))))
            points=[]
            for f in np.arange(1,count)/count:
                point=end.copy();point[:3,3]=(1-f)*start[:3,3]+f*end[:3,3];points.append(point.tolist())
            selected['phase_waypoints_world'][phase]=points
        selected['description']=('香蕉保持中段抓取和原目标碗落点，先沿远离尖角外壳的方向低位移出，再分段抬升；抬升段CPU样本已避开此前携物包络冲突。转运直线仍有源包络冲突警告，transfer不施加直线约束，留给一次完整cuRobo规划寻找绕行。' if not mustard else '芥末瓶更换瓶身抓取方位和局部高度，检查全部张爪与源几何闭合，按短段接近/初抬升限制实际规划曲线偏离；布局和物理不变。')+' 选中'+final_trial['candidate']['name']+'。'
        selected['grasp_info'].update(round4_rule='declared_piecewise_targets_with_independent_actual_FK_gate',
            prior_physical_result=published[sid]['latest_physical_result'])
        if mustard:
            selected['grasp_info']['selected_candidate']=deepcopy(final_trial['candidate'])
            selected['grasp_info']['selected_candidate']['planning_gripper_joint_rad']=selected['planning_gripper_joint_rad']
            selected['grasp_info']['source_estimated_contact_angle_rad']=selected['planning_gripper_joint_rad']
        if not passed:selected['description']+=' 有限CPU检查仍有警告，具体见绑定报告，不代表已经可规划或可执行。'
        validate_recipe(selected,read(ROOT/row['result_path']),read(ROOT/row['config_path']))
        entry=dict(scene_id=sid,previous_candidate=row['candidate_planning_result'],latest_actual_result=published[sid]['latest_physical_result'],
            config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
            original_masks=masks,trials=trials,selected_index=selected_index,cpu_screen_passed=passed,
            selected=deepcopy(selected),elapsed_seconds=time.monotonic()-begin)
        report['entries'].append(entry);recipes[sid]=selected;save(REPORT.with_suffix('.partial.json'),report)
        del probe
        if mustard:del shapes,gripper
    save(REPORT,report)
    for r in recipes.values():r['evidence']=ref(REPORT)
    save(CONFIG,dict(kind='physical_retry_round4_other_recipes',recipes=recipes,evidence=ref(REPORT),layout_changed=False))
    print(json.dumps(dict(config=ref(CONFIG),report=ref(REPORT))),flush=True)


if __name__=='__main__':main()
