#!/usr/bin/env python3
"""One bounded refinement: rejoin recorded branch early, split vertical descent."""
from copy import deepcopy
from pathlib import Path
import json, os, sys, time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
from scripts.extensions.replicator.prepare_round12_human_carry_cpu import SID,OUT,CONFIG,PREVIOUS,ANALYSIS,MANIFEST


def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
    report_path=OUT/'directed_refinement.json'
    if report_path.exists() or CONFIG.exists():raise FileExistsError('Refinement already exists')
    prior=read(PREVIOUS)['recipes'][SID];row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
    result=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
    probe=geo.Probe(row,result);opener=OpenGeometry(probe);masks=masks_for(original)
    old=read(ANALYSIS);recorded=old['reverse_carrying_samples']
    qstart=np.asarray(old['transfer_q_rad']);qbranch=np.asarray(old['same_tcp_recorded_retreat_q_rad'])
    lower=np.asarray(prior['target_poses_world']['lower']);start=np.asarray(prior['target_poses_world']['transfer'])
    high=np.asarray(prior['phase_waypoints_world']['retreat'][0]);attachment=np.asarray(prior['nominal_attachment'])
    # Recorded fraction is over transfer -> lower+100mm. Preserve exact branch anchors.
    def target_at(f):
        rotation=Slerp([0.,1.],Rotation.from_matrix([start[:3,:3],high[:3,:3]]))
        t=np.eye(4);t[:3,:3]=rotation(f).as_matrix();t[:3,3]=(1-f)*start[:3,3]+f*high[:3,3]
        return t
    def check(q):
        probe.phase='lower';probe.carrying=True;probe.grip=prior['planning_gripper_joint_rad']
        probe.attachment=attachment;probe.masks=masks['lower']
        q=np.asarray(q);tcp=probe.kin.fk(q);goal=tcp@attachment
        coll=probe.check(q);full=opener.bowl_world(goal)
        excess=float(np.maximum(probe.kin.limits[:,0]-q,q-probe.kin.limits[:,1]).clip(min=0).max())
        return dict(q_rad=q.tolist(),tcp_world=tcp.tolist(),nominal_payload_world=goal.tolist(),collision=coll,full_source_bowl=full,joint_limit_excess_rad=excess,
            passed=bool(excess==0 and not coll['unique_world_pairs'] and not coll['self_sphere_overlaps'] and full['passed']))
    declarations=[dict(name='x30_y25_rejoin_f50',offset_xyz_m=[-.03,-.025,0.],rejoin_fraction=.5),
                  dict(name='x40_y15_rejoin_f50',offset_xyz_m=[-.04,-.015,0.],rejoin_fraction=.5),
                  dict(name='x30_y25_rejoin_f40',offset_xyz_m=[-.03,-.025,0.],rejoin_fraction=.4)]
    declaration_path=OUT/'directed_refinement_declarations.json'
    save(declaration_path,dict(kind='round12_human_bounded_carry_refinement_declarations',source_coarse_report=ref(OUT/'preparation.json'),
        maximum_candidates=3,maximum_ik_seeds_per_sample=1,maximum_lower_waypoints=8,candidates=declarations,
        vertical_waypoint_height_offsets_m=[.1,.075,.05,.025],source_script=ref(Path(__file__))))
    trials=[];selected=None
    for declaration in declarations:
        begin=time.monotonic();detour=target_at(.3);detour[:3,3]+=declaration['offset_xyz_m']
        targets=[start,detour,target_at(declaration['rejoin_fraction']),target_at(.8),high]
        seeds=[qbranch,np.asarray(recorded[3]['q_rad']),np.asarray(recorded[round(10*declaration['rejoin_fraction'])]['q_rad']),np.asarray(recorded[8]['q_rad']),np.asarray(prior['joint_space_target_q_by_phase']['retreat'][0])]
        for dz in (.075,.05,.025):
            t=lower.copy();t[2,3]+=dz;targets.append(t)
            seed=next(v for v in recorded if abs(np.asarray(v['tcp_world'])[2,3]-t[2,3])<.0051)
            seeds.append(np.asarray(seed['q_rad']))
        targets.append(lower);seeds.append(np.asarray(prior['joint_space_target_q_by_phase']['lower'][-1]))
        endpoints=[];qs=[]
        for i,(target,seed) in enumerate(zip(targets,seeds)):
            if i in (0,4,8):q=seed
            else:q=probe.kin.ik(target,seed,attempts=1)
            fk=probe.kin.fk(q);geometry=check(q)
            pe=float(np.linalg.norm(fk[:3,3]-target[:3,3]));re=float(Rotation.from_matrix(fk[:3,:3]@target[:3,:3].T).magnitude())
            endpoints.append(dict(index=i,target_world=target.tolist(),position_error_m=pe,orientation_error_rad=re,**geometry,
                endpoint_passed=bool(geometry['passed'] and pe<=.001 and re<=.01)))
            qs.append(q)
        chords=[];ideals=[]
        for i,(a,b) in enumerate(zip([qstart,*qs[:-1]],qs)):
            samples=[dict(fraction=float(f),**check((1-f)*a+f*b)) for f in np.linspace(0,1,31)]
            chords.append(dict(index=i,samples=samples,passed=all(s['passed'] for s in samples),max_abs_joint_delta_rad=float(np.max(np.abs(a-b)))))
            if i==0:continue
            ta=targets[i-1];tb=targets[i];rotation=Slerp([0.,1.],Rotation.from_matrix([ta[:3,:3],tb[:3,:3]]))
            ideal=[]
            for f in np.linspace(0,1,11):
                t=np.eye(4);t[:3,:3]=rotation(f).as_matrix();t[:3,3]=(1-f)*ta[:3,3]+f*tb[:3,3]
                ik=pose_check(probe,t,(1-f)*a+f*b,'lower',prior,masks)
                g=check(np.asarray(ik['q_rad'])) if 'q_rad' in ik else None
                ideal.append(dict(fraction=float(f),target_world=t.tolist(),ik=ik,geometry=g,passed=bool(ik['passed'] and g and g['passed'])))
            ideals.append(dict(index=i,samples=ideal,passed=all(s['passed'] for s in ideal)))
        passed=all(e['endpoint_passed'] for e in endpoints) and all(c['passed'] for c in chords) and all(c['passed'] for c in ideals)
        trial=dict(declaration=declaration,endpoints=endpoints,joint_chords=chords,ideal_segments=ideals,passed=passed,seconds=time.monotonic()-begin)
        trials.append(trial);save(OUT/'directed_refinement.partial.json',dict(scene_id=SID,trials=trials))
        print(json.dumps(dict(name=declaration['name'],passed=passed,endpoints=[e['endpoint_passed'] for e in endpoints],
            chord_failures=[sum(not s['passed'] for s in c['samples']) for c in chords],
            ideal_failures=[sum(not s['passed'] for s in c['samples']) for c in ideals],seconds=time.monotonic()-begin)),flush=True)
        if passed:selected=trial;break
    # Exact original opened withdrawal remains unchanged; bind its existing sample evidence.
    report=dict(kind='round12_human_fixed_layout_carry_waypoint_refinement',scene_id=SID,source_script=ref(Path(__file__)),
        source_previous_recipe=ref(PREVIOUS),source_previous_manifest=ref(MANIFEST),source_previous_candidate=row['candidate_planning_result'],
        source_reverse_analysis=ref(ANALYSIS),source_coarse_screen=ref(OUT/'preparation.json'),declarations=ref(declaration_path),
        retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),retained_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        retained_scene_revision=row['scene_revision'],source_opened_retreat_evidence=prior['withdraw_direction_correction'],
        unchanged_nominal_destination=prior['destination_region_check'],planning_gripper_joint_rad=prior['planning_gripper_joint_rad'],nominal_attachment=prior['nominal_attachment'],
        trials=trials,selected_name=selected['declaration']['name'] if selected else None,passed=selected is not None,
        actual_gpu_paths_checked=False,layout_changed=False,curobo_calls=0,kit_calls=0,physical_episodes=0,
        limitations=['Finite source-geometry CPU ideal/chord samples are not continuous-path proof or actual cuRobo output.',
            'All CPU chords use the recorded prior transfer branch; the new GPU prefix may choose a different branch.',
            'FCL triangle distance is unsigned and can miss complete containment; GPU sphere collision checker remains required.',
            'Original opened-retreat source geometry is retained from the bound prior evidence, not reinterpreted as new physical execution.'])
    save(report_path,report)
    if selected:
        r=deepcopy(prior);r.update(retry_round=12,evidence=ref(report_path),previous_frozen_recipe=ref(PREVIOUS),
            previous_candidate_planning_result=row['candidate_planning_result'],source_joint_goal_evidence=ref(report_path),
            cpu_screen_passed=True,cpu_screen_scope='Frozen endpoints plus 31 carrying source-geometry joint samples per lower segment and 11 ideal TCP samples; opened retreat retained exactly.',
            ideal_segment_samples_passed=True,ideal_segment_samples_checked_phases=['lower'],remaining_cpu_warnings=[],
            planning_path_policy=dict(kind='collision_checked_graph_joint_goals_with_carry_waypoints',previous_straight_line_gate_removed=True,
                reason_zh='保留第十一轮最终目标与关节分支，加入携碗侧绕及分段垂直下放路点；仅将路点关节目标交给单次图搜索/轨迹优化，CPU有限路线不替代实际碰撞检查。'),
            description='保持人体布局、抓取、全部最终TCP与退出目标。lower先在相同TCP换至已检查分支，侧绕人体后回到原路线，最后经上方100/75/50/25mm逐段下降；本轮唯一冻结候选仍须cuRobo与原生物理验证。',finalizer_source=ref(Path(__file__)))
        r['phase_waypoints_world']['lower']=[e['target_world'] for e in selected['endpoints'][:-1]]
        r['joint_space_target_q_by_phase']['lower']=[e['q_rad'] for e in selected['endpoints']]
        for key in ('target_poses_world','nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad'):
            assert r[key]==prior[key],key
        assert r['phase_waypoints_world']['retreat']==prior['phase_waypoints_world']['retreat']
        assert r['joint_space_target_q_by_phase']['retreat']==prior['joint_space_target_q_by_phase']['retreat']
        assert r['joint_space_target_q_by_phase']['lower'][-1]==prior['joint_space_target_q_by_phase']['lower'][-1]
        validate_recipe(r,original,probe.layout)
        save(CONFIG,dict(kind='physical_retry_round12_human_recipes',recipes={SID:r},evidence=ref(report_path),layout_changed=False))
    print(json.dumps(dict(report=ref(report_path),recipe=ref(CONFIG) if CONFIG.exists() else None)),flush=True)


if __name__=='__main__':main()
