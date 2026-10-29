#!/usr/bin/env python3
"""One declared early-upright candidate after identifying a sink-back arc."""
from copy import deepcopy
from pathlib import Path
import json,os,sys,time
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
from scripts.extensions.replicator.prepare_round12_human_carry_cpu import SID,OUT,CONFIG,PREVIOUS,ANALYSIS,MANIFEST


def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
    report_path=OUT/'early_upright_final_candidate.json'
    if report_path.exists() or CONFIG.exists():raise FileExistsError('Final candidate already checked')
    prior=read(PREVIOUS)['recipes'][SID];row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
    result=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
    probe=geo.Probe(row,result);opener=OpenGeometry(probe);masks=masks_for(original)
    earlier=read(OUT/'directed_refinement.json')['trials'][0]
    lower=np.asarray(prior['target_poses_world']['lower']);attachment=np.asarray(prior['nominal_attachment'])
    qstart=np.asarray(read(ANALYSIS)['transfer_q_rad'])
    targets=[np.asarray(v['target_world']) for v in earlier['endpoints'][:3]]
    fixed_q=[np.asarray(v['q_rad']) for v in earlier['endpoints'][:3]]
    for dz in (.160,.130,.100,.075,.050,.025,0.):
        t=lower.copy();t[2,3]+=dz;targets.append(t)
    declaration=OUT/'early_upright_declaration.json'
    save(declaration,dict(kind='round12_human_one_early_upright_carry_candidate',scene_id=SID,source_script=ref(Path(__file__)),
        source_prior_screen=ref(OUT/'directed_refinement.json'),candidate_count=1,maximum_lower_waypoints=12,
        target_poses_world=[t.tolist() for t in targets],lower_waypoint_count=len(targets)-1,
        reason_zh='原携碗最后转正弧在槽后壁出现球包碰撞；提前在原lower正上方160mm完成转正，再以30/25mm固定关节路点下放。',
        all_terminal_targets_unchanged=True,layout_changed=False))
    def check(q):
        q=np.asarray(q);probe.phase='lower';probe.carrying=True;probe.grip=prior['planning_gripper_joint_rad'];probe.attachment=attachment;probe.masks=masks['lower']
        tcp=probe.kin.fk(q);goal=tcp@attachment;collision=probe.check(q);full=opener.bowl_world(goal)
        excess=float(np.maximum(probe.kin.limits[:,0]-q,q-probe.kin.limits[:,1]).clip(min=0).max())
        return dict(q_rad=q.tolist(),tcp_world=tcp.tolist(),nominal_payload_world=goal.tolist(),collision=collision,full_source_bowl=full,
            joint_limit_excess_rad=excess,passed=bool(excess==0 and not collision['unique_world_pairs'] and not collision['self_sphere_overlaps'] and full['passed']))
    endpoints=[];qs=[]
    for i,t in enumerate(targets):
        if i<3:q=fixed_q[i]
        elif i==5:q=np.asarray(prior['joint_space_target_q_by_phase']['retreat'][0])
        elif i==len(targets)-1:q=np.asarray(prior['joint_space_target_q_by_phase']['lower'][-1])
        else:
            # Seed from the existing nearby branch, not an unrelated authored reset.
            nearest=min(earlier['endpoints'],key=lambda e:np.linalg.norm(np.asarray(e['target_world'])[:3,3]-t[:3,3]))
            q=probe.kin.ik(t,np.asarray(nearest['q_rad']),attempts=1)
        g=check(q);fk=np.asarray(g['tcp_world']);pe=float(np.linalg.norm(fk[:3,3]-t[:3,3]));re=float(Rotation.from_matrix(fk[:3,:3]@t[:3,:3].T).magnitude())
        endpoints.append(dict(index=i,target_world=t.tolist(),position_error_m=pe,orientation_error_rad=re,**g,
            endpoint_passed=bool(g['passed'] and pe<=.001 and re<=.01)));qs.append(q)
    chords=[];ideals=[]
    for i,(a,b) in enumerate(zip([qstart,*qs[:-1]],qs)):
        samples=[dict(fraction=float(f),**check((1-f)*a+f*b)) for f in np.linspace(0,1,31)]
        chords.append(dict(index=i,samples=samples,passed=all(s['passed'] for s in samples),max_abs_joint_delta_rad=float(np.max(np.abs(a-b)))))
        if i==0:continue
        ta,tb=targets[i-1],targets[i];rotation=Slerp([0,1],Rotation.from_matrix([ta[:3,:3],tb[:3,:3]]));samples=[]
        for f in np.linspace(0,1,11):
            t=np.eye(4);t[:3,:3]=rotation(f).as_matrix();t[:3,3]=(1-f)*ta[:3,3]+f*tb[:3,3]
            ik=pose_check(probe,t,(1-f)*a+f*b,'lower',prior,masks);g=check(np.asarray(ik['q_rad'])) if 'q_rad' in ik else None
            samples.append(dict(fraction=float(f),target_world=t.tolist(),ik=ik,geometry=g,passed=bool(ik['passed'] and g and g['passed'])))
        ideals.append(dict(index=i,samples=samples,passed=all(s['passed'] for s in samples)))
    passed=all(e['endpoint_passed'] for e in endpoints) and all(c['passed'] for c in chords) and all(c['passed'] for c in ideals)
    report=dict(kind='round12_human_fixed_layout_early_upright_carry_candidate',scene_id=SID,source_script=ref(Path(__file__)),declaration=ref(declaration),
        source_previous_recipe=ref(PREVIOUS),source_previous_manifest=ref(MANIFEST),source_previous_candidate=row['candidate_planning_result'],
        source_previous_ideal_analysis=ref(ANALYSIS),source_coarse_screen=ref(OUT/'preparation.json'),source_directed_screen=ref(OUT/'directed_refinement.json'),
        retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),retained_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        retained_scene_revision=row['scene_revision'],source_opened_retreat_evidence=prior['withdraw_direction_correction'],
        source_opened_gripper_provenance=opener.provenance,unchanged_nominal_destination=prior['destination_region_check'],
        nominal_attachment=prior['nominal_attachment'],planning_gripper_joint_rad=prior['planning_gripper_joint_rad'],
        endpoints=endpoints,joint_chords=chords,ideal_segments=ideals,passed=passed,
        endpoint_count=len(endpoints),joint_chord_sample_count=sum(len(c['samples']) for c in chords),ideal_pose_sample_count=sum(len(c['samples']) for c in ideals),
        actual_gpu_paths_checked=False,layout_changed=False,new_curobo_calls=0,new_kit_calls=0,new_physical_episodes=0,
        limitations=['Finite CPU ideal poses and joint chords are not actual GPU paths or a continuous collision proof.',
            'Robot sphere buffer is 5mm and attached bowl is the unchanged 100-sphere cover; full source bowl triangle mesh is also checked.',
            'FCL triangle distance is unsigned and may miss complete containment; actual cuRobo world/self collision checks remain mandatory.',
            '31-sample first bridge begins at recorded R11 transfer q; actual R12 transfer branch may differ.',
            'Opened retreat is unchanged and referenced to prior CPU/GPU static evidence; no new contact or support success is claimed.'])
    save(report_path,report)
    if passed:
        r=deepcopy(prior);r.update(retry_round=12,evidence=ref(report_path),previous_frozen_recipe=ref(PREVIOUS),
            previous_candidate_planning_result=row['candidate_planning_result'],source_joint_goal_evidence=ref(report_path),
            cpu_screen_passed=True,cpu_screen_scope='10 endpoints, 310 carrying joint-chord samples and 99 ideal TCP samples with source robot/payload spheres and full source bowl; original opened retreat unchanged.',
            ideal_segment_samples_passed=True,ideal_segment_samples_checked_phases=['lower'],remaining_cpu_warnings=[],
            planning_path_policy=dict(kind='collision_checked_graph_joint_goals_with_carry_waypoints',previous_straight_line_gate_removed=True,
                reason_zh='保留第十一轮布局和全部最终目标，仅加入携碗绕行及提前转正的分段下放关节目标；CPU有限理想路线不注入优化器，实际由单次cuRobo图搜索与碰撞约束轨迹优化判定。'),
            description='保持人体站位与抓姿。下放先同TCP换支，侧绕人体后于落点上方160mm转正，再经130/100/75/50/25mm路点下降；所有原最终TCP、最终关节目标、夹爪与退出路点保持。有限CPU几何通过仍不等于真实路径或物理成功。',finalizer_source=ref(Path(__file__)))
        r['phase_waypoints_world']['lower']=[e['target_world'] for e in endpoints[:-1]]
        r['joint_space_target_q_by_phase']['lower']=[e['q_rad'] for e in endpoints]
        for key in ('target_poses_world','nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad'):
            assert r[key]==prior[key],key
        assert r['phase_waypoints_world']['retreat']==prior['phase_waypoints_world']['retreat']
        assert r['joint_space_target_q_by_phase']['retreat']==prior['joint_space_target_q_by_phase']['retreat']
        assert r['joint_space_target_q_by_phase']['lower'][-1]==prior['joint_space_target_q_by_phase']['lower'][-1]
        validate_recipe(r,original,probe.layout)
        save(CONFIG,dict(kind='physical_retry_round12_human_recipes',recipes={SID:r},evidence=ref(report_path),layout_changed=False))
    print(json.dumps(dict(passed=passed,endpoints=[e['endpoint_passed'] for e in endpoints],
        chord_failures=[sum(not s['passed'] for s in c['samples']) for c in chords],ideal_failures=[sum(not s['passed'] for s in c['samples']) for c in ideals],
        report=ref(report_path),recipe=ref(CONFIG) if CONFIG.exists() else None)),flush=True)


if __name__=='__main__':main()
