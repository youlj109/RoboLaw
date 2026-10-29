#!/usr/bin/env python3
"""Check one 25mm raised release endpoint already reached by round12 planning."""
from copy import deepcopy
from pathlib import Path
import json,os,sys
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
SID='u_shape_bowl_sink__human_default_mpfb'
OUT=ROOT/'reports/replicator_physical_retry_round13_20261029/human_cpu'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round13_human_recipes_20261029.json'
PREVIOUS=ROOT/'configs/extensions/replicator/physical_retry_round12_human_recipes_20261029.json'
MANIFEST=ROOT/'exports/replicator_physical_retry_round12_20261029/manifest.json'


def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
    if OUT.exists() or CONFIG.exists():raise FileExistsError('Round13 candidate already exists')
    OUT.mkdir(parents=True)
    prior=read(PREVIOUS)['recipes'][SID];row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
    result=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
    probe=geo.Probe(row,result);opener=OpenGeometry(probe);masks=masks_for(original)
    lower=np.asarray(prior['phase_waypoints_world']['lower'][-1]);attachment=np.asarray(prior['nominal_attachment'])
    qlower=np.asarray(prior['joint_space_target_q_by_phase']['lower'][-2]);qhigh=np.asarray(prior['joint_space_target_q_by_phase']['retreat'][0])
    high=np.asarray(prior['phase_waypoints_world']['retreat'][0]);goal=lower@attachment
    source_segment=next(p for p in result['phases'] if p['phase']=='lower')['segments'][-2]
    assert source_segment['segment_index']==8 and source_segment['solver_success'] and source_segment['status']=='generated'
    with np.load(geo.bound(source_segment['trajectory']),allow_pickle=False) as z:actual_q=z['q'][-1].copy()
    initial_goal=np.asarray(original['targets']['lower'])@np.asarray(original['nominal_attachment'])
    floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'))
    points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];bounds=np.array([points.min(0),points.max(0)])
    delta=goal[:3,3]-initial_goal[:3,3];bottom_gap=float(bounds[0,2]-floor[3][2])
    bowl=opener.bowl_world(goal);release=opener.check(lower,goal)
    region_passed=bool(np.all(bounds[0,:2]>=floor[2][:2]) and np.all(bounds[1,:2]<=floor[3][:2])
        and np.linalg.norm(delta[:2])<=.06 and abs(delta[2])<=.06 and abs(bottom_gap-.050)<1e-6 and bowl['passed'])
    centers,radii=probe.cover;cover=centers@goal[:3,:3].T+goal[:3,3]
    region=dict(kind='round13_retained_layout_nominal_raised_bowl_destination',passed=region_passed,
        config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        scene_revision=row['scene_revision'],source_previous_candidate=row['candidate_planning_result'],source_successful_lower_segment=source_segment['trajectory'],
        nominal_attachment=attachment.tolist(),nominal_lower_tcp_world=lower.tolist(),nominal_goal_world=goal.tolist(),original_goal_world=initial_goal.tolist(),
        nominal_goal_delta_xyz_m=delta.tolist(),nominal_xy_limit_m=.06,nominal_z_limit_m=.06,
        sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[np.asarray(floor[2]).tolist(),np.asarray(floor[3]).tolist()],
        predicted_source_bowl_world_bounds_m=bounds.tolist(),predicted_bowl_bottom_to_source_floor_m=bottom_gap,
        payload_sphere_cover_bounds_m=[np.min(cover-radii[:,None],axis=0).tolist(),np.max(cover+radii[:,None],axis=0).tolist()],
        source_drain_bounds_m=[[np.asarray(x[2]).tolist(),np.asarray(x[3]).tolist()] for x in probe.world.objects if x[0].endswith('/Sink/Drain_0')],
        source_bowl=ref(geo.local(probe.layout['objects']['bowl']['usd_path'])),historical_measured_attachment_reused=False,new_physical_support_measured=False,
        scope='Nominal complete source bowl at 50mm release gap inside original basin XY. This is not floor support, settled placement, or successful physics evidence.')
    region_path=OUT/'raised_destination.json';save(region_path,region)
    def robot(q,carrying):
        probe.phase='lower' if carrying else 'retreat';probe.carrying=carrying;probe.grip=prior['planning_gripper_joint_rad'] if carrying else 0.
        probe.attachment=attachment;probe.masks=masks[probe.phase]
        q=np.asarray(q);c=probe.check(q);fk=probe.kin.fk(q)
        excess=float(np.maximum(probe.kin.limits[:,0]-q,q-probe.kin.limits[:,1]).clip(min=0).max())
        return dict(q_rad=q.tolist(),tcp_world=fk.tolist(),collision=c,joint_limit_excess_rad=excess,
            passed=bool(excess==0 and not c['unique_world_pairs'] and not c['self_sphere_overlaps']))
    frozen_endpoint=robot(qlower,True);actual_endpoint=robot(actual_q,True)
    chord=[]
    for f in np.linspace(0.,1.,31):
        q=(1-f)*qlower+f*qhigh;check=robot(q,False);source=opener.check(np.asarray(check['tcp_world']),goal,angles=(0.,))
        clear=source['fully_open_fingers_clear'] and source['fully_open_gripper_hulls_clear'] and source['all_sampled_gripper_world_clear']
        chord.append(dict(fraction=float(f),robot=check,open_source_gripper=source,passed=bool(check['passed'] and clear)))
    ideal=[];rotation=Slerp([0.,1.],Rotation.from_matrix([lower[:3,:3],high[:3,:3]]))
    for f in np.linspace(0.,1.,16):
        target=np.eye(4);target[:3,:3]=rotation(f).as_matrix();target[:3,3]=(1-f)*lower[:3,3]+f*high[:3,3]
        q=probe.kin.ik(target,(1-f)*qlower+f*qhigh,attempts=1);check=robot(q,False)
        source=opener.check(np.asarray(check['tcp_world']),goal,angles=(0.,))
        clear=source['fully_open_fingers_clear'] and source['fully_open_gripper_hulls_clear'] and source['all_sampled_gripper_world_clear']
        ideal.append(dict(fraction=float(f),target_world=target.tolist(),robot=check,open_source_gripper=source,passed=bool(check['passed'] and clear)))
    release_passed=bool(release['fully_open_fingers_clear'] and release['fully_open_gripper_hulls_clear'] and release['all_sampled_gripper_world_clear'])
    passed=region_passed and release_passed and frozen_endpoint['passed'] and actual_endpoint['passed'] and all(x['passed'] for x in chord+ideal)
    report=dict(kind='round13_single_raised_release_cpu_preparation',scene_id=SID,source_script=ref(Path(__file__)),source_previous_recipe=ref(PREVIOUS),
        source_previous_manifest=ref(MANIFEST),source_previous_candidate=row['candidate_planning_result'],source_successful_lower_segment=source_segment['trajectory'],
        source_retained_open_retreat=prior['withdraw_direction_correction'],source_gripper_provenance=opener.provenance,
        raised_destination=ref(region_path),previous_failed_final_segment_status=next(p for p in result['phases'] if p['phase']=='lower')['segments'][-1]['solver_status'],
        frozen_lower_q_check=frozen_endpoint,actual_previous_segment_endpoint_check=actual_endpoint,
        actual_endpoint_to_frozen_q_max_abs_difference_rad=float(np.max(np.abs(actual_q-qlower))),
        release_17_angles=release,release_passed=release_passed,opened_retreat_joint_chord=chord,opened_retreat_ideal_samples=ideal,
        passed=passed,candidate_count=1,layout_changed=False,new_gpu_calls=0,new_kit_calls=0,new_physical_episodes=0,
        limitations=['The previous final failed optimizer path was not saved; its constraint summary cannot identify collision or dynamic-bound causes.',
            'The new nominal release remains 50mm above the source floor; falling, support, retention and settling must be determined by the unchanged native predicate.',
            'Finite source-gripper expansion, ideal TCP and joint-chord samples are not actual cuRobo curves or a continuous collision proof.',
            'Retreat checks keep the nominal released bowl at its high pose; actual object motion after opening remains unmeasured.'])
    report_path=OUT/'preparation.json';save(report_path,report)
    if passed:
        r=deepcopy(prior);r.update(retry_round=13,evidence=ref(report_path),previous_frozen_recipe=ref(PREVIOUS),
            previous_candidate_planning_result=row['candidate_planning_result'],source_joint_goal_evidence=ref(report_path),
            final_payload_pose_world=goal.tolist(),destination_region_check=dict(passed=True,evidence_path=str(region_path.relative_to(ROOT)),sha256=geo.sha(region_path)),
            cpu_screen_passed=True,cpu_screen_scope='Prior successful segment endpoint; source full bowl region, 17 opening angles, and 31 joint/16 ideal samples to unchanged open-retreat waypoint.',
            ideal_segment_samples_passed=True,ideal_segment_samples_checked_phases=['retreat'],remaining_cpu_warnings=[],
            planning_path_policy=dict(kind='collision_checked_graph_joint_goals_raised_release',previous_straight_line_gate_removed=True,
                reason_zh='仅删除第十二轮失败的最后25mm下放段，在已规划到达且张爪源几何净空通过的上一端点释放；落槽支撑仍由原物理成功判据检验。'),
            description='保留第十二轮抓姿、人体、下放前缀与退出目标，仅将lower改为已成功的原终点上25mm并删除最后一段；名义碗底离原槽底50mm，XY及Z仍在最初目标60mm界内，打开后实际落槽成败不预判。',
            finalizer_source=ref(Path(__file__)))
        r['target_poses_world']['lower']=prior['phase_waypoints_world']['lower'][-1]
        r['phase_waypoints_world']['lower']=prior['phase_waypoints_world']['lower'][:-1]
        r['joint_space_target_q_by_phase']['lower']=prior['joint_space_target_q_by_phase']['lower'][:-1]
        for key in ('nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad'):
            assert r[key]==prior[key],key
        for phase in ('pregrasp','approach','lift','transfer','retreat'):assert r['target_poses_world'][phase]==prior['target_poses_world'][phase]
        assert r['phase_waypoints_world']['retreat']==prior['phase_waypoints_world']['retreat']
        assert r['joint_space_target_q_by_phase']['retreat']==prior['joint_space_target_q_by_phase']['retreat']
        validate_recipe(r,original,probe.layout)
        save(CONFIG,dict(kind='physical_retry_round13_human_recipes',recipes={SID:r},evidence=ref(report_path),layout_changed=False))
    print(json.dumps(dict(passed=passed,region=region_passed,delta=delta.tolist(),bottom_gap_m=bottom_gap,release=release_passed,
        chord_failed=sum(not s['passed'] for s in chord),ideal_failed=sum(not s['passed'] for s in ideal),report=ref(report_path),recipe=ref(CONFIG) if CONFIG.exists() else None)),flush=True)


if __name__=='__main__':main()
