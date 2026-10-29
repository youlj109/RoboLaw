#!/usr/bin/env python3
"""Bounded carrying detours around the recorded human near-contact, CPU only.

Finite ideal pose samples and joint chords are evidence, not an injected path.
No Kit, cuRobo solve or native episode is started by this preparer.
"""
from copy import deepcopy
from pathlib import Path
import json, os, sys, time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo, read, save, ref, masks_for, pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe

SID = 'u_shape_bowl_sink__human_default_mpfb'
OUT = ROOT/'reports/replicator_physical_retry_round12_20261029/human_cpu'
CONFIG = ROOT/'configs/extensions/replicator/physical_retry_round12_human_recipes_20261029.json'
PREVIOUS = ROOT/'configs/extensions/replicator/physical_retry_round11_human_recipes_20261029.json'
ANALYSIS = ROOT/'reports/replicator_physical_retry_round11_20261029/human_reverse_retreat_analysis.json'
MANIFEST = ROOT/'exports/replicator_physical_retry_round11_20261029/manifest.json'


def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES') == ''
    if OUT.exists() or CONFIG.exists():
        raise FileExistsError('Round12 human preparation already exists')
    OUT.mkdir(parents=True)
    prior = read(PREVIOUS)['recipes'][SID]
    row = next(v for v in read(MANIFEST)['entries'] if v['scene_id'] == SID)
    previous = read(geo.bound(row['candidate_planning_result']))
    original = read(geo.bound(dict(path=row['result_path'], sha256=row['result_sha256'])))
    probe = geo.Probe(row, previous)
    opener = OpenGeometry(probe)
    masks = masks_for(original)
    attachment = np.asarray(prior['nominal_attachment'])
    old = read(ANALYSIS)
    qstart = np.asarray(old['transfer_q_rad'])
    qbranch = np.asarray(old['same_tcp_recorded_retreat_q_rad'])
    qlower = np.asarray(old['fixed_lower_q_rad'])
    qhigh = np.asarray(prior['joint_space_target_q_by_phase']['retreat'][0])
    start = np.asarray(prior['target_poses_world']['transfer'])
    lower = np.asarray(prior['target_poses_world']['lower'])
    high = np.asarray(prior['phase_waypoints_world']['retreat'][0])
    rotation = Slerp([0., 1.], Rotation.from_matrix([start[:3, :3], high[:3, :3]]))
    # The only known obstructed band is f=0.2..0.3 along this old ideal segment.
    detour_base = np.eye(4)
    detour_base[:3, :3] = rotation(.3).as_matrix()
    detour_base[:3, 3] = .7*start[:3, 3] + .3*high[:3, 3]
    declarations = [dict(name='away_y35mm', offset_xyz_m=[0., -.035, 0.]),
                    dict(name='away_y60mm', offset_xyz_m=[0., -.060, 0.]),
                    dict(name='away_x30_y25mm', offset_xyz_m=[-.030, -.025, 0.])]
    save(OUT/'declarations.json', dict(scene_id=SID, maximum_detour_candidates=3,
        maximum_ik_seeds_per_sample=1, ideal_samples_per_segment=11,
        joint_chord_samples_per_segment=31, candidates=declarations,
        source_script=ref(Path(__file__)), source_analysis=ref(ANALYSIS),
        scope='One fixed detour band and three finite displacement choices; no layout or terminal pose changes.'))

    def check(q, label):
        q = np.asarray(q)
        probe.phase='lower'; probe.carrying=True; probe.grip=prior['planning_gripper_joint_rad']
        probe.attachment=attachment; probe.masks=masks['lower']
        tcp=probe.kin.fk(q); goal=tcp@attachment
        collision=probe.check(q); full=opener.bowl_world(goal)
        excess=float(np.maximum(probe.kin.limits[:,0]-q, q-probe.kin.limits[:,1]).clip(min=0).max())
        return dict(label=label,q_rad=q.tolist(),tcp_world=tcp.tolist(),nominal_payload_world=goal.tolist(),
            collision=collision,full_source_bowl=full,joint_limit_excess_rad=excess,
            passed=bool(excess==0 and not collision['unique_world_pairs'] and not collision['self_sphere_overlaps'] and full['passed']))

    def chord(a,b,label):
        samples=[dict(fraction=float(f),**check((1-f)*a+f*b,label)) for f in np.linspace(0,1,31)]
        return dict(passed=all(x['passed'] for x in samples),samples=samples,
            max_abs_joint_delta_rad=float(np.max(np.abs(b-a))))

    bridge=chord(qstart,qbranch,'actual_transfer_to_same_tcp_branch')
    save(OUT/'branch_bridge.json',dict(kind='round12_human_recorded_transfer_joint_bridge',
        source_analysis=ref(ANALYSIS),source_previous_candidate=row['candidate_planning_result'],
        planning_gripper_joint_rad=probe.grip,nominal_attachment=attachment.tolist(),
        scope='31 CPU joint samples carrying the bowl; actual GPU branch/path can differ.',**bridge))
    trials=[]; chosen=None
    for declaration in declarations:
        begin=time.monotonic()
        detour=detour_base.copy();detour[:3,3]+=declaration['offset_xyz_m']
        targets=[detour,high,lower]
        segments=[]; q=qbranch.copy(); first=start.copy(); ends=[]; passed=True
        for segment,end in enumerate(targets):
            interpolation=Slerp([0.,1.],Rotation.from_matrix([first[:3,:3],end[:3,:3]]))
            samples=[]
            for f in np.linspace(0.,1.,11):
                target=np.eye(4);target[:3,:3]=interpolation(f).as_matrix()
                target[:3,3]=(1-f)*first[:3,3]+f*end[:3,3]
                ik=pose_check(probe,target,q,'lower',prior,masks)
                if 'q_rad' in ik:q=np.asarray(ik['q_rad'])
                geometry=check(q,'ideal_carrying') if 'q_rad' in ik else None
                ok=bool(ik['passed'] and geometry and geometry['passed'])
                samples.append(dict(fraction=float(f),target_world=target.tolist(),ik=ik,geometry=geometry,passed=ok))
                passed=passed and ok
            # Preserve the two existing vertical endpoints and their exact branch.
            fixed=qhigh if segment==1 else qlower if segment==2 else q.copy()
            end_check=check(fixed,'frozen_lower_waypoint')
            end_tcp=probe.kin.fk(fixed)
            pe=float(np.linalg.norm(end_tcp[:3,3]-end[:3,3]))
            re=float(Rotation.from_matrix(end_tcp[:3,:3]@end[:3,:3].T).magnitude())
            endpoint=dict(q_rad=fixed.tolist(),target_world=end.tolist(),position_error_m=pe,
                orientation_error_rad=re,geometry=end_check,passed=bool(end_check['passed'] and pe<=.001 and re<=.01))
            segments.append(dict(index=segment,ideal_samples=samples,endpoint=endpoint,
                final_ideal_to_frozen_q_max_delta_rad=float(np.max(np.abs(q-fixed)))))
            ends.append(fixed);q=fixed;first=end
            passed=passed and endpoint['passed']
        joints=[qbranch,*ends]
        chords=[chord(a,b,f'lower_joint_chord_{i}') for i,(a,b) in enumerate(zip(joints,joints[1:]))]
        passed=passed and bridge['passed'] and all(x['passed'] for x in chords)
        trial=dict(declaration=declaration,waypoint_targets=[start.tolist(),detour.tolist(),high.tolist()],
            waypoint_q_rad=[qbranch.tolist(),ends[0].tolist(),qhigh.tolist(),qlower.tolist()],
            ideal_segments=segments,joint_chords=chords,passed=passed,elapsed_seconds=time.monotonic()-begin)
        trials.append(trial)
        save(OUT/'candidates.partial.json',dict(scene_id=SID,trials=trials))
        print(json.dumps(dict(name=declaration['name'],passed=passed,
            ideal_failures=[sum(not x['passed'] for x in s['ideal_samples']) for s in segments],
            joint_chord_failures=[sum(not x['passed'] for x in c['samples']) for c in chords],
            endpoints=[s['endpoint']['passed'] for s in segments],seconds=time.monotonic()-begin)),flush=True)
        if passed:chosen=trial;break
    report=dict(kind='round12_human_fixed_layout_carry_waypoint_cpu_preparation',scene_id=SID,
        source_script=ref(Path(__file__)),source_previous_recipe=ref(PREVIOUS),source_previous_manifest=ref(MANIFEST),
        source_previous_candidate=row['candidate_planning_result'],source_reverse_analysis=ref(ANALYSIS),
        retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),
        retained_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        scene_revision=row['scene_revision'],source_gripper_provenance=opener.provenance,
        planning_gripper_joint_rad=prior['planning_gripper_joint_rad'],nominal_attachment=prior['nominal_attachment'],
        declarations=ref(OUT/'declarations.json'),branch_bridge=ref(OUT/'branch_bridge.json'),
        trials=trials,selected_name=chosen['declaration']['name'] if chosen else None,passed=chosen is not None,
        new_curobo_calls=0,new_kit_calls=0,new_physical_episodes=0,layout_changed=False,
        limitations=['Finite CPU source-geometry checks do not certify continuous collision clearance.',
            'FCL triangle distance is unsigned and may miss complete containment; future cuRobo collision checks remain required.',
            'These ideal TCP and linear joint samples are not injected as a cuRobo trajectory seed and are not actual GPU paths.',
            'The historical transfer endpoint supplies the bridge start; a new GPU prefix may select another branch.'])
    report_path=OUT/'preparation.json';save(report_path,report)
    if chosen:
        r=deepcopy(prior)
        r.update(retry_round=12,evidence=ref(report_path),previous_frozen_recipe=ref(PREVIOUS),
            previous_candidate_planning_result=row['candidate_planning_result'],source_joint_goal_evidence=ref(report_path),
            cpu_screen_passed=True,cpu_screen_scope='Source robot/payload spheres and full bowl at finite ideal TCP and 31-point joint chords, with unchanged opened retreat from prior evidence.',
            ideal_segment_samples_passed=True,ideal_segment_samples_checked_phases=['lower'],remaining_cpu_warnings=[],
            planning_path_policy=dict(kind='collision_checked_graph_joint_goals_with_carry_waypoints',previous_straight_line_gate_removed=True,
                reason_zh='固定人体、全部最终TCP与抓放姿态，仅加入携碗避开人体球包近邻的下放中间关节目标；图搜索可为单次轨迹优化供种子，CPU理想路线不替代实际cuRobo碰撞检查。'),
            description='保持第十一轮人体布局、抓姿、名义落点和退出目标。下放先在相同TCP切至已筛查关节分支，再向远离人体方向绕行并经落点上方100mm垂直下放；每段仅一个冻结关节目标，真实路径与抓放成功仍需单次规划和物理执行。',
            finalizer_source=ref(Path(__file__)))
        r['phase_waypoints_world']['lower']=chosen['waypoint_targets']
        r['joint_space_target_q_by_phase']['lower']=chosen['waypoint_q_rad']
        for key in ('target_poses_world','nominal_attachment','nominal_gripper_joint_rad','planning_gripper_joint_rad'):
            assert r[key]==prior[key],key
        assert r['phase_waypoints_world']['retreat']==prior['phase_waypoints_world']['retreat']
        assert r['joint_space_target_q_by_phase']['retreat']==prior['joint_space_target_q_by_phase']['retreat']
        validate_recipe(r,original,probe.layout)
        save(CONFIG,dict(kind='physical_retry_round12_human_recipes',recipes={SID:r},evidence=ref(report_path),layout_changed=False))
    print(json.dumps(dict(report=ref(report_path),recipe=ref(CONFIG) if CONFIG.exists() else None),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
