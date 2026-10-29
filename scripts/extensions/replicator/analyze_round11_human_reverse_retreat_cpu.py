#!/usr/bin/env python3
"""Check recorded retreat states in carrying mode; no new IK or candidate."""
from pathlib import Path
import json,os,sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
SID='u_shape_bowl_sink__human_default_mpfb'
OUT=ROOT/'reports/replicator_physical_retry_round11_20261029/human_reverse_retreat_analysis.json'


def main():
    assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
    if OUT.exists():raise FileExistsError(OUT)
    plan_path=ROOT/'configs/extensions/replicator/physical_retry_round11_20261029.json'
    row=next(r for r in read(plan_path)['entries'] if r['scene_id']==SID)
    result=read(geo.bound(row['candidate_planning_result']));probe=geo.Probe(row,result)
    probe.phase='lower';probe.carrying=True;probe.grip=result['planning_gripper_joint_rad']
    probe.masks=next(p['contact_masks'] for p in result['phases'] if p['phase']=='lower')
    opener=OpenGeometry(probe);attachment=np.asarray(result['nominal_attachment'])
    cpu_path=ROOT/'reports/replicator_physical_retry_round9_20261029/human_cpu/withdraw_direction_correction.json'
    cpu=read(cpu_path);selected=next(t for t in cpu['trials'] if t['declaration']['name']==cpu['selected_name'])
    recorded=selected['ideal_samples'];assert len(recorded)==22
    partial=result['artifacts']['partial_trajectory']
    with np.load(geo.bound(partial),allow_pickle=False) as z:
        i=int(np.flatnonzero(z['phase']=='transfer')[-1]);qstart=z['q'][i].copy()
    qbranch=np.array(recorded[-1]['ik_collision']['q_rad']);qlower=np.array(result['recipe']['joint_space_target_q_by_phase']['lower'][0])
    def check(q,label):
        tcp=probe.kin.fk(q);goal=tcp@attachment;collision=probe.check(q)
        excess=float(np.maximum(probe.kin.limits[:,0]-q,q-probe.kin.limits[:,1]).clip(min=0).max())
        full_bowl=opener.bowl_world(goal)
        return dict(label=label,q_rad=q.tolist(),tcp_world=tcp.tolist(),nominal_payload_world=goal.tolist(),joint_limit_excess_rad=excess,
            collision=collision,full_source_bowl=full_bowl,
            passed=excess==0 and not collision['unique_world_pairs'] and not collision['self_sphere_overlaps'] and full_bowl['passed'])
    reverse=[check(np.array(s['ik_collision']['q_rad']),f'reversed_retreat_sample{21-i}') for i,s in enumerate(recorded[::-1])]
    # This is the minimum explicit continuity bridge between the real generated
    # transfer branch and the recorded retreat branch at the same TCP pose.
    bridge=[dict(fraction=float(f),**check((1-f)*qstart+f*qbranch,'linear_joint_branch_bridge')) for f in np.linspace(0,1,31)]
    direct=[dict(fraction=float(f),**check((1-f)*qstart+f*qlower,'direct_joint_lower_chord')) for f in np.linspace(0,1,31)]
    def summary(samples):
        failed=[dict(index=i,label=s['label'],fraction=s.get('fraction'),joint_limit_excess_rad=s['joint_limit_excess_rad'],
            world_pairs=s['collision']['unique_world_pairs'],self_pairs=s['collision']['self_sphere_overlaps'],source_bowl_pairs=s['full_source_bowl']['world_pairs']) for i,s in enumerate(samples) if not s['passed']]
        return dict(passed=not failed,samples=len(samples),failed_count=len(failed),first_failure=failed[0] if failed else None,failures=failed)
    summaries={k:summary(v) for k,v in [('reverse_carrying',reverse),('branch_bridge',bridge),('direct_joint_lower',direct)]}
    report=dict(kind='round11_human_existing_retreat_reverse_carrying_cpu_analysis',scene_id=SID,source_script=ref(Path(__file__)),source_plan=ref(plan_path),source_current_planning=row['candidate_planning_result'],
        source_partial_trajectory=partial,actual_transfer_index=i,source_existing_open_retreat_cpu=ref(cpu_path),retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),retained_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
        planning_gripper_joint_rad=probe.grip,carrying=True,nominal_attachment=result['nominal_attachment'],masks=probe.masks,
        transfer_q_rad=qstart.tolist(),same_tcp_recorded_retreat_q_rad=qbranch.tolist(),fixed_lower_q_rad=qlower.tolist(),
        joint_difference_transfer_to_lower_rad=(qlower-qstart).tolist(),joint_difference_transfer_to_same_tcp_retreat_rad=(qbranch-qstart).tolist(),
        max_abs_joint_delta_to_lower_rad=float(np.max(np.abs(qlower-qstart))),max_abs_joint_delta_to_same_tcp_retreat_rad=float(np.max(np.abs(qbranch-qstart))),
        same_tcp_branch_position_error_m=float(np.linalg.norm(probe.kin.fk(qstart)[:3,3]-probe.kin.fk(qbranch)[:3,3])),
        same_tcp_branch_orientation_error_rad=float(Rotation.from_matrix(probe.kin.fk(qstart)[:3,:3]@probe.kin.fk(qbranch)[:3,:3].T).magnitude()),
        summaries=summaries,reverse_carrying_samples=reverse,branch_bridge_samples=bridge,direct_lower_joint_chord_samples=direct,
        candidate_waypoint_advice=dict(status='suggestion_only_not_a_new_frozen_candidate',intermediate_tcp_world=recorded[10]['target_world'],intermediate_q_rad=recorded[10]['ik_collision']['q_rad'],
            description_zh='先到旧退出路径末端相同TCP的另一关节分支，再逆行到lower上方100mm，最后按已记录路径垂直下降；是否可采用取决于携碗复核与连接段检查，不能跳接两个不同q。'),
        new_ik_calls=0,new_gpu_calls=0,new_planning_calls=0,new_physical_episodes=0,
        limitations=['Finite CPU FCL/source-sphere diagnostics only; not the failed cuRobo path and not a continuous collision proof.',
            'FCL triangle distance is an unsigned surface query and can miss complete containment. Earlier GPU endpoint/open-retreat validation does not validate the carrying reverse or bridge.',
            'Round11 summary constraint combines joint/state bounds, world collision and self collision; no failed q array was preserved, so these chords cannot identify its exact failure root cause.'])
    save(OUT,report)
    print(json.dumps(dict(report=ref(OUT),max_delta_lower=report['max_abs_joint_delta_to_lower_rad'],max_delta_branch=report['max_abs_joint_delta_to_same_tcp_retreat_rad'],summaries=summaries),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
