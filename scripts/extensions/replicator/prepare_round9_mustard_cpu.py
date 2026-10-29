#!/usr/bin/env python3
"""One fixed-layout lift-corridor candidate, finite independent source CPU checks.

The previous measured lift failed after early lateral/rotational motion. These
ideal paths are only candidate preparation; every future cuRobo lift sample must
separately pass the declared corridor and orientation contract before physics.
"""
from copy import deepcopy
import json,os,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator import planning_failure_cpu_geometry as geo
from scripts.extensions.replicator.prepare_round4_other_cpu import read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_round7_other_layout_cpu import spheres,clearance
SID='u_shape_mustard_bowl__robot_sharp_equipment_housing'
OUT=ROOT/'reports/replicator_physical_retry_round9_20261029'
REPORT=OUT/'mustard_lift_corridor_cpu_preparation.json'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round9_mustard_recipes_20261029.json'
MANIFEST=ROOT/'exports/replicator_physical_retry_round8_20261029/manifest.json'


def check_seed(probe,recipe,seed,label,masks):
    q=np.asarray(seed);reference=probe.kin.fk(q);start=reference.copy();rows=[]
    ends=[np.asarray(x) for x in recipe['phase_waypoints_world']['lift']]+[np.asarray(recipe['target_poses_world']['lift'])]
    for segment,goal in enumerate(ends):
        interpolation=Slerp([0,1],Rotation.from_matrix([start[:3,:3],goal[:3,:3]]))
        for f in np.linspace(0,1,13):
            target=goal.copy();target[:3,3]=(1-f)*start[:3,3]+f*goal[:3,3];target[:3,:3]=interpolation(float(f)).as_matrix()
            check=pose_check(probe,target,q,'lift',recipe,masks);check.update(segment_index=segment,fraction=float(f))
            if not check['passed']:rows.append(check);return dict(seed_kind=label,seed_q=seed.tolist(),reference_tcp_world=reference.tolist(),passed=False,samples=rows)
            q=np.asarray(check['q_rad']);actual=probe.kin.fk(q);offset=actual[:3,3]-reference[:3,3];angle=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@reference[:3,:3].T).magnitude()))
            check.update(actual_tcp_world=actual.tolist(),lateral_from_reference_m=float(np.linalg.norm(offset[:2])),orientation_from_reference_deg=angle,z_from_reference_m=float(offset[2]),hazard=clearance(probe,spheres(probe,q,'lift',recipe),masks['lift'],True))
            gate=recipe['lift_path_validation'];check['declared_lift_contract_passed']=bool(check['lateral_from_reference_m']<=gate['max_xy_deviation_m'] and angle<=gate['max_orientation_deviation_deg'] and offset[2]>=gate['min_z_offset_m'])
            rows.append(check)
            if not check['declared_lift_contract_passed']:return dict(seed_kind=label,seed_q=seed.tolist(),reference_tcp_world=reference.tolist(),passed=False,samples=rows)
        start=goal
    return dict(seed_kind=label,seed_q=seed.tolist(),reference_tcp_world=reference.tolist(),passed=True,samples=rows,max_ideal_cpu_xy_m=max(r['lateral_from_reference_m'] for r in rows),max_ideal_cpu_orientation_deg=max(r['orientation_from_reference_deg'] for r in rows),min_ideal_cpu_z_m=min(r['z_from_reference_m'] for r in rows))


def main():
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA for CPU-only preparation')
    if REPORT.exists() or CONFIG.exists():raise FileExistsError('Frozen round9 inputs already exist')
    row=next(r for r in read(MANIFEST)['entries'] if r['scene_id']==SID);previous=read(geo.bound(row['candidate_planning_result']));recipe=deepcopy(previous['recipe']);probe=geo.Probe(row,dict(previous,failed_phase='lift'));masks=masks_for(read(ROOT/row['result_path']))
    grasp=np.asarray(recipe['target_poses_world']['approach']);points=[]
    for rise in (.030,.060):
        t=grasp.copy();t[2,3]+=rise;points.append(t.tolist())
    for k in ('evidence','new_layout'):recipe.pop(k,None)
    recipe.update(retry_round=9,retained_scene_revision=True,scene_settings_changed_this_round=False,layout_fixed=True,
      phase_waypoints_world={'lift':points},hold_orientation_phases=['lift'],linear_motion_phases=['lift'],segment_deviation_limits_m={},
      planning_options_by_phase={'lift':{'constraint_weight':20.,'enable_graph':False}},
      lift_path_validation=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002),
      planning_path_policy=dict(kind='collision_checked_lift_corridor',previous_straight_line_gate_removed=True,reason_zh='仅对起提lift施加相对approach末实际FK的8mm横向/4度姿态/不得下沉超过2mm合同；其他阶段保留自由碰撞绕行，不恢复旧全阶段5mm直线门槛。'),
      description='保留第八轮场景、45度瓶身抓姿和0.70闭合指令/0.22规划夹持角；lift先同XY、同姿态上提30mm、60mm，再到原152.485mm抬升终点。仅lift采用强姿态/竖直代价且关闭graph，并以实际返回FK硬检查8mm横移、4度旋转和-2mm下沉限制；离台后的transfer保留自由避障。',
      limitations='The numeric lift limits are a declared design contract chosen to suppress the previous 41mm/14deg early motion, not an empirically certified safe threshold. CPU ideal samples do not validate actual cuRobo paths or physical grasp retention.')
    source_trajectory=previous['artifacts']['trajectory']
    with np.load(geo.bound(source_trajectory),allow_pickle=False) as z:planned_seed=z['q'][np.flatnonzero(z['phase']=='approach')[-1]]
    raw=read(geo.bound(dict(path=row['execution_result_path'],sha256=row['execution_result_sha256'])));states=dict(path=raw['physics_evidence']['q_path'],sha256=raw['physics_evidence']['q_sha256']);meta_ref=dict(path=raw['physics_evidence']['metadata_path'],sha256=raw['physics_evidence']['metadata_sha256']);metadata=read(geo.bound(meta_ref))
    with np.load(geo.bound(states),allow_pickle=False) as z:
        i=np.flatnonzero(z['phase']=='close')[-1];indices=[metadata['joint_names'].index(f'panda_joint{j}') for j in range(1,8)];actual_seed=z['q'][i,indices]
    checked=[check_seed(probe,recipe,planned_seed,'retained_round8_planned_approach_endpoint',masks),check_seed(probe,recipe,actual_seed,'retained_round8_actual_close_endpoint_comparison',masks)]
    report=dict(kind='physical_retry_round9_mustard_lift_corridor_preparation',scene_id=SID,source_manifest=ref(MANIFEST),previous_candidate=row['candidate_planning_result'],source_trajectory=source_trajectory,source_actual_result=dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']),source_actual_states=states,source_actual_metadata=meta_ref,source_prior_lift_motion=ref(ROOT/'reports/replicator_physical_retry_round8_20261029/mustard_lift_recorded_motion_review.json'),source_prior_visual_review=ref(ROOT/'reports/replicator_physical_retry_round8_20261029/failure_visual_review.json'),retained_scene_revision=row['scene_revision'],retained_config=dict(path=row['config_path'],sha256=row['config_sha256']),retained_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),source_script=ref(Path(__file__)),source_geometry_helper=ref(Path(geo.__file__)),scene_settings_changed_this_round=False,candidate_count=1,cpu_ik_continuation_branches=2,cpu_ik_restarts_per_pose=0,new_gpu_calls=0,physics_episodes=0,original_masks=masks,checks=checked,cpu_screen_passed=all(x['passed'] for x in checked),selected_recipe=deepcopy(recipe),scope='Same source robot spheres, payload cover and world meshes as earlier finite CPU screens; nominal grasp/attachment retained. Source closure checks were already performed for this exact grasp/layout in round8. Future GPU path must independently pass its actual FK lift gate; future physical contacts remain untested.')
    save(REPORT,report);recipe['evidence']=ref(REPORT);save(CONFIG,dict(kind='physical_retry_round9_mustard_recipes',recipes={SID:recipe},evidence=ref(REPORT),scene_settings_changed_this_round=False));print(json.dumps(dict(recipe=ref(CONFIG),report=ref(REPORT),checks=[{k:v for k,v in c.items() if k not in ('samples','seed_q','reference_tcp_world')} for c in checked])))

if __name__=='__main__':main()
