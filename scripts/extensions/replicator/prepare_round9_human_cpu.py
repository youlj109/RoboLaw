#!/usr/bin/env python3
"""Eight fixed-layout upright placement candidates; no GPU/Kit/physics."""
from copy import deepcopy
from pathlib import Path
import json,os,sys,time
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe,PHASES
SID='u_shape_bowl_sink__human_default_mpfb'
MANIFEST=ROOT/'exports/replicator_physical_retry_round8_20261029/manifest.json'
OUT=ROOT/'reports/replicator_physical_retry_round9_20261029/human_cpu'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round9_human_recipes_20261029.json'
YAWS=(135.,150.,120.,165.,105.,225.,210.,240.)

def branches(probe,r,target,phase,seed,masks,authored):
 seeds=[seed,authored,seed+np.array([.8,0.,-1.2,0.,.4,0.,0.]),seed+np.array([-.8,0.,1.2,0.,-.4,0.,0.])]
 attempts=[]
 for s in seeds:
  s=np.clip(s,probe.kin.limits[:,0]+1e-5,probe.kin.limits[:,1]-1e-5)
  check=pose_check(probe,target,s,phase,r,masks);attempts.append(dict(seed_q_rad=s.tolist(),**check))
  if check['passed']:break
 best=min(attempts,key=lambda c:(not c['passed'],c['score'],c.get('position_error_m',99.)))
 return dict(best,phase=phase,branches=attempts)

def ideal_segment(probe,r,start,end,phase,seed,masks):
 rotation=Slerp([0.,1.],Rotation.from_matrix([start[:3,:3],end[:3,:3]]))
 samples=[];q=seed.copy()
 for fraction in np.linspace(0.,1.,7):
  target=np.eye(4);target[:3,:3]=rotation(fraction).as_matrix();target[:3,3]=(1-fraction)*start[:3,3]+fraction*end[:3,3]
  check=pose_check(probe,target,q,phase,r,masks);samples.append(dict(fraction=float(fraction),target_world=target.tolist(),**check))
  if 'q_rad' in check:q=np.array(check['q_rad'])
 return dict(passed=all(c['passed'] for c in samples),samples=samples,scope='Finite ideal straight translation and shortest rotation only; not a cuRobo curve or actual physics path.')

def main():
 assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
 if OUT.exists() or CONFIG.exists():raise FileExistsError('Round9 human output already exists')
 OUT.mkdir(parents=True);row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
 previous=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
 probe=geo.Probe(row,previous);opener=OpenGeometry(probe);masks=masks_for(original)
 partial_ref=previous['artifacts']['partial_trajectory']
 with np.load(geo.bound(partial_ref),allow_pickle=False) as array:
  transfer_index=int(np.flatnonzero(array['phase']=='transfer')[-1]);transfer_q=array['q'][transfer_index].copy()
 authored=np.array(previous['start_q']);attachment=np.array(previous['nominal_attachment']);goal0=np.array(original['targets']['lower'])@np.array(original['nominal_attachment'])
 previous_goal=np.array(previous['targets']['lower'])@attachment;floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'))
 declarations=[dict(upright_bowl_yaw_deg=yaw,release_floor_gap_m=.025,retreat_translation_world_m=[0.,-.10,.25]) for yaw in YAWS]
 save(OUT/'declarations.json',dict(scene_id=SID,maximum_candidates=8,maximum_endpoint_ik_seeds=4,candidates=declarations,source_manifest=ref(MANIFEST),source_script=ref(Path(__file__))))
 trials=[];selected=None;best=None
 for index,yaw in enumerate(YAWS):
  begin=time.monotonic();goal=previous_goal.copy();goal[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
  points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];goal[2,3]+=floor[3][2]+.025-points[:,2].min()
  lower=goal@np.linalg.inv(attachment);retreat=lower.copy();retreat[:3,3]+=[0.,-.10,.25]
  r=dict(scene_id=SID,task_id=row['task_id'],payload_name='bowl',layout_fixed=True,layout_changed=False,retry_round=9,retained_scene_revision=True,scene_settings_changed_this_round=False,
   target_poses_world=deepcopy(previous['targets']),nominal_attachment=attachment.tolist(),initial_payload_pose_world=deepcopy(previous['recipe']['initial_payload_pose_world']),final_payload_pose_world=goal.tolist(),
   nominal_gripper_joint_rad=previous['nominal_gripper_joint_rad'],planning_gripper_joint_rad=previous['planning_gripper_joint_rad'],grasp_info=dict(source_previous_grasp=previous['grasp_info'],nominal_only=True,physical_contacts_verified=False,round9_upright_destination_yaw_deg=yaw),
   phase_waypoints_world={},linear_motion_phases=[],hold_orientation_phases=[],segment_deviation_limits_m={},planning_options_by_phase={},planning_path_policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='固定r8人体布局，仅调整直立下放朝向与退出目标；默认笛卡尔碰撞检查自由绕行，不沿用额外直线门槛。'))
  r['target_poses_world'].update(lower=lower.tolist(),retreat=retreat.tolist())
  points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];bounds=np.array([points.min(0),points.max(0)]);offset=goal[:3,3]-goal0[:3,3]
  region=bool(np.all(bounds[0,:2]>=floor[2][:2]) and np.all(bounds[1,:2]<=floor[3][:2]) and np.linalg.norm(offset[:2])<=.06 and abs(offset[2])<=.06)
  destination=opener.bowl_world(goal);release=opener.check(lower,goal);retreat_geometry=opener.check(retreat,goal,angles=(0.,))
  lc=branches(probe,r,lower,'lower',transfer_q,masks,authored)
  rc=branches(probe,r,retreat,'retreat',np.array(lc.get('q_rad',transfer_q)),masks,authored)
  endpoint_passed=bool(lc['passed'] and rc['passed'])
  source_passed=bool(region and destination['passed'] and release['fully_open_fingers_clear'] and release['fully_open_gripper_hulls_clear'] and release['all_sampled_gripper_world_clear'] and retreat_geometry['fully_open_fingers_clear'] and retreat_geometry['fully_open_gripper_hulls_clear'] and retreat_geometry['all_sampled_gripper_world_clear'])
  segments={}
  if endpoint_passed and source_passed:
   segments['lower']=ideal_segment(probe,r,np.array(previous['targets']['transfer']),lower,'lower',transfer_q,masks)
   segments['retreat']=ideal_segment(probe,r,lower,retreat,'retreat',np.array(lc['q_rad']),masks)
  cover_centers,cover_radii=probe.cover;cover_world=cover_centers@goal[:3,:3].T+goal[:3,3]
  region_evidence=dict(kind='round9_retained_layout_nominal_source_bowl_destination',passed=region and destination['passed'],config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),scene_revision=row['scene_revision'],nominal_attachment=attachment.tolist(),nominal_lower_tcp_world=lower.tolist(),nominal_goal_world=goal.tolist(),original_goal_world=goal0.tolist(),nominal_goal_delta_xyz_m=offset.tolist(),nominal_xy_limit_m=.06,nominal_z_limit_m=.06,sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[np.array(floor[2]).tolist(),np.array(floor[3]).tolist()],predicted_source_bowl_world_bounds_m=bounds.tolist(),predicted_bowl_bottom_to_source_floor_m=float(bounds[0,2]-floor[3][2]),payload_sphere_cover_bounds_m=[np.min(cover_world-cover_radii[:,None],axis=0).tolist(),np.max(cover_world+cover_radii[:,None],axis=0).tolist()],source_drain_bounds_m=[[np.array(x[2]).tolist(),np.array(x[3]).tolist()] for x in probe.world.objects if x[0].endswith('/Sink/Drain_0')],source_bowl=ref(geo.local(probe.layout['objects']['bowl']['usd_path'])),historical_measured_attachment_reused=False,new_physical_support_measured=False,scope='Nominal complete source bowl and sphere cover; 25mm release gap, no measured support guarantee.')
  passed=endpoint_passed and source_passed
  tr=dict(index=index,declaration=declarations[index],recipe=r,lower_endpoint=lc,retreat_endpoint=rc,source_bowl_destination=destination,source_release=release,source_retreat_endpoint=retreat_geometry,nominal_destination=region_evidence,ideal_segments=segments,endpoints_passed=endpoint_passed,source_geometry_passed=source_passed,passed=passed,elapsed_seconds=time.monotonic()-begin)
  trials.append(tr);save(OUT/'candidates.partial.json',dict(scene_id=SID,trials=trials))
  score=(passed,source_passed,lc['passed'],rc['passed'],-lc['score']-rc['score'])
  if best is None or score>best[0]:best=(score,tr)
  print(json.dumps(dict(index=index,yaw=yaw,source_passed=source_passed,lower_passed=lc['passed'],lower_score=lc['score'],retreat_passed=rc['passed'],retreat_score=rc['score'],ideal={p:s['passed'] for p,s in segments.items()},passed=passed)),flush=True)
  if passed:selected=tr;break
 if selected is None:selected=best[1]
 report=dict(kind='round9_human_retained_layout_finite_cpu_preparation',scene_id=SID,source_manifest=ref(MANIFEST),source_previous_candidate=row['candidate_planning_result'],source_original_planning=dict(path=row['result_path'],sha256=row['result_sha256']),config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),retained_scene_revision=row['scene_revision'],source_script=ref(Path(__file__)),source_gripper_provenance=opener.provenance,
  actual_previous_planning_transfer_seed=dict(trajectory=partial_ref,index=transfer_index,q_rad=transfer_q.tolist(),scope='Only the prior solver endpoint is an IK seed, not a measured physical state or a seed enforced on the next GPU solve.'),
  maximum_candidates=8,maximum_endpoint_ik_seeds=4,selected_index=selected['index'],cpu_endpoint_and_source_screen_passed=selected['passed'],ideal_segment_samples_passed=all(s['passed'] for s in selected['ideal_segments'].values()) if selected['ideal_segments'] else None,trials=trials,layout_changed=False,kit_started=False,curobo_calls=0,physical_episodes=0,
  limitations=['Endpoint candidates and finite ideal samples do not establish the future cuRobo curve or physical contacts.','Only upright nominal payload orientations were declared; real grasp may deform or slip.','Original r8 layout, original assets, masks, controller and task predicate remain unchanged.'])
 save(OUT/'candidates.json',report);save(OUT/'selected_nominal_destination.json',selected['nominal_destination'])
 r=deepcopy(selected['recipe']);r.update(config_path=row['config_path'],config_sha256=row['config_sha256'],source_scene_path=row['source_scene_path'],source_scene_sha256=row['source_scene_sha256'],evidence=ref(OUT/'candidates.json'),previous_candidate_planning_result=row['candidate_planning_result'],frozen_candidate_count=1,cpu_screen_passed=selected['passed'],selected_round9_candidate=selected['declaration'],destination_region_check=dict(passed=selected['nominal_destination']['passed'],evidence_path=str((OUT/'selected_nominal_destination.json').relative_to(ROOT)),sha256=geo.sha(OUT/'selected_nominal_destination.json')),
  description=f'保持r8人体和源场景不变，保留已规划通过的抓取、抬升、搬运目标；仅把直立碗下放朝向改为{selected["declaration"]["upright_bowl_yaw_deg"]:g}度并调整退出目标。原名义落点60mm邻域、底距槽25mm和Drain净空保持；CPU端点/源几何与有限理想段分别记录，真实轨迹与任务成功仍需本轮单次规划/物理判定。',
  remaining_cpu_warnings=[dict(phase=c['phase'],score=c['score'],collision=c.get('collision'),error=c.get('error')) for c in (selected['lower_endpoint'],selected['retreat_endpoint']) if not c['passed']],
  ideal_segment_samples_passed=report['ideal_segment_samples_passed'])
 validate_recipe(r,original,probe.layout);save(CONFIG,dict(kind='physical_retry_round9_human_recipes',recipes={SID:r},evidence=ref(OUT/'candidates.json'),layout_changed=False))
 print(json.dumps(dict(config=ref(CONFIG),evidence=ref(OUT/'candidates.json'),selected_index=selected['index'],cpu_endpoint_passed=selected['passed']),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
