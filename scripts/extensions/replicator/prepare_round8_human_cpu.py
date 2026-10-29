#!/usr/bin/env python3
"""At most twelve declared human-layout/placement candidates, CPU only."""
from copy import deepcopy
from pathlib import Path
import json,os,sys,time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from scripts.extensions.replicator.prepare_physical_retry_round2_bowls_cpu import contact_scan,gripper_models
from scripts.extensions.replicator.prepare_round7_human_cpu import source_distance
from kitchen_safety.extensions.replicator.scene_build_runtime import compose
from kitchen_safety.extensions.replicator.physical_retry_round2 import PHASES,validate_recipe
SID='u_shape_bowl_sink__human_default_mpfb'
MANIFEST=ROOT/'exports/replicator_physical_retry_round7_20261029/manifest.json'
OUT=ROOT/'reports/replicator_physical_retry_round8_20261029/human_cpu'
CONFIG=ROOT/'configs/extensions/replicator/round8_layouts_20261029'/f'{SID}.json'
RECIPES=ROOT/'configs/extensions/replicator/physical_retry_round8_human_recipes_20261029.json'
LAYOUTS=(('outward12',(.12,0.,0.),0.),('outward18',(.18,0.,0.),0.),
         ('outward12_back6_yawminus20',(.12,-.06,0.),-20.),('outward18_forward6_yaw10',(.18,.06,0.),10.))
YAWS=(0.,-90.,180.)

def endpoint_branches(probe,r,previous,masks):
 q=np.array(previous['start_q']);checks=[]
 for phase in PHASES:
  seeds=[q.copy(),np.array(previous['start_q']),q+np.array([.6,-.5,-.7,0.,.5,.2,0.])]
  branches=[]
  for seed in seeds:
   seed=np.clip(seed,probe.kin.limits[:,0]+1e-5,probe.kin.limits[:,1]-1e-5)
   c=pose_check(probe,np.array(r['target_poses_world'][phase]),seed,phase,r,masks)
   branches.append(dict(seed_q_rad=seed.tolist(),**c))
   if c['passed']:break
  selected=min(branches,key=lambda x:(not x['passed'],x['score'],x.get('position_error_m',99.)))
  check=dict(selected,phase=phase,branches=branches);checks.append(check)
  if 'q_rad' in selected:q=np.array(selected['q_rad'])
 return checks

def main():
 assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
 if OUT.exists() or CONFIG.exists() or RECIPES.exists():raise FileExistsError('Frozen round8 outputs already exist')
 OUT.mkdir(parents=True);row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
 previous=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
 oldpath=geo.bound(dict(path=row['config_path'],sha256=row['config_sha256']));oldlayout=read(oldpath)
 masks=masks_for(original);w,meshes,provenance=gripper_models();trials=[];selected=None;best=None
 declared=[dict(layout=name,position_delta_from_round7_m=list(delta),yaw_deg=yaw,bowl_yaw_deg=byaw,release_floor_gap_m=.025,transfer_from_goal_delta_m=[0.,-.10,.292823834000636]) for name,delta,yaw in LAYOUTS for byaw in YAWS]
 save(OUT/'declarations.json',dict(scene_id=SID,maximum_candidates=12,maximum_endpoint_ik_seeds=3,candidates=declared,source_manifest=ref(MANIFEST),source_script=ref(Path(__file__))))
 for li,(name,delta,yaw) in enumerate(LAYOUTS):
  layout=deepcopy(oldlayout);layout['obstacle']['position']=(np.array(oldlayout['obstacle']['position'])+delta).tolist();layout['obstacle']['yaw_deg']=yaw
  draft=OUT/name/'layout.json';draft.parent.mkdir();save(draft,layout);scene=draft.parent/'source_scene.usda';composition=compose(draft,scene)
  entry=dict(row,config_path=str(draft.relative_to(ROOT)),config_sha256=geo.sha(draft),source_scene_path=str(scene.relative_to(ROOT)),source_scene_sha256=geo.sha(scene))
  probe=geo.Probe(entry,previous);opener=OpenGeometry(probe)
  for byaw in YAWS:
   begin=time.monotonic();a=np.array(previous['targets']['approach']);initial=np.array(original['targets']['approach'])@np.array(original['nominal_attachment']);attachment=np.linalg.inv(a)@initial
   goal0=np.array(original['targets']['lower'])@np.array(original['nominal_attachment']);goal=goal0.copy();goal[:2,3]+=[-.055,-.02];goal[:3,:3]=Rotation.from_euler('z',byaw,degrees=True).as_matrix()
   floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'));points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];goal[2,3]+=floor[3][2]+.025-points[:,2].min()
   lower=goal@np.linalg.inv(attachment);transfer_goal=goal.copy();transfer_goal[1,3]-=.10;transfer_goal[2,3]=initial[2,3]+.17;transfer=transfer_goal@np.linalg.inv(attachment)
   pre=np.array(previous['targets']['pregrasp']);lift=np.array(previous['targets']['lift']);retreat=lower.copy();retreat[:3,3]+=[0.,-.08,.23]
   r=dict(scene_id=SID,task_id=layout['task_id'],payload_name='bowl',layout_fixed=True,retry_round=8,frozen_candidate_count=1,
    target_poses_world={p:t.tolist() for p,t in zip(PHASES,(pre,a,lift,transfer,lower,retreat))},nominal_attachment=attachment.tolist(),initial_payload_pose_world=initial.tolist(),final_payload_pose_world=goal.tolist(),
    nominal_gripper_joint_rad=.78,planning_gripper_joint_rad=.76,grasp_info=dict(source_previous_grasp=previous['grasp_info'],nominal_only=True,physical_contacts_verified=False,historical_measured_attachment_reused=False),
    phase_waypoints_world={},linear_motion_phases=[],hold_orientation_phases=[],segment_deviation_limits_m={},planning_options_by_phase={},planning_path_policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='继续以冻结的新人体站位和搬运端点做碰撞检查自由绕行；不用额外直线门槛，保留原物理成功判据。'))
   contact=contact_scan(a,initial,probe.geometry,w,meshes)
   if contact['passed']:r['planning_gripper_joint_rad']=float(min(contact['candidate_angles_rad']))
   closing=opener.check(a,initial,angles=np.linspace(0,r['planning_gripper_joint_rad'],9));closing_hits=[h for s in closing['samples'] for h in s['source_gripper_world_pairs'] if not probe.world._matches(h['world_object'],'/World/objects/bowl')]
   points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];bounds=np.array([points.min(0),points.max(0)]);offset=goal[:3,3]-goal0[:3,3]
   region=bool(np.all(bounds[0,:2]>=floor[2][:2]) and np.all(bounds[1,:2]<=floor[3][:2]) and np.linalg.norm(offset[:2])<=.06 and abs(offset[2])<=.06)
   destination=opener.bowl_world(goal);release=opener.check(lower,goal);sweep=[]
   for f in np.linspace(0,1,5):
    t=lower.copy();t[:3,3]=(1-f)*lower[:3,3]+f*retreat[:3,3];sweep.append(dict(fraction=float(f),**opener.check(t,goal,angles=(0.,))))
   probe.phase='pregrasp';probe.carrying=False;probe.grip=0.;probe.masks=masks['pregrasp'];start_collision=probe.check(np.array(previous['start_q']))
   checks=endpoint_branches(probe,r,previous,masks)
   proximity=[dict(phase=p,**source_distance(probe,opener,np.array(r['target_poses_world'][p])@attachment)) for p in ('approach','lift','transfer','lower')]
   for f in (.25,.5,.75):
    t=initial.copy();t[:3,3]=(1-f)*(lift@attachment)[:3,3]+f*transfer_goal[:3,3];proximity.append(dict(phase='straight_bowl_center_transfer_probe',fraction=f,**source_distance(probe,opener,t)))
   source_ok=bool(contact['passed'] and not closing_hits and region and destination['passed'] and release['fully_open_fingers_clear'] and release['fully_open_gripper_hulls_clear'] and release['all_sampled_gripper_world_clear'] and all(s['fully_open_fingers_clear'] and s['fully_open_gripper_hulls_clear'] and s['all_sampled_gripper_world_clear'] for s in sweep))
   start_ok=not start_collision['unique_world_pairs'] and not start_collision['self_sphere_overlaps'];passed=bool(source_ok and start_ok and all(c['passed'] for c in checks))
   cover_centers,cover_radii=probe.cover;cover_world=cover_centers@goal[:3,:3].T+goal[:3,3]
   foot=[dict(path=n,world_min_z_m=float(low[2]),world_max_z_m=float(high[2])) for n,_,low,high in probe.world.objects if n.startswith('/World/hazard') and 'shoe' in n.lower()]
   region_evidence=dict(kind='round8_new_layout_nominal_source_bowl_destination',passed=region and destination['passed'],new_config=ref(draft),new_source_scene=ref(scene),nominal_attachment=attachment.tolist(),nominal_lower_tcp_world=lower.tolist(),nominal_goal_world=goal.tolist(),original_goal_world=goal0.tolist(),nominal_goal_delta_xyz_m=offset.tolist(),nominal_xy_limit_m=.06,nominal_z_limit_m=.06,sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[np.array(floor[2]).tolist(),np.array(floor[3]).tolist()],predicted_source_bowl_world_bounds_m=bounds.tolist(),predicted_bowl_bottom_to_source_floor_m=float(bounds[0,2]-floor[3][2]),source_bowl=ref(geo.local(layout['objects']['bowl']['usd_path'])),payload_sphere_cover_bounds_m=[np.min(cover_world-cover_radii[:,None],axis=0).tolist(),np.max(cover_world+cover_radii[:,None],axis=0).tolist()],source_drain_bounds_m=[[np.array(x[2]).tolist(),np.array(x[3]).tolist()] for x in probe.world.objects if x[0].endswith('/Sink/Drain_0')],derived_from_new_nominal_attachment=True,historical_measured_attachment_reused=False,new_physical_support_measured=False,scope='Nominal complete source bowl and conservative sphere-cover envelope; 25mm floor gap is a release target, not observed settling or support.')
   tr=dict(index=len(trials),declaration=declared[len(trials)],new_config=ref(draft),new_source_scene=ref(scene),composition=composition,recipe=r,source_contact=contact,source_closing=closing,closing_nonpayload_world_pairs=closing_hits,source_bowl_destination=destination,source_release=release,source_open_retreat_sweep=sweep,initial_robot=start_collision,endpoint_checks=checks,human_bowl_source_mesh_proximity=proximity,human_source_feet=foot,nominal_destination=region_evidence,source_geometry_passed=source_ok,passed=passed,elapsed_seconds=time.monotonic()-begin)
   trials.append(tr);save(OUT/'candidates.partial.json',dict(scene_id=SID,trials=trials))
   rank=(source_ok,start_ok,sum(c['passed'] for c in checks),-sum(c['score'] for c in checks))
   if best is None or rank>best[0]:best=(rank,tr)
   print(json.dumps(dict(index=tr['index'],layout=name,yaw=byaw,source_ok=source_ok,start_ok=start_ok,phases=[dict(phase=c['phase'],passed=c['passed'],score=c['score'],branches=len(c['branches'])) for c in checks],passed=passed)),flush=True)
   if passed:selected=tr;break
  if selected:break
 if selected is None:selected=best[1]
 selected_layout=read(geo.bound(selected['new_config']));save(CONFIG,selected_layout);final_scene=OUT/'selected_source_scene.usda';final_composition=compose(CONFIG,final_scene)
 copy=deepcopy(selected_layout);copy['obstacle']['position']=oldlayout['obstacle']['position'];copy['obstacle']['yaw_deg']=oldlayout['obstacle']['yaw_deg'];assert copy==oldlayout
 region=deepcopy(selected['nominal_destination']);region['new_config']=ref(CONFIG);region['new_source_scene']=ref(final_scene);save(OUT/'selected_nominal_destination.json',region)
 report=dict(kind='round8_human_new_layout_finite_cpu_preparation',scene_id=SID,source_manifest=ref(MANIFEST),source_previous_candidate=row['candidate_planning_result'],source_original_planning=dict(path=row['result_path'],sha256=row['result_sha256']),original_config=ref(oldpath),new_config=ref(CONFIG),new_source_scene=ref(final_scene),new_composition=final_composition,source_script=ref(Path(__file__)),source_gripper_provenance=provenance,
  layout_revision=dict(only_changed_json_fields=['obstacle.position','obstacle.yaw_deg'],old_position=oldlayout['obstacle']['position'],new_position=selected_layout['obstacle']['position'],position_delta_m=(np.array(selected_layout['obstacle']['position'])-oldlayout['obstacle']['position']).tolist(),old_yaw_deg=oldlayout['obstacle']['yaw_deg'],new_yaw_deg=selected_layout['obstacle']['yaw_deg'],all_other_json_equal=True,source_asset_paths_sha256=final_composition['source_paths_sha256']),maximum_declared_candidates=12,maximum_endpoint_ik_seeds=3,trials=trials,selected_index=selected['index'],cpu_screen_passed=selected['passed'],kit_started=False,curobo_calls=0,physical_episodes=0,
  limitations=['Finite nominal source geometry and at most three IK branches per endpoint, not a continuous path or real contact audit.','25mm release gap is not a measured support or physical stability claim.','Standing hazard has unchanged root Z and source foot geometry; no physical human balance test.','One final candidate only; no frozen CPU joint sequence is sent to cuRobo.'])
 save(OUT/'candidates.json',report)
 r=deepcopy(selected['recipe']);r.update(config_path=str(CONFIG.relative_to(ROOT)),config_sha256=geo.sha(CONFIG),evidence=ref(OUT/'candidates.json'),previous_candidate_planning_result=row['candidate_planning_result'],cpu_screen_passed=selected['passed'],selected_round8_candidate=selected['declaration'],destination_region_check=dict(passed=region['passed'],evidence_path=str((OUT/'selected_nominal_destination.json').relative_to(ROOT)),sha256=geo.sha(OUT/'selected_nominal_destination.json')),
  description='人体在r7基础上有限外移/转向；保留已经规划通过的竖直碗缘抓姿，把搬运端点改到槽口初始侧的高位，避免复用原碰撞目标。原目标60mm邻域内直立偏心释放，源碗底距槽底25mm以使完整保守携物球包也避开原排水口；实际接触和落稳由一次原生物理判断。',
  remaining_cpu_warnings=[dict(phase=c['phase'],score=c['score'],collision=c.get('collision'),error=c.get('error')) for c in selected['endpoint_checks'] if not c['passed']])
 validate_recipe(r,original,selected_layout);save(RECIPES,dict(kind='physical_retry_round8_human_recipes',recipes={SID:r},evidence=ref(OUT/'candidates.json'),layout_changed=True))
 print(json.dumps(dict(selected_index=selected['index'],cpu_screen_passed=selected['passed'],config=ref(CONFIG),recipes=ref(RECIPES)),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
