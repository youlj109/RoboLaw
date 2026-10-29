#!/usr/bin/env python3
"""At most eight new-layout/source-geometry candidates; no Kit/GPU/physics.

Only the added human's position and yaw change. Candidate targets use a fresh
nominal attachment, never the earlier measured 90 mm compensation.
"""
from copy import deepcopy
from pathlib import Path
import json, os, sys, time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,pose_check
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from scripts.extensions.replicator.prepare_physical_retry_round2_bowls_cpu import contact_scan,gripper_models
from kitchen_safety.extensions.replicator.scene_build_runtime import compose
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe,PHASES

SID='u_shape_bowl_sink__human_default_mpfb'
MANIFEST=ROOT/'exports/replicator_physical_retry_round6_20261029/manifest.json'
OUT=ROOT/'reports/replicator_physical_retry_round7_20261029/human_cpu'
CONFIG=ROOT/'configs/extensions/replicator/round7_layouts_20261029'/f'{SID}.json'
RECIPES=ROOT/'configs/extensions/replicator/physical_retry_round7_human_recipes_20261029.json'
LAYOUTS=(('sinkward16',(.05,.16,0.),0.),('sinkward20',(.08,.20,0.),0.),
         ('sinkward12_yawminus10',(.05,.12,0.),-10.),('sinkward16_yawplus10',(.08,.16,0.),10.))

def recipe_for(previous,original,probe,tilt):
 initial=np.array(original['targets']['approach'])@np.array(original['nominal_attachment'])
 a=np.array(previous['targets']['approach']);a[:3,:3]=a[:3,:3]@Rotation.from_euler('y',tilt,degrees=True).as_matrix()
 attachment=np.linalg.inv(a)@initial
 pre=a.copy();pre[:3,3]+=[.02,.02,.10]
 lift=a.copy();lift[2,3]+=.12
 goal0=np.array(original['targets']['lower'])@np.array(original['nominal_attachment'])
 goal=goal0.copy();goal[:2,3]+=[-.055,-.02]
 goal[:3,:3]=Rotation.from_euler('z',180. if tilt else -90.,degrees=True).as_matrix()
 floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'))
 points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3]
 goal[2,3]+=floor[3][2]+.015-points[:,2].min()
 lower=goal@np.linalg.inv(attachment)
 transfer=lower.copy();transfer[2,3]=max(lift[2,3],lower[2,3]+.25)
 retreat=lower.copy();retreat[:3,3]+=[0.,-.08,.23]
 targets=dict(zip(PHASES,(pre,a,lift,transfer,lower,retreat)))
 r=dict(scene_id=SID,task_id=probe.layout['task_id'],payload_name='bowl',layout_fixed=True,
  target_poses_world={p:t.tolist() for p,t in targets.items()},nominal_attachment=attachment.tolist(),
  initial_payload_pose_world=initial.tolist(),final_payload_pose_world=goal.tolist(),
  nominal_gripper_joint_rad=.78,planning_gripper_joint_rad=.76,grasp_info=dict(
   source_previous_grasp=previous['grasp_info'],round7_local_y_tilt_delta_deg=tilt,
   nominal_only=True,physical_contacts_verified=False,historical_measured_attachment_reused=False),
  phase_waypoints_world={},linear_motion_phases=[],hold_orientation_phases=[],
  segment_deviation_limits_m={},planning_options_by_phase={},planning_path_policy=dict(
   kind='collision_checked_free_path',previous_straight_line_gate_removed=True,
   reason_zh='用户允许新增人体小幅移位后，使用碰撞检查下的自由绕行；取消额外直线偏差硬门槛，原物理成功判据保持。'),
  retry_round=7,frozen_candidate_count=1)
 return r,goal0,goal,floor

def source_distance(probe,opener,bowl_pose):
 fcl=opener.fcl;obj=fcl.CollisionObject(opener.bowl,fcl.Transform(bowl_pose[:3,:3],bowl_pose[:3,3]));dist=[]
 for i,(name,_,_,_) in enumerate(probe.world.objects):
  if name.startswith('/World/hazard'):
   other=probe.obstacle(i);dist.append(dict(source_path=name,distance_m=float(fcl.distance(obj,other,fcl.DistanceRequest(),fcl.DistanceResult())),intersects=bool(fcl.collide(obj,other,fcl.CollisionRequest(),fcl.CollisionResult()))))
 return dict(minimum_distance_m=min(x['distance_m'] for x in dist),meshes=dist)

def main():
 assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
 if OUT.exists() or CONFIG.exists() or RECIPES.exists():raise FileExistsError('Round7 outputs exist')
 OUT.mkdir(parents=True);row=next(x for x in read(MANIFEST)['entries'] if x['scene_id']==SID)
 previous=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])))
 oldpath=geo.bound(dict(path=row['config_path'],sha256=row['config_sha256']));oldlayout=read(oldpath)
 masks=masks_for(original);w,meshes,provenance=gripper_models();trials=[];selected=None;best=None
 declared=[dict(layout=name,delta_xyz_m=list(delta),yaw_deg=yaw,local_y_tilt_delta_deg=tilt) for name,delta,yaw in LAYOUTS for tilt in (-15.,0.)]
 save(OUT/'declarations.json',dict(scene_id=SID,maximum_candidates=8,candidates=declared,source_manifest=ref(MANIFEST),source_script=ref(Path(__file__))))
 for li,(name,delta,yaw) in enumerate(LAYOUTS):
  layout=deepcopy(oldlayout);layout['obstacle']['position']=(np.array(oldlayout['obstacle']['position'])+delta).tolist();layout['obstacle']['yaw_deg']=yaw
  draft=OUT/name/'layout.json';draft.parent.mkdir();save(draft,layout);scene=draft.parent/'source_scene.usda';composition=compose(draft,scene)
  entry=dict(row,config_path=str(draft.relative_to(ROOT)),config_sha256=geo.sha(draft),source_scene_path=str(scene.relative_to(ROOT)),source_scene_sha256=geo.sha(scene))
  probe=geo.Probe(entry,previous);opener=OpenGeometry(probe)
  for tilt in (-15.,0.):
   begin=time.monotonic();r,goal0,goal,floor=recipe_for(previous,original,probe,tilt);a=np.array(r['target_poses_world']['approach']);initial=np.array(r['initial_payload_pose_world'])
   contact=contact_scan(a,initial,probe.geometry,w,meshes)
   if contact['passed']:r['planning_gripper_joint_rad']=float(min(contact['candidate_angles_rad']))
   closing=opener.check(a,initial,angles=np.linspace(0,r['planning_gripper_joint_rad'],9))
   closing_hits=[h for s in closing['samples'] for h in s['source_gripper_world_pairs'] if not probe.world._matches(h['world_object'],'/World/objects/bowl')]
   points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];bounds=np.array([points.min(0),points.max(0)]);offset=goal[:3,3]-goal0[:3,3]
   region=bool(np.all(bounds[0,:2]>=floor[2][:2]) and np.all(bounds[1,:2]<=floor[3][:2]) and np.linalg.norm(offset[:2])<=.06 and abs(offset[2])<=.06)
   destination=opener.bowl_world(goal);release=opener.check(np.array(r['target_poses_world']['lower']),goal)
   sweep=[]
   for f in np.linspace(0,1,5):
    t=np.array(r['target_poses_world']['lower']);t[:3,3]=(1-f)*t[:3,3]+f*np.array(r['target_poses_world']['retreat'])[:3,3]
    sweep.append(dict(fraction=float(f),**opener.check(t,goal,angles=(0.,))))
   checks=[];q=np.array(previous['start_q'])
   probe.phase='pregrasp';probe.carrying=False;probe.grip=0.;probe.masks=masks['pregrasp'];start_collision=probe.check(q)
   for phase in PHASES:
    c=pose_check(probe,np.array(r['target_poses_world'][phase]),q,phase,r,masks);c['phase']=phase;checks.append(c)
    if 'q_rad' in c:q=np.array(c['q_rad'])
   # Distances use the entire unchanged source human and bowl meshes, not a
   # reduced proxy. These pose samples do not establish a safe actual path.
   proximity=[]
   for phase in ('approach','lift','transfer','lower'):
    t=np.array(r['target_poses_world'][phase])@np.array(r['nominal_attachment']);proximity.append(dict(phase=phase,**source_distance(probe,opener,t)))
   lift=np.array(r['target_poses_world']['lift']);transfer=np.array(r['target_poses_world']['transfer'])
   for f in (.25,.5,.75):
    t=initial.copy();t[:3,3]=(1-f)*(lift@np.array(r['nominal_attachment']))[:3,3]+f*(transfer@np.array(r['nominal_attachment']))[:3,3]
    proximity.append(dict(phase='straight_bowl_center_transfer_probe',fraction=f,**source_distance(probe,opener,t)))
   source_ok=bool(contact['passed'] and not closing_hits and region and destination['passed'] and release['fully_open_fingers_clear'] and release['fully_open_gripper_hulls_clear'] and release['all_sampled_gripper_world_clear'] and all(s['fully_open_fingers_clear'] and s['fully_open_gripper_hulls_clear'] and s['all_sampled_gripper_world_clear'] for s in sweep))
   start_ok=not start_collision['unique_world_pairs'] and not start_collision['self_sphere_overlaps'];passed=bool(source_ok and start_ok and all(c['passed'] for c in checks))
   tr=dict(index=len(trials),declaration=declared[len(trials)],new_config=ref(draft),new_source_scene=ref(scene),composition=composition,recipe=r,source_contact=contact,closing_nonpayload_world_pairs=closing_hits,source_closing=closing,source_bowl_destination=destination,source_release=release,source_open_retreat_sweep=sweep,initial_robot=start_collision,endpoint_checks=checks,human_bowl_source_mesh_proximity=proximity,source_geometry_passed=source_ok,passed=passed,elapsed_seconds=time.monotonic()-begin,
    nominal_destination=dict(kind='round7_new_layout_nominal_source_bowl_destination',passed=region and destination['passed'],new_config=ref(draft),new_source_scene=ref(scene),nominal_attachment=r['nominal_attachment'],nominal_lower_tcp_world=r['target_poses_world']['lower'],nominal_goal_world=goal.tolist(),original_goal_world=goal0.tolist(),nominal_goal_delta_xyz_m=offset.tolist(),nominal_xy_limit_m=.06,nominal_z_limit_m=.06,sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[np.array(floor[2]).tolist(),np.array(floor[3]).tolist()],predicted_source_bowl_world_bounds_m=bounds.tolist(),predicted_bowl_bottom_to_source_floor_m=float(bounds[0,2]-floor[3][2]),source_bowl=ref(geo.local(layout['objects']['bowl']['usd_path'])),derived_from_new_nominal_attachment=True,historical_measured_attachment_reused=False,new_physical_support_measured=False,scope='Nominal complete source bowl mesh above unchanged floor; 15mm release gap permits settling. Not measured support or physical success.'))
   trials.append(tr);save(OUT/'candidates.partial.json',dict(scene_id=SID,trials=trials))
   rank=(source_ok,start_ok,sum(c['passed'] for c in checks),-sum(c['score'] for c in checks))
   if best is None or rank>best[0]:best=(rank,tr)
   print(json.dumps(dict(index=tr['index'],layout=name,tilt=tilt,source_ok=source_ok,contact=contact['passed'],start_ok=start_ok,endpoint_passes=sum(c['passed'] for c in checks),phases=[dict(phase=c['phase'],passed=c['passed'],score=c['score']) for c in checks],passed=passed)),flush=True)
   if passed:selected=tr;break
  if selected:break
 if selected is None:selected=best[1]
 selected_layout=read(geo.bound(selected['new_config']));save(CONFIG,selected_layout);final_scene=OUT/'selected_source_scene.usda';final_composition=compose(CONFIG,final_scene)
 copy=deepcopy(selected_layout);copy['obstacle']['position']=oldlayout['obstacle']['position'];copy['obstacle']['yaw_deg']=oldlayout['obstacle']['yaw_deg'];assert copy==oldlayout
 unchanged_assets={p:geo.sha(geo.local(p)) for p in [oldlayout['kitchen_usd'],oldlayout['obstacle']['usd_path'],*[o['usd_path'] for o in oldlayout['objects'].values()]]}
 region=deepcopy(selected['nominal_destination']);region['new_config']=ref(CONFIG);region['new_source_scene']=ref(final_scene);save(OUT/'selected_nominal_destination.json',region)
 report=dict(kind='round7_human_new_layout_finite_cpu_preparation',scene_id=SID,source_manifest=ref(MANIFEST),source_previous_candidate=row['candidate_planning_result'],source_original_planning=dict(path=row['result_path'],sha256=row['result_sha256']),original_config=ref(oldpath),new_config=ref(CONFIG),new_source_scene=ref(final_scene),new_composition=final_composition,source_script=ref(Path(__file__)),source_gripper_provenance=provenance,
  layout_revision=dict(only_changed_json_fields=['obstacle.position','obstacle.yaw_deg'],old_position=oldlayout['obstacle']['position'],new_position=selected_layout['obstacle']['position'],position_delta_m=(np.array(selected_layout['obstacle']['position'])-oldlayout['obstacle']['position']).tolist(),old_yaw_deg=oldlayout['obstacle']['yaw_deg'],new_yaw_deg=selected_layout['obstacle']['yaw_deg'],all_other_json_equal=True,unchanged_source_asset_sha256=unchanged_assets),
  maximum_declared_candidates=8,trials=trials,selected_index=selected['index'],cpu_screen_passed=selected['passed'],kit_started=False,curobo_calls=0,physical_episodes=0,
  limitations=['Finite source geometry and one IK branch per endpoint, not a continuous path or cooked contact audit.','Source bowl mesh and conservative sphere cover differ; endpoint warnings are retained.','15mm nominal floor gap is a release target, not observed support.','Original model, collision geometry, task objects and original physical success predicate unchanged.'])
 save(OUT/'candidates.json',report)
 r=deepcopy(selected['recipe']);r.update(config_path=str(CONFIG.relative_to(ROOT)),config_sha256=geo.sha(CONFIG),evidence=ref(OUT/'candidates.json'),previous_candidate_planning_result=row['candidate_planning_result'],cpu_screen_passed=selected['passed'],selected_round7_candidate=selected['declaration'],destination_region_check=dict(passed=region['passed'],evidence_path=str((OUT/'selected_nominal_destination.json').relative_to(ROOT)),sha256=geo.sha(OUT/'selected_nominal_destination.json')),
  description='新增人体仅小幅向水槽侧平移/转向，保留搬运区影响；重新按原碗源几何与本轮名义抓取计算直抬和偏心直立释放，退出时向初始侧上方远离水龙头。使用碰撞检查自由绕行，原物理成功判据保持。',
  remaining_cpu_warnings=[dict(phase=c['phase'],score=c['score'],collision=c.get('collision'),error=c.get('error')) for c in selected['endpoint_checks'] if not c['passed']])
 validate_recipe(r,original,selected_layout);save(RECIPES,dict(kind='physical_retry_round7_human_recipes',recipes={SID:r},evidence=ref(OUT/'candidates.json'),layout_changed=True))
 print(json.dumps(dict(selected_index=selected['index'],cpu_screen_passed=selected['passed'],config=ref(CONFIG),recipes=ref(RECIPES)),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
