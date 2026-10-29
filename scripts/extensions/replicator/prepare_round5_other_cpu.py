#!/usr/bin/env python3
"""Bounded round5 preparation: three recipes, no GPU/Kit/physical execution."""
import os,json,time
from pathlib import Path
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.prepare_round4_other_cpu import (ROOT,geo,read,save,ref,masks_for,endpoint_checks,sampled_segments)
from scripts.extensions.replicator.prepare_round3_mustard import source_screen
from scripts.extensions.replicator.prepare_round2_mustard import Gripper,scene_shapes
from scripts.extensions.replicator.prepare_physical_retry_round2_bowls_cpu import contact_scan,gripper_models
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from kitchen_safety.extensions.replicator.physical_retry_round2_bowl import make_targets as bowl_targets
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe,PHASES
OUT=ROOT/'reports/replicator_physical_retry_round5_20261029'
REPORT=OUT/'other_cpu_preparation.json'
CONFIG=ROOT/'configs/extensions/replicator/physical_retry_round5_other_recipes_20261029.json'
MANIFEST=ROOT/'exports/replicator_physical_retry_round4_20261029/manifest.json'
SIDS=('l_island_banana_bowl__robot_sharp_equipment_housing','u_shape_mustard_bowl__robot_sharp_equipment_housing','u_shape_bowl_sink__human_default_mpfb')

def points(start,end,fractions):
 out=[]
 for f in fractions:
  t=end.copy();t[:3,3]=(1-f)*start[:3,3]+f*end[:3,3];out.append(t.tolist())
 return out

def declarations(sid,previous,probe):
 r=deepcopy(previous['recipe']);a=np.array(r['target_poses_world']['approach']);lift=np.array(r['target_poses_world']['lift'])
 if 'banana' in sid:
  elbow=lift.copy();elbow[2,3]=a[2,3]+.008
  # Reuse the source-clear route; halve initial step, bounded total 8 intermediates.
  r['phase_waypoints_world']={'lift':points(a,elbow,[.125,.25,.5,.75,1.])+points(elbow,lift,[.25,.5,.75])}
  r['planning_options_by_phase']={'lift':dict(constraint_weight=20.,enable_graph=False)}
  yield dict(name='same_clear_housing_route_half_first_step_strong_orientation_no_graph'),r
 elif 'mustard' in sid:
  pre=a.copy();pre[2,3]+=.06;r['target_poses_world']['pregrasp']=pre.tolist()
  r['phase_waypoints_world']={'approach':points(pre,a,np.arange(1,6)/6),'lift':points(a,lift,np.arange(1,9)/9)}
  r['planning_options_by_phase']={p:dict(constraint_weight=20.,enable_graph=False) for p in ('approach','lift')}
  yield dict(name='same_source_body_grasp_shorter_60mm_pregrasp_10mm_descent_strong_cost_no_graph'),r
 else:
  initial=a@np.array(previous['nominal_attachment']);away=initial[:3,3]-np.array(probe.layout['obstacle']['position']);away[2]=0;away/=np.linalg.norm(away)
  for az,tilt,dz in ((30.,-10.,-.003),(60.,-10.,-.003),(-30.,-10.,-.003),(30.,0.,-.002),(60.,0.,-.002),(-30.,0.,-.002)):
   r=bowl_targets(previous,dict(jaw_axis_tilt_deg=tilt,tcp_raise_m=dz),rim_azimuth_delta_deg=az)
   for k in ('scene_id','task_id','payload_name','layout_fixed','destination_nominal_z_limit_m','destination_region_check'):
    if k in previous['recipe']:r[k]=deepcopy(previous['recipe'][k])
   r['planning_gripper_joint_rad']=.68;r['nominal_gripper_joint_rad']=.78
   a=np.array(r['target_poses_world']['approach']);lift=a.copy();lift[:3,3]+=.07*away+[0,0,.075];r['target_poses_world']['lift']=lift.tolist()
   # First lift is upward 20mm, then sideways away from human with gentle rise.
   elbow=a.copy();elbow[2,3]+=.020
   r['phase_waypoints_world']={'lift':points(a,elbow,[.5,1.])+points(elbow,lift,[.2,.4,.6,.8])}
   r['linear_motion_phases']=['retreat'];r['segment_deviation_limits_m']={'lift':.005}
   r['planning_options_by_phase']={'lift':dict(constraint_weight=20.,enable_graph=False)}
   yield dict(name=f'rim_delta{az:g}_tilt_delta{tilt:g}_z{dz:g}_away_human',azimuth_delta_deg=az,tilt_delta_deg=tilt,tcp_raise_m=dz,away_from_human_world=away.tolist()),r

def main():
 if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA')
 if REPORT.exists() or CONFIG.exists():raise FileExistsError('Frozen outputs already exist')
 OUT.mkdir(parents=True,exist_ok=True);records={r['scene_id']:r for r in read(MANIFEST)['entries']}
 report=dict(kind='physical_retry_round5_other_cpu_preparation',source_manifest=ref(MANIFEST),source_script=ref(Path(__file__)),source_geometry_helper=ref(Path(geo.__file__)),source_analysis=ref(OUT.parent/'replicator_physical_retry_round4_20261029/remaining_failures_grasp_analysis.json'),source_visual_review=ref(OUT.parent/'replicator_physical_retry_round4_20261029/failure_visual_review.json'),entries=[],layout_changed=False,physics_launched=False,kit_started=False,curobo_called=False)
 recipes={}
 for sid in SIDS:
  row=records[sid];previous=read(geo.bound(row['candidate_planning_result']));original=read(ROOT/row['result_path']);probe=geo.Probe(row,dict(previous,failed_phase='approach'));masks=masks_for(original);trials=[];best=None
  if 'mustard' in sid:gripper=Gripper();shapes=scene_shapes(geo.bound(dict(path=row['source_scene_path'],sha256=row['source_scene_sha256'])))
  if 'bowl_sink' in sid:w,meshes,provenance=gripper_models();opener=OpenGeometry(probe)
  for name,r in declarations(sid,previous,probe):
   tr=dict(candidate=name);r['scene_id']=sid;r['task_id']=row['task_id'];r['payload_name']=previous['payload_name'];r['layout_fixed']=True
   for k in ('evidence','cpu_preparation_evidence','source_config_sha256','source_result_sha256'):r.pop(k,None)
   a=np.array(r['target_poses_world']['approach']);initial=np.array(original['targets']['approach'])@np.array(original['nominal_attachment'])
   if 'mustard' in sid:
    tr['source_contact']=source_screen(r,shapes,gripper)
    if not tr['source_contact']['passed']:trials.append(tr);continue
   if 'bowl_sink' in sid:
    tr['source_contact']=contact_scan(a,initial,probe.geometry,w,meshes)
    if not tr['source_contact']['passed']:trials.append(tr);print(sid,name['name'],'contact failed',flush=True);continue
    r['planning_gripper_joint_rad']=min(tr['source_contact']['candidate_angles_rad'],key=lambda x:abs(x-.68))
    tr['source_gripper_world_closing']=opener.check(a,initial,angles=np.linspace(0,r['planning_gripper_joint_rad'],9))
    # The helper also returns child meshes of the intended bowl. Classify
    # existing payload contacts separately; no world/hazard mask is changed.
    closing=tr['source_gripper_world_closing']
    closing['nonpayload_world_pairs']=[h for s in closing['samples'] for h in s['source_gripper_world_pairs'] if not probe.world._matches(h['world_object'],'/World/objects/bowl')]
    closing['all_nonpayload_world_clear']=not closing['nonpayload_world_pairs']
   end=endpoint_checks(probe,r,previous,masks,'approach' if 'bowl_sink' in sid else ('approach' if 'mustard' in sid else 'lift'));tr['endpoints']=end
   if end['passed']:tr['sampled_ideal_segments']=sampled_segments(probe,r,end,masks,('approach','lift') if 'mustard' in sid else ('lift',))
   passed=end['passed'] and tr.get('sampled_ideal_segments',{}).get('passed',False)
   if 'bowl_sink' in sid:passed=passed and tr['source_gripper_world_closing']['all_nonpayload_world_clear']
   tr['passed']=passed;trials.append(tr)
   score=(int(passed),sum(x['passed'] for x in end['checks']),sum(x['passed'] for x in tr.get('sampled_ideal_segments',{}).get('samples',[])))
   if best is None or score>best[0]:best=(score,deepcopy(r),len(trials)-1)
   print(sid,name['name'],'endpoint',end['passed'],'samples',tr.get('sampled_ideal_segments',{}).get('passed'),'passed',passed,flush=True)
   save(REPORT.with_suffix('.partial.json'),dict(report,active_scene=sid,active_trials=trials))
   if passed:break
  if best is None:raise RuntimeError('No contact viable candidate '+sid)
  r=best[1];i=best[2];targets={p:np.array(t) for p,t in r['target_poses_world'].items()}
  r['hold_orientation_phases']=[p for p in PHASES[1:] if np.allclose(targets[p][:3,:3],targets[PHASES[PHASES.index(p)-1]][:3,:3],atol=1e-7)]
  r.update(retry_round=5,frozen_candidate_count=1,selected_round5_candidate=trials[i]['candidate'],cpu_screen_passed=trials[i]['passed'],previous_candidate_planning_result=row['candidate_planning_result'],latest_actual_result=(dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']) if row['executed'] else row['previous_retry_result']),limitations='Finite ideal source-geometry/IK checks only; actual GPU path must independently pass frozen 5mm finite-segment gate; no physical grasp/safety guarantee.')
  r['description']=('更换碗缘方位、略降抓取点，先短抬升再远离人体；根据新抓取attachment重算原同水槽放置补偿目标。' if 'bowl_sink' in sid else ('保留已通过源几何筛选的避壳折线，首段缩半，强姿态代价20且禁用graph。' if 'banana' in sid else '保留已通过源闭合筛选的瓶身抓取，预抓缩至60mm并按10mm下降，强姿态/XY代价20且禁用graph，禁止首段反向超调超过原5mm。'))+' 选中'+trials[i]['candidate']['name']+'。'
  validate_recipe(r,original,read(ROOT/row['config_path']))
  report['entries'].append(dict(scene_id=sid,previous_candidate=row['candidate_planning_result'],latest_actual_result=r['latest_actual_result'],config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),original_masks=masks,trials=trials,selected_index=i,cpu_screen_passed=trials[i]['passed'],selected=deepcopy(r)))
  recipes[sid]=r;save(REPORT.with_suffix('.partial.json'),report);del probe
 save(REPORT,report)
 for r in recipes.values():r['evidence']=ref(REPORT)
 save(CONFIG,dict(kind='physical_retry_round5_other_recipes',recipes=recipes,evidence=ref(REPORT),layout_changed=False))
 print(json.dumps(dict(config=ref(CONFIG),report=ref(REPORT))),flush=True)
if __name__=='__main__':main()
