#!/usr/bin/env python3
"""Finite, evidence-specific CPU bowl candidate selection. No CUDA/Kit/physics.

The selected recipe is a hypothesis for one subsequent cuRobo call and native
episode. Source mesh intersections do not establish force closure or safety.
"""
from pathlib import Path
import argparse
from copy import deepcopy
import json
import os
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.round3_bowl_cpu_probe import Probe,sha,binding,bound
from scripts.extensions.replicator.prepare_physical_retry_round2_bowls_cpu import gripper_models,contact_scan
from kitchen_safety.extensions.replicator.physical_retry_round2_bowl import PHASES,pose_checked

MANIFEST=ROOT/'exports/replicator_physical_retry_round2_20261028/manifest.json'
PUBLICATION=ROOT/'configs/extensions/replicator/physical_retry_results_round2_20261028.json'
ANALYSIS=ROOT/'reports/replicator_physical_retry_round2_20261028/remaining_failures_grasp_analysis.json'

def write(path,data):
 path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text(json.dumps(data,ensure_ascii=False,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist())+'\n')

def pose(value):
 result=np.eye(4);result[:3,3]=value[:3];result[:3,:3]=Rotation.from_quat(value[3:7]).as_matrix();return result

def declarations(sid,probe,refine=False):
 """Different physical/CPU evidence determines each finite candidate family."""
 floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'))
 front=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinFront'))
 back=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBack'))
 inward=(np.asarray(back[2])+back[3]-np.asarray(front[2])-front[3])/2;inward[2]=0;inward/=np.linalg.norm(inward)
 if refine and sid=='l_island_bowl_sink__human_default_mpfb':
  return 'human_lift_and_faucet_branch',[dict(grasp_azimuth_delta_deg=60.,grasp_tilt_delta_deg=-10.,
   lift_shift_world_m=[0.,-.035,0.],carry_yaw_deg=yaw,destination_shift_world_m=[0.,dy,0.])
   for yaw,dy in [(-90.,-.025),(-120.,-.025),(-60.,-.025),(-90.,0.),(-120.,0.),(180.,-.025)]]
 if refine and sid=='peninsula_bowl_sink__human_default_mpfb':
  return 'human_faucet_and_basin_right_clearance',[dict(carry_yaw_deg=yaw,destination_shift_world_m=[dx,dy,0.])
   for yaw,dx,dy in [(-45.,-.025,-.015),(-45.,-.04,0.),(-60.,-.025,0.),(-30.,-.04,0.),(-60.,-.04,-.015),(-30.,-.025,-.015)]]
 if refine and sid=='peninsula_bowl_sink__robot_projecting_crossbar':
  return 'crossbar_and_open_basin_right_clearance',[dict(grasp_azimuth_delta_deg=30.,grasp_tilt_delta_deg=-10.,destination_shift_world_m=[dx,dy,0.])
   for dx,dy in [(-.025,0.),(-.04,0.),(-.025,.015),(-.04,.015)]]
 if refine and sid in ('l_shape_bowl_sink__property_glass_bottle','l_shape_bowl_sink__robot_narrow_opening'):
  return 'observed_contact_height_release',[dict(placement_correction_fraction=f,placement_contact_height=True,release_seconds=4.)
   for f in (0.,.10,.20,.30,.80,1.)]
 if sid.endswith('robot_sharp_equipment_housing'):
  # The housing is above/north of the bowl; a pure vertical lift enters it.
  return 'sharp_lift_clearance',[dict(lift_shift_world_m=[x,y,z],grasp_tilt_delta_deg=tilt) for x,y,z,tilt in
   [(0.,-.065,0.,0.),(-.035,-.065,0.,0.),(.035,-.065,0.,0.),(0.,-.085,-.025,-10.),(-.04,-.07,-.025,-10.)]]
 if sid.endswith('robot_projecting_crossbar'):
  # Rotate around this bowl, not around a global kitchen reference.
  return 'crossbar_wrist_branch',[dict(grasp_azimuth_delta_deg=az,grasp_tilt_delta_deg=tilt) for az,tilt in
   [(-30.,-10.),(30.,-10.),(-60.,-10.),(60.,-10.),(-90.,-10.),(90.,-10.),(-30.,0.),(30.,0.)]]
 if sid=='l_island_bowl_sink__human_default_mpfb':
  return 'human_lift_branch',[dict(grasp_azimuth_delta_deg=az,grasp_tilt_delta_deg=tilt,lift_shift_world_m=shift)
   for az,tilt,shift in [(-60.,-10.,[0.,-.035,0.]),(-90.,-10.,[0.,-.035,0.]),(-120.,-10.,[0.,-.035,0.]),
    (-60.,0.,[0.,-.06,-.02]),(-90.,0.,[0.,-.06,-.02]),(60.,-10.,[0.,-.035,0.])]]
 if sid.endswith('human_default_mpfb'):
  # The previous transfer endpoint's wrist intersects the faucet: yaw the
  # carrying gripper about the nominal bowl, keeping the release basin.
  return 'human_faucet_transfer',[dict(carry_yaw_deg=yaw,transfer_height_delta_m=z,
    destination_shift_world_m=(-inward*distance).tolist()) for yaw,z,distance in
    [(45.,0.,.025),(-45.,0.,.025),(90.,0.,.025),(-90.,0.,.025),(45.,-.04,.04),(-45.,-.04,.04),(90.,-.04,.04),(-90.,-.04,.04)]]
 if sid.startswith('l_island_') or sid=='peninsula_bowl_sink__property_tablet_stand':
  # BasinFront contact: move inside the same floor bounds; inspect both
  # closed lower and opened retreat start rather than masking the wall.
  return 'basin_front_open_clearance',[dict(destination_shift_world_m=(inward*distance).tolist(),carry_yaw_deg=yaw)
   for distance,yaw in [(.025,0.),(.04,0.),(.055,0.),(.025,30.),(.025,-30.),(.04,30.),(.04,-30.)]]
 if sid in ('l_shape_bowl_sink__property_glass_bottle','l_shape_bowl_sink__robot_narrow_opening'):
  return 'observed_placement_compensation',[dict(placement_correction_fraction=f,release_seconds=4.) for f in (.5,.35,.75,1.)]
 if sid in ('l_shape_bowl_sink__property_wine_glass','l_shape_bowl_sink__property_tablet_stand','peninsula_bowl_sink__property_wine_glass'):
  return 'deeper_less_tilted_rim',[dict(grasp_tilt_delta_deg=t,grasp_height_delta_m=z) for t,z in
   [(-10.,-.004),(-10.,-.002),(-15.,-.004),(-15.,-.002),(-10.,0.),(-15.,0.),(-5.,-.004),(-5.,-.002)]]
 raise ValueError('Undeclared bowl scene: '+sid)

def targets(previous,param,measured_attachment=None,geometry=None,floor_top=None):
 old={p:pose_checked(previous['targets'][p]) for p in PHASES};a0=pose_checked(previous['nominal_attachment']);initial=old['approach']@a0
 grasp=old['approach'].copy();az=param.get('grasp_azimuth_delta_deg',0.)
 if az:
  rot=initial[:3,:3]@Rotation.from_euler('z',az,degrees=True).as_matrix()@initial[:3,:3].T
  grasp[:3,3]=initial[:3,3]+rot@(grasp[:3,3]-initial[:3,3]);grasp[:3,:3]=rot@grasp[:3,:3]
 grasp[:3,:3]=grasp[:3,:3]@Rotation.from_euler('y',param.get('grasp_tilt_delta_deg',0.),degrees=True).as_matrix()
 grasp[2,3]+=param.get('grasp_height_delta_m',0.)
 attachment=np.linalg.inv(grasp)@initial
 goals={p:old[p]@a0@np.linalg.inv(attachment) for p in PHASES};goals['approach']=grasp
 goals['pregrasp']=grasp.copy();goals['pregrasp'][:3,3]+=old['pregrasp'][:3,3]-old['approach'][:3,3]
 goals['lift'][:2,3]=grasp[:2,3];goals['lift'][:3,3]+=param.get('lift_shift_world_m',[0.,0.,0.])
 yaw=Rotation.from_euler('z',param.get('carry_yaw_deg',0.),degrees=True).as_matrix()
 for p in ('transfer','lower','retreat'):
  object_pose=old[p]@a0;object_pose[:3,:3]=yaw@object_pose[:3,:3]
  object_pose[:3,3]+=param.get('destination_shift_world_m',[0.,0.,0.])
  goals[p]=object_pose@np.linalg.inv(attachment)
 goals['transfer'][2,3]+=param.get('transfer_height_delta_m',0.)
 if 'placement_correction_fraction' in param:
  if measured_attachment is None:raise ValueError('Actual transfer attachment required')
  desired=old['lower']@a0;corrected=desired@np.linalg.inv(measured_attachment);fraction=param['placement_correction_fraction']
  goals['lower'][:3,3]=old['lower'][:3,3]+fraction*(corrected[:3,3]-old['lower'][:3,3])
  delta=Rotation.from_matrix(corrected[:3,:3]@old['lower'][:3,:3].T)
  goals['lower'][:3,:3]=Rotation.from_rotvec(delta.as_rotvec()*fraction).as_matrix()@old['lower'][:3,:3]
  if param.get('placement_contact_height'):
   predicted=goals['lower']@measured_attachment
   lowest=float((geometry.vertices@predicted[:3,:3].T+predicted[:3,3])[:,2].min())
   goals['lower'][2,3]+=floor_top+.003-lowest
  goals['retreat']=goals['lower'].copy();goals['retreat'][2,3]+=.15
 for p in PHASES:pose_checked(goals[p])
 final=goals['lower']@attachment;prior_final=old['lower']@a0
 if np.linalg.norm(final[:2,3]-prior_final[:2,3])>.06 or abs(final[2,3]-prior_final[2,3])>.06:
  raise ValueError('Candidate exceeds previous same-basin destination neighborhood')
 info=deepcopy(previous.get('grasp_info',{}));info.update(round3_parameters=param,target_tcp_world_m=grasp[:3,3].tolist(),nominal_only=True,physical_contacts_verified=False)
 return dict(target_poses_world={p:goals[p].tolist() for p in PHASES},nominal_attachment=attachment.tolist(),
  initial_payload_pose_world=initial.tolist(),final_payload_pose_world=final.tolist(),grasp_info=info,
  nominal_gripper_joint_rad=.78,planning_gripper_joint_rad=float(previous['planning_gripper_joint_rad']))

def observed(row,analysis,probe):
 if row['scene_id'] not in analysis:return None,None
 record=analysis[row['scene_id']];key=next(k for k in record['measured_keyframes'] if k['label']=='approach_end');initial=pose(key['object_pose_xyz_xyzw'])
 if row['scene_id'] not in ('l_shape_bowl_sink__property_glass_bottle','l_shape_bowl_sink__robot_narrow_opening'):return initial,None
 key=next(k for k in record['measured_keyframes'] if k['label']=='transfer_end');meta=json.loads(bound(record['physics_metadata']).read_text())
 with np.load(bound(record['physics_states']),allow_pickle=False) as data:
  q=data['q'][key['state_index'],[meta['joint_names'].index(f'panda_joint{i}') for i in range(1,8)]]
 return initial,np.linalg.inv(probe.kin.fk(q))@pose(key['object_pose_xyz_xyzw'])

def destination_check(recipe,probe):
 floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'));final=np.asarray(recipe['final_payload_pose_world'])
 points=probe.geometry.vertices@final[:3,:3].T+final[:3,3]
 low,high=points.min(0),points.max(0)
 return dict(passed=bool(np.all(low[:2]>=np.asarray(floor[2])[:2]) and np.all(high[:2]<=np.asarray(floor[3])[:2])),
  sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[floor[2],floor[3]],full_source_bowl_world_bounds_m=[low,high],
  scope='Nominal source bowl inside the unchanged basin floor XY domain only; actual contact and support unverified.')

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--scene',action='append');parser.add_argument('--refine',action='store_true');args=parser.parse_args();out=args.output.resolve()
 if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA devices for CPU candidate screening')
 if out.exists():raise FileExistsError(out)
 out.mkdir(parents=True)
 manifest=json.loads(MANIFEST.read_text());publication=json.loads(PUBLICATION.read_text());published={r['scene_id']:r['retry_round2'] for r in publication['entries'] if r.get('retry_round2')}
 analysis={r['scene_id']:r for r in json.loads(ANALYSIS.read_text())['entries']}
 rows=[r for r in manifest['entries'] if 'bowl_sink' in r['scene_id'] and r['execution_status'] in ('failed','planning_failed')]
 if len(rows)!=17:raise ValueError('Expected exactly 17 remaining bowl scenes')
 rows=[r for r in rows if not args.scene or r['scene_id'] in args.scene]
 gripper,meshes,contact_provenance=gripper_models();recipes={}
 write(out/'declaration.json',dict(kind='physical_retry_round3_bowl_cpu_selection',scene_ids=[r['scene_id'] for r in rows],
  source_manifest=binding(MANIFEST),source_publication=binding(PUBLICATION),source_analysis=binding(ANALYSIS),
  selection='Finite per-evidence declarations; first candidate with all six CPU IK/world endpoints, open-retreat start, source contacts if changed, and basin XY bound. If none pass, freeze least world-colliding documented candidate, explicitly cpu_screen_passed=false.',
  source_contact_provenance=contact_provenance,physical_episodes=0,curobo_calls=0,layout_changed=False))
 for row in rows:
  begin=time.monotonic();sid=row['scene_id'];previous=json.loads(bound(row['candidate_planning_result']).read_text());probe=Probe(row,previous)
  original=json.loads(bound(dict(path=row['result_path'],sha256=row['result_sha256'])).read_text());masks={p['phase']:p['contact_masks'] for p in original['phases']}
  source_path=bound(original['artifacts']['trajectory']);seeds={}
  with np.load(source_path,allow_pickle=False) as data:
   for p in PHASES:seeds[p]=data['q'][np.flatnonzero(data['phase'].astype(str)==p)[-1]]
  initial,measured=observed(row,analysis,probe)
  if initial is None:initial=np.asarray(previous['targets']['approach'])@np.asarray(previous['nominal_attachment'])
  family,params=declarations(sid,probe,args.refine);trials=[];selected=None
  floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'))
  diagnosis=published[sid].get('planning_failure_diagnostic') or published[sid].get('diagnostic_visual_review') or {}
  order=list(PHASES);failed=previous.get('failed_phase')
  if failed in order:order.remove(failed);order.insert(0,failed)
  write(out/'scenes'/sid/'declaration.json',dict(scene_id=sid,family=family,candidates=params,previous_planning_result=row['candidate_planning_result'],
   original_masks=masks,observed_initial_payload_pose=initial,measured_transfer_attachment=measured,diagnostic=diagnosis))
  for index,param in enumerate(params):
   trial=dict(index=index,parameters=param,passed=False,endpoint_ik={},world_pair_count=99999)
   try:
    recipe=targets(previous,param,measured,probe.geometry,float(floor[3][2]));trial['recipe']=recipe
    changed=any(param.get(k,0.) for k in ('grasp_tilt_delta_deg','grasp_height_delta_m','grasp_azimuth_delta_deg'))
    contact=contact_scan(np.asarray(recipe['target_poses_world']['approach']),initial,probe.geometry,gripper,meshes) if changed else dict(passed=True,scope='Grasp retained unchanged from prior candidate; no new source-contact claim.')
    trial['source_contact']=contact
    if changed and contact['passed']:
     recipe['planning_gripper_joint_rad']=min(contact['candidate_angles_rad'],key=lambda x:(abs(x-previous['planning_gripper_joint_rad']),x))
    region=destination_check(recipe,probe);trial['destination_region']=region
    hits=0;ikok=True
    for p in order:
     target=np.asarray(recipe['target_poses_world'][p]);probe.set_phase(p,masks[p],recipe['nominal_attachment'],recipe['planning_gripper_joint_rad'])
     try:
      q=probe.kin.ik(target,seeds[p],attempts=2);actual=probe.kin.fk(q);check=probe.check(q)
      pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));re=float(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude());ok=pe<.002 and re<.035
      trial['endpoint_ik'][p]=dict(solved=ok,q=q,position_error_m=pe,rotation_error_rad=re,world_pairs=check['unique_world_pairs'],
       self_sphere_overlap_count=len(check['self_sphere_overlaps']))
      hits+=len(check['unique_world_pairs']);ikok &=ok
      if p=='lower':
       probe.set_phase('retreat',masks['retreat'],recipe['nominal_attachment'],0.);opened=probe.check(q)
       trial['open_retreat_start']=dict(world_pairs=opened['unique_world_pairs']);hits+=len(opened['unique_world_pairs'])
     except RuntimeError as error:trial['endpoint_ik'][p]=dict(solved=False,error=str(error));ikok=False;hits+=100
    trial.update(world_pair_count=hits,passed=bool(ikok and hits==0 and contact['passed'] and region['passed']))
   except Exception as error:trial['error']=repr(error)
   trials.append(trial)
   write(out/'scenes'/sid/'cpu_candidates.partial.json',dict(scene_id=sid,candidates=trials))
   print(json.dumps(dict(scene_id=sid,index=index,family=family,passed=trial['passed'],world_pairs=trial['world_pair_count'],contact=trial.get('source_contact',{}).get('passed'),error=trial.get('error'))),flush=True)
   if trial['passed']:selected=trial;break
  if selected is None:
   usable=[t for t in trials if 'recipe' in t]
   if not usable:raise RuntimeError('No bounded candidate for '+sid)
   selected=min(usable,key=lambda t:(not t.get('source_contact',{}).get('passed'),not t.get('destination_region',{}).get('passed'),t['world_pair_count'],t['index']))
  recipe=selected['recipe'];path=out/'scenes'/sid/'cpu_candidates.json'
  write(path,dict(scene_id=sid,selected_index=selected['index'],cpu_screen_passed=selected['passed'],candidates=trials,elapsed_seconds=time.monotonic()-begin,
   source_config=dict(path=row['config_path'],sha256=row['config_sha256']),source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
   previous_planning_result=row['candidate_planning_result'],observed_transfer_attachment=measured,
   limits='CPU endpoints are not a continuous CUDA route, cooked-shape contact, stable grasp, task success or safety certificate. Self-sphere overlaps retained for inspection; exact GPU self-collision remains mandatory.'))
  recipe.update(scene_id=sid,task_id=row['task_id'],payload_name='bowl',layout_fixed=True,config_path=row['config_path'],config_sha256=row['config_sha256'],
   previous_planning_result=row['candidate_planning_result'],one_selected_candidate=True,candidate_index=selected['index'],cpu_screen_passed=selected['passed'],
   evidence=binding(path),description=family+'：'+json.dumps(selected['parameters'],ensure_ascii=False)+'。依据上轮逐景视觉/CPU诊断调整；仅CPU候选，不代表物理成功。',
   provenance=dict(publication=binding(PUBLICATION),analysis=binding(ANALYSIS),source_manifest=binding(MANIFEST)),
   hold_orientation_phases=['lift','transfer','lower'])
  region_path=out/'scenes'/sid/'destination_region.json';write(region_path,selected['destination_region'])
  recipe['destination_region_check']=dict(passed=selected['destination_region']['passed'],evidence_path=str(region_path.relative_to(ROOT)),sha256=sha(region_path))
  if selected['parameters'].get('release_seconds'):recipe['execution_overrides']=dict(release_seconds=selected['parameters']['release_seconds'])
  recipes[sid]=recipe;write(out/'recipes.json',dict(kind='physical_retry_round3_bowl_recipes',schema_version=1,recipes=recipes))
  print('SELECTED',sid,selected['index'],selected['passed'],round(time.monotonic()-begin,2),flush=True)
 if any(k.startswith(('omni.','isaacsim.')) for k in sys.modules):raise RuntimeError('Unexpected simulator import')
 write(out/'summary.json',dict(scenes=len(recipes),cpu_screen_passed=sum(r['cpu_screen_passed'] for r in recipes.values()),
  physical_episodes=0,curobo_calls=0,layout_changed=False,source_files={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),ROOT/'scripts/extensions/replicator/round3_bowl_cpu_probe.py']}))

if __name__=='__main__':main()
