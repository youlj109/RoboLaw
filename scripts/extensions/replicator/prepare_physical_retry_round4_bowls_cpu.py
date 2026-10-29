#!/usr/bin/env python3
"""Bounded round4 bowl endpoint/open-gripper screening; no Kit/CUDA/physics."""
from pathlib import Path
from copy import deepcopy
import argparse
import json
import os
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.round3_bowl_cpu_probe import Probe,binding,bound,sha
from scripts.extensions.replicator.prepare_physical_retry_round2_bowls_cpu import gripper_models
from kitchen_safety.extensions.replicator.physical_retry_round2_bowl import PHASES

MANIFEST=ROOT/'exports/replicator_physical_retry_round3_20261029/manifest.json'
PUBLICATION=ROOT/'configs/extensions/replicator/physical_retry_results_round3_20261029.json'
ANALYSIS=ROOT/'reports/replicator_physical_retry_round3_20261029/remaining_failures_grasp_analysis.json'
SCENES=('l_shape_bowl_sink__human_default_mpfb','l_shape_bowl_sink__property_wine_glass',
        'l_shape_bowl_sink__robot_narrow_opening','u_shape_bowl_sink__human_default_mpfb')

def write(path,data):
 path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text(json.dumps(data,ensure_ascii=False,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist())+'\n')

def pose(state):
 t=np.eye(4);t[:3,:3]=Rotation.from_quat(state[3:7]).as_matrix();t[:3,3]=state[:3];return t

def measured(row):
 ref=(dict(path=row['execution_result_path'],sha256=row['execution_result_sha256'])
      if row['executed'] else row['previous_retry_result'])
 result=json.loads(bound(ref).read_text());p=result['physics_evidence']
 executed_plan_ref=dict(path=result['planning_result_path'],sha256=result['planning_result_sha256'])
 executed_plan=json.loads(bound(executed_plan_ref).read_text())
 meta_ref=dict(path=p['metadata_path'],sha256=p['metadata_sha256']);state_ref=dict(path=p['q_path'],sha256=p['q_sha256'])
 meta=json.loads(bound(meta_ref).read_text())
 with np.load(bound(state_ref),allow_pickle=False) as a:
  i=int(np.flatnonzero(a['phase']=='transfer')[-1]);palm=pose(a['robot_body_pose_w'][i,meta['robot_body_names'].index('base_link')])
  tcp=palm.copy();tcp[:3,3]+=tcp[:3,:3]@np.array([.131,0,0])
  obj=pose(a['task_object_states'][i,meta['object_names'].index('bowl'),:7])
  info=dict(execution_result=ref,physics_states=state_ref,physics_metadata=meta_ref,state_index=i,
   physics_step=int(a['physics_step'][i]),phase=str(a['phase'][i]),action_index=int(a['action_index'][i]),
   source_trajectory_index=int(a['source_trajectory_index'][i]),measured_tcp_world=tcp,measured_bowl_world=obj,
   measured_attachment=np.linalg.inv(tcp)@obj,
   executed_planning_result=executed_plan_ref,executed_lower_tcp_world=executed_plan['targets']['lower'],
   scope='Measured USD base_link plus local X=0.131m; no nominal FK substituted. Transfer is a sensitivity reference, not a future rigid grasp guarantee.')
 return info

class OpenGeometry:
 def __init__(self,probe):
  import fcl
  self.fcl=fcl;self.probe=probe;self.gripper,self.meshes,self.provenance=gripper_models()
  model=fcl.BVHModel();model.beginModel(len(probe.geometry.vertices),len(probe.geometry.triangles));model.addSubModel(probe.geometry.vertices,probe.geometry.triangles);model.endModel()
  self.bowl=model

 def check(self,tcp,bowl_pose,angles=None):
  if angles is None:angles=np.linspace(.78,0.,17)
  fcl=self.fcl;probe=self.probe;w=self.gripper;palm=tcp.copy();palm[:3,3]-=palm[:3,:3]@np.array([.131,0,0])
  bowl=fcl.CollisionObject(self.bowl,fcl.Transform(bowl_pose[:3,:3],bowl_pose[:3,3]));rows=[]
  for angle in angles:
   links=w._gripper_links(dict(zip(w.GRIPPER_JOINT_NAMES,np.array([1,1,-1,1,-1,-1])*angle)))
   hits=[];finger_hits={};bowl_hull_hits=[];minimum_clearance=100.;bowl_distance=100.
   for body,parts in self.meshes.items():
    t=palm@links[body];count=0
    for _,model in parts:
     obj=fcl.CollisionObject(model,fcl.Transform(t[:3,:3],t[:3,3]));count+=int(fcl.collide(obj,bowl,fcl.CollisionRequest(),fcl.CollisionResult()))
     bowl_distance=min(bowl_distance,float(fcl.distance(obj,bowl,fcl.DistanceRequest(),fcl.DistanceResult())))
    finger_hits[body.rsplit('/',1)[-1]]=count
   for body,model,vertices in w.gripper_models:
    t=palm@links[body];points=vertices@t[:3,:3].T+t[:3,3];low,high=points.min(0),points.max(0)
    near=np.flatnonzero(np.all(low-.001<=probe.high,axis=1)&np.all(high+.001>=probe.low,axis=1))
    obj=fcl.CollisionObject(model,fcl.Transform(t[:3,:3],t[:3,3]))
    if fcl.collide(obj,bowl,fcl.CollisionRequest(),fcl.CollisionResult()):bowl_hull_hits.append(body.rsplit('/',1)[-1])
    for j in near:
     name=probe.world.objects[j][0]
     if name=='/World/objects/bowl':continue
     other=probe.obstacle(j);colliding=bool(fcl.collide(obj,other,fcl.CollisionRequest(),fcl.CollisionResult()))
     distance=float(fcl.distance(obj,other,fcl.DistanceRequest(),fcl.DistanceResult()))
     minimum_clearance=min(minimum_clearance,distance)
     if colliding or distance<.001:hits.append(dict(body=body.rsplit('/',1)[-1],world_object=name,intersection=colliding,distance_m=distance))
   rows.append(dict(angle_rad=float(angle),source_finger_bowl_intersections=finger_hits,source_gripper_bowl_hull_intersections=bowl_hull_hits,minimum_finger_bowl_distance_m=bowl_distance,source_gripper_world_pairs=hits,minimum_near_world_distance_m=minimum_clearance))
  return dict(samples=rows,fully_open_fingers_clear=not any(rows[-1]['source_finger_bowl_intersections'].values()) and rows[-1]['minimum_finger_bowl_distance_m']>=.001,
   fully_open_gripper_hulls_clear=not rows[-1]['source_gripper_bowl_hull_intersections'],
   all_sampled_gripper_world_clear=all(not r['source_gripper_world_pairs'] for r in rows),
   scope='Enabled source finger triangles versus measured-reference bowl; source-derived gripper hulls versus original world. Finite angle samples, no cooked PhysX/contact forces or physical stability guarantee.')

 def bowl_world(self,bowl_pose):
  fcl=self.fcl;probe=self.probe;points=probe.geometry.vertices@bowl_pose[:3,:3].T+bowl_pose[:3,3]
  low,high=points.min(0),points.max(0);near=np.flatnonzero(np.all(low-.001<=probe.high,axis=1)&np.all(high+.001>=probe.low,axis=1))
  obj=fcl.CollisionObject(self.bowl,fcl.Transform(bowl_pose[:3,:3],bowl_pose[:3,3]));hits=[]
  for j in near:
   name=probe.world.objects[j][0]
   if name=='/World/objects/bowl':continue
   other=probe.obstacle(j);hit=bool(fcl.collide(obj,other,fcl.CollisionRequest(),fcl.CollisionResult()))
   distance=float(fcl.distance(obj,other,fcl.DistanceRequest(),fcl.DistanceResult()))
   if hit or distance<.001:hits.append(dict(world_object=name,intersection=hit,distance_m=distance))
  return dict(passed=not hits,world_pairs=hits,scope='Complete source bowl triangle surface against original world, including floor; predicted pose only.')

def make_targets(previous,original,observed,probe,param):
 old={p:np.array(previous['targets'][p]) for p in PHASES};nominal=np.array(previous['nominal_attachment']);actual=np.array(observed['measured_attachment'])
 if param.get('lower_baseline')=='latest_executed':old['lower']=np.array(observed['executed_lower_tcp_world'])
 goal=np.array(original['targets']['lower'])@np.array(original['nominal_attachment']);desired=goal.copy()
 desired[:3,:3]=Rotation.from_euler('z',param['yaw_deg'],degrees=True).as_matrix()
 desired[:3,3]+=np.array(param.get('destination_shift_m',[0.,0.,0.]))
 corrected=desired@np.linalg.inv(actual);fraction=param['correction_fraction']
 lower=old['lower'].copy();delta=Rotation.from_matrix(corrected[:3,:3]@lower[:3,:3].T)
 lower[:3,:3]=Rotation.from_rotvec(delta.as_rotvec()*fraction).as_matrix()@lower[:3,:3]
 lower[:3,3]=(1-fraction)*old['lower'][:3,3]+fraction*corrected[:3,3]
 floor=next(o for o in probe.world.objects if o[0].endswith('/Sink/BasinBottom'))
 predicted=lower@actual;points=probe.geometry.vertices@predicted[:3,:3].T+predicted[:3,3]
 lower[2,3]+=float(floor[3][2])+.003+param.get('release_height_m',0.)-points[:,2].min()
 old['lower']=lower;old['retreat']=lower.copy();old['retreat'][:3,3]+=param.get('retreat_delta_m',[0.,0.,.20])
 final=lower@nominal;predicted=lower@actual
 delta=final[:3,3]-goal[:3,3]
 nominal_z_limit=.09 if previous['scene_id'] in ('l_shape_bowl_sink__human_default_mpfb','u_shape_bowl_sink__human_default_mpfb') else .06
 bound_ok=np.linalg.norm(delta[:2])<=.06 and abs(delta[2])<=nominal_z_limit
 points=probe.geometry.vertices@predicted[:3,:3].T+predicted[:3,3];low,high=points.min(0),points.max(0)
 region=bool(np.all(low[:2]>=np.array(floor[2])[:2]) and np.all(high[:2]<=np.array(floor[3])[:2]))
 recipe=dict(target_poses_world={p:t.tolist() for p,t in old.items()},nominal_attachment=nominal.tolist(),
  nominal_gripper_joint_rad=.78,planning_gripper_joint_rad=previous['planning_gripper_joint_rad'],
  grasp_info=dict(previous.get('grasp_info',{}),round4_parameters=param),hold_orientation_phases=['lift','transfer','lower','retreat'])
 recipe['linear_motion_phases']=list(dict.fromkeys([*previous['recipe'].get('linear_motion_phases',[]),'retreat']))
 if nominal_z_limit>.06:recipe['destination_nominal_z_limit_m']=nominal_z_limit
 check=dict(passed=region,nominal_goal_in_authorized_bounds=bool(bound_ok),nominal_xy_limit_m=.06,nominal_z_limit_m=nominal_z_limit,nominal_goal_delta_xyz_m=delta,
  original_goal_world=goal,nominal_goal_world=final,measured_reference_predicted_bowl_world=predicted,
  sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[floor[2],floor[3]],predicted_source_bowl_world_bounds_m=[low,high],
  predicted_bowl_bottom_to_source_floor_m=float(low[2]-floor[3][2]),observed_reference=observed,
  original_config=dict(path=probe.entry['config_path'],sha256=probe.entry['config_sha256']),
  original_source_scene=dict(path=probe.entry['source_scene_path'],sha256=probe.entry['source_scene_sha256']),
  scope='Fixed measured attachment sensitivity estimate; predicted complete bowl XY within original basin floor. The two human scenes permit nominal Z<=90mm solely for measured TCP/bowl compensation; actual floor target, layout, physics and task threshold unchanged. Not actual future contact/containment.')
 return recipe,check

def parameters(sid,refine=False):
 # Finite declarations: prefer sufficient upright compensation and then vary
 # yaw/XY to place wrist and open fingers inside the unchanged sink aperture.
 fractions=(0.,1.,.75,.5) if 'narrow_opening' not in sid else (1.,0.,.25,.5,.75)
 yaw0=-45. if 'human_' in sid else 0.
 if refine and 'narrow_opening' in sid:
  return [dict(correction_fraction=f,yaw_deg=y,lower_baseline='latest_executed',
   destination_shift_m=[dx,dy,0.],release_height_m=-.001,retreat_delta_m=[0.,0.,.20],
   rationale='Small opposite wrist-roll extrapolation avoids the observed upright-compensation wrist/counter envelope; actual bowl tilt remains an explicit stability warning.')
   for f,y,dx,dy in [(-.35,0.,0.,0.),(-.5,0.,0.,0.),(-.15,0.,0.,0.),(-.25,0.,0.,0.),
    (-.25,45.,0.,0.),(-.25,-45.,0.,0.),(-.25,0.,-.025,0.),(-.25,0.,0.,.025)]]
 return [dict(correction_fraction=f,yaw_deg=y,destination_shift_m=[dx,dy,0.],release_height_m=h,
              lower_baseline='latest_executed' if 'narrow_opening' in sid else 'previous_candidate',
              retreat_delta_m=[rx,ry,.20])
   for f,y,dx,dy,h,rx,ry in [(f,yaw0,0.,0.,0.,0.,0.) for f in fractions]
   +[(1. if 'narrow_opening' in sid else .5,yaw0+delta,0.,0.,0.,0.,0.) for delta in (45.,-45.,90.,-90.,135.,-135.,180.)]
   +[(.5,yaw0,dx,dy,0.,0.,0.) for dx,dy in ((-.025,0.),(0.,.025),(.025,0.),(0.,-.025))]]

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--scene',action='append');p.add_argument('--inspect-only',action='store_true');p.add_argument('--refine',action='store_true');args=p.parse_args()
 if os.environ.get('CUDA_VISIBLE_DEVICES')!='':raise RuntimeError('Hide CUDA devices for CPU screening')
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False);manifest=json.loads(MANIFEST.read_text());rows={r['scene_id']:r for r in manifest['entries']}
 scenes=args.scene or list(SCENES);recipes={};summary=[]
 write(out/'declaration.json',dict(kind='round4_bowl_finite_cpu_selection',scenes=scenes,source_manifest=binding(MANIFEST),source_publication=binding(PUBLICATION),source_analysis=binding(ANALYSIS),physical_episodes=0,curobo_calls=0,layout_changed=False))
 for sid in scenes:
  start=time.monotonic();row=rows[sid];previous=json.loads(bound(row['candidate_planning_result']).read_text());original=json.loads(bound(dict(path=row['result_path'],sha256=row['result_sha256'])).read_text())
  probe=Probe(row,previous);observed=measured(row);opener=OpenGeometry(probe);folder=out/'scenes'/sid
  write(folder/'observed_reference.json',observed)
  masks={x['phase']:x['contact_masks'] for x in original['phases']};seeds={}
  source=previous.get('artifacts',{}).get('trajectory') or original['artifacts']['trajectory']
  with np.load(bound(source),allow_pickle=False) as a:
   for phase in PHASES:seeds[phase]=a['q'][np.flatnonzero(a['phase'].astype(str)==phase)[-1]]
  trials=[];selected=None
  for index,param in enumerate(parameters(sid,args.refine)):
   recipe,region=make_targets(previous,original,observed,probe,param);t=dict(index=index,parameters=param,recipe=recipe,destination_region=region,passed=False)
   target=np.array(recipe['target_poses_world']['lower']);predicted=np.array(region['measured_reference_predicted_bowl_world'])
   t['predicted_bowl_world']=opener.bowl_world(predicted)
   t['gripper_expansion']=opener.check(target,predicted)
   if args.inspect_only or not region['nominal_goal_in_authorized_bounds']:
    t['scope']='Geometry-only/bounds screen; no IK endpoint assertion.'
   else:
    hits=0;allik=True;t['endpoints']={}
    for phase in ('lower','retreat',*PHASES[:4]):
     probe.set_phase(phase,masks[phase],recipe['nominal_attachment'],recipe['planning_gripper_joint_rad']);target=np.array(recipe['target_poses_world'][phase])
     try:
      q=probe.kin.ik(target,seeds[phase],attempts=2);actual=probe.kin.fk(q);check=probe.check(q)
      pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));re=float(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude());ok=pe<.002 and re<.035
      t['endpoints'][phase]=dict(solved=ok,q=q,position_error_m=pe,rotation_error_rad=re,world_pairs=check['unique_world_pairs'],self_sphere_overlap_count=len(check['self_sphere_overlaps']))
      hits+=len(check['unique_world_pairs']);allik&=ok
     except RuntimeError as error:t['endpoints'][phase]=dict(solved=False,error=str(error));hits+=100;allik=False
    t['world_pair_count']=hits;t['six_endpoint_ik_passed']=allik
    # Test a straight open-gripper retreat against the stationary predicted
    # bowl and source kitchen; the later cuRobo path is not claimed identical.
    retreat=np.array(recipe['target_poses_world']['retreat']);lower=np.array(recipe['target_poses_world']['lower'])
    t['open_retreat_sweep']=[]
    for fraction in np.linspace(0.,1.,9):
     tcp=lower.copy();tcp[:3,3]=(1-fraction)*lower[:3,3]+fraction*retreat[:3,3]
     g=opener.check(tcp,predicted,angles=(0.,));t['open_retreat_sweep'].append(dict(fraction=float(fraction),**g))
    sweep_ok=all(s['fully_open_fingers_clear'] and s['fully_open_gripper_hulls_clear'] and s['all_sampled_gripper_world_clear'] for s in t['open_retreat_sweep'])
    t['passed']=bool(allik and hits==0 and region['passed'] and t['predicted_bowl_world']['passed'] and t['gripper_expansion']['all_sampled_gripper_world_clear'] and t['gripper_expansion']['fully_open_fingers_clear'] and t['gripper_expansion']['fully_open_gripper_hulls_clear'] and sweep_ok)
   trials.append(t);write(folder/'cpu_candidates.partial.json',dict(scene_id=sid,candidates=trials))
   print(json.dumps(dict(scene_id=sid,index=index,parameters=param,bounds=region['nominal_goal_in_authorized_bounds'],nominal_delta=region['nominal_goal_delta_xyz_m'].tolist(),region=region['passed'],open_clear=t['gripper_expansion']['fully_open_fingers_clear'],expand_world_clear=t['gripper_expansion']['all_sampled_gripper_world_clear'],pairs=t.get('world_pair_count'),passed=t['passed'])),flush=True)
   if t['passed']:selected=t;break
  report=folder/'cpu_candidates.json';write(report,dict(scene_id=sid,candidates=trials,selected_index=selected['index'] if selected else None,source_gripper_geometry=opener.provenance,observed_reference=binding(folder/'observed_reference.json'),elapsed_seconds=time.monotonic()-start))
  summary.append(dict(scene_id=sid,selected_index=selected['index'] if selected else None,candidates=len(trials),report=binding(report)))
  if selected:
   recipe=selected['recipe'];recipe.update(scene_id=sid,task_id=row['task_id'],payload_name='bowl',layout_fixed=True,
    config_path=row['config_path'],config_sha256=row['config_sha256'],source_planning_result=row['candidate_planning_result'],
    one_selected_candidate=True,candidate_index=selected['index'],cpu_screen_passed=True,evidence=binding(report),
    description='依据上轮实测TCP—碗变换修正放置和退出姿态，并用源几何筛选全展开夹爪净空；候选未经新GPU规划或实际物理执行。')
   region_path=folder/'destination_region.json';write(region_path,selected['destination_region']);recipe['destination_region_check']=dict(passed=True,evidence_path=str(region_path.relative_to(ROOT)),sha256=sha(region_path))
   recipes[sid]=recipe;write(out/'recipes.json',dict(kind='physical_retry_round4_bowl_recipes',schema_version=1,recipes=recipes))
 write(out/'summary.json',dict(scenes=summary,selected=len(recipes),physical_episodes=0,curobo_calls=0,layout_changed=False,source=binding(Path(__file__))))

if __name__=='__main__':main()
