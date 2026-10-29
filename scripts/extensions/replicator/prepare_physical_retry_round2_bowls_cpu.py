#!/usr/bin/env python3
"""Freeze one CPU-screened bowl candidate per remaining failed scene; no Kit/GPU."""
from pathlib import Path
import argparse,json,hashlib,sys,time
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.physical_retry_round2_bowl import CANDIDATES,PHASES,make_targets
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from scripts.extensions.replicator.audit_banana_grasp_cpu import source_finger_meshes

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ref(p):return dict(path=str(Path(p).relative_to(ROOT)),sha256=sha(p))
def write(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist())+'\n')

def gripper_models():
 import fcl
 w=CollisionWorld.__new__(CollisionWorld);w.fcl=fcl
 w.gripper_models,_=w._gripper_geometry()
 meshes,provenance=source_finger_meshes(w)
 return w,meshes,provenance

def contact_scan(target,bowl_pose,geometry,w,meshes):
 import fcl
 model=fcl.BVHModel();model.beginModel(len(geometry.vertices),len(geometry.triangles));model.addSubModel(geometry.vertices,geometry.triangles);model.endModel()
 bowl=fcl.CollisionObject(model,fcl.Transform(bowl_pose[:3,:3],bowl_pose[:3,3]));palm=target.copy();palm[:3,3]-=palm[:3,:3]@np.array([.131,0,0])
 support_z=float((geometry.vertices@bowl_pose[:3,:3].T+bowl_pose[:3,3])[:,2].min())
 rows=[]
 for angle in np.arange(.60,.781,.01):
  links=w._gripper_links(dict(zip(w.GRIPPER_JOINT_NAMES,np.array([1,1,-1,1,-1,-1])*angle)))
  fingers={}
  for body,parts in meshes.items():
   t=palm@links[body];count=0
   for _,m in parts:
    response=fcl.CollisionResult();count+=int(fcl.collide(fcl.CollisionObject(m,fcl.Transform(t[:3,:3],t[:3,3])),bowl,fcl.CollisionRequest(num_max_contacts=1024,enable_contact=False),response))
   fingers[body.rsplit('/',1)[-1]]=count
  low=min(float((v@(palm@links[b])[:3,:3].T+(palm@links[b])[:3,3])[:,2].min()) for b,_,v in w.gripper_models)
  rows.append(dict(angle_rad=float(round(angle,5)),finger_source_triangle_intersections=fingers,bilateral=all(x>0 for x in fingers.values()),minimum_gripper_source_z_m=low,support_z_m=support_z,source_support_clearance_m=low-support_z))
 candidates=[x for x in rows if x['bilateral'] and x['source_support_clearance_m']>=-.0005]
 return dict(passed=bool(candidates),rows=rows,candidate_angles_rad=[x['angle_rad'] for x in candidates],scope='Source triangle intersection and support-plane estimate only; no cooked PhysX contacts, force closure, friction, obstacle collision or actual grasp proof.')

def main():
 pa=argparse.ArgumentParser();pa.add_argument('--output',type=Path,required=True);pa.add_argument('--scene',action='append');args=pa.parse_args();args.output=args.output.resolve()
 if args.output.exists():raise FileExistsError(args.output)
 args.output.mkdir(parents=True)
 source=ROOT/'reports/replicator_physical_retry_round1_20261028/remaining_failures_grasp_analysis.json';analysis=json.loads(source.read_text());rows=[x for x in analysis['entries'] if x['payload_name']=='bowl'];rows=[x for x in rows if not args.scene or x['scene_id'] in args.scene]
 w,meshes,provenance=gripper_models();geometry=PayloadGeometry(ROOT/'assets/robolab/objects/ycb/bowl.usd');recipes=[]
 write(args.output/'declaration.json',dict(kind='finite_cpu_bowl_candidate_selection',candidates=list(CANDIDATES),scenes=[x['scene_id'] for x in rows],physics_execution=False,cuda=False,selection='First declared candidate with six original-source IK endpoints and bilateral source-triangle intersections above the source support plane. If no passing candidate, retain best explicit unverified candidate, never claim acceptance.',angle_grid_rad=np.arange(.60,.781,.01).round(5).tolist(),source_contact_provenance=provenance,prior_analysis=ref(source)))
 for a in rows:
  started=time.monotonic();sid=a['scene_id'];previous_path=ROOT/a['source_planning_result']['path'];previous=json.loads(previous_path.read_text());assert sha(previous_path)==a['source_planning_result']['sha256'];cfg=ROOT/previous['config_path'];assert sha(cfg)==previous['config_sha256'];layout=json.loads(cfg.read_text());kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
  with np.load(ROOT/previous['artifacts']['trajectory']['path'],allow_pickle=False) as z:oldq=z['q'];oldphase=z['phase'].astype(str);seeds={p:oldq[np.flatnonzero(oldphase==p)[-1]] for p in PHASES}
  oldinitial=np.asarray(previous['targets']['approach'])@np.asarray(previous['nominal_attachment'])
  execution=ROOT/a['result']['path'];meta=json.loads(execution.with_name('physics_metadata.json').read_text());observed=oldinitial.copy()
  with np.load(execution.with_name('physics_states.npz'),allow_pickle=False) as z:
   phases=z['phase'].astype(str);i=int(np.flatnonzero(phases=='initial_settle')[-1]);state=z['task_object_states'][i,meta['object_names'].index('bowl'),:7];observed[:3,:3]=Rotation.from_quat(state[3:7]).as_matrix();observed[:3,3]=state[:3]
  azdelta=60. if sid=='g_shape_bowl_sink__robot_sharp_equipment_housing' else 0.
  reverse=sid=='u_shape_bowl_sink__robot_steel_clamp'
  goal_delta=np.array([-.05,.03,.012]) if sid=='peninsula_bowl_sink__property_wine_glass' else np.zeros(3)
  attempts=[];selected=None
  for n,param in enumerate(CANDIDATES):
   recipe=make_targets(previous,param,rim_azimuth_delta_deg=azdelta,reverse_jaw=reverse,destination_translation_world_m=goal_delta)
   ik={};qs={};allik=True
   for p in PHASES:
    try:
     q=kin.ik(np.asarray(recipe['target_poses_world'][p]),seeds[p],attempts=2);fk=kin.fk(q);goal=np.asarray(recipe['target_poses_world'][p]);pe=float(np.linalg.norm(fk[:3,3]-goal[:3,3]));re=float(Rotation.from_matrix(fk[:3,:3]@goal[:3,:3].T).magnitude());ok=pe<.002 and re<np.deg2rad(2);ik[p]=dict(passed=ok,position_error_m=pe,rotation_error_rad=re,q=q.tolist());qs[p]=q.tolist();allik&=ok
    except Exception as err:ik[p]=dict(passed=False,error=str(err));allik=False
   contact=contact_scan(np.asarray(recipe['target_poses_world']['approach']),observed,geometry,w,meshes)
   row=dict(index=n,parameters=dict(param,rim_azimuth_delta_deg=azdelta,reverse_jaw=reverse,destination_translation_world_m=goal_delta.tolist()),six_endpoint_ik_passed=bool(allik),endpoint_ik=ik,source_contact=contact,passed=bool(allik and contact['passed']))
   attempts.append(row)
   if row['passed']:
    # Retain original .73 command whenever present in the source-contact set;
    # otherwise use the closest declared 0.01-rad candidate, bounded <=.78.
    angles=contact['candidate_angles_rad'];recipe['planning_gripper_joint_rad']=min(angles,key=lambda x:(abs(x-.73),x));recipe['nominal_gripper_joint_rad']=.78
    selected=(n,recipe);break
  if selected is None:
   n=max(range(len(attempts)),key=lambda k:(sum(x['passed'] for x in attempts[k]['endpoint_ik'].values()),attempts[k]['source_contact']['passed'],-k));selected=(n,make_targets(previous,CANDIDATES[n],rim_azimuth_delta_deg=azdelta,reverse_jaw=reverse,destination_translation_world_m=goal_delta))
  n,recipe=selected;recipe.update(scene_id=sid,task_id=previous['task_id'],payload_name='bowl',layout_fixed=True,config_path=previous['config_path'],config_sha256=previous['config_sha256'],source_planning_result=ref(previous_path),prior_physical_result=ref(execution),one_selected_candidate=True,candidate_index=n,cpu_screen_passed=attempts[n]['passed'],description_zh='调整现有碗沿抓取姿态与夹入高度；重算名义携物变换，首个抬升端点取消横拖，保留场景布局。'+('同水槽落点X偏移-50mm/Y偏移+30mm、抬高12mm，避免继续压向原高接触异常落点。' if goal_delta.any() else ''),evidence=dict(prior_grasp_analysis=ref(source),cpu_candidates_path=str((args.output/'scenes'/sid/'cpu_candidates.json').relative_to(ROOT)),scope='Finite CPU selection only. One later cuRobo pass and one native episode; no force/cooked-shape/safety or task-success claim.'))
  write(args.output/'scenes'/sid/'cpu_candidates.json',dict(scene_id=sid,candidates=attempts,selected_index=n,elapsed_seconds=time.monotonic()-started,observed_initial_bowl_pose=observed.tolist(),source_support_plane_estimate_only=True))
  recipe['evidence']['cpu_candidates_sha256']=sha(args.output/'scenes'/sid/'cpu_candidates.json');write(args.output/'scenes'/sid/'recipe.json',recipe);recipes.append(recipe);write(args.output/'recipes.json',dict(schema_version=1,kind='physical_retry_round2_bowl_recipes',recipes=recipes));print(json.dumps(dict(scene_id=sid,candidate=n,cpu_passed=recipe['cpu_screen_passed'],grip=recipe['nominal_gripper_joint_rad'],seconds=round(time.monotonic()-started,2))),flush=True)
 if any(k.startswith(('omni.','isaacsim.')) for k in sys.modules):raise RuntimeError('Unexpected simulator import')
 write(args.output/'summary.json',dict(scenes=len(recipes),cpu_passed=sum(x['cpu_screen_passed'] for x in recipes),source_files={str(Path(__file__).relative_to(ROOT)):sha(__file__),'kitchen_safety/extensions/replicator/physical_retry_round2_bowl.py':sha(ROOT/'kitchen_safety/extensions/replicator/physical_retry_round2_bowl.py')},physics_episodes=0,curobo_calls=0,layout_changes=0))
if __name__=='__main__':main()
