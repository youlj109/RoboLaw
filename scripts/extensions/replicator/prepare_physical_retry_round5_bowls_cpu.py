#!/usr/bin/env python3
"""Finite fixed-layout bowl/support CPU study; never imports Kit or starts physics."""
from pathlib import Path
from copy import deepcopy
import argparse,json,os,sys,time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.round3_bowl_cpu_probe import Probe,bound,binding,sha
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry,write,pose
from kitchen_safety.extensions.replicator.physical_retry_round2_bowl import PHASES
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
MANIFEST=ROOT/'exports/replicator_physical_retry_round4_20261029/manifest.json'
SCENES=('l_shape_bowl_sink__property_wine_glass','l_shape_bowl_sink__robot_narrow_opening')

def observed(row):
 result_ref=dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']);result=json.loads(bound(result_ref).read_text());ev=result['physics_evidence']
 states_ref=dict(path=ev['q_path'],sha256=ev['q_sha256']);meta_ref=dict(path=ev['metadata_path'],sha256=ev['metadata_sha256']);meta=json.loads(bound(meta_ref).read_text())
 predmeta=Path(row['execution_result_path']).parent/'task_predicate_metadata.json';pm=json.loads((ROOT/predmeta).read_text())
 out=dict(result=result_ref,physics_states=states_ref,physics_metadata=meta_ref,predicate_metadata=binding(ROOT/predmeta),sensor_identity=pm['sensor_identity'],thresholds=pm['thresholds'],samples={})
 with np.load(bound(states_ref),allow_pickle=False) as a:
  for phase in ('transfer','lower','release','final_settle'):
   i=int(np.flatnonzero(a['phase']==phase)[-1]);palm=pose(a['robot_body_pose_w'][i,meta['robot_body_names'].index('base_link')]);tcp=palm.copy();tcp[:3,3]+=tcp[:3,:3]@np.array([.131,0,0]);obj=pose(a['task_object_states'][i,meta['object_names'].index('bowl'),:7])
   out['samples'][phase]=dict(state_index=i,physics_step=int(a['physics_step'][i]),phase=phase,action_index=int(a['action_index'][i]),source_trajectory_index=int(a['source_trajectory_index'][i]),tcp_world=tcp,bowl_world=obj,attachment=np.linalg.inv(tcp)@obj)
 return out

def choices(sid,refine=False):
 if refine:
  return [(f,dx,dy,yaw,raise_z) for f,dx,dy,yaw,raise_z in ([(.5,-.055,-.020,0.,.013),(.5,-.055,-.020,-15.,.013),(.25,-.055,-.020,15.,.010),(0.,-.055,-.020,30.,.008)] if "wine_glass" in sid else [(0.,-.055,-.020,60.,0.),(0.,-.055,-.020,90.,0.),(0.,-.055,-.020,-60.,0.),(0.,-.055,-.020,-90.,0.)])]
 # At most eight declared candidates, no adaptive/unbounded search.
 return [(f,dx,dy,0.,0.) for f,dx,dy in ((.5,-.055,-.020),(.75,-.055,-.020),(1.,-.055,-.020),(.25,-.055,-.020),(.5,-.05,.03),(.75,-.05,.03),(1.,-.05,.03),(0.,-.055,-.020))]

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--inspect-only',action='store_true');p.add_argument('--refine',action='store_true');args=p.parse_args();assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False);m=json.loads(MANIFEST.read_text());recipes={};summ=[]
 for sid in SCENES:
  row=next(r for r in m['entries'] if r['scene_id']==sid);prev=json.loads(bound(row['candidate_planning_result']).read_text());original=json.loads(bound(dict(path=row['result_path'],sha256=row['result_sha256'])).read_text());probe=Probe(row,prev);geo=OpenGeometry(probe);obs=observed(row);folder=out/'scenes'/sid;write(folder/'observed.json',obs)
  floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'));drain=next(x for x in probe.world.objects if x[0].endswith('/Sink/Drain_0'))
  att=np.array(prev['nominal_attachment']);actual=np.array(obs['samples']['lower']['attachment']);base=np.array(prev['targets']['lower']);originalgoal=np.array(original['targets']['lower'])@np.array(original['nominal_attachment'])
  bowl=base@actual;upright=Rotation.from_euler('z',Rotation.from_matrix(bowl[:3,:3]).as_euler('xyz')[2]).as_matrix();correct=upright@actual[:3,:3].T
  masks={x['phase']:x['contact_masks'] for x in original['phases']};seeds={}
  with np.load(bound(prev['artifacts']['trajectory']),allow_pickle=False) as a:
   for ph in PHASES:seeds[ph]=a['q'][np.flatnonzero(a['phase'].astype(str)==ph)[-1]]
  trials=[];selected=None
  for index,(f,dx,dy,yaw,raise_z) in enumerate(choices(sid,args.refine)):
   lower=base.copy();lower[:3,:3]=Rotation.from_rotvec(Rotation.from_matrix(correct@base[:3,:3].T).as_rotvec()*f).as_matrix()@base[:3,:3]
   lower[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@lower[:3,:3]
   # Set nominal final XY inside unchanged 60mm neighborhood; measured relation determines actual predicted bowl.
   lower[:2,3]=(originalgoal[:3,3]+[dx,dy,0]-lower[:3,:3]@att[:3,3])[:2]
   pred=lower@actual;pts=probe.geometry.vertices@pred[:3,:3].T+pred[:3,3]
   lower[2,3]+=floor[3][2]+.003+raise_z-pts[:,2].min();pred=lower@actual;pts=probe.geometry.vertices@pred[:3,:3].T+pred[:3,3]
   nominal=lower@att;delta=nominal[:3,3]-originalgoal[:3,3];bounds=bool(np.linalg.norm(delta[:2])<=.06 and abs(delta[2])<=.06)
   region=bool(np.all(pts.min(0)[:2]>=floor[2][:2]) and np.all(pts.max(0)[:2]<=floor[3][:2]));uprightpose=pred.copy();uprightpose[:3,:3]=upright;upoints=probe.geometry.vertices@uprightpose[:3,:3].T+uprightpose[:3,3];uprightpose[2,3]+=floor[3][2]+.003-upoints[:,2].min()
   target={k:np.array(prev['targets'][k]) for k in PHASES};target['lower']=lower;target['retreat']=lower.copy();target['retreat'][2,3]+=.20
   rec=dict(scene_id=sid,task_id=row['task_id'],payload_name='bowl',layout_fixed=True,target_poses_world=target,nominal_attachment=att,nominal_gripper_joint_rad=.78,planning_gripper_joint_rad=prev['planning_gripper_joint_rad'],hold_orientation_phases=['lift','transfer','lower','retreat'],linear_motion_phases=list(dict.fromkeys([*prev['recipe'].get('linear_motion_phases',[]),'retreat'])),grasp_info=dict(prev.get('grasp_info',{}),round5=dict(orientation_fraction=f,nominal_xy_offset_m=[dx,dy],basis='round4 measured lower attachment',release_source_floor_clearance_m=.003+raise_z,yaw_delta_deg=yaw)))
   t=dict(index=index,parameters=dict(orientation_fraction=f,nominal_xy_offset_m=[dx,dy],yaw_delta_deg=yaw,release_raise_m=raise_z),recipe=rec,nominal_delta_xyz_m=delta,bounds_passed=bounds,region_passed=region,predicted_bowl_world=pred,predicted_tilt_deg=float(np.degrees(np.arccos(np.clip(pred[2,2],-1,1)))),source_bowl_world=geo.bowl_world(pred),upright_settle_hypothesis=geo.bowl_world(uprightpose),upright_settle_pose=uprightpose,passed=False)
   if bounds and not args.inspect_only:
    t['gripper_expansion']=geo.check(lower,pred);t['endpoints']={};hits=0;ik=True
    for phase in ('lower','retreat',*PHASES[:4]):
     probe.set_phase(phase,masks[phase],att,rec['planning_gripper_joint_rad']);goal=target[phase]
     try:
      q=probe.kin.ik(goal,seeds[phase],attempts=2);fk=probe.kin.fk(q);check=probe.check(q);pe=float(np.linalg.norm(fk[:3,3]-goal[:3,3]));re=float(Rotation.from_matrix(fk[:3,:3]@goal[:3,:3].T).magnitude());okay=pe<.002 and re<.035;hits+=len(check['unique_world_pairs']);ik&=okay
      t['endpoints'][phase]=dict(solved=okay,q=q,position_error_m=pe,rotation_error_rad=re,world_pairs=check['unique_world_pairs'],self_sphere_overlap_count=len(check['self_sphere_overlaps']))
     except RuntimeError as e:ik=False;hits+=100;t['endpoints'][phase]=dict(solved=False,error=str(e))
    t['world_pair_count']=hits;t['six_endpoint_ik_passed']=ik;t['open_retreat_sweep']=[]
    for s in np.linspace(0,1,9):
     tcp=lower.copy();tcp[2,3]+=.20*s;g=geo.check(tcp,pred,angles=(0.,));t['open_retreat_sweep'].append(dict(fraction=float(s),**g))
    gs=t['gripper_expansion'];clear=all(g['fully_open_fingers_clear'] and g['fully_open_gripper_hulls_clear'] and g['all_sampled_gripper_world_clear'] for g in t['open_retreat_sweep'])
    t['passed']=bool(bounds and region and ik and hits==0 and t['source_bowl_world']['passed'] and t['upright_settle_hypothesis']['passed'] and gs['fully_open_fingers_clear'] and gs['fully_open_gripper_hulls_clear'] and gs['all_sampled_gripper_world_clear'] and clear)
   trials.append(t);write(folder/'cpu_candidates.partial.json',dict(scene_id=sid,candidates=trials));print(json.dumps(dict(scene_id=sid,index=index,bounds=bounds,nominal_delta=delta.tolist(),tilt=t['predicted_tilt_deg'],payload=t['source_bowl_world']['passed'],settle=t['upright_settle_hypothesis']['passed'],ik=t.get('six_endpoint_ik_passed'),world=t.get('world_pair_count'),open=t.get('gripper_expansion',{}).get('fully_open_fingers_clear'),passed=t['passed'])),flush=True)
   if t['passed']:selected=t;break
  report=folder/'cpu_candidates.json';write(report,dict(scene_id=sid,source_manifest=binding(MANIFEST),source_candidate_planning_result=row['candidate_planning_result'],source_original_result=dict(path=row['result_path'],sha256=row['result_sha256']),observed=binding(folder/'observed.json'),floor=dict(path=floor[0],bounds=[floor[2],floor[3]]),drain=dict(path=drain[0],bounds=[drain[2],drain[3]],top_above_floor_m=float(drain[3][2]-floor[3][2]),independent_of_original_filtered_sensor=True),source_gripper_geometry=geo.provenance,candidates=trials,selected_index=selected['index'] if selected else None,scope='Finite CPU source geometry / endpoint checks only; no CUDA, Kit, physics, continuous-path certificate, cooked shape or guaranteed support. Upright settle is only a conservative geometric hypothesis, not simulated settling.'))
  if selected:
   rec=selected['recipe'];rec.update(one_selected_candidate=True,candidate_index=selected['index'],cpu_screen_passed=True,evidence=binding(report),config_path=row['config_path'],config_sha256=row['config_sha256'],source_planning_result=row['candidate_planning_result'],description='保留已成功抓持和开爪退出，依据实测夹持变换轻调释放姿态，把落点偏离独立排水口并保持在原槽底。原支撑传感器、布局、物理、控制和判据不变。')
   validate_recipe(rec,original,probe.layout);recipes[sid]=rec
  summ.append(dict(scene_id=sid,selected_index=selected['index'] if selected else None,report=binding(report)))
 write(out/'recipes.json',dict(kind='physical_retry_round5_bowl_recipes',schema_version=1,recipes=recipes))
 write(out/'summary.json',dict(entries=summ,selected=len(recipes),physical_episodes=0,curobo_calls=0,source_script=binding(Path(__file__))))
if __name__=='__main__':main()
