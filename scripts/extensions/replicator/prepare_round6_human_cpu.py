#!/usr/bin/env python3
"""Eight bounded nominal-only bowl destinations, fixed round5 grasp and layout.
No measured attachment is reused and no GPU/Kit/physical episode is started.
"""
from pathlib import Path
from copy import deepcopy
import argparse,json,os,sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import geo,read,save,ref,masks_for,endpoint_checks,sampled_segments
from scripts.extensions.replicator.prepare_physical_retry_round4_bowls_cpu import OpenGeometry
from scripts.extensions.replicator.prepare_physical_retry_round2_bowls_cpu import contact_scan,gripper_models
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe,PHASES
SID='u_shape_bowl_sink__human_default_mpfb'
MANIFEST=ROOT/'exports/replicator_physical_retry_round5_20261029/manifest.json'
OUT=ROOT/'reports/replicator_physical_retry_round6_20261029'

def declarations(previous,original,probe):
 attachment=np.array(previous['nominal_attachment']);goal0=np.array(original['targets']['lower'])@np.array(original['nominal_attachment']);floor=next(x for x in probe.world.objects if x[0].endswith('/Sink/BasinBottom'))
 oldgoal=np.array(previous['targets']['lower'])@attachment
 # Fixed finite list. Nominal destination is built afresh from full source bowl
 # geometry and the NEW grasp attachment, never historical measured contacts.
 for yaw in (None,-45.,-90.,0.,45.,90.,135.,180.):
  r=deepcopy(previous['recipe'])
  for key in ('destination_nominal_z_limit_m','destination_region_check','evidence','cpu_screen_passed','selected_round5_candidate','latest_actual_result','previous_candidate_planning_result','final_payload_pose_world','retry_round','frozen_candidate_count','limitations','description'):
   r.pop(key,None)
  # Remove the old grasp-derived placement narrative, keep only real source
  # grasp data. These discarded entries remain in bound historical inputs.
  for key in ('round3_parameters','round4_parameters'):r.get('grasp_info',{}).pop(key,None)
  goal=goal0.copy();goal[:2,3]+=[-.05,0.]
  goal[:3,:3]=oldgoal[:3,:3] if yaw is None else Rotation.from_euler('z',yaw,degrees=True).as_matrix()
  verts=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];goal[2,3]+=floor[3][2]+.003-verts[:,2].min()
  lower=goal@np.linalg.inv(attachment);retreat=lower.copy();retreat[2,3]+=.20
  r['target_poses_world']['lower']=lower.tolist();r['target_poses_world']['retreat']=retreat.tolist()
  r['linear_motion_phases']=['retreat'];r['hold_orientation_phases']=['approach','lift','retreat'];r['segment_deviation_limits_m']={'lift':.005}
  r['planning_options_by_phase']={'lift':dict(constraint_weight=20.,enable_graph=False)}
  r['nominal_attachment']=attachment.tolist();r['final_payload_pose_world']=goal.tolist();r['grasp_info']['round6_nominal_destination_only']=True
  yield dict(payload_yaw_deg=yaw,orientation_source='retained nominal bowl orientation' if yaw is None else 'upright source bowl yaw',nominal_xy_offset_m=[-.05,0.],source_bowl_floor_gap_m=.003),r,goal,floor,goal0

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=OUT/'human_cpu');args=p.parse_args();assert os.environ.get('CUDA_VISIBLE_DEVICES')==''
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False);m=read(MANIFEST);row=next(r for r in m['entries'] if r['scene_id']==SID);previous=read(geo.bound(row['candidate_planning_result']));original=read(geo.bound(dict(path=row['result_path'],sha256=row['result_sha256'])));probe=geo.Probe(row,previous);opener=OpenGeometry(probe);masks=masks_for(original);w,meshes,provenance=gripper_models()
 initial=np.array(original['targets']['approach'])@np.array(original['nominal_attachment']);a=np.array(previous['targets']['approach']);contact=contact_scan(a,initial,probe.geometry,w,meshes)
 closing=opener.check(a,initial,angles=np.linspace(0,previous['planning_gripper_joint_rad'],9));closing['nonpayload_world_pairs']=[h for s in closing['samples'] for h in s['source_gripper_world_pairs'] if not probe.world._matches(h['world_object'],'/World/objects/bowl')];closing['all_nonpayload_world_clear']=not closing['nonpayload_world_pairs']
 trials=[];selected=None
 for index,(declaration,recipe,goal,floor,goal0) in enumerate(declarations(previous,original,probe)):
  points=probe.geometry.vertices@goal[:3,:3].T+goal[:3,3];bounds=np.array([points.min(0),points.max(0)]);delta=goal[:3,3]-goal0[:3,3]
  region=bool(np.all(bounds[0,:2]>=floor[2][:2]) and np.all(bounds[1,:2]<=floor[3][:2]));bounded=bool(np.linalg.norm(delta[:2])<=.06 and abs(delta[2])<=.06)
  geometry=opener.bowl_world(goal);lower=np.array(recipe['target_poses_world']['lower']);release=opener.check(lower,goal);sweep=[]
  for fraction in np.linspace(0.,1.,9):
   tcp=lower.copy();tcp[2,3]+=.20*fraction;sweep.append(dict(fraction=float(fraction),**opener.check(tcp,goal,angles=(0.,))))
  end=endpoint_checks(probe,recipe,previous,masks,'approach');segments=sampled_segments(probe,recipe,end,masks,('lift',)) if end['passed'] else dict(passed=False,not_run_reason='endpoint rejected')
  source_ok=geometry['passed'] and release['fully_open_fingers_clear'] and release['fully_open_gripper_hulls_clear'] and release['all_sampled_gripper_world_clear'] and all(g['fully_open_fingers_clear'] and g['fully_open_gripper_hulls_clear'] and g['all_sampled_gripper_world_clear'] for g in sweep)
  passed=bool(region and bounded and contact['passed'] and closing['all_nonpayload_world_clear'] and source_ok and end['passed'] and segments['passed'])
  region_evidence=dict(kind='new_grasp_nominal_source_bowl_destination',passed=bool(region and bounded and geometry['passed']),nominal_attachment=recipe['nominal_attachment'],nominal_lower_tcp_world=recipe['target_poses_world']['lower'],nominal_goal_world=goal.tolist(),original_goal_world=goal0.tolist(),nominal_goal_delta_xyz_m=delta.tolist(),nominal_xy_limit_m=.06,nominal_z_limit_m=.06,sink_floor_source_path=floor[0],sink_floor_world_bounds_m=[np.array(floor[2]).tolist(),np.array(floor[3]).tolist()],predicted_source_bowl_world_bounds_m=bounds.tolist(),predicted_bowl_bottom_to_source_floor_m=float(bounds[0,2]-floor[3][2]),original_config=dict(path=row['config_path'],sha256=row['config_sha256']),original_source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),source_bowl=dict(path=str(probe.geometry.path.relative_to(ROOT)) if hasattr(probe.geometry,'path') else probe.layout['objects']['bowl']['usd_path'],sha256=geo.sha(geo.local(probe.layout['objects']['bowl']['usd_path']))),source_previous_grasp_planning_result=row['candidate_planning_result'],derived_from_new_nominal_attachment=True,historical_measured_attachment_reused=False,new_physical_support_measured=False,scope='Full original source bowl geometry under lower TCP @ current nominal grasp attachment; only nominal source geometry. Not an observed/rigid grasp, cooked-contact, settling, or physical support certificate.')
  tr=dict(index=index,declaration=declaration,recipe=recipe,nominal_destination=region_evidence,source_bowl_world=geometry,source_gripper_release=release,source_open_retreat_sweep=sweep,endpoints=end,sampled_lift_segments=segments,passed=passed);trials.append(tr)
  save(out/'candidates.partial.json',dict(scene_id=SID,trials=trials));print(json.dumps(dict(index=index,yaw=declaration['payload_yaw_deg'],bounded=bounded,delta=delta.tolist(),region=region,payload_clear=geometry['passed'],release_clear=release['all_sampled_gripper_world_clear'],endpoints=end['passed'],last=end['checks'][-1]['phase'],segments=segments['passed'],passed=passed)),flush=True)
  if passed:selected=tr;break
 report=dict(kind='round6_human_bowl_finite_nominal_cpu_preparation',scene_id=SID,source_manifest=ref(MANIFEST),source_previous_planning=row['candidate_planning_result'],source_original_planning=dict(path=row['result_path'],sha256=row['result_sha256']),source_previous_diagnosis=ref(ROOT/'reports/replicator_physical_retry_round5_20261029/planning_failure_diagnostics.json'),source_previous_scope_review=ref(ROOT/'reports/replicator_physical_retry_round5_20261029/round5_adapter_readonly_review.json'),source_script=ref(Path(__file__)),source_gripper_provenance=provenance,source_contact=contact,source_closing=closing,trials=trials,selected_index=selected['index'] if selected else None,maximum_declared_candidates=8,physical_episodes=0,curobo_calls=0,layout_changed=False,historical_measured_attachment_reused=False)
 save(out/'candidates.json',report)
 if selected:
  save(out/'selected_nominal_destination.json',selected['nominal_destination']);r=selected['recipe'];r['destination_region_check']=dict(passed=True,evidence_path=str((out/'selected_nominal_destination.json').relative_to(ROOT)),sha256=geo.sha(out/'selected_nominal_destination.json'));r['evidence']=ref(out/'candidates.json');r['cpu_screen_passed']=True;r['description']='保留round5新抓姿与先竖直后远离人体的抬升；原名义目标60mm邻域内，以本轮nominal_attachment和完整源碗几何重新计算放置，取消旧90mm实测补偿继承。实际支撑仍待本轮物理验证。';validate_recipe(r,original,probe.layout);save(out/'selected_recipe.json',r)
 print(json.dumps(dict(report=ref(out/'candidates.json'),selected_index=report['selected_index']),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
