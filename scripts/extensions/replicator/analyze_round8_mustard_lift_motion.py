#!/usr/bin/env python3
"""Read-only measured body-pose lift review; no FK substitution or new physics."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
SID='u_shape_mustard_bowl__robot_sharp_equipment_housing';OUT=ROOT/'reports/replicator_physical_retry_round8_20261029'
def ref(p):
 p=Path(p);return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def read(p):return json.loads(Path(p).read_text())
def bound(r):
 p=ROOT/r['path'];assert ref(p)==r;return p

def main():
 a=read(OUT/'grasp_analysis.json')['entries'][0];raw=read(bound(a['result']));meta=read(bound(a['physics_metadata']))
 with np.load(bound(a['physics_states']),allow_pickle=False) as z:phase=z['phase'];palm=z['robot_body_pose_w'][:,meta['robot_body_names'].index('base_link')]
 rotation=Rotation.from_quat(palm[:,3:7]);tcp=palm[:,:3]+rotation.apply([.131,0,0]);close=a['phase_summaries']['close']['last_state_index'];lift=np.flatnonzero(phase=='lift');loss=next(f['state_index'] for f in a['measured_keyframes'] if f['label']=='first_state_after_last_carry_loaded_contact')
 relative=rotation*rotation[close].inv();angles=relative.magnitude()*180/np.pi;delta=tcp-tcp[close]
 before=lift[(lift<=loss)&(delta[lift,2]<.020)]
 samples=[]
 for f in a['measured_keyframes']:
  i=f['state_index'];samples.append(dict(label=f['label'],state_index=i,phase=str(phase[i]),tcp_xyz_world_m=tcp[i].tolist(),tcp_delta_from_close_m=delta[i].tolist(),tcp_orientation_delta_from_close_deg=float(angles[i]),object_centroid_rise_m=f['centroid_rise_m'],object_tilt_from_world_up_deg=f['object_tilt_from_world_up_deg'],bilateral_loaded=f['bilateral_loaded']))
 report=dict(kind='measured_mustard_lift_motion_review',scene_id=SID,source_script=ref(Path(__file__)),source_analysis=ref(OUT/'grasp_analysis.json'),source_result=a['result'],source_native_states=a['physics_states'],source_native_metadata=a['physics_metadata'],source_planning_result=a['source_planning_result'],new_physics_episodes=0,new_gpu_planning_calls=0,source='Measured USD base_link position/quaternion plus source local TCP +X0.131m, same definition as grasp_analysis; no nominal path substituted.',
  samples=samples,initial_lift_before_contact_loss_and_before_20mm_tcp_rise=dict(samples=len(before),max_tcp_horizontal_displacement_m=float(np.linalg.norm(delta[before,:2],axis=1).max()),min_tcp_z_delta_m=float(delta[before,2].min()),max_tcp_orientation_delta_deg=float(angles[before].max())),
  inference_zh='实际抬升前段在离台高度不足时已有明显横移及腕部姿态变化，瓶体随后倾倒并失去双指接触；这与贴台横拖/姿态扰动导致失持的解释相容，但没有反事实测试，不能排除接触位置或局部接触模型影响。',root_cause_confirmed=False,scene_infeasibility_established=False)
 path=OUT/'mustard_lift_recorded_motion_review.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(report=ref(path),early_lift=report['initial_lift_before_contact_loss_and_before_20mm_tcp_rise'])))
if __name__=='__main__':main()
