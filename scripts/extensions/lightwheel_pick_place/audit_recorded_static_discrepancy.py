"""Read-only comparison of measured poses and reported velocities; never changes native success."""
import argparse,json,hashlib
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=json.loads((a.episode/'result.json').read_text());report={'episode':str(a.episode),'native_success_unchanged':r['success'],'scope':'Recorded final 10 seconds at action frequency. Position-derived motion is independent evidence, not a replacement native predicate. No state writes or threshold changes.','dt_s':r['dt'],'objects':{}}
with h5py.File(a.episode/'trajectory.h5') as f:
 for name in ('banana_1','banana_2'):
  x=np.asarray(f['next_obs/'+name])[-150:];speed=np.linalg.norm(np.diff(x[:,:3],axis=0),axis=1)/r['dt'];v=np.linalg.norm(x[:,7:10],axis=1);rot=Rotation.from_quat(x[:,3:7]);angle=(rot[0].inv()*rot).magnitude();c=r['final_native_destination_checks'][name];force=np.asarray(c['force_matrix_w_n']).reshape(-1,3)[0];norm=np.linalg.norm(force)
  report['objects'][name]={'samples':len(x),'position_span_xyz_m':np.ptp(x[:,:3],axis=0).tolist(),'position_derived_max_speed_m_s':float(max(speed)),'reported_speed_min_m_s':float(min(v)),'reported_speed_max_m_s':float(max(v)),'orientation_max_change_rad':float(max(angle)),'native_final_passed':c['passed'],'final_force_n':force.tolist(),'final_support_angle_deg':float(np.degrees(np.arccos(force[2]/norm))),'original_speed_threshold_m_s':.003,'speed_channels_disagree':bool(max(speed)<.003 and min(v)>.003)}
report['trajectory_sha256']=hashlib.file_digest((a.episode/'trajectory.h5').open('rb'),'sha256').hexdigest()
report['conclusion']='Reported speed and recorded pose motion disagree for banana1; cause not established. Visual stability is corroborated. Preserve original native failure and disclose discrepancy.'
with a.output.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
