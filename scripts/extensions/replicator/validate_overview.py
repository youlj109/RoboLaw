"""Independently validate supplementary overview RGB/pose/action synchronization."""
import argparse,hashlib,io,json
from pathlib import Path
import h5py,numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from kitchen_safety.dataset import CAMERA_NAMES,camera_pose_errors,decode_video

def validate(out):
 result=json.loads((out/'result.json').read_text());spec=result['overview_camera'];extra=out/'overview';meta=json.loads((extra/'cameras.json').read_text());alignment=json.loads((extra/'frame_alignment.json').read_text())['measurements'];checks={};details={}
 with h5py.File(out/'trajectory.h5') as trajectory:
  n=len(trajectory['action']);dt=float(trajectory.attrs['dt']);phases=trajectory['phase'].asstr()[:];times=trajectory['time'][:]
 checks['nonempty']=n>0;checks['overview_metadata']=set(meta)=={'overview_camera'} and isinstance(spec,dict)
 with h5py.File(out/'images.h5') as primary:
  checks['original_three_camera_names_and_lengths']=set(primary)==set(CAMERA_NAMES) and all(len(primary[name+'/jpeg'])==n for name in CAMERA_NAMES)
 sync=len(alignment)==n
 for i,item in enumerate(alignment):
  sync &= (i<n and item.get('action_index')==i and item.get('primary_frame_index')==i and item.get('overview_frame_index')==i and item.get('phase')==phases[i] and isinstance(item.get('physics_step_before'),int) and item.get('physics_step_before')==item.get('physics_step_after'))
 captured_times=np.array([x.get('simulation_time_s',float('nan')) for x in alignment]);checks['frame_physics_synchronization']=bool(sync and np.isfinite(captured_times).all() and len(captured_times)==n and np.allclose(captured_times-captured_times[0],times,atol=1e-6,rtol=1e-6))
 camera=meta['overview_camera']
 with h5py.File(extra/'images.h5') as f:
  g=f['overview_camera'];checks['overview_array_shapes']=int(f.attrs['frames'])==n and abs(float(f.attrs['dt'])-dt)<1e-9 and len(g['jpeg'])==n and g['position_world'].shape==(n,3) and g['quaternion_world_ros_xyzw'].shape==(n,4) and (camera['width'],camera['height'])==(1280,720)
  jpeg=True
  for blob in g['jpeg']:
   with Image.open(io.BytesIO(blob.tobytes())) as image:image.load();jpeg &= image.size==(1280,720)
  checks['all_jpeg_frames_decoded']=jpeg;positions=g['position_world'][:];quaternions=g['quaternion_world_ros_xyzw'][:];ros=(Rotation.from_quat(spec['quaternion_opengl_xyzw'])*Rotation.from_quat([1,0,0,0])).as_quat();expected={'position_world':np.tile(spec['position_world'],(n,1)),'quaternion_world_ros_xyzw':np.tile(ros,(n,1))};error=camera_pose_errors(positions,quaternions,expected);details['pose_errors']=error;checks['fixed_camera_pose_matches_spec']=bool(np.isfinite(positions).all() and np.isfinite(quaternions).all() and error['position_max_error_m']<.001 and error['rotation_max_error_deg']<.1)
 root_video=out/'overview_camera.mp4';count=decode_video(root_video,n,expected_dt=dt,expected_size=(1280,720));panel=decode_video(extra/'all_cameras.mp4',n,expected_dt=dt,expected_size=(640,360));checks['all_mp4_frames_decoded_and_duration']=count==n and panel==n;checks['root_video_matches_raw_export']=hashlib.sha256(root_video.read_bytes()).digest()==hashlib.sha256((extra/'overview_camera.mp4').read_bytes()).digest();details.update(frames=n,dt=dt,decoded_video_frames=count,panel_frames=panel,alignment_records=len(alignment))
 return dict(passed=all(checks.values()),checks=checks,details=details,scope='Supplementary fixed overview before the same original three camera actions; unchanged physics step during capture; all JPEG/MP4 frames decoded and measured camera poses checked. Source/physical/self/fixed-base and original-camera validation remain separate hard gates. Whole-human/task visibility requires separate image review.')

def main():
 p=argparse.ArgumentParser();p.add_argument('episode');a=p.parse_args();out=Path(a.episode)
 try:report=validate(out)
 except Exception as ex:report=dict(passed=False,error=f'{type(ex).__name__}: {ex}')
 (out/'overview_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return int(not report['passed'])
if __name__=='__main__':raise SystemExit(main())
