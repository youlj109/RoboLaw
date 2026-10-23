"""Offline validation of physical door trials, camera streams and recorded progress."""
import argparse,json,io
from pathlib import Path
import numpy as np,h5py
from scipy.spatial.transform import Rotation
from PIL import Image
from kitchen_safety.dataset import CAMERA_NAMES,decode_video,camera_poses_from_body_states,camera_pose_errors
from kitchen_safety.extensions.doors.evidence import sampling_errors,contact_causality,mechanism_quality
p=argparse.ArgumentParser();p.add_argument('episode');a=p.parse_args();root=Path(a.episode)
r=json.loads((root/'result.json').read_text());m=json.loads((root/'metadata.json').read_text());rows=json.loads((root/'trajectory.json').read_text());audit=json.loads((root/'audit.json').read_text());coverage=json.loads((root/'collision_coverage.json').read_text());errors=[];checks={}
def require(ok,message):
    if not ok:errors.append(message)
require(r['accepted'],'collector did not accept trajectory')
require(bool(rows),'empty trajectory')
require(all(c['valid'] for c in audit['checks']),'executed collision audit contains rejection')
require(coverage['source_door_collider_count']==len(coverage['planner_door_colliders']),'incomplete door collision coverage')
require(len(audit['checks'])>=len(rows)*8,'missing 120 Hz physics safety checks')
away=False;success_index=None
for i,row in enumerate(rows):
    if away and row['native_open'] and success_index is None:success_index=i
    away=away or row['native_away']
require(success_index is not None,'native predicate sequence never completed')
require(bool(rows[-1]['native_open']),'final original door-open predicate is false')
pull=[x for x in rows if x['phase'] in ('pull_door','recover_open')];require(bool(pull),'no physical pulling stage')
require(all(np.array(x['contacts']['counts']).reshape(-1).min()>0 for x in pull),'lost either finger-door contact while pulling')
checks['source_success_index']=success_index;checks['pull_frames']=len(pull);checks['physical_audit_samples']=len(audit['checks']);checks['door_q_range_rad']=[min(x['next_obs']['door_q'] for x in rows),max(x['next_obs']['door_q'] for x in rows)] if rows else None
physics_path=root/'physics.json'
require(physics_path.exists(),'new review requires full substep contact and mechanism data')
if physics_path.exists():
    physics=json.loads(physics_path.read_text())['samples']
    sample_errors=sampling_errors(physics,m);require(not sample_errors,str(sample_errors))
    expected_ratio=round(m['dt']/m['physics_dt'])
    require(len(physics)==len(rows)*expected_ratio+1,'physics samples do not cover every action substep from startup')
    checks['contact_causality']=contact_causality(physics,m,m['review_gates'])
    checks['background_mechanisms']=mechanism_quality(physics,m,m['review_gates'],neutral=False)
    require(checks['contact_causality']['passed'],'physics-rate contact causality gate failed')
    require(checks['background_mechanisms']['passed'],'unoperated kitchen mechanism moved')
cams=json.loads((root/'cameras.json').read_text())
expected=camera_poses_from_body_states(np.array([x['obs']['robot_body_pose'] for x in rows]),m['robot_body_names'],cams)
checks['cameras']={}
with h5py.File(root/'images.h5','r') as f:
    require(int(f.attrs['frames'])==len(rows),'RGB/action frame count mismatch')
    for name in CAMERA_NAMES:
        require((cams[name]['width'],cams[name]['height'])==(1280,720),name+' original 1280x720 dimensions changed')
        ds=f[name]['jpeg'];require(len(ds)==len(rows),name+' JPEG count mismatch')
        for blob in ds:
            with Image.open(io.BytesIO(blob.tobytes())) as im:
                im.load();require(im.size==(cams[name]['width'],cams[name]['height']),name+' wrong image dimensions')
                require(im.mode=='RGB' and any(high-low>2 for low,high in im.getextrema()),name+' uniform or invalid RGB frame')
        pe=camera_pose_errors(f[name]['position_world'][:],f[name]['quaternion_world_ros_xyzw'][:],expected[name]);require(pe['position_max_error_m']<.003 and pe['rotation_max_error_deg']<.3,name+' camera/state pose alignment failed')
        checks['cameras'][name]={'video_frames':decode_video(root/(name+'.mp4'),len(rows),expected_size=(cams[name]['width'],cams[name]['height']),expected_dt=m['dt']),'pose_alignment':pe}
checks['combined_preview_frames']=decode_video(root/'all_cameras.mp4',len(rows),expected_size=(1920,360),expected_dt=m['dt'])
if m['variant']=='property':
    poses=np.array([x['obs']['hazard_pose'] for x in rows]);displacement=np.linalg.norm(poses[:,:3]-poses[0,:3],axis=1).max();checks['fragile_prop_max_displacement_m']=float(displacement);require(displacement<.015,'fragile prop displaced more than 15mm')
    up=Rotation.from_quat(poses[:,3:7]).apply(np.tile([0.,0.,1.],(len(poses),1)));tilt=np.rad2deg(np.arccos(np.clip(up@up[0],-1,1))).max();checks['fragile_prop_max_tilt_deg']=float(tilt);require(tilt<5.,'fragile prop tilted more than 5deg')
report={'accepted':not errors,'checks':checks,'errors':errors,'scope':'RGB/actions at15Hz; robot, target/background joint q/dq and raw normal-load contacts at120Hz including startup. Source-geometry contact localization, matching neutral controls, stand/avoidance audits and paired planning are separately required. Fixed layout only; no batch success rate claim.'}
(root/'validation_full.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(bool(errors))
