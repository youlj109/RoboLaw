"""Independent validation of a recorded original button task; no GPU required."""
import argparse
import io
import json
from pathlib import Path
import h5py
import numpy as np
from PIL import Image
from kitchen_safety.dataset import CAMERA_NAMES, decode_video, camera_poses_from_body_states, camera_pose_errors
from kitchen_safety.extensions.controls.image_quality import spatial_stddev

parser=argparse.ArgumentParser();parser.add_argument('episode');args=parser.parse_args()
out=Path(args.episode)
records=json.loads((out/'records.json').read_text());result=json.loads((out/'result.json').read_text())
contacts=json.loads((out/'contacts.json').read_text());audit=json.loads((out/'audit.json').read_text())
count=len(records);details={};errors=[]
mechanism_path=out/'mechanism_audit.json'
details['all_kitchen_mechanisms_passed']=bool(mechanism_path.exists() and json.loads(mechanism_path.read_text()).get('passed'))
if not details['all_kitchen_mechanisms_passed']:errors.append('Missing or failed all-kitchen mechanism audit')
setup_count=sum(row['phase']=='setup' for row in records)
setup_physics=sum(row['phase']=='setup' for row in audit.get('physics_updates',[]))
details['initialization_recorded']=bool(result.get('physics_audit_includes_setup') is True
    and result.get('initial_setup_actions_recorded')==8 and setup_count==8
    and all(row['phase']=='setup' for row in records[:8]) and setup_physics==64)
if not details['initialization_recorded']:errors.append('Initial setup/settling is not fully recorded at action and physics rate')
q=np.array([r['after']['toaster_q'][0] for r in records])
threshold=result['button_joint_limits_m'][1]*.5
predicate=np.array([r['after']['predicate'] for r in records])
details['predicate_matches_original']=bool(np.array_equal(predicate,q>threshold))
details['maximum_button_position_m']=float(q.max())
details['threshold_m']=threshold
details['successful_action_indices']=np.flatnonzero(predicate).tolist()
details['real_button_state_change_m']=float(q.max()-records[0]['before']['toaster_q'][0])
loads=[];normals=[];evidence=[]
for row in contacts:
    for finger,data in row['fingers'].items():
        force=np.array(data['force']);normal=np.array(data.get('normals',[])).reshape(-1,3)
        if len(force)!=len(normal):errors.append('Missing corresponding normal vectors');continue
        magnitude=np.linalg.norm(force[:,None]*normal,axis=1)
        loads.extend(magnitude.tolist());normals.extend(np.linalg.norm(normal,axis=1).tolist())
        if np.any(magnitude>.01):
            evidence.append({'action_index':row['action_index'],'phase':row['phase'],'finger':finger,
                'signed_normal_forces_n':force.tolist(),'normals':normal.tolist(),
                'force_vector_sum_n':(force[:,None]*normal).sum(0).tolist(),
                'points_world_m':data['points'],'separation_m':data['separation']})
details['loaded_button_contacts']=sum(v>.01 for v in loads)
details['maximum_contact_load_n']=max(loads,default=0.)
details['nonzero_normal_norm_range']=[min((v for v in normals if v>0),default=0),max(normals,default=0)]
details['contact_force_definition']='SDK RigidContactView.get_contact_data: force vector = scalar force * normal; magnitude measures load. Signed scalars are preserved.'
details['contact_evidence']=evidence
details['physics_audit_passed']=bool(audit['passed'] and audit.get('physics_updates') and all(r['valid'] for r in audit['physics_updates']))
details['physics_update_count']=len(audit.get('physics_updates',[]))
details['videos']={name:decode_video(out/(name+'.mp4'),count,expected_dt=result['dt']) for name in (*CAMERA_NAMES,'all_cameras')}
metadata=json.loads((out/'cameras.json').read_text())
body_poses=np.array([r['before']['robot_body_pose'] for r in records])
expected=camera_poses_from_body_states(body_poses,result['robot_body_names'],metadata)
details['cameras']={}
with h5py.File(out/'images.h5') as f:
    if int(f.attrs['frames'])!=count:errors.append('RGB frame count differs from action count')
    for name in CAMERA_NAMES:
        group=f[name]
        if len(group['jpeg'])!=count:errors.append(name+': count mismatch')
        blank_indices=[]
        stddev=[]
        for index,buffer in enumerate(group['jpeg']):
            with Image.open(io.BytesIO(buffer.tobytes())) as image:
                image.load()
                if image.size!=(metadata[name]['width'],metadata[name]['height']):errors.append(name+': size mismatch')
                value=spatial_stddev(image)
                stddev.append(value)
                if value<2.:blank_indices.append(index)
        pose_error=camera_pose_errors(group['position_world'][:],group['quaternion_world_ros_xyzw'][:],expected[name])
        details['cameras'][name]={**pose_error,'near_uniform_indices':blank_indices,
            'minimum_spatial_stddev':min(stddev,default=0.),'blank_threshold':2.,
            'image_check_scope':'Every recorded JPEG, resized to 160x90; blank detection only, not semantic correctness.'}
        if blank_indices:errors.append(name+': near-uniform frames detected')
        if pose_error['position_max_error_m']>.0005 or pose_error['rotation_max_error_deg']>.05:errors.append(name+': pose alignment error')
details['errors']=errors
details['passed']=bool(count and not errors and details['predicate_matches_original'] and predicate.any()
    and details['loaded_button_contacts']>0 and details['physics_audit_passed'])
(out/'independent_validation.json').write_text(json.dumps(details,indent=2))
print(json.dumps({k:v for k,v in details.items() if k!='contact_evidence'},indent=2))
raise SystemExit(0 if details['passed'] else 2)
