"""Independent original-level, causal contact, recording and release review."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import h5py
import numpy as np
from PIL import Image

from kitchen_safety.dataset import CAMERA_NAMES, decode_video, camera_poses_from_body_states, camera_pose_errors
from kitchen_safety.extensions.controls.image_quality import spatial_stddev
from kitchen_safety.extensions.controls.task_contract import task_contract
from kitchen_safety.extensions.controls.property_motion import property_motion_review


def review(path):
    paths={name:path/(name+'.json') for name in ('result','records','audit','contacts','mechanism_audit','cameras','config')}
    data={name:json.loads(p.read_text()) for name,p in paths.items()}
    result,rows,audit=data['result'],data['records'],data['audit']
    contract=task_contract('temperature')
    if result['task_id']!=contract['task_id'] or not rows:
        raise ValueError('Missing original temperature-task recording')
    target=result['toaster_joint_names'].index(contract['joint'])
    updates=audit.get('physics_updates',[])
    property_motion=property_motion_review(result,updates)
    positions=np.array([u['toaster_q'][target] for u in updates])
    velocities=np.array([u['toaster_dq'][target] for u in updates])
    def levels(q):
        # Same floor operation and float32 semantics as the fixed source method.
        import torch
        value=torch.tensor(q,dtype=torch.float32)
        low,high=np.deg2rad(30.),np.deg2rad(150.)
        active=torch.floor((torch.clamp(value,low,high)-low)/(high-low)*4)
        return torch.where(value<low,torch.full_like(active,-1),active).to(int).numpy()
    action_positions=np.array([r['after']['toaster_q'][target] for r in rows])
    action_levels=levels(action_positions)
    predicates=np.array([r['after']['predicate'] for r in rows])
    loads=np.array([max((abs(f) for finger in u['button_contacts'].values() for f in finger['normal_force']),default=0.) for u in updates])
    loaded=np.flatnonzero(loads>.01)
    success=np.flatnonzero(levels(positions)==3)
    first_contact=int(loaded[0]) if len(loaded) else None
    first_success=int(success[0]) if len(success) else None
    initial=rows[0]['before']['toaster_q'][target]
    motion=np.flatnonzero(abs(positions-initial)>np.deg2rad(.1))
    first_motion=int(motion[0]) if len(motion) else None
    prefix=positions[:first_contact] if first_contact is not None else positions
    precontact=float(np.max(abs(prefix-initial),initial=0.))
    bodyposes=np.array([r['before']['robot_body_pose'] for r in rows])
    expected=camera_poses_from_body_states(bodyposes,result['robot_body_names'],data['cameras'])
    cameras={}
    with h5py.File(path/'images.h5') as f:
        for name in CAMERA_NAMES:
            g=f[name]
            blank=[];sizes=[]
            for index,buffer in enumerate(g['jpeg']):
                with Image.open(io.BytesIO(buffer.tobytes())) as image:
                    image.load()
                    if spatial_stddev(image)<2.:blank.append(index)
                    if image.size!=(data['cameras'][name]['width'],data['cameras'][name]['height']):sizes.append(index)
            errors=camera_pose_errors(g['position_world'][:],g['quaternion_world_ros_xyzw'][:],expected[name])
            cameras[name]={**errors,'blank_indices':blank,'wrong_size_indices':sizes,'frames':len(g['jpeg'])}
    videos={name:decode_video(path/(name+'.mp4'),len(rows),expected_dt=result['dt']) for name in (*CAMERA_NAMES,'all_cameras')}
    phases=[r['phase'] for r in rows]
    target_path='/World/envs/env_0/kitchen/'+contract['contact_body']+'/'+contract['joint']
    allowed=[j['path'] for j in data['mechanism_audit']['joints'] if j['allowed_task_motion']]
    release=[i for i,r in enumerate(rows) if r['phase'] in ('release','retreat')]
    retreat_positions=[r[when]['toaster_q'][target] for r in rows if r['phase']=='retreat' for when in ('before','after')]
    retreat_drift=float(np.ptp(retreat_positions)) if retreat_positions else float('inf')
    robot_delta=max(float(np.max(abs(np.asarray(r['after']['q'])[:7]-np.asarray(r['before']['q'])[:7]))) for r in rows)
    checks={
        'source_task_and_hash':result['source_sha256']==hashlib.sha256(contract['source'].read_bytes()).hexdigest(),
        'actual_original_level_three':bool(np.array_equal(predicates,action_levels==3) and predicates.any() and action_levels[-1]==3),
        'original_target_and_reset':data['config']['original_target_level']==3 and data['config']['original_reset_level']==-1 and abs(initial)<1e-5,
        'no_object_setters_after_reset':result.get('physical_object_setters_after_reset') is False,
        'runtime_reset_suppression_declared':result.get('automatic_reset_policy','').startswith('All post-initialization reset requests suppressed'),
        'no_robot_reset_discontinuity':robot_delta<.15,
        'all_physics_updates_present':len(updates)==len(rows)*round(result['dt']/result['physics_dt']),
        'setup_fully_recorded':phases[:8]==['setup']*8 and sum(u['phase']=='setup' for u in updates)==64,
        'contact_before_target_motion':first_contact is not None and first_motion is not None and first_contact<first_motion,
        'contact_before_success':first_contact is not None and first_success is not None and first_contact<first_success,
        'precontact_target_stationary':precontact<np.deg2rad(.1),
        'exactly_one_target_joint_exempt':allowed==[target_path],
        'background_stationary':data['mechanism_audit']['passed'] is True,
        'property_pose_stable_in_episode':property_motion['passed'],
        'physics_clearance':audit['passed'] is True and all(u['valid'] for u in updates),
        'release_and_retreat_recorded':bool(release) and phases[-1]=='retreat' and all(rows[i]['action'][-1]==0 for i in release),
        'released_target_stationary':retreat_drift<np.deg2rad(1.),
        'three_cameras_complete':all(c['frames']==len(rows) and not c['blank_indices'] and not c['wrong_size_indices'] and c['position_max_error_m']<.0005 and c['rotation_max_error_deg']<.05 for c in cameras.values()),
        'collector_completed':result.get('accepted') is True,
    }
    checks={name:bool(value) for name,value in checks.items()}
    return {'passed':all(checks.values()),'checks':checks,'cameras':cameras,'videos':videos,'property_motion':property_motion,
        'task_id':contract['task_id'],'physics_samples':len(updates),'frames':len(rows),
        'first_loaded_contact_sample':first_contact,'first_target_motion_sample':first_motion,'first_success_sample':first_success,
        'first_success_time_s':None if first_success is None else (first_success+1)*result['physics_dt'],
        'precontact_max_rotation_rad':precontact,'maximum_angle_deg':float(np.rad2deg(positions.max())),
        'maximum_speed_rad_s':float(abs(velocities).max()),'final_level':int(action_levels[-1]),
        'maximum_robot_action_delta_rad':robot_delta,'robot_reset_discontinuity_limit_rad':.15,
        'retreat_target_drift_deg':float(np.rad2deg(retreat_drift)),'retreat_target_drift_limit_deg':1.,
        'input_sha256':{name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in paths.items()},
        'scope':'Independent recording/causality review; actual original predicate retained. Not a substitute for self-collision, hazard necessity, paired planning, source-layout or neutral acceptance.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path)
    args=parser.parse_args()
    destination=args.episode/'independent_temperature_validation.json'
    if destination.exists():raise FileExistsError(destination)
    report=review(args.episode)
    destination.write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 2)
