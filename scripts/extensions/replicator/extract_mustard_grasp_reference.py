#!/usr/bin/env python3
"""Extract measured bottle-local grasp candidates from a saved real G episode.

Read-only CPU provenance/pose extraction; no claim that a transferred candidate
passes the stricter independent family contact or environment checks.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import h5py
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import ROBOT_USD,DroidKinematics
from kitchen_safety.extensions.replicator.mustard_family_preparation import sha
from scripts.extensions.replicator.variant_geometry import code_evidence


def pose(xyz,quat):
    value=np.eye(4);value[:3,:3]=Rotation.from_quat(quat).as_matrix();value[:3,3]=xyz
    return value


def extract(episode):
    episode=Path(episode);result=json.loads((episode/'result.json').read_text())
    provenance=json.loads((episode/'collection_provenance.json').read_text())
    if not result['success'] or not result['arena_success'] or not provenance['passed']:
        raise ValueError('Require a successful real source episode with unchanged collection provenance')
    if result['layout']['task_id']!='kitchen_bench_replicator_g_shape_mustard_bowl':
        raise ValueError('Reference extraction is explicitly scoped to original G mustard')
    snapshot={path:entry for path,entry in provenance['entries'].items() if entry.get('source_text') is not None}
    controls={path:entry for path,entry in snapshot.items() if path.endswith(('collect.py','grasp_geometry.py','collision_planner.py'))}
    if len(controls)!=3:raise ValueError('Require original collector/grasp geometry/contact checker snapshots')
    for path,entry in controls.items():
        if (hashlib.sha256(entry['source_text'].encode()).hexdigest()!=entry['initial_sha256']
                or entry['initial_sha256']!=entry['checked_sha256'] or not entry['unchanged']):
            raise ValueError('Corrupt execution source snapshot: '+path)
    layout=result['layout'];kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
    observations={}
    with h5py.File(episode/'trajectory.h5') as f:
        if f.attrs['quaternion_order']!='xyzw':raise ValueError('Unknown actual quaternion order')
        phases=f['phase'].asstr()[:];names=json.loads(f.attrs['joint_names'])
        for phase in ('pregrasp','approach','close'):
            indices=np.flatnonzero(phases==phase)
            if not len(indices) or not np.all(np.diff(indices)==1):raise ValueError('Missing/noncontiguous source phase')
            index=int(indices[-1]);group=f['next_obs'];bottle=np.asarray(group['mustard'][index])[:7]
            tcp=pose(group['tcp'][index],group['tcp_quat_xyzw'][index]);bp=pose(bottle[:3],bottle[3:7]);q=group['q'][index]
            fk=kin.fk(q[:7])
            observations[phase]=dict(action_index=index,state='next_obs: after this actual action',
                time_after_action_s=float((index+1)*f.attrs['dt']),bottle_world=bp.tolist(),tcp_world=tcp.tolist(),
                bottle_to_tcp_local=(np.linalg.inv(bp)@tcp).tolist(),q_rad=q.tolist(),joint_names=names,
                fk_to_observed_tcp_translation_m=float(np.linalg.norm(fk[:3,3]-tcp[:3,3])),
                fk_to_observed_tcp_rotation_deg=float(np.rad2deg(Rotation.from_matrix(fk[:3,:3]@tcp[:3,:3].T).magnitude())))
        start=int(np.flatnonzero(phases=='pregrasp')[0]);initial=f['obs/mustard'][start][:7]
    requested=result['requested_grasp'];planned=next(p for p in result['plans'] if p['phase']=='approach')
    planned_local=np.linalg.inv(pose(initial[:3],initial[3:]))@np.asarray(planned['effective_goal_pose_world'])
    candidates=[dict(name='G_actual_approach_end',bottle_to_tcp_local=observations['approach']['bottle_to_tcp_local'],
                     derivation='Measured open-finger TCP/bottle at the end of the actual approach'),
        dict(name='G_actual_close_end',bottle_to_tcp_local=observations['close']['bottle_to_tcp_local'],
             derivation='Measured closed-gripper TCP/bottle after physical grasp; open-finger migration is unverified'),
        dict(name='G_planned_approach',bottle_to_tcp_local=planned_local.tolist(),
             derivation='Saved effective approach target in the measured pregrasp-start bottle frame; not measured execution')]
    inputs={name:sha(episode/name) for name in ('result.json','trajectory.h5','collection_provenance.json','grasp_contacts.json')}
    return dict(reference_episode=str(episode),reference_input_sha256=inputs,source_physical_success=True,
        observations=observations,ordered_transfer_candidates=candidates,
        requested_controls={key:requested[key] for key in ('explicit_orientation','tilt_deg','yaw_deg','offset_z_m','offset_y_m','source_bottle_position_world_m','target_world_m')},
        source_plans=[p for p in result['plans'] if p['phase'] in ('pregrasp','approach')],
        source_execution_code=controls,source_robot_sha256=sha(ROBOT_USD),
        source_bottle_sha256=sha(ROOT/'assets/robolab/objects/hope/mustard_bottle.usd'),
        selected_finger_phase='approach/close',migration_finger_angles='Original open fingers, all six joint angles zero; closed reference angles are retained but do not initialize a grasp',
        scope='Source observed poses and saved planner targets only. G success does not certify these candidates under a new layout, full-pair policy, physical contact, source stability or cuRobo.',
        execution_ready=False,migration_passed=False,**code_evidence([Path(__file__)]))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--episode',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=extract(args.episode);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),candidates=[dict(name=x['name'],local_xyz=np.asarray(x['bottle_to_tcp_local'])[:3,3].tolist()) for x in result['ordered_transfer_candidates']],execution_ready=False)))


if __name__=='__main__':main()
