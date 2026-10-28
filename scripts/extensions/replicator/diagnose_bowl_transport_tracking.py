#!/usr/bin/env python3
"""Retained transport endpoint tracking; CPU only, no execution or controller change."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from scripts.extensions.replicator.task_audit_sidecars import load_episode, sha
from scripts.extensions.replicator.diagnose_bowl_transport import json_write


def run(episode):
    meta, data, _ = load_episode(episode)
    layout = meta['source_layout']
    kin = DroidKinematics(np.asarray(layout['robot_position'])+[0., 0., layout['stand_height']], layout['robot_yaw_deg'])
    result = json.loads((episode/'result.json').read_text())
    plans = [row for row in result['plans'] if row['phase'] == 'transport']
    if len(plans) != 1:
        raise ValueError('Require one retained transport attempt')
    plan = plans[0]
    index = result['plans'].index(plan)+1
    archive = episode/f'plan_{index:02d}_transport.npz'
    with np.load(archive, allow_pickle=False) as raw:
        goal, final_q = raw['goal_tcp'], raw['positions'][-1]
    arm = [meta['joint_names'].index(f'panda_joint{i}') for i in range(1, 8)]
    actions = [row for row in json.loads((episode/'actions.json').read_text()) if row['phase'] == 'transport']
    repeats = []
    for row in reversed(actions):
        if not np.allclose(np.asarray(row['action'])[:7], final_q, atol=1e-12, rtol=0.):
            break
        repeats.append(row)
    if not repeats:
        raise ValueError('No retained final target command')
    start, end = repeats[-1]['before_physics_step'], repeats[0]['after_physics_step']
    rows = []
    for i in np.flatnonzero((data['physics_step'] >= start) & (data['physics_step'] <= end)):
        actual = pose_matrix(data['tcp_pose_w'][i])
        q, dq = data['q'][i, arm], data['dq'][i, arm]
        fk = kin.fk(q)
        rows.append(dict(physics_step=int(data['physics_step'][i]), actual_q=q, actual_dq_rad_s=dq,
            actual_minus_command_rad=q-final_q,
            maximum_joint_error_rad=float(np.max(np.abs(q-final_q))),
            maximum_joint_speed_rad_s=float(np.max(np.abs(dq))),
            tcp_goal_position_error_m=float(np.linalg.norm(actual[:3, 3]-goal[:3, 3])),
            tcp_goal_xy_error_m=float(np.linalg.norm(actual[:2, 3]-goal[:2, 3])),
            tcp_goal_rotation_error_deg=float(np.rad2deg(Rotation.from_matrix(actual[:3, :3]@goal[:3, :3].T).magnitude())),
            actual_tcp_source_fk_position_error_m=float(np.linalg.norm(actual[:3, 3]-fk[:3, 3]))))
    return dict(accepted=False, physical_execution=False, controller_changed=False,
        episode=str(episode), repeated_final_command_actions=len(repeats),
        repeated_final_command_physics_steps=end-start,
        repeated_final_command_seconds=(end-start)*meta['physics_dt_s'],
        requested_goal_tcp=goal, final_command_q=final_q, commanded_q_source_fk=kin.fk(final_q),
        final_actual=rows[-1], full_endpoint_hold_samples=rows,
        input_sha256={name: sha(episode/name) for name in ['result.json', 'physics_metadata.json', 'physics_states.npz', 'actions.json', archive.name]},
        interpretation='Final action tracking and attachment slip are distinct. A bounded future hold cannot be assumed to converge and may permit further slip; all original collision/contact/5 mm landing gates remain mandatory.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = run(args.episode)
    json_write(args.output, report)
    print(json.dumps(dict(output=str(args.output), repeated_final_command_seconds=report['repeated_final_command_seconds'],
        final_tcp_error_m=report['final_actual']['tcp_goal_position_error_m'],
        final_joint_speed_rad_s=report['final_actual']['maximum_joint_speed_rad_s'])))


if __name__ == '__main__':
    main()
