"""Offline bounded position retreat diagnosis on observed release states only."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.release_pose_lwx import release_progress
from kitchen_safety.planning import transform


def bounded_position_command(kin, seed, target, reference_rotation):
    """A proposal only: prioritize position, penalize rotation and joint travel."""
    seed = np.asarray(seed, dtype=float)
    low = np.maximum(kin.limits[:, 0] + 1e-8, seed - .025)
    high = np.minimum(kin.limits[:, 1] - 1e-8, seed + .025)
    if np.any(low >= high):
        raise ValueError('No bounded feasible joint interval')
    def residual(q):
        pose = kin.fk(q)
        angle = Rotation.from_matrix(pose[:3, :3] @ reference_rotation.T).as_rotvec()
        return np.r_[pose[:3, 3] - target, .005*angle, .0005*(q-seed)]
    result = least_squares(residual, np.clip(seed, low+1e-10, high-1e-10),
                           bounds=(low, high), max_nfev=80, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    return result.x


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Evidence exists')
    sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    m = json.loads((a.episode/'metadata.json').read_text())
    rows = json.loads((a.episode/'trajectory.json').read_text())
    config = json.loads(Path(m['variant_config']).read_text())
    stage, kin, world, geometry = reconstruct(m, config, m['variant_spec'])
    world.allow_handle_contact = True
    cp = m['controller_parameters']
    margin = cp.get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    before = [r for r in rows if r['phase'] in ['success_hold', 'recover_open']][-1]['next_obs']
    start = kin.fk(np.asarray(before['q'][:7]))
    approach = Rotation.from_euler('z', m['robot_yaw_deg'], degrees=True).apply([1., 0., 0.])
    normal = Rotation.from_rotvec(np.asarray(geometry['axis'])*(before['door_q']-m['door_motion_reference_joint_rad'])).apply(approach)
    direction = -normal*np.cos(np.deg2rad(cp['release_tilt_deg'])) + np.array([0., 0., np.sin(np.deg2rad(cp['release_tilt_deg']))])
    direction /= np.linalg.norm(direction)
    initial = transform(m['initial_door_pose'][:3], m['initial_door_pose'][3:])
    report = dict(episode=str(a.episode), trajectory_sha256=sha(a.episode/'trajectory.json'),
                  metadata_sha256=sha(a.episode/'metadata.json'), implementation_sha256=sha(__file__),
                  scope='CPU bounded position-only command proposals on actual release observations. No GPU self collision, response prediction, future state, simulation execution or acceptance claim.',
                  physics_replay=False, unsafe_path_executed=False, orientation_limit_deg=10., samples=[])
    for i, row in enumerate(r for r in rows if r['phase']=='release_handle'):
        observed = np.asarray(row['obs']['q'][:7])
        original = np.asarray(row['action'][:7])
        actual = np.asarray(row['next_obs']['q'][:7])
        progress = release_progress((i+1)/20., cp.get('release_retreat_profile', 'linear'))
        target = start[:3, 3] + direction*cp['release_retreat_m']*progress
        pose = row['next_obs']['door_body_pose']
        delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(initial)
        for body in geometry['moving_bodies']:
            world.move_body(body, delta)
        fingers = {n: row['next_obs']['q'][m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
        item = dict(index=i, original_cap_joint=int(np.argmax(np.abs(original-observed)))+1,
                    original_max_joint_step_rad=float(np.abs(original-observed).max()), modes={})
        for mode in ['original', 'bounded_position']:
            try:
                q = original if mode=='original' else bounded_position_command(kin, observed, target, start[:3, :3])
                fk = kin.fk(q)
                checks = [world.clearance(observed+(q-observed)*f, mode='execution', gripper_q=fingers) for f in np.linspace(0.,1.,5)]
                orientation = float(np.rad2deg(Rotation.from_matrix(fk[:3, :3] @ start[:3, :3].T).magnitude()))
                item['modes'][mode] = dict(command=q.tolist(), position_error_m=float(np.linalg.norm(fk[:3,3]-target)),
                    orientation_departure_deg=orientation, orientation_within_limit=orientation<=10.,
                    tcp_release_advance_m=float(np.dot(fk[:3,3]-kin.fk(observed)[:3,3],direction)),
                    max_joint_step_rad=float(np.abs(q-observed).max()),
                    conservative_world_valid=all(c['valid'] for c in checks),
                    minimum_slack_m=min(c['slack_m'] for c in checks))
            except (ValueError, RuntimeError) as exc:
                item['modes'][mode] = dict(error=str(exc))
        report['samples'].append(item)
    report['complete'] = True
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    for mode in ['original', 'bounded_position']:
        values=[r['modes'][mode] for r in report['samples']]
        print('POSITION_DIAG',mode,'samples',len(values),'errors',sum('error' in r for r in values),
              'invalid',sum(not r.get('conservative_world_valid',False) for r in values),
              'max_orientation_deg',max((r.get('orientation_departure_deg',0) for r in values),default=0),
              'min_tcp_advance_m',min((r.get('tcp_release_advance_m',0) for r in values),default=0),flush=True)


if __name__ == '__main__':
    main()
