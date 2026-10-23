"""Diagnose release IK from a retained physical failure; never execute it."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    metadata = json.loads((args.episode / 'metadata.json').read_text())
    trajectory = args.episode / 'trajectory.json'
    rows = json.loads(trajectory.read_text())
    first = next(i for i, row in enumerate(rows) if row['phase'] == 'release_handle')
    observation = rows[first]['obs']
    config = json.loads(Path(metadata['variant_config']).read_text())
    _, kin, _, geometry = reconstruct(metadata, config)
    margin = metadata['controller_parameters'].get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    start_q = np.asarray(observation['q'][:7])
    start = kin.fk(start_q)
    approach = Rotation.from_euler('z', metadata['robot_yaw_deg'], degrees=True).apply([1., 0., 0.])
    from kitchen_safety.extensions.doors.task_model import joint_position_from_openness
    reference = joint_position_from_openness(geometry['reset_openness'], *geometry['limits_rad'])
    normal = Rotation.from_rotvec(np.asarray(geometry['axis']) * (observation['door_q'] - reference)).apply(approach)
    report = {
        'episode': str(args.episode),
        'trajectory_sha256': hashlib.sha256(trajectory.read_bytes()).hexdigest(),
        'release_first_action': first,
        'start_q': start_q.tolist(),
        'scope': 'Offline bounded IK diagnostic only. No dynamic rollout, collision acceptance, contact acceptance or task success claim.',
        'candidates': [],
    }
    for tilt in [0., -30., 30., -45., 45.]:
        direction = -normal * np.cos(np.deg2rad(tilt)) + np.asarray([0., 0., np.sin(np.deg2rad(tilt))])
        direction /= np.linalg.norm(direction)
        for length in [.005, .01, .015, .02, .025, .03]:
            q = start_q.copy()
            entry = {'length_m': length, 'tilt_deg': tilt, 'ik_all_targets_solved': False}
            try:
                for fraction in np.linspace(.05, 1., 20):
                    target = start.copy()
                    target[:3, 3] += direction * length * fraction
                    command = kin.ik(target, q, attempts=1)
                    delta = command - q
                    q += delta / max(1., float(np.max(np.abs(delta))) / .025)
                entry['ik_all_targets_solved'] = True
            except Exception as error:
                entry.update(error=str(error), rejected_fraction=float(fraction))
            report['candidates'].append(entry)
            print(entry, flush=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
